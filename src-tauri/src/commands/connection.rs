// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Connection Commands — serial port listing, connect, disconnect, BLE scanning

use tauri::{AppHandle, Emitter, State};

use crate::flightlog::msp_raw_logger::MspRawSink;
use crate::flightlog::recorder::FlightRecorder;
use crate::flightlog::types::{FlightLogSettings, InavStats};
use crate::mavlink_proto;
use crate::msp::{
    FcInfo, FeatureSet, InavVersion, MspTransport, MSP_API_VERSION, MSP_BLACKBOX_CONFIG, MSP_BOARD_INFO, MSP_EEPROM_WRITE,
    MSP_FC_VARIANT, MSP_FC_VERSION, MSP_NAME, MSP_SET_NAME, MSP_UID, MSP_WP, MSPV2_INAV_MIXER,
};
use crate::msp::features::is_version_supported;
use crate::scheduler;
use crate::scheduler::TelemetryConfig;
use crate::state::{ActiveProtocol, AppState};
use crate::transport::{ByteTransport, Transport, TransportType};
use crate::vehicle_registry::emitter::VehicleEmitter;
use crate::vehicle_registry::{LinkEntry, LinkId, VehicleId, VehicleInfo, ERR_NOT_CONNECTED};
use crate::transport::PortInfo;
use crate::transport::serial::SerialConnection;
use crate::transport::tcp::TcpTransport;
use crate::transport::udp::UdpTransport;
// `transport::ble` resolves per platform behind one name (btleplug on desktop, CoreBluetooth on iOS).
use crate::transport::ble::{self as ble_backend, BleDeviceInfo};

/// Home position pushed to the frontend (event `home-position`). Same shape/name regardless of
/// protocol so MAVLink (HOME_POSITION) can emit it identically later.
#[derive(serde::Serialize, Clone)]
struct HomeEvent {
    lat: f64,
    lon: f64,
    alt: f64,
}

/// What `connect` hands back: the registry ids of the new link / its primary vehicle plus the
/// handshake info (the pre-multi-vehicle return value).
#[derive(serde::Serialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct ConnectResult {
    pub link_id: LinkId,
    pub vehicle_id: String,
    pub fc_info: FcInfo,
}

/// A protocol path's result: the started handler and what it learned about the vehicle. `connect`
/// turns this into a registry entry.
struct Opened {
    protocol: ActiveProtocol,
    protocol_name: &'static str,
    fc_info: FcInfo,
    /// MAVLink system id of the handshake vehicle; 0 for MSP / passive (one vehicle per link).
    sysid: u8,
    /// MAVLink autopilot component id; 0 for MSP / passive.
    compid: u8,
}

/// `link-closed` event payload.
#[derive(serde::Serialize, Clone)]
#[serde(rename_all = "camelCase")]
struct LinkClosed {
    link_id: LinkId,
    reason: &'static str,
}

/// `vehicle-lost` event payload.
#[derive(serde::Serialize, Clone)]
#[serde(rename_all = "camelCase")]
struct VehicleLost {
    vehicle_id: String,
    link_id: LinkId,
    reason: &'static str,
}

/// `active-vehicle-changed` event payload.
#[derive(serde::Serialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct ActiveVehicleChanged {
    pub vehicle_id: Option<String>,
}

/// List available serial ports. On iOS this is always empty — `transport::serial` resolves to the
/// stand-in there (no serial access exists), and the UI hides serial on mobile anyway.
#[tauri::command]
pub fn list_serial_ports() -> Vec<PortInfo> {
    crate::transport::serial::list_ports()
}

/// Scan for BLE devices matching known serial profiles
#[tauri::command]
pub async fn scan_ble_devices() -> Result<Vec<BleDeviceInfo>, String> {
    ble_backend::scan_ble_devices().await
}

/// Start a live BLE scan session. Discovered/updated devices are emitted as `ble-device` events
/// for the frontend to populate in real time. Restarts any previous session.
#[tauri::command]
pub async fn ble_scan_start(app: AppHandle, state: State<'_, AppState>) -> Result<(), String> {
    let (tx, rx) = tokio::sync::oneshot::channel::<()>();
    {
        // Replace any existing sender — dropping the old one ends the previous session.
        let mut guard = state.ble_scan_stop.lock().map_err(|e| e.to_string())?;
        *guard = Some(tx);
    }
    tauri::async_runtime::spawn(async move {
        if let Err(e) = ble_backend::run_scan_session(app, rx).await {
            log::warn!("BLE scan session ended: {}", e);
        }
    });
    Ok(())
}

/// Stop the live BLE scan session (if any).
#[tauri::command]
pub fn ble_scan_stop(state: State<'_, AppState>) -> Result<(), String> {
    let mut guard = state.ble_scan_stop.lock().map_err(|e| e.to_string())?;
    *guard = None; // drop the sender → the session's stop future resolves
    Ok(())
}

/// Write a craft name to the connected INAV FC (MSP_SET_NAME) and persist it (MSP_EEPROM_WRITE),
/// then update the cached `fc_info`. INAV/MSP only; rejected for non-MSP / disconnected links. Used
/// post-flight to push a newly chosen craft name to the FC so future flights auto-link to a vehicle.
#[tauri::command(async)]
pub fn inav_set_craft_name(name: String, state: State<'_, AppState>) -> Result<(), String> {
    let trimmed = name.trim();
    // INAV stores the craft name in a fixed 16-byte buffer; clamp to the conventional limit.
    if trimmed.len() > 16 {
        return Err("Craft name too long (max 16 characters)".into());
    }
    let mut reg = state.links.lock().map_err(|e| e.to_string())?;
    let active = reg.active().cloned().ok_or(ERR_NOT_CONNECTED)?;
    {
        let handle = match reg.active_protocol() {
            Some(ActiveProtocol::Msp(h)) => h,
            Some(_) => return Err("FC is not running MSP (INAV)".into()),
            None => return Err(ERR_NOT_CONNECTED.into()),
        };
        handle.msp_request(MSP_SET_NAME, trimmed.as_bytes())?;
        handle.msp_request(MSP_EEPROM_WRITE, &[])?;
    }
    // Keep the cached craft name in sync so the UI reflects it without a reconnect.
    if let Some(entry) = reg.get_mut(active.link) {
        entry.fc_info.craft_name = trimmed.to_string();
    }
    Ok(())
}

/// Read the INAV lifetime flight statistics from the FC `stats` settings (MSP2_COMMON_SETTING by
/// name). `enabled` mirrors the `stats` toggle; the totals are only meaningful when it is on. Used
/// to offer the FC's lifetime totals as a vehicle baseline. INAV/MSP only.
#[tauri::command(async)]
pub fn inav_read_stats(state: State<'_, AppState>) -> Result<InavStats, String> {
    use crate::commands::fc_settings::read_uint_setting;
    let proto = state.links.lock().map_err(|e| e.to_string())?;
    let handle = match proto.active_protocol() {
        Some(ActiveProtocol::Msp(h)) => h,
        Some(_) => return Err("FC is not running MSP (INAV)".into()),
        None => return Err("Not connected".into()),
    };
    let enabled = read_uint_setting(handle, "stats").unwrap_or(0) != 0;
    let mut stats = InavStats { enabled, ..Default::default() };
    if enabled {
        stats.flight_count = read_uint_setting(handle, "stats_flight_count").unwrap_or(0) as i64;
        stats.total_time_s = read_uint_setting(handle, "stats_total_time").unwrap_or(0) as i64;
        stats.total_dist_m = read_uint_setting(handle, "stats_total_dist").unwrap_or(0) as i64;
        stats.total_energy = read_uint_setting(handle, "stats_total_energy").unwrap_or(0) as i64;
    }
    Ok(stats)
}

/// Connect to a flight controller on the given transport and protocol.
/// MSP: Performs handshake + starts telemetry scheduler.
/// MAVLink: Waits for HEARTBEAT + starts handler thread.
#[tauri::command]
#[allow(clippy::too_many_arguments)] // Tauri command — args map to frontend invoke() params
pub async fn connect(
    transport_type: TransportType,
    // Protocol selection ("msp" or "mavlink", defaults to "msp")
    protocol: Option<String>,
    // Serial params
    port: Option<String>,
    baud_rate: Option<u32>,
    // TCP/UDP params
    host: Option<String>,
    tcp_port: Option<u16>,
    // BLE params
    ble_device_id: Option<String>,
    // Telemetry config
    attitude_rate_hz: Option<f64>,
    position_rate_hz: Option<f64>,
    airspeed_enabled: Option<bool>,
    wind_enabled: Option<bool>,
    // MAVLink: when true, request no stream rates — FC streams per its own SRn_* params (ADR-043)
    mavlink_full_telemetry: Option<bool>,
    // Flight log config
    flight_log_enabled: Option<bool>,
    flight_log_db_enabled: Option<bool>,
    flight_log_path: Option<String>,
    flight_log_raw_path: Option<String>,
    flight_log_raw: Option<bool>,
    flight_log_raw_always: Option<bool>,
    state: State<'_, AppState>,
    app_handle: AppHandle,
) -> Result<ConnectResult, String> {
    // Multi-vehicle: every connect opens an additional link. Reserve its id now (the handlers stamp
    // their events with it) and, on the first link only, start RC injection from a clean slate — with
    // another link already up, resetting would yank the other vehicle's engaged override.
    let link_id = {
        let mut reg = state.links.lock().map_err(|e| e.to_string())?;
        if reg.is_empty() {
            if let Ok(mut rc) = state.rc_tx.lock() {
                *rc = crate::scheduler::rc_tx::RcTxState::default();
            }
        }
        reg.reserve_id()
    };

    let proto = protocol.as_deref().unwrap_or("msp");

    log::info!(
        "Connect requested: protocol={} transport={:?} port={:?} baud={:?} host={:?} tcp_port={:?} ble={:?}",
        proto, transport_type, port, baud_rate, host, tcp_port, ble_device_id,
    );

    // Open byte-level transport based on type. Every variant is handled on every platform — the
    // platform differences live behind `transport::serial` / `transport::ble` (on iOS a serial open
    // fails with a clear runtime error; BLE is CoreBluetooth there). TCP/UDP are platform-independent.
    // The OS-facing link status names the transport ("MSP over Serial"); the protocol paths
    // below only know their protocol.
    crate::link_status::set_transport(&transport_type.to_string());
    let byte_transport: Box<dyn ByteTransport> = match transport_type {
        TransportType::Serial => {
            let port_name = port.ok_or("Serial port name required")?;
            let baud = baud_rate.unwrap_or(115200);
            Box::new(SerialConnection::open(&port_name, baud)?)
        }
        TransportType::Tcp => {
            let h = host.ok_or("TCP host required")?;
            let p = tcp_port.ok_or("TCP port required")?;
            Box::new(TcpTransport::connect(&h, p)?)
        }
        TransportType::Udp => {
            let h = host.ok_or("UDP host required")?;
            let p = tcp_port.ok_or("UDP port required")?;
            Box::new(UdpTransport::connect(&h, p)?)
        }
        TransportType::Ble => {
            let dev_id = ble_device_id.ok_or("BLE device ID required")?;
            if proto == "telemetry" {
                // Passive mode: no known profile required — auto-discover + subscribe to all
                // Notify/Indicate characteristics and dump the GATT table to the Debug Monitor.
                Box::new(ble_backend::connect_ble_listen(&dev_id, app_handle.clone()).await?)
            } else {
                Box::new(ble_backend::connect_ble(&dev_id).await?)
            }
        }
    };

    let transport_desc = byte_transport.description().to_string();
    let transport_note = byte_transport.diagnostic_note();
    log::info!("Transport opened, protocol={} link=L{} ({})", proto, link_id, transport_desc);

    // One link per endpoint. A second socket talking to the same UDP/TCP peer (or the same serial
    // port / BLE device) does not give a second view of the vehicles — the peer answers whichever
    // socket spoke last, so both links starve each other (PX4 SITL: 0 bytes on the newcomer, dropped
    // vehicles on the first). Every vehicle behind one endpoint is already discovered on one link.
    {
        let reg = state.links.lock().map_err(|e| e.to_string())?;
        let duplicate = reg.iter().find(|e| e.transport == transport_desc).map(|e| e.id);
        drop(reg);
        if let Some(existing) = duplicate {
            log::warn!("Connect refused: {} is already open as L{}", transport_desc, existing);
            return Err(format!(
                "Already connected to {} as link L{}. Every vehicle on that endpoint is discovered there — select it from the Links list instead of opening a second link.",
                transport_desc, existing
            ));
        }
    }

    let result = match proto {
        "mavlink" => {
            // ── MAVLink Path ─────────────────────────────────────────────
            connect_mavlink(
                link_id,
                byte_transport,
                attitude_rate_hz,
                position_rate_hz,
                airspeed_enabled,
                wind_enabled,
                mavlink_full_telemetry,
                flight_log_enabled,
                flight_log_db_enabled,
                flight_log_path,
                flight_log_raw_path,
                flight_log_raw,
                flight_log_raw_always,
                state.clone(),
                app_handle.clone(),
            )
        }
        "telemetry" => {
            // ── Passive Telemetry Path (listen-only, auto-detect) ────────
            connect_passive_telemetry(
                link_id,
                byte_transport,
                flight_log_enabled,
                flight_log_db_enabled,
                flight_log_path,
                flight_log_raw_path,
                state.clone(),
                app_handle.clone(),
            )
        }
        _ => {
            // ── MSP Path ────────────────────────────────────────────────
            connect_msp(
                link_id,
                byte_transport,
                attitude_rate_hz,
                position_rate_hz,
                airspeed_enabled,
                wind_enabled,
                flight_log_enabled,
                flight_log_db_enabled,
                flight_log_path,
                flight_log_raw_path,
                flight_log_raw,
                flight_log_raw_always,
                state.clone(),
                app_handle.clone(),
            )
        }
    };

    // Central success/failure log — a failed connect otherwise only surfaces in the UI toast and
    // leaves no trace in the diagnostics log (the original PX4 report had nothing to go on).
    let opened = match result {
        Ok(o) => {
            log::info!(
                "Connection established: {} {} (platform={}) as L{}:S{}",
                o.fc_info.fc_variant, o.fc_info.fc_version, o.fc_info.platform_type, link_id, o.sysid,
            );
            o
        }
        Err(e) => {
            log::error!("Connection failed (protocol={}): {}", proto, e);
            // A transport-level explanation (busy local UDP port → the vehicles' pushes go elsewhere)
            // turns a bare "no HEARTBEAT" into something the user can act on.
            return Err(match transport_note {
                Some(note) => format!("{e}

{note}"),
                None => e,
            });
        }
    };

    // Register the link. The first link's primary vehicle becomes the active one.
    let primary = VehicleId::new(link_id, opened.sysid);
    let entry = LinkEntry {
        id: link_id,
        protocol: opened.protocol,
        protocol_name: opened.protocol_name,
        transport: transport_desc,
        fc_info: opened.fc_info.clone(),
        primary: primary.clone(),
    };
    let discovered = VehicleInfo::primary_of(&entry, opened.compid);
    let activated = {
        let mut reg = state.links.lock().map_err(|e| e.to_string())?;
        reg.insert(entry)
    };
    crate::link_presence::link_up(&opened.fc_info, opened.protocol_name);
    let _ = app_handle.emit("vehicle-discovered", &discovered);
    if activated {
        state.retarget_rc(Some(&primary));
        let _ = app_handle.emit("active-vehicle-changed", ActiveVehicleChanged { vehicle_id: Some(primary.to_key()) });
    }

    Ok(ConnectResult { link_id, vehicle_id: primary.to_key(), fc_info: opened.fc_info })
}

/// MSP connection path: handshake → scheduler
#[allow(clippy::too_many_arguments)] // mirrors the connect() command's parameter set
fn connect_msp(
    link_id: LinkId,
    byte_transport: Box<dyn ByteTransport>,
    attitude_rate_hz: Option<f64>,
    position_rate_hz: Option<f64>,
    airspeed_enabled: Option<bool>,
    wind_enabled: Option<bool>,
    flight_log_enabled: Option<bool>,
    flight_log_db_enabled: Option<bool>,
    flight_log_path: Option<String>,
    flight_log_raw_path: Option<String>,
    flight_log_raw: Option<bool>,
    flight_log_raw_always: Option<bool>,
    state: State<'_, AppState>,
    app_handle: AppHandle,
) -> Result<Opened, String> {
    // Shared MSP raw-serial log sink (ADR-049): the transport writes into it, the recorder owns its
    // lifecycle. Created up front so both share the same slot.
    let msp_raw_sink: MspRawSink = std::sync::Arc::new(std::sync::Mutex::new(None));

    // In CONTINUOUS raw mode, open the raw logger NOW — before the handshake — so the handshake's
    // identity frames (MSP_NAME / MSP_FC_VARIANT / …) are captured in the log and the offline parser
    // can recover the vehicle info. The recorder later adopts this same logger (ADR-049). Per-flight
    // mode opens on arm instead, so it intentionally has no handshake.
    if flight_log_enabled.unwrap_or(false)
        && flight_log_raw.unwrap_or(false)
        && flight_log_raw_always.unwrap_or(false)
    {
        let portable = std::env::current_exe()
            .ok()
            .and_then(|p| p.parent().map(|p| p.join(".portable").exists()))
            .unwrap_or(false);
        let raw_dir = crate::flightlog::db::resolve_raw_log_dir(
            flight_log_raw_path.as_deref().unwrap_or(""),
            portable,
        );
        match crate::flightlog::msp_raw_logger::MspRawLogger::new(&raw_dir, 0, &chrono::Utc::now()) {
            Ok(logger) => {
                if let Ok(mut g) = msp_raw_sink.lock() {
                    *g = Some(logger);
                }
                log::info!("Continuous MSP raw log opened pre-handshake");
            }
            Err(e) => log::warn!("Failed to open pre-handshake MSP raw log: {}", e),
        }
    }

    // Wrap in MSP protocol layer (adds MSP v2 framing + response parser)
    let mut transport = MspTransport::new(byte_transport, msp_raw_sink.clone());

    // One vehicle per MSP link → sysid 0. Everything this link emits carries "L{id}:S0".
    let emitter = VehicleEmitter::new(app_handle.clone(), VehicleId::new(link_id, 0));

    // ── MSP Handshake ──────────────────────────────────────────────
    let mut fc_info = FcInfo::default();

    // 1) MSP_API_VERSION → [mspProtocol, apiVersionMajor, apiVersionMinor]
    let resp = transport.msp_request(MSP_API_VERSION, &[])?;
    if resp.payload.len() >= 3 {
        fc_info.msp_protocol = resp.payload[0];
        fc_info.api_version = format!("{}.{}", resp.payload[1], resp.payload[2]);
    }

    // 2) MSP_FC_VARIANT → 4-byte identifier string (e.g. "INAV")
    let resp = transport.msp_request(MSP_FC_VARIANT, &[])?;
    fc_info.fc_variant = String::from_utf8_lossy(&resp.payload).trim().to_string();

    // 3) MSP_FC_VERSION → [major, minor, patch]
    let resp = transport.msp_request(MSP_FC_VERSION, &[])?;
    if resp.payload.len() >= 3 {
        fc_info.fc_version = format!(
            "{}.{}.{}",
            resp.payload[0], resp.payload[1], resp.payload[2]
        );
    }

    // 4) MSP_BOARD_INFO → board identifier (4 bytes) + hw revision (u16 LE)
    let resp = transport.msp_request(MSP_BOARD_INFO, &[])?;
    if resp.payload.len() >= 4 {
        fc_info.board_id = String::from_utf8_lossy(&resp.payload[..4])
            .trim()
            .to_string();
    }
    if resp.payload.len() >= 6 {
        fc_info.hardware_revision =
            (resp.payload[4] as u16) | ((resp.payload[5] as u16) << 8);
    }

    // ── Version check & feature detection ────────────────────────────
    if fc_info.fc_variant != "INAV" {
        return Err(format!(
            "Unsupported firmware variant: '{}'. Only INAV is currently supported.",
            fc_info.fc_variant
        ));
    }

    let version = InavVersion::parse(&fc_info.fc_version).ok_or_else(|| {
        format!("Cannot parse firmware version: '{}'", fc_info.fc_version)
    })?;

    if !is_version_supported(version) {
        return Err(format!(
            "INAV {} is not supported. Minimum required version is 7.0.0.",
            version
        ));
    }

    let feature_set = FeatureSet::for_version(version);
    log::info!(
        "Feature gates for INAV {}: autoland={}, geozones={}, msp_rc={}, aux_rc={}",
        version,
        feature_set.autoland_config,
        feature_set.geozones,
        feature_set.msp_rc,
        feature_set.aux_rc
    );
    let link_stats_supported = feature_set.link_stats;
    let wind_supported = feature_set.wind_estimate;
    fc_info.features = Some(feature_set);

    // 5) MSP2_INAV_MIXER → platform type and mixer preset
    match transport.msp_request(MSPV2_INAV_MIXER, &[]) {
        Ok(resp) => {
            if resp.payload.len() >= 7 {
                fc_info.platform_type = resp.payload[3];
                fc_info.mixer_preset =
                    (resp.payload[5] as i16) | ((resp.payload[6] as i16) << 8);
            }
        }
        Err(e) => {
            log::warn!("Failed to query mixer config: {}", e);
        }
    }

    // 6) MSP_NAME → craft name configured in the FC
    match transport.msp_request(MSP_NAME, &[]) {
        Ok(resp) => {
            fc_info.craft_name = String::from_utf8_lossy(&resp.payload).trim().to_string();
        }
        Err(e) => {
            log::warn!("Failed to query craft name: {}", e);
        }
    }

    // 6b) MSP_UID → the MCU's 96-bit unique id (three little-endian u32 words), rendered as 24 hex
    // chars. Informational: reconnect identity for the platform-type override, stored per flight.
    match transport.msp_request(MSP_UID, &[]) {
        Ok(resp) if resp.payload.len() >= 12 => {
            fc_info.fc_uid = Some(resp.payload[..12].iter().map(|b| format!("{:02X}", b)).collect());
        }
        Ok(_) => log::warn!("MSP_UID: short reply"),
        Err(e) => log::warn!("Failed to query MSP_UID: {}", e),
    }

    // 6c) MSP_BLACKBOX_CONFIG → [supported, device, …]; device 0 = NONE. Seeds the vehicle library's
    // "blackbox available" flag when the craft is saved from the UAV Info panel.
    match transport.msp_request(MSP_BLACKBOX_CONFIG, &[]) {
        Ok(resp) if resp.payload.len() >= 2 => {
            fc_info.blackbox = Some(resp.payload[0] != 0 && resp.payload[1] != 0);
        }
        Ok(_) => log::debug!("MSP_BLACKBOX_CONFIG: short reply"),
        Err(e) => log::debug!("MSP_BLACKBOX_CONFIG not answered: {}", e),
    }

    // 7) Home position — MSP_WP #0 is INAV's RTH home (GPS_home, lat/lon in deg·1e7). One-shot at
    //    connect so a mid-flight connect / app restart recovers Home; the live arm-transition path
    //    only sets it when we actually witness the arm. Raw-parse the 21-byte WP payload (the home
    //    WP's action byte isn't a normal nav action, so we don't go through decode_wp). lat==lon==0
    //    means no home is set yet (on the ground, pre-arm) → skip; arm will set it live.
    match transport.msp_request(MSP_WP, &[0]) {
        Ok(resp) if resp.payload.len() >= 14 => {
            let p = &resp.payload;
            let lat_e7 = i32::from_le_bytes([p[2], p[3], p[4], p[5]]);
            let lon_e7 = i32::from_le_bytes([p[6], p[7], p[8], p[9]]);
            let alt_cm = i32::from_le_bytes([p[10], p[11], p[12], p[13]]);
            if lat_e7 != 0 || lon_e7 != 0 {
                let home = HomeEvent {
                    lat: lat_e7 as f64 / 1e7,
                    lon: lon_e7 as f64 / 1e7,
                    alt: alt_cm as f64 / 100.0,
                };
                log::info!("Home from FC (MSP_WP 0): {:.7}, {:.7}", home.lat, home.lon);
                crate::link_status::on_home(home.lat, home.lon);
                let _ = emitter.emit("home-position", home);
            } else {
                log::info!("MSP_WP(0): no home set on FC yet");
            }
        }
        Ok(_) => log::warn!("MSP_WP(0) home response too short"),
        Err(e) => log::warn!("Failed to query home (MSP_WP 0): {}", e),
    }

    let transport_desc = transport.description();
    log::info!(
        "Connected to {} {} v{} via {} (board: {}, API: {}, platform: {})",
        fc_info.fc_variant,
        fc_info.fc_version,
        fc_info.api_version,
        transport_desc,
        fc_info.board_id,
        fc_info.api_version,
        fc_info.platform_type,
    );

    // ── Start telemetry scheduler ────────────────────────────────────────
    let config = TelemetryConfig {
        attitude_rate_hz: attitude_rate_hz.unwrap_or(5.0),
        position_rate_hz: position_rate_hz.unwrap_or(2.0),
        airspeed_enabled: airspeed_enabled.unwrap_or(false),
        // RC link stats poll (MSP2_INAV_GET_LINK_STATS) — INAV 9.1+ only.
        link_stats_enabled: link_stats_supported,
        // Wind poll (MSP2_INAV_WIND) — opt-in AND INAV 10.0+ only.
        wind_enabled: wind_enabled.unwrap_or(false) && wind_supported,
    };

    // ── Flight recorder setup ────────────────────────────────────────────
    let flight_log_settings = FlightLogSettings {
        enabled: flight_log_enabled.unwrap_or(false),
        db_enabled: flight_log_db_enabled.unwrap_or(false),
        db_path: flight_log_path.unwrap_or_default(),
        raw_log_path: flight_log_raw_path.unwrap_or_default(),
        raw_enabled: flight_log_raw.unwrap_or(false),
        raw_always: flight_log_raw_always.unwrap_or(false),
    };

    let recorder_handle = if flight_log_settings.enabled {
        let portable = std::env::current_exe()
            .ok()
            .and_then(|p| p.parent().map(|p| p.join(".portable").exists()))
            .unwrap_or(false);

        match FlightRecorder::new(flight_log_settings, fc_info.clone(), "MSP", portable, app_handle.clone(), state.pending_session.clone(), state.resume_pending.clone(), state.active_temp_path.clone(), msp_raw_sink.clone()) {
            Ok(mut rec) => {
                rec.start_continuous_log();
                let handle = std::sync::Arc::new(std::sync::Mutex::new(rec));
                log::info!("Flight recorder initialized");
                Some(handle)
            }
            Err(e) => {
                log::error!("Failed to initialize flight recorder: {}", e);
                None
            }
        }
    } else {
        None
    };

    // Fresh link starts with RC injection off (frontend re-engages explicitly).
    if let Ok(mut rc) = state.rc_tx.lock() {
        *rc = crate::scheduler::rc_tx::RcTxState::default();
    }

    store_recorder(&state, &recorder_handle);
    let handle = scheduler::start(
        Box::new(transport),
        config,
        emitter,
        recorder_handle,
        state.radar_ingest.clone(),
        state.radar_msp_enabled.clone(),
        state.rc_tx.clone(),
    );

    Ok(Opened { protocol: ActiveProtocol::Msp(handle), protocol_name: "MSP", fc_info, sysid: 0, compid: 0 })
}

/// MAVLink connection path: handshake → handler
#[allow(clippy::too_many_arguments)]
fn connect_mavlink(
    link_id: LinkId,
    mut byte_transport: Box<dyn ByteTransport>,
    attitude_rate_hz: Option<f64>,
    position_rate_hz: Option<f64>,
    airspeed_enabled: Option<bool>,
    wind_enabled: Option<bool>,
    mavlink_full_telemetry: Option<bool>,
    flight_log_enabled: Option<bool>,
    flight_log_db_enabled: Option<bool>,
    flight_log_path: Option<String>,
    flight_log_raw_path: Option<String>,
    flight_log_raw: Option<bool>,
    flight_log_raw_always: Option<bool>,
    state: State<'_, AppState>,
    app_handle: AppHandle,
) -> Result<Opened, String> {
    // MAVLink handshake: wait for FC HEARTBEAT, send GCS HEARTBEAT back
    let (fc_info, fc_sysid, fc_compid) = mavlink_proto::perform_handshake(&mut *byte_transport)?;

    log::info!(
        "MAVLink connected: {} (sysid={}) via {}",
        fc_info.fc_variant,
        fc_sysid,
        byte_transport.description(),
    );

    // Configure telemetry stream rates (ADR-043) — mirrors the MSP poll-rate knobs. Skipped when
    // "Full MAVLink Telemetry" is on, so the FC streams purely per its own SRn_* params (.tlog gets
    // everything). Applied here, before the handler thread starts, while we still own the transport.
    // Which of the two wind messages this FC speaks — WIND on ArduPilot, WIND_COV on PX4.
    let is_px4 = fc_info.fc_variant == "PX4";
    if mavlink_full_telemetry.unwrap_or(false) {
        // SET_MESSAGE_INTERVAL is sticky on the FC until reboot, so a prior reduced session would
        // otherwise keep the link narrow. Reset our managed messages to the FC's SRn defaults.
        mavlink_proto::streamrates::reset_stream_rates(&mut *byte_transport, fc_sysid, is_px4);
    } else {
        mavlink_proto::streamrates::apply_stream_rates(
            &mut *byte_transport,
            fc_sysid,
            attitude_rate_hz.unwrap_or(5.0),
            position_rate_hz.unwrap_or(2.0),
            airspeed_enabled.unwrap_or(false),
            wind_enabled.unwrap_or(false),
            is_px4,
        );
    }

    // Ask the FC which EKF core is active (AHRS_EKF_TYPE) for the header EKF indicator. One-shot,
    // fire-and-forget — the PARAM_VALUE reply is decoded by the handler thread once it starts.
    mavlink_proto::params::request_ekf_type(&mut *byte_transport, fc_sysid);

    // One-shot HOME_POSITION request — recovers the real FC home on a mid-flight connect (MAVLink
    // counterpart of the MSP_WP(0) read below in the MSP path). Unconditional: in Full-telemetry
    // mode this is the ONLY source (no interval push), in reduced mode it beats the first 0.2 Hz tick.
    mavlink_proto::params::request_home_position(&mut *byte_transport, fc_sysid);

    // ArduPilot: read Q_ENABLE to detect a QuadPlane (which reports MAV_TYPE_FIXED_WING, so the mission
    // vehicle class can't be told from the HEARTBEAT alone). Copter/Rover/Sub lack the param → no reply.
    if fc_info.fc_variant.starts_with("Ardu") {
        mavlink_proto::params::request_quadplane_flag(&mut *byte_transport, fc_sysid);
    }

    // Logging backend for the vehicle library's "blackbox available" flag: ArduPilot LOG_BACKEND_TYPE
    // (bitmask, 0 = none), PX4 SDLOG_MODE (-1 = disabled). The reply lands in the handler, which
    // updates the stored FC info and emits `telemetry-vehicle { blackbox }`.
    if fc_info.fc_variant.starts_with("Ardu") {
        mavlink_proto::params::request_param(&mut *byte_transport, fc_sysid, "LOG_BACKEND_TYPE");
    } else if fc_info.fc_variant.contains("PX4") {
        mavlink_proto::params::request_param(&mut *byte_transport, fc_sysid, "SDLOG_MODE");
    }

    // ── Flight recorder setup ────────────────────────────────────────────
    let flight_log_settings = FlightLogSettings {
        enabled: flight_log_enabled.unwrap_or(false),
        db_enabled: flight_log_db_enabled.unwrap_or(false),
        db_path: flight_log_path.unwrap_or_default(),
        raw_log_path: flight_log_raw_path.unwrap_or_default(),
        raw_enabled: flight_log_raw.unwrap_or(false),
        raw_always: flight_log_raw_always.unwrap_or(false),
    };

    let recorder_handle = if flight_log_settings.enabled {
        let portable = std::env::current_exe()
            .ok()
            .and_then(|p| p.parent().map(|p| p.join(".portable").exists()))
            .unwrap_or(false);

        // MAVLink records via .tlog; the MSP raw sink is unused here (kept empty).
        let msp_raw_sink: MspRawSink = std::sync::Arc::new(std::sync::Mutex::new(None));
        match FlightRecorder::new(flight_log_settings.clone(), fc_info.clone(), "MAVLink", portable, app_handle.clone(), state.pending_session.clone(), state.resume_pending.clone(), state.active_temp_path.clone(), msp_raw_sink) {
            Ok(mut rec) => {
                rec.start_continuous_log();
                let handle = std::sync::Arc::new(std::sync::Mutex::new(rec));
                log::info!("Flight recorder initialized (MAVLink)");
                Some(handle)
            }
            Err(e) => {
                log::error!("Failed to initialize flight recorder: {}", e);
                None
            }
        }
    } else {
        None
    };

    // Start the MAVLink handler thread. Its events carry "L{link}:S{sysid}" of the handshake vehicle;
    // vehicles it discovers later on the same link get their own sysid stamped. The stream shaping
    // applied above is handed along so those vehicles are configured identically.
    let rates = mavlink_proto::handler::StreamRateConfig {
        full_telemetry: mavlink_full_telemetry.unwrap_or(false),
        attitude_hz: attitude_rate_hz.unwrap_or(5.0),
        position_hz: position_rate_hz.unwrap_or(2.0),
        airspeed_enabled: airspeed_enabled.unwrap_or(false),
        wind_enabled: wind_enabled.unwrap_or(false),
    };
    store_recorder(&state, &recorder_handle);
    // Vehicles discovered later on this link record unattended (DB only) with the same settings.
    let secondary_recording = if flight_log_settings.enabled && flight_log_settings.db_enabled {
        let portable = std::env::current_exe()
            .ok()
            .and_then(|p| p.parent().map(|p| p.join(".portable").exists()))
            .unwrap_or(false);
        Some(mavlink_proto::handler::SecondaryRecording { settings: flight_log_settings.clone(), portable })
    } else {
        None
    };
    let emitter = VehicleEmitter::new(app_handle, VehicleId::new(link_id, fc_sysid));
    let handle = mavlink_proto::handler::start(byte_transport, fc_sysid, fc_compid, fc_info.fc_variant.clone(), (fc_info.platform_type, fc_info.mav_type), emitter, recorder_handle, state.rc_tx.clone(), rates, secondary_recording);

    Ok(Opened { protocol: ActiveProtocol::Mavlink(handle), protocol_name: "MAVLink", fc_info, sysid: fc_sysid, compid: fc_compid })
}

/// Passive telemetry path: no handshake — start the listen-only handler immediately.
/// The wire protocol is auto-detected by the handler; nothing is ever transmitted. When flight logging
/// is enabled, a recorder is attached and fed the decoded telemetry (arm/disarm derived from the FC's
/// flight-mode field — e.g. FrSky MODES).
fn connect_passive_telemetry(
    link_id: LinkId,
    byte_transport: Box<dyn ByteTransport>,
    flight_log_enabled: Option<bool>,
    flight_log_db_enabled: Option<bool>,
    flight_log_path: Option<String>,
    flight_log_raw_path: Option<String>,
    state: State<'_, AppState>,
    app_handle: AppHandle,
) -> Result<Opened, String> {
    log::info!(
        "Passive telemetry connect (listen-only) via {}",
        byte_transport.description()
    );

    // Synthesize a minimal FcInfo so the frontend enters the connected state. Passive telemetry carries
    // no FC identity (no handshake) — leave firmware empty (shown as "N/A") and use the Generic platform
    // (255) so the map shows the generic arrow rather than defaulting to a multirotor.
    let fc_info = FcInfo {
        platform_type: 255, // PLATFORM_GENERIC
        ..FcInfo::default()
    };

    // Flight recorder (no raw byte log on this path — FrSky has no MSP raw stream).
    let flight_log_settings = FlightLogSettings {
        enabled: flight_log_enabled.unwrap_or(false),
        db_enabled: flight_log_db_enabled.unwrap_or(false),
        db_path: flight_log_path.unwrap_or_default(),
        raw_log_path: flight_log_raw_path.unwrap_or_default(),
        raw_enabled: false,
        raw_always: false,
    };

    let recorder_handle = if flight_log_settings.enabled {
        let portable = std::env::current_exe()
            .ok()
            .and_then(|p| p.parent().map(|p| p.join(".portable").exists()))
            .unwrap_or(false);
        let msp_raw_sink: MspRawSink = std::sync::Arc::new(std::sync::Mutex::new(None));
        match FlightRecorder::new(flight_log_settings, fc_info.clone(), "Telemetry", portable, app_handle.clone(), state.pending_session.clone(), state.resume_pending.clone(), state.active_temp_path.clone(), msp_raw_sink) {
            Ok(mut rec) => {
                rec.start_continuous_log();
                log::info!("Flight recorder initialized (passive telemetry)");
                Some(std::sync::Arc::new(std::sync::Mutex::new(rec)))
            }
            Err(e) => {
                log::error!("Failed to initialize flight recorder: {}", e);
                None
            }
        }
    } else {
        None
    };

    store_recorder(&state, &recorder_handle);
    // One vehicle per passive link → sysid 0.
    let emitter = VehicleEmitter::new(app_handle, VehicleId::new(link_id, 0));
    let handle = crate::passive_telemetry::start(byte_transport, emitter, recorder_handle);

    Ok(Opened { protocol: ActiveProtocol::PassiveTelemetry(handle), protocol_name: "Telemetry", fc_info, sysid: 0, compid: 0 })
}

/// Disconnect one link (`link_id`) or, with `None`, every link (the pre-multi-vehicle behaviour the
/// single connect/disconnect toggle still relies on). Stops the handler(s), drops the transport(s) and
/// moves the active vehicle to a remaining link if its own went away.
#[tauri::command]
pub async fn disconnect(link_id: Option<LinkId>, state: State<'_, AppState>, app_handle: AppHandle) -> Result<(), String> {
    // Take the entries out under the lock, stop them after releasing it (stop() joins the thread).
    let (entries, active_change) = {
        let mut reg = state.links.lock().map_err(|e| e.to_string())?;
        if reg.is_empty() {
            return Err(ERR_NOT_CONNECTED.into());
        }
        match link_id {
            None => (reg.drain(), Some(None)),
            Some(id) => {
                let (entry, change) = reg.remove(id).ok_or_else(|| format!("Unknown link L{id}"))?;
                (vec![entry], change)
            }
        }
    };

    for entry in entries {
        let vehicle_key = entry.primary.to_key();
        match entry.protocol {
            ActiveProtocol::Msp(handle) => {
                let _transport = handle.stop(); // transport dropped here
                log::info!("MSP scheduler stopped (L{})", entry.id);
            }
            ActiveProtocol::Mavlink(handle) => {
                let _transport = handle.stop(); // transport dropped here
                log::info!("MAVLink handler stopped (L{})", entry.id);
            }
            ActiveProtocol::PassiveTelemetry(handle) => {
                let _transport = handle.stop(); // transport dropped here
                log::info!("Passive telemetry handler stopped (L{})", entry.id);
            }
        }
        let _ = app_handle.emit("vehicle-lost", VehicleLost { vehicle_id: vehicle_key, link_id: entry.id, reason: "user" });
        let _ = app_handle.emit("link-closed", LinkClosed { link_id: entry.id, reason: "user" });
    }

    if let Some(new_active) = active_change {
        state.retarget_rc(new_active.as_ref());
        let _ = app_handle.emit("active-vehicle-changed", ActiveVehicleChanged { vehicle_id: new_active.map(|v| v.to_key()) });
    }

    let none_left = state.links.lock().map(|r| r.is_empty()).unwrap_or(true);
    if none_left {
        if let Ok(mut rec) = state.recorder.lock() {
            *rec = None;
        }
        crate::link_presence::link_down();
        log::info!("Disconnected");
    } else {
        log::info!("Link closed; other links remain");
    }
    Ok(())
}

/// Publish the connection's recorder handle for the command layer (cleared again on disconnect).
fn store_recorder(state: &State<'_, AppState>, rec: &Option<crate::flightlog::recorder::FlightRecorderHandle>) {
    if let Ok(mut slot) = state.recorder.lock() {
        *slot = rec.clone();
    }
}

/// Override the platform type of the connected vehicle for this session (UAV Info panel dropdown).
/// Updates the stored FC info and the recorder, so the flight being recorded — and any flight started
/// later on this link — is saved with the chosen type. RAM only; nothing is persisted.
#[tauri::command]
pub fn set_platform_type(platform_type: u8, state: State<'_, AppState>) -> Result<(), String> {
    {
        // Multi-vehicle: the override applies to the active vehicle's link (its primary FcInfo).
        let mut reg = state.links.lock().map_err(|e| e.to_string())?;
        let link = reg.active().map(|v| v.link).ok_or_else(|| crate::vehicle_registry::ERR_NOT_CONNECTED.to_string())?;
        match reg.get_mut(link) {
            Some(entry) => entry.fc_info.platform_type = platform_type,
            None => return Err(crate::vehicle_registry::ERR_NOT_CONNECTED.to_string()),
        }
    }
    if let Ok(slot) = state.recorder.lock() {
        if let Some(rec) = slot.as_ref() {
            if let Ok(mut r) = rec.lock() {
                r.set_platform_type(platform_type);
            }
        }
    }
    log::info!("Platform type override: {}", platform_type);
    Ok(())
}
