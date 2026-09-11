// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Application State
// Holds the shared state for the Tauri application, including the active connection.

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use std::sync::mpsc;

use crate::aero::AeroCache;
use crate::flightlog::recorder::PendingSessionHandle;
use crate::mavlink_proto::handler::MavlinkCommand;
use crate::mavlink_proto::MavlinkHandle;
use crate::msp::FcInfo;
use crate::passive_telemetry::PassiveHandle;
use crate::radar::source::SourceUpdate;
use crate::radar::RadarManager;
use crate::scheduler::rc_tx::{RcTxHandle, RcTxState};
use crate::scheduler::SchedulerHandle;
use crate::vehicle_registry::LinkRegistry;

/// Which protocol is currently active
pub enum ActiveProtocol {
    Msp(SchedulerHandle),
    Mavlink(MavlinkHandle),
    /// Passive, listen-only telemetry (FrSkyX/CRSF/LTM/MAVLink-passive), protocol auto-detected.
    PassiveTelemetry(PassiveHandle),
}

/// A resolved MAVLink command target: the link's handler channel plus the system id to address.
/// Returned by [`AppState::mav_target`] so command handlers never hold the registry lock while they
/// wait on the vehicle.
pub struct MavTarget {
    pub cmd_tx: mpsc::Sender<MavlinkCommand>,
    pub sysid: u8,
    /// FC variant of the link's primary vehicle ("ArduPlane"/"ArduCopter"/"PX4"/…).
    pub fc_variant: String,
}

/// Global application state managed by Tauri
pub struct AppState {
    /// Every open link (protocol handler + handshake info) and the active-vehicle selection. Replaces
    /// the former single `protocol` / `fc_info` slots — see `vehicle_registry`.
    pub links: Mutex<LinkRegistry>,
    /// Radar (foreign-vehicle tracking) subsystem — fully independent of `protocol`.
    pub radar: Mutex<RadarManager>,
    /// Bridge for scheduler-fed radar sources (ADS-B via MSP): the radar aggregator's ingest channel
    /// (Some while radar runs) and a runtime on/off flag the MSP scheduler polls.
    pub radar_ingest: Arc<Mutex<Option<std::sync::mpsc::Sender<SourceUpdate>>>>,
    pub radar_msp_enabled: Arc<AtomicBool>,
    /// GCS RC-injection state (docs/archive/MSP_RC_CONTROL.md §10 Phase 4c). Written by the rc_stream_*
    /// commands, read+streamed by the MSP scheduler thread. Independent of `protocol` lifecycle.
    pub rc_tx: RcTxHandle,
    /// Stop handle for the live BLE scan session (Some while scanning). Dropping/replacing the
    /// sender ends the session — see `commands::connection::ble_scan_start` / `ble_scan_stop`.
    pub ble_scan_stop: Mutex<Option<tokio::sync::oneshot::Sender<()>>>,
    /// Airspace Manager (aeronautical data) — last fetched region cached in RAM, or None.
    pub aero: Mutex<Option<AeroCache>>,
    /// Pending live-recording session awaiting commit/discard (deferred commit, ADR-041). Set by the
    /// recorder on disarm; resolved by the Save/Discard commands or the recorder's grace-arm path.
    /// Lives here (not in the recorder) so it survives a disconnect while the End-Flight dialog is open.
    pub pending_session: PendingSessionHandle,
    /// A recovered orphan session the user chose to **continue on reconnect** (ADR-042). The next
    /// recorder consults it on its first polled status: armed → resume the same `.ktmp`; disarmed →
    /// finalize it into `pending_session` + the End-Flight dialog.
    pub resume_pending: PendingSessionHandle,
}

impl AppState {
    pub fn new() -> Self {
        let radar = RadarManager::new();
        let radar_ingest = radar.ingest_handle();
        Self {
            links: Mutex::new(LinkRegistry::new()),
            radar: Mutex::new(radar),
            radar_ingest,
            radar_msp_enabled: Arc::new(AtomicBool::new(false)),
            rc_tx: Arc::new(Mutex::new(RcTxState::default())),
            ble_scan_stop: Mutex::new(None),
            aero: Mutex::new(None),
            pending_session: Arc::new(Mutex::new(None)),
            resume_pending: Arc::new(Mutex::new(None)),
        }
    }
}

impl AppState {
    /// Resolve a MAVLink command target. `vehicle_id` is the frontend's `"L1:S1"` key, or `None` for the
    /// active vehicle. Errors keep the legacy texts ("Not connected", "FC is not running MAVLink").
    pub fn mav_target(&self, vehicle_id: Option<&str>) -> Result<MavTarget, String> {
        let reg = self.links.lock().map_err(|e| e.to_string())?;
        let (entry, sysid) = reg.resolve(vehicle_id)?;
        match &entry.protocol {
            ActiveProtocol::Mavlink(h) => Ok(MavTarget {
                cmd_tx: h.cmd_tx_clone(),
                sysid,
                fc_variant: h.fc_variant.clone(),
            }),
            _ => Err("FC is not running MAVLink".into()),
        }
    }

    /// Like `mav_target` but `Ok(None)` for non-MAVLink / disconnected — for the fence/rally readers
    /// that answer with an empty config instead of an error.
    pub fn mav_target_opt(&self, vehicle_id: Option<&str>) -> Result<Option<MavTarget>, String> {
        match self.mav_target(vehicle_id) {
            Ok(t) => Ok(Some(t)),
            Err(e) if e == crate::vehicle_registry::ERR_NOT_CONNECTED || e == "FC is not running MAVLink" => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Point the MAVLink RC-injection stream at `vehicle` (the new active vehicle) and DISENGAGE it.
    /// Re-pointing a live stream would hand the sticks to a different aircraft without the operator
    /// noticing; they re-engage on the new vehicle explicitly (the frontend mirrors this). The PX4 /
    /// ArduPilot choice follows the link's handshake variant (vehicles sharing one link share it).
    pub fn retarget_rc(&self, vehicle: Option<&crate::vehicle_registry::VehicleId>) {
        let target = vehicle.and_then(|v| {
            let reg = self.links.lock().ok()?;
            let entry = reg.get(v.link)?;
            Some(crate::scheduler::rc_tx::RcTarget {
                link: v.link,
                sysid: v.sysid,
                px4: entry.fc_info.fc_variant.eq_ignore_ascii_case("px4"),
            })
        });
        if let Ok(mut rc) = self.rc_tx.lock() {
            if rc.mav_target != target || target.is_none() {
                rc.enabled = false;
                rc.mav_override_us.clear();
                rc.mav_manual = None;
            }
            rc.mav_target = target;
        }
    }

    /// The active link's handshake info, if connected.
    #[allow(dead_code)] // Phase C (link status / relay per-vehicle)
    pub fn active_fc_info(&self) -> Option<FcInfo> {
        self.links.lock().ok().and_then(|r| r.active_fc_info().cloned())
    }
}

