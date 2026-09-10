// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Vehicle / link commands — the multi-vehicle surface next to `connection::{connect, disconnect}`:
// list the open links and pick which vehicle the singleton UI (widgets, control panel, untargeted
// commands) follows. See docs/02-design/features/multi-vehicle.design.md §3.1.

use tauri::{AppHandle, Emitter, State};

use crate::commands::connection::ActiveVehicleChanged;
use crate::state::AppState;
use crate::vehicle_registry::{LinkSummary, VehicleId};

/// Every open link with its protocol, transport and handshake info.
#[tauri::command]
pub fn list_links(state: State<'_, AppState>) -> Result<Vec<LinkSummary>, String> {
    let reg = state.links.lock().map_err(|e| e.to_string())?;
    Ok(reg.summaries())
}

/// The active vehicle's key (`"L1:S1"`), or `None` when nothing is connected.
#[tauri::command]
pub fn get_active_vehicle(state: State<'_, AppState>) -> Result<Option<String>, String> {
    let reg = state.links.lock().map_err(|e| e.to_string())?;
    Ok(reg.active().map(|v| v.to_key()))
}

/// Make `vehicle_id` the active vehicle. The link must be open; for MAVLink the sysid may be one the
/// handler discovered after the handshake (those never enter the registry). Emits
/// `active-vehicle-changed` so every frontend consumer re-targets at once.
#[tauri::command]
pub fn set_active_vehicle(vehicle_id: String, state: State<'_, AppState>, app_handle: AppHandle) -> Result<(), String> {
    let vid = VehicleId::parse(&vehicle_id).ok_or_else(|| format!("Invalid vehicle id '{vehicle_id}'"))?;
    {
        let mut reg = state.links.lock().map_err(|e| e.to_string())?;
        if reg.active() == Some(&vid) {
            return Ok(());
        }
        reg.set_active(vid.clone())?;
    }
    log::info!("Active vehicle → {}", vid);
    let _ = app_handle.emit("active-vehicle-changed", ActiveVehicleChanged { vehicle_id: Some(vid.to_key()) });
    Ok(())
}
