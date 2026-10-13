// Copyright 2026 Sachin Kumar
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Z Control device runtime.
//!
//! Exposes a [`Device`] to the `zenbedded_hardware_interface` ros2_control
//! plugin over `zenoh-nostd`:
//!
//! * publishes the device state on `state_key` every `period` (free-running,
//!   latest value; the host never waits for it),
//! * applies the latest command received on `command_key`,
//! * calls [`Device::on_command_timeout`] when no command arrived for
//!   `watchdog`, or when the zenoh session is lost.
//!
//! The payloads are packed structs from [`zc_proto`] and must match the
//! `interface_schema.yaml` the host plugin loads.
//!
//! The same code runs on the ESP32-S3 (embassy + `zenoh-embassy`) and on a
//! laptop as a virtual device (`zenoh-std`). Only the link manager differs.

#![no_std]

mod config;
mod node;

pub use config::ZcSessionConfig;
pub use node::{Node, NodeConfig, Stats, run};

pub use zc_proto::{self, Field, Packed, packed_struct};
pub use zenoh_nostd;

use embassy_time::Instant;

/// Largest state or command payload in bytes.
pub const MAX_PAYLOAD: usize = 256;

/// The hardware behind one `<ros2_control>` block.
///
/// All methods are called from the control loop and must not block.
pub trait Device {
    /// `state_interfaces` of the schema, in schema order.
    type State: Packed;
    /// `command_interfaces` of the schema, in schema order.
    type Command: Packed;

    /// Samples the current state. Called once per control period.
    fn read(&mut self, now: Instant) -> Self::State;

    /// Applies a new command. Called at most once per control period, with
    /// the latest command received since the previous period.
    fn write(&mut self, now: Instant, command: &Self::Command);

    /// No command for longer than the watchdog timeout, or the connection to
    /// the host was lost. Bring all actuators into a safe state.
    fn on_command_timeout(&mut self, now: Instant);
}
