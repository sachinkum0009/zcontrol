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

//! Simulated devices: fake data that behaves plausibly, for testing the whole
//! ros2_control path without motors or sensors.

use embassy_time::Instant;
use zc_core::{Device, packed_struct};

/// A wheel whose velocity follows the command with an acceleration limit and
/// whose position integrates the velocity.
///
/// Integrating velocity into position is what makes `diff_drive_controller`
/// odometry move (unlike a plain command-to-state mirror).
#[derive(Clone, Copy, Debug)]
pub struct FakeWheel {
    /// rad
    pub position: f64,
    /// rad/s
    pub velocity: f64,
    /// rad/s
    pub velocity_command: f64,
    /// rad/s^2; `f64::INFINITY` follows the command instantly.
    pub max_acceleration: f64,
}

impl FakeWheel {
    pub const fn new(max_acceleration: f64) -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            velocity_command: 0.0,
            max_acceleration,
        }
    }

    /// Advances the simulation by `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        let max_dv = self.max_acceleration * dt;
        let dv = (self.velocity_command - self.velocity).clamp(-max_dv, max_dv);
        self.velocity += dv;
        self.position += self.velocity * dt;
    }
}

packed_struct! {
    /// `state_interfaces` of `zc_bringup/config/fake_rover_schema.yaml`.
    pub struct FakeRoverState {
        pub left_wheel_joint_position: f64,
        pub left_wheel_joint_velocity: f64,
        pub right_wheel_joint_position: f64,
        pub right_wheel_joint_velocity: f64,
    }
}

packed_struct! {
    /// `command_interfaces` of `zc_bringup/config/fake_rover_schema.yaml`.
    pub struct FakeRoverCommand {
        pub left_wheel_joint_velocity: f64,
        pub right_wheel_joint_velocity: f64,
    }
}

/// Two velocity-controlled wheels, as driven by `diff_drive_controller`.
pub struct FakeRover {
    pub left: FakeWheel,
    pub right: FakeWheel,
    last_step: Option<Instant>,
}

impl FakeRover {
    /// `max_acceleration` in rad/s^2 for both wheels.
    pub const fn new(max_acceleration: f64) -> Self {
        Self {
            left: FakeWheel::new(max_acceleration),
            right: FakeWheel::new(max_acceleration),
            last_step: None,
        }
    }
}

impl Device for FakeRover {
    type State = FakeRoverState;
    type Command = FakeRoverCommand;

    fn read(&mut self, now: Instant) -> FakeRoverState {
        if let Some(last) = self.last_step {
            let dt = now.duration_since(last).as_micros() as f64 * 1e-6;
            self.left.step(dt);
            self.right.step(dt);
        }
        self.last_step = Some(now);

        FakeRoverState {
            left_wheel_joint_position: self.left.position,
            left_wheel_joint_velocity: self.left.velocity,
            right_wheel_joint_position: self.right.position,
            right_wheel_joint_velocity: self.right.velocity,
        }
    }

    fn write(&mut self, _now: Instant, command: &FakeRoverCommand) {
        self.left.velocity_command = command.left_wheel_joint_velocity;
        self.right.velocity_command = command.right_wheel_joint_velocity;
    }

    fn on_command_timeout(&mut self, _now: Instant) {
        // Ramp down with the acceleration limit, like a real motor driver would.
        self.left.velocity_command = 0.0;
        self.right.velocity_command = 0.0;
    }
}

packed_struct! {
    /// State of [`Echo`]: `zc_bringup/config/echo_schema.yaml`.
    pub struct EchoState {
        /// Control loop iterations since boot. A gap means lost state samples.
        pub tick: u64,
        /// Sequence number of the last applied command.
        pub last_seq: u64,
        /// Commands applied since boot. Commands sent minus this = commands
        /// lost or overwritten before the control loop picked them up.
        pub commands: u64,
    }
}

packed_struct! {
    /// Command of [`Echo`].
    pub struct EchoCommand {
        pub seq: u64,
    }
}

/// Loopback device for timing measurements: the state echoes the sequence
/// number of the last applied command (see `examples/latency_probe`).
#[derive(Default)]
pub struct Echo {
    tick: u64,
    last_seq: u64,
    commands: u64,
}

impl Device for Echo {
    type State = EchoState;
    type Command = EchoCommand;

    fn read(&mut self, _now: Instant) -> EchoState {
        self.tick += 1;
        EchoState {
            tick: self.tick,
            last_seq: self.last_seq,
            commands: self.commands,
        }
    }

    fn write(&mut self, _now: Instant, command: &EchoCommand) {
        self.last_seq = command.seq;
        self.commands += 1;
    }

    fn on_command_timeout(&mut self, _now: Instant) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use zc_core::Packed;

    #[test]
    fn wheel_integrates_velocity_with_acceleration_limit() {
        let mut w = FakeWheel::new(10.0);
        w.velocity_command = 2.0;
        for _ in 0..10 {
            w.step(0.01); // 0.1 s at 10 rad/s^2 -> reaches 1.0 rad/s
        }
        assert!((w.velocity - 1.0).abs() < 1e-9);
        assert!(w.position > 0.0);
        for _ in 0..100 {
            w.step(0.01);
        }
        assert!((w.velocity - 2.0).abs() < 1e-9);
    }

    #[test]
    fn payload_sizes_match_schema() {
        // 4 x float64 state, 2 x float64 command
        assert_eq!(FakeRoverState::SIZE, 32);
        assert_eq!(FakeRoverCommand::SIZE, 16);
    }
}
