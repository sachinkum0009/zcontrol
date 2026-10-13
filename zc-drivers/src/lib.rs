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

//! Sensor and actuator implementations for Z Control devices.
//!
//! Real drivers (encoders, PWM motors, IMU, ...) will live here behind feature
//! flags on top of `embedded-hal(-async)`. For now there are only simulated
//! devices.

#![no_std]

#[cfg(feature = "fake")]
pub mod fake;
