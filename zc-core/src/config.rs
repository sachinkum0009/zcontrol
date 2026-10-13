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

use zenoh_nostd::{
    platform::ZLinkManager,
    session::{
        FixedCapacityGetCallbacks, FixedCapacityQueryableCallbacks, FixedCapacitySubCallbacks,
        TransportLinkManager, ZSessionConfig, zenoh::storage::RawOrBox,
    },
};

/// zenoh-nostd session configuration for a Z Control device.
///
/// `BUFF` is the transport buffer size in bytes (one zenoh batch). It must hold
/// the largest state payload plus zenoh framing; 512 is plenty for typical
/// devices.
///
/// Callback storage is sized for what [`crate::Node`] declares: one subscriber
/// (commands), no queries. Capacities must be powers of two >= 2 (heapless
/// `IndexMap`).
pub struct ZcSessionConfig<LM: ZLinkManager, const BUFF: usize> {
    transports: TransportLinkManager<LM>,
}

impl<LM: ZLinkManager, const BUFF: usize> ZcSessionConfig<LM, BUFF> {
    pub fn new(link_manager: LM) -> Self {
        Self {
            transports: TransportLinkManager::from(link_manager),
        }
    }
}

impl<LM: ZLinkManager, const BUFF: usize> ZSessionConfig for ZcSessionConfig<LM, BUFF> {
    type Buff = [u8; BUFF];
    type LinkManager = LM;

    type SubCallbacks<'res> = FixedCapacitySubCallbacks<'res, 4, RawOrBox<56>, RawOrBox<600>>;
    type GetCallbacks<'res> = FixedCapacityGetCallbacks<'res, 2, RawOrBox<1>, RawOrBox<32>>;
    type QueryableCallbacks<'res>
        = FixedCapacityQueryableCallbacks<'res, Self, 2, RawOrBox<32>, RawOrBox<952>>
    where
        Self: 'res;

    fn transports(&self) -> &TransportLinkManager<Self::LinkManager> {
        &self.transports
    }

    fn buff(&self) -> Self::Buff {
        [0u8; BUFF]
    }
}
