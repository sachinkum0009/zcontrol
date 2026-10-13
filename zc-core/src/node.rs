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

use core::mem::ManuallyDrop;

use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker, Timer};
use heapless::Vec;
use zenoh_nostd::session::{Endpoint, Resources, Session, ZSessionConfig, zenoh};

use crate::{Device, MAX_PAYLOAD, Packed};

/// Latest command payload received from the host.
///
/// A `Signal` keeps only the newest value, which is exactly the latest-value
/// semantics we want: a command that is overwritten before the control loop
/// picks it up is simply skipped. It has to be a `static` because zenoh-nostd
/// callbacks live as long as the session. This also means one `Node` per
/// firmware, which matches "one board per ros2_control plugin instance".
static COMMAND: Signal<CriticalSectionRawMutex, Vec<u8, MAX_PAYLOAD>> = Signal::new();

/// Delay between reconnect attempts in [`run`].
const RECONNECT_DELAY: Duration = Duration::from_secs(1);

/// Interval of the statistics log line.
const STATS_INTERVAL: Duration = Duration::from_secs(10);

/// Where and how fast a device talks to the host.
///
/// `state_key` and `command_key` must equal the `state_topic` and
/// `command_topic` hardware parameters in the URDF.
#[derive(Clone, Copy, Debug)]
pub struct NodeConfig {
    pub state_key: &'static str,
    pub command_key: &'static str,
    /// State publish and command apply period. 10 ms = 100 Hz.
    pub period: Duration,
    /// Command watchdog timeout. 100 ms = 10 missed cycles at 100 Hz.
    pub watchdog: Duration,
}

impl NodeConfig {
    /// Keys as given, 100 Hz period and 100 ms watchdog.
    pub const fn new(state_key: &'static str, command_key: &'static str) -> Self {
        Self {
            state_key,
            command_key,
            period: Duration::from_millis(10),
            watchdog: Duration::from_millis(100),
        }
    }
}

/// Counters since boot.
#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub states_sent: u32,
    pub commands_applied: u32,
    /// Command payloads whose size did not match `Device::Command::SIZE`.
    pub commands_rejected: u32,
    pub watchdog_trips: u32,
    pub sessions: u32,
}

/// A [`Device`] plus its watchdog state. Survives reconnects.
pub struct Node<D: Device> {
    device: D,
    config: NodeConfig,
    last_command: Option<Instant>,
    safe: bool,
    stats: Stats,
}

impl<D: Device> Node<D> {
    pub fn new(device: D, config: NodeConfig) -> Self {
        const {
            assert!(
                D::State::SIZE <= MAX_PAYLOAD,
                "State is larger than MAX_PAYLOAD"
            );
            assert!(
                D::Command::SIZE <= MAX_PAYLOAD,
                "Command is larger than MAX_PAYLOAD"
            );
        }
        Self {
            device,
            config,
            last_command: None,
            safe: true,
            stats: Stats::default(),
        }
    }

    pub fn device(&self) -> &D {
        &self.device
    }

    pub fn device_mut(&mut self) -> &mut D {
        &mut self.device
    }

    pub fn stats(&self) -> Stats {
        self.stats
    }

    /// Serves the device on an open session until the session fails.
    ///
    /// On return the device has been put into its safe state.
    pub async fn serve<'res, C: ZSessionConfig>(
        &mut self,
        session: &Session<'res, C>,
    ) -> zenoh::ZResult<()> {
        self.stats.sessions += 1;
        let result = self.serve_inner(session).await;
        self.enter_safe_state(Instant::now());
        result
    }

    async fn serve_inner<'res, C: ZSessionConfig>(
        &mut self,
        session: &Session<'res, C>,
    ) -> zenoh::ZResult<()> {
        let state_ke = zenoh::keyexpr::new(self.config.state_key)?;
        let command_ke = zenoh::keyexpr::new(self.config.command_key)?;

        // Drop a command left over from a previous session.
        COMMAND.reset();
        let _subscriber = session
            .declare_subscriber(command_ke)
            .callback_sync(|sample| match Vec::from_slice(sample.payload()) {
                Ok(bytes) => COMMAND.signal(bytes),
                Err(_) => zenoh::warn!("command payload larger than MAX_PAYLOAD, dropped"),
            })
            .finish()
            .await?;

        zenoh::info!(
            "serving: state '{}' ({} B), command '{}' ({} B)",
            self.config.state_key,
            D::State::SIZE,
            self.config.command_key,
            D::Command::SIZE
        );

        match select(session.run(), self.control_loop(session, state_ke)).await {
            Either::First(r) => r.map_err(Into::into),
            Either::Second(r) => r,
        }
    }

    async fn control_loop<'res, C: ZSessionConfig>(
        &mut self,
        session: &Session<'res, C>,
        state_ke: &zenoh::keyexpr,
    ) -> zenoh::ZResult<()> {
        let mut buf = [0u8; MAX_PAYLOAD];
        let mut ticker = Ticker::every(self.config.period);
        let mut next_stats = Instant::now() + STATS_INTERVAL;

        loop {
            let now = Instant::now();
            self.apply_command(now);

            let state = self.device.read(now);
            state.encode(&mut buf);
            session
                .put(state_ke, &buf[..D::State::SIZE])
                .finish()
                .await?;
            self.stats.states_sent = self.stats.states_sent.wrapping_add(1);

            if now >= next_stats {
                next_stats = now + STATS_INTERVAL;
                let s = self.stats;
                zenoh::info!(
                    "stats: states={} commands={} rejected={} watchdog_trips={} sessions={}",
                    s.states_sent,
                    s.commands_applied,
                    s.commands_rejected,
                    s.watchdog_trips,
                    s.sessions
                );
            }

            ticker.next().await;
        }
    }

    fn apply_command(&mut self, now: Instant) {
        if let Some(bytes) = COMMAND.try_take() {
            match D::Command::decode(&bytes) {
                Some(command) => {
                    if self.safe && self.last_command.is_some() {
                        zenoh::info!("commands resumed");
                    }
                    self.device.write(now, &command);
                    self.last_command = Some(now);
                    self.safe = false;
                    self.stats.commands_applied = self.stats.commands_applied.wrapping_add(1);
                }
                None => {
                    // Log only the first one: a schema mismatch repeats every cycle.
                    if self.stats.commands_rejected == 0 {
                        zenoh::warn!(
                            "command payload of {} B dropped, expected {} B (schema mismatch?)",
                            bytes.len(),
                            D::Command::SIZE
                        );
                    }
                    self.stats.commands_rejected = self.stats.commands_rejected.wrapping_add(1);
                }
            }
            return;
        }

        if let Some(last) = self.last_command
            && !self.safe
            && now.duration_since(last) > self.config.watchdog
        {
            zenoh::warn!(
                "no command for {} ms, entering safe state",
                now.duration_since(last).as_millis()
            );
            self.stats.watchdog_trips += 1;
            self.enter_safe_state(now);
        }
    }

    fn enter_safe_state(&mut self, now: Instant) {
        if !self.safe {
            self.device.on_command_timeout(now);
            self.safe = true;
        }
    }
}

/// Connects to `endpoint` and serves `node` forever, reconnecting after
/// session loss.
pub async fn run<C: ZSessionConfig, D: Device>(
    config: &C,
    endpoint: &str,
    node: &mut Node<D>,
) -> ! {
    loop {
        match Endpoint::try_from(endpoint) {
            Ok(ep) => session_once(config, ep, endpoint, node).await,
            Err(e) => zenoh::error!("invalid endpoint {}: {}", endpoint, e),
        }
        Timer::after(RECONNECT_DELAY).await;
    }
}

/// One connect + serve cycle with its own transport resources.
///
/// zenoh-nostd ties a session to `&'res mut Resources<'res, _>`. That borrow
/// can only end when `Resources` itself goes away, so a plain local cannot be
/// dropped (and its socket released) after the session ends. The upstream
/// examples sidestep this with `StaticCell`, which allows only one connection
/// per boot. Here the resources are borrowed through a raw pointer and dropped
/// explicitly once the session is gone, which frees the socket buffers for the
/// next attempt (the embassy link manager has only a few).
async fn session_once<C: ZSessionConfig, D: Device>(
    config: &C,
    ep: Endpoint<'_>,
    endpoint: &str,
    node: &mut Node<D>,
) {
    // ManuallyDrop: if this future is cancelled mid-session, the resources leak
    // instead of being dropped while the session still borrows them.
    let mut storage = ManuallyDrop::new(Resources::<C>::default());
    let resources: *mut Resources<'_, C> = &mut *storage;
    {
        // SAFETY: `resources` points to `storage`, which outlives this block.
        // Everything that borrows it (the session and what `serve` declares on
        // it) is created and dropped inside this block.
        let resources = unsafe { &mut *resources };
        match zenoh::connect(resources, config, ep).await {
            Ok(session) => {
                zenoh::info!("connected to {}", endpoint);
                if let Err(e) = node.serve(&session).await {
                    zenoh::warn!("session ended: {}", e);
                }
            }
            Err(e) => zenoh::warn!("connect to {} failed: {}", endpoint, e),
        }
    }
    // SAFETY: the session borrowing `storage` was dropped at the end of the
    // block above, and `storage` is not used afterwards.
    unsafe { ManuallyDrop::drop(&mut storage) };
}
