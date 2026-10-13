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

//! Z Control firmware for the ESP32-S3: a fake two-wheel rover exposed to
//! `zenbedded_hardware_interface` over WiFi.
//!
//! Configured at build time:
//!
//! | env            | default              |
//! |----------------|----------------------|
//! | `WIFI_SSID`     | (required)           |
//! | `WIFI_PASSWORD` | (required)           |
//! | `ZC_ENDPOINT`   | `tcp/192.168.1.100:7447` (host running the zenoh router) |
//! | `ZC_DEVICE`     | `rover1` → keys `zc/rover1/state`, `zc/rover1/cmd` |
//!
//! `--features echo` builds the timing loopback device instead of the fake
//! rover (see `examples/latency_probe`).
//!
//! ```bash
//! WIFI_SSID=lab WIFI_PASSWORD=*** ZC_ENDPOINT=tcp/192.168.178.56:7447 cargo run --release
//! ```

#![no_std]
#![no_main]

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, Runner, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::{clock::CpuClock, rng::Rng, timer::timg::TimerGroup};
use esp_radio::{
    Controller,
    wifi::{
        ClientConfig, ModeConfig, PowerSaveMode, WifiController, WifiDevice, WifiEvent,
        WifiStaState,
    },
};
use static_cell::StaticCell;
use zc_core::{Node, NodeConfig, ZcSessionConfig, run, zenoh_nostd::session::zenoh};
#[cfg(feature = "echo")]
use zc_drivers::fake::Echo;
#[cfg(not(feature = "echo"))]
use zc_drivers::fake::FakeRover;
use zenoh_embassy::EmbassyLinkManager;

use esp_println as _;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");
const ENDPOINT: &str = match option_env!("ZC_ENDPOINT") {
    Some(v) => v,
    None => "tcp/192.168.1.100:7447",
};
const DEVICE: &str = match option_env!("ZC_DEVICE") {
    Some(v) => v,
    None => "rover1",
};

/// 512 B MTU per socket, 3 sockets: what the upstream examples use.
type LinkManager = EmbassyLinkManager<'static, 512, 3>;
type Config = ZcSessionConfig<LinkManager, 512>;

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);
    zenoh::info!("zc-esp32s3: device '{}' -> {}", DEVICE, ENDPOINT);

    // WiFi + network stack
    static RADIO: StaticCell<Controller<'static>> = StaticCell::new();
    let radio = RADIO.init(esp_radio::init().expect("radio init"));
    let (wifi_controller, interfaces) =
        esp_radio::wifi::new(radio, peripherals.WIFI, Default::default()).expect("wifi init");

    let rng = Rng::new();
    let seed = rng.random() as u64 | ((rng.random() as u64) << 32);
    static NET_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, net_runner) = embassy_net::new(
        interfaces.sta,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        NET_RESOURCES.init(StackResources::new()),
        seed,
    );

    spawner.must_spawn(wifi_task(wifi_controller));
    spawner.must_spawn(net_task(net_runner));

    stack.wait_config_up().await;
    if let Some(cfg) = stack.config_v4() {
        zenoh::info!("got IP {}", cfg.address);
    }

    // Z Control node
    static STATE_KEY: StaticCell<heapless::String<64>> = StaticCell::new();
    static COMMAND_KEY: StaticCell<heapless::String<64>> = StaticCell::new();
    let state_key = STATE_KEY.init(key("state"));
    let command_key = COMMAND_KEY.init(key("cmd"));

    static CONFIG: StaticCell<Config> = StaticCell::new();
    let config = CONFIG.init(Config::new(LinkManager::new(stack)));

    #[cfg(not(feature = "echo"))]
    let device = FakeRover::new(20.0);
    #[cfg(feature = "echo")]
    let device = Echo::default();
    let mut node = Node::new(
        device,
        NodeConfig::new(state_key.as_str(), command_key.as_str()),
    );
    run(config, ENDPOINT, &mut node).await
}

/// `zc/<DEVICE>/<suffix>`
fn key(suffix: &str) -> heapless::String<64> {
    let mut s = heapless::String::new();
    write!(s, "zc/{DEVICE}/{suffix}").expect("ZC_DEVICE too long");
    s
}

/// Keeps the station connected, with power saving off (modem sleep adds 100 ms+
/// latency spikes, which breaks 100 Hz control).
#[embassy_executor::task]
async fn wifi_task(mut controller: WifiController<'static>) {
    loop {
        if esp_radio::wifi::sta_state() == WifiStaState::Connected {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            zenoh::warn!("WiFi disconnected");
            Timer::after(Duration::from_millis(1000)).await;
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let mode = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(WIFI_SSID.into())
                    .with_password(WIFI_PASSWORD.into()),
            );
            controller.set_config(&mode).expect("wifi config");
            controller.start_async().await.expect("wifi start");
            if controller.set_power_saving(PowerSaveMode::None).is_err() {
                zenoh::warn!("could not disable WiFi power saving");
            }
        }
        zenoh::info!("connecting to WiFi '{}'", WIFI_SSID);
        match controller.connect_async().await {
            Ok(()) => zenoh::info!("WiFi connected"),
            Err(e) => {
                zenoh::warn!("WiFi connect failed: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

// zenoh-nostd needs randomness (session IDs); use the hardware RNG.
getrandom::register_custom_getrandom!(hw_getrandom);
fn hw_getrandom(bytes: &mut [u8]) -> Result<(), getrandom::Error> {
    Rng::new().read(bytes);
    Ok(())
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    zenoh::error!("panic: {}", defmt::Display2Format(info));
    // A reset is the safest recovery: motors are off while the chip reboots.
    esp_hal::system::software_reset()
}
