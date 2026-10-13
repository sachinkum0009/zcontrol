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

//! Fake two-wheel rover on a laptop: the same `zc-core` + `FakeRover` code as
//! the ESP32-S3 firmware, over `zenoh-std` instead of WiFi.
//!
//! ```bash
//! ros2 run rmw_zenoh_cpp rmw_zenohd &
//! cargo run -p zc-virtual-device -- --endpoint tcp/127.0.0.1:7447 --device rover1
//! ```

use zc_core::{Node, NodeConfig, ZcSessionConfig, run};
use zc_drivers::fake::{Echo, FakeRover};
use zenoh_std::StdLinkManager;

type Config = ZcSessionConfig<StdLinkManager, 512>;

fn usage() -> ! {
    eprintln!(
        "usage: zc-virtual-device [--endpoint tcp/127.0.0.1:7447] [--device rover1] \
         [--period-ms 10] [--watchdog-ms 100] [--echo]"
    );
    std::process::exit(2)
}

fn leak(s: String) -> &'static str {
    Box::leak(s.into_boxed_str())
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let mut endpoint = String::from("tcp/127.0.0.1:7447");
    let mut device = String::from("rover1");
    let mut period_ms = 10u64;
    let mut watchdog_ms = 100u64;
    let mut echo = false;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().unwrap_or_else(|| usage());
        match arg.as_str() {
            "--endpoint" => endpoint = value(),
            "--device" => device = value(),
            "--period-ms" => period_ms = value().parse().unwrap_or_else(|_| usage()),
            "--watchdog-ms" => watchdog_ms = value().parse().unwrap_or_else(|_| usage()),
            "--echo" => echo = true,
            _ => usage(),
        }
    }

    let mut node_config = NodeConfig::new(
        leak(format!("zc/{device}/state")),
        leak(format!("zc/{device}/cmd")),
    );
    node_config.period = embassy_time::Duration::from_millis(period_ms);
    node_config.watchdog = embassy_time::Duration::from_millis(watchdog_ms);

    log::info!(
        "virtual device '{device}' -> {endpoint}, {} Hz, watchdog {watchdog_ms} ms",
        1000 / period_ms.max(1)
    );

    let config = Config::new(StdLinkManager);
    if echo {
        // Timing loopback for examples/latency_probe
        let mut node = Node::new(Echo::default(), node_config);
        run(&config, &endpoint, &mut node).await
    } else {
        let mut node = Node::new(FakeRover::new(20.0), node_config);
        run(&config, &endpoint, &mut node).await
    }
}
