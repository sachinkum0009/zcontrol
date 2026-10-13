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

//! Timing measurement against an `Echo` device (ESP32-S3 or virtual).
//!
//! Sends `EchoCommand { seq }` at a fixed rate, the way the ros2_control plugin
//! sends commands on every `write()`, and records when each `EchoState` arrives.
//!
//! * round trip: command sent -> first state that echoes it. Includes waiting
//!   for the device's next control tick (0..period), i.e. it is the
//!   command-to-state latency ros2_control would see.
//! * state jitter: inter-arrival time of state samples.
//! * state loss: gaps in the device tick counter.
//! * commands: sent vs applied by the device (latest-value: two commands in one
//!   device tick count as one applied).
//!
//! ```bash
//! zc-latency-probe --endpoint tcp/127.0.0.1:7447 --device echo1 --seconds 60 --csv /tmp/x.csv
//! ```

use std::{
    fmt::Write as _,
    sync::Mutex,
    time::{Duration, Instant},
};

use embassy_futures::select::{Either, select};
use zc_core::{
    Packed, ZcSessionConfig,
    zenoh_nostd::session::{Endpoint, Error, ZSessionConfig, zenoh},
};
use zc_drivers::fake::{EchoCommand, EchoState};
use zenoh_std::StdLinkManager;

type Config = ZcSessionConfig<StdLinkManager, 4096>;

/// (arrival time, state) of every state sample.
static STATES: Mutex<Vec<(Instant, EchoState)>> = Mutex::new(Vec::new());

struct Args {
    endpoint: String,
    device: String,
    seconds: u64,
    rate_hz: u64,
    csv: Option<String>,
}

fn parse_args() -> Args {
    let mut a = Args {
        endpoint: "tcp/127.0.0.1:7447".into(),
        device: "echo1".into(),
        seconds: 60,
        rate_hz: 100,
        csv: None,
    };
    let usage = || -> ! {
        eprintln!(
            "usage: zc-latency-probe [--endpoint E] [--device echo1] [--seconds 60] [--rate 100] [--csv FILE]"
        );
        std::process::exit(2)
    };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        let mut v = || it.next().unwrap_or_else(|| usage());
        match arg.as_str() {
            "--endpoint" => a.endpoint = v(),
            "--device" => a.device = v(),
            "--seconds" => a.seconds = v().parse().unwrap_or_else(|_| usage()),
            "--rate" => a.rate_hz = v().parse().unwrap_or_else(|_| usage()),
            "--csv" => a.csv = Some(v()),
            _ => usage(),
        }
    }
    a
}

fn leak(s: String) -> &'static str {
    Box::leak(s.into_boxed_str())
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    let args = parse_args();
    // The embassy std executor never returns from main, so exit explicitly.
    match probe(&args).await {
        Ok(()) => std::process::exit(0),
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(1);
        }
    }
}

async fn probe(args: &Args) -> zenoh::ZResult<()> {
    let state_ke = zenoh::keyexpr::new(leak(format!("zc/{}/state", args.device)))?;
    let cmd_ke = zenoh::keyexpr::new(leak(format!("zc/{}/cmd", args.device)))?;

    let session = zenoh::connect!(
        Config: Config::new(StdLinkManager),
        Endpoint::try_from(args.endpoint.as_str())?
    );

    let _sub = session
        .declare_subscriber(state_ke)
        .callback_sync(|sample| {
            let now = Instant::now();
            if let Some(state) = EchoState::decode(sample.payload()) {
                STATES.lock().unwrap().push((now, state));
            }
        })
        .finish()
        .await?;

    eprintln!(
        "probing {} via {} for {} s at {} Hz ...",
        args.device, args.endpoint, args.seconds, args.rate_hz
    );

    let period = embassy_time::Duration::from_micros(1_000_000 / args.rate_hz);
    let total = (args.seconds * args.rate_hz) as usize;
    let mut sent: Vec<Instant> = Vec::with_capacity(total);
    // Start above anything a previous run left in the device's `last_seq`, so
    // stale echoes are never matched; seq start_seq + i was sent at sent[i].
    let start_seq: u64 = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64
        * 1000;

    let send_loop = async {
        let mut ticker = embassy_time::Ticker::every(period);
        let mut buf = [0u8; EchoCommand::SIZE];
        for i in 0..total {
            EchoCommand {
                seq: start_seq + i as u64,
            }
            .encode(&mut buf);
            sent.push(Instant::now());
            session.put(cmd_ke, &buf).finish().await?;
            ticker.next().await;
        }
        // Let the last echoes arrive.
        embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
        Ok::<(), Error>(())
    };

    let run_start = Instant::now();
    match select(session.run(), send_loop).await {
        Either::First(r) => r?,
        Either::Second(r) => r?,
    }
    let elapsed = run_start.elapsed();

    let states = std::mem::take(&mut *STATES.lock().unwrap());
    print!("{}", report(args, &sent, start_seq, &states, elapsed));
    if let Some(path) = &args.csv {
        write_csv(path, run_start, &sent, start_seq, &states).expect("write csv");
        eprintln!("raw data: {path}");
    }
    Ok(())
}

fn pct(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    let idx = ((p / 100.0) * (sorted.len() - 1) as f64).round() as usize;
    sorted[idx]
}

fn summary(name: &str, mut v: Vec<f64>) -> String {
    if v.is_empty() {
        return format!("{name:<22} no data\n");
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = v.len() as f64;
    let mean = v.iter().sum::<f64>() / n;
    let std = (v.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n).sqrt();
    format!(
        "{name:<22} n={:<6} min {:6.2}  p50 {:6.2}  p95 {:6.2}  p99 {:6.2}  p99.9 {:7.2}  max {:7.2}  mean {:6.2}  std {:5.2}  [ms]\n",
        v.len(),
        v[0],
        pct(&v, 50.0),
        pct(&v, 95.0),
        pct(&v, 99.0),
        pct(&v, 99.9),
        v[v.len() - 1],
        mean,
        std
    )
}

fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1e3
}

fn report(
    args: &Args,
    sent: &[Instant],
    start_seq: u64,
    states: &[(Instant, EchoState)],
    elapsed: Duration,
) -> String {
    let mut out = String::new();
    let period_ms = 1000.0 / args.rate_hz as f64;
    writeln!(
        out,
        "\n=== {} via {}: {:.1} s, {} commands sent, {} states received ===",
        args.device,
        args.endpoint,
        elapsed.as_secs_f64(),
        sent.len(),
        states.len()
    )
    .ok();

    if states.len() < 2 {
        writeln!(
            out,
            "not enough state samples (is the Echo device running?)"
        )
        .ok();
        return out;
    }

    // State stream: inter-arrival and loss (device tick gaps).
    let gaps: Vec<f64> = states.windows(2).map(|w| ms(w[1].0 - w[0].0)).collect();
    let mut lost_states = 0u64;
    let mut reboots = 0u32;
    for w in states.windows(2) {
        let (a, b) = (w[0].1.tick, w[1].1.tick);
        if b > a {
            lost_states += b - a - 1;
        } else {
            reboots += 1;
        }
    }
    let ticks = states
        .last()
        .unwrap()
        .1
        .tick
        .saturating_sub(states[0].1.tick)
        + 1;
    let state_rate = states.len() as f64 / ms(states.last().unwrap().0 - states[0].0) * 1e3;

    out += &summary("state inter-arrival", gaps.clone());
    for thr in [2.0, 5.0] {
        let n = gaps.iter().filter(|&&g| g > thr * period_ms).count();
        writeln!(out, "  gaps > {:>3.0} ms: {n}", thr * period_ms).ok();
    }
    let n100 = gaps.iter().filter(|&&g| g > 100.0).count();
    writeln!(out, "  gaps > 100 ms: {n100}  (watchdog default)").ok();
    writeln!(
        out,
        "  rate {:.2} Hz, lost {} of {} device ticks ({:.3} %), device reboots seen: {}",
        state_rate,
        lost_states,
        ticks,
        100.0 * lost_states as f64 / ticks as f64,
        reboots
    )
    .ok();

    // Round trip: first state echoing seq N.
    let mut rtt = Vec::new();
    let mut max_seen = 0u64;
    for (t, s) in states {
        let seq = s.last_seq;
        if seq > max_seen && seq >= start_seq {
            max_seen = seq;
            let idx = (seq - start_seq) as usize;
            if let Some(t_sent) = sent.get(idx) {
                rtt.push(ms(*t - *t_sent));
            }
        }
    }
    out += &summary("round trip (cmd->state)", rtt.clone());
    for thr in [20.0, 50.0, 100.0] {
        let n = rtt.iter().filter(|&&r| r > thr).count();
        writeln!(out, "  round trip > {thr:>3.0} ms: {n}").ok();
    }

    // Command delivery.
    let first = states.iter().find(|(_, s)| s.last_seq >= start_seq);
    let applied = match first {
        Some((_, f)) => states.last().unwrap().1.commands - f.commands + 1,
        None => 0,
    };
    let last_echoed = states.last().unwrap().1.last_seq;
    writeln!(
        out,
        "commands: sent {}, applied by device {} ({:.2} %), last echoed seq {} of {}",
        sent.len(),
        applied,
        100.0 * applied as f64 / sent.len() as f64,
        last_echoed,
        start_seq + sent.len() as u64 - 1
    )
    .ok();
    writeln!(
        out,
        "  (applied < sent: commands overwritten within one device tick or lost; TCP itself does not drop)"
    )
    .ok();
    out
}

fn write_csv(
    path: &str,
    t0: Instant,
    sent: &[Instant],
    start_seq: u64,
    states: &[(Instant, EchoState)],
) -> std::io::Result<()> {
    let mut s = String::from("kind,t_ms,cmd_index_or_tick,last_seq,commands\n");
    for (i, t) in sent.iter().enumerate() {
        writeln!(s, "cmd,{:.3},{},,", ms(*t - t0), i).ok();
    }
    for (t, st) in states {
        writeln!(
            s,
            "state,{:.3},{},{},{}",
            ms(t.saturating_duration_since(t0)),
            st.tick,
            st.last_seq as i64 - start_seq as i64,
            st.commands
        )
        .ok();
    }
    std::fs::write(path, s)
}
