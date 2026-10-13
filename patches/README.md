# Local patches of upstream crates

| Crate | Base | Change | Why |
|---|---|---|---|
| `zenoh-embassy` | zenoh-nostd `e88f73a` (`platforms/zenoh-embassy`) | `set_nagle_enabled(false)` on every TCP socket (`src/lib.rs`) | smoltcp defaults to Nagle on (embassy-net 0.8 does not expose the 10 ms delayed ACK setting). Over WiFi at 100 Hz this bunched state samples (17 % arrived < 2 ms after the previous one) and pushed p99 round trip to ~45 ms. `zenoh-std` already sets `TCP_NODELAY`. |

Used via `[patch]` in `zc-boards/esp32s3/Cargo.toml`. Drop the patch once upstream
disables Nagle in `zenoh-embassy` (to be proposed upstream).
