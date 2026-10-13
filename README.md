# Z Control

Rust firmware library that exposes an embedded board (ESP32-S3 first) to
**ros2_control** through the existing
[`zenbedded_hardware_interface`](../zenbedded_hardware_interface) plugin, over
zenoh using [`zenoh-nostd`](https://github.com/eclipse-zenoh/zenoh-nostd).
Design and roadmap: `project.md` in the workspace root.

Toolchain setup: [installation_instructions.md](installation_instructions.md).

```
 ros2_control (diff_drive_controller, joint_state_broadcaster, ...)
        │
 zenbedded_hardware_interface/ZenbeddedHardware   ← unchanged C++ plugin
        │  zc/<device>/state  (device → host, packed struct, 100 Hz)
        │  zc/<device>/cmd    (host → device, packed struct, every write())
 zenoh router (rmw_zenohd)
        │  WiFi / TCP
 zc-core Node ── Device impl (zc-drivers::fake::FakeRover for now)
        │
 ESP32-S3 (embassy, esp-hal)  or  laptop (zc-virtual-device)
```

## Layout

| Path | Role |
|---|---|
| `zc-proto` | `no_std` wire format: `Packed` trait and `packed_struct!` macro, layout compatible with the plugin's `interface_schema.yaml` |
| `zc-core` | `no_std` runtime: `Device` trait, `Node` (100 Hz state publish, latest command, watchdog, safe state on disconnect), reconnecting `run()` |
| `zc-drivers` | Device implementations. Now: `fake::FakeRover` (two wheels, velocity integrated into position) |
| `zc-boards/esp32s3` | ESP32-S3 firmware (separate workspace, `esp` toolchain) |
| `examples/virtual_device` | Same `FakeRover` on a laptop via `zenoh-std`, for testing without hardware |
| `zc_bringup` | ament package: URDF, schema, controllers, launch for the fake rover |

## Wire format (compatible with the current plugin)

The plugin loads `schema_path` (YAML) and expects each payload to be the listed
fields packed back to back, little-endian, no padding, no header, in YAML order.
A payload of the wrong size is dropped. The firmware writes the matching struct
by hand:

```rust
zc_core::packed_struct! {
    /// state_interfaces of fake_rover_schema.yaml, same order
    pub struct FakeRoverState {
        pub left_wheel_joint_position: f64,
        pub left_wheel_joint_velocity: f64,
        pub right_wheel_joint_position: f64,
        pub right_wheel_joint_velocity: f64,
    }
}
```

| schema | Rust |
|---|---|
| `float32` | `f32` |
| `float64` | `f64` |
| `int32` | `i32` |
| `uint64` | `u64` |
| `int16` | `i16` |
| `uint8` | `u8` |

The plugin has no sequence numbers, timestamps, description query or liveliness
yet, so neither does the firmware. Generating the structs from the YAML is
planned (option b).

## Writing a device

```rust
impl zc_core::Device for MyBoard {
    type State = MyState;      // packed_struct!
    type Command = MyCommand;  // packed_struct!
    fn read(&mut self, now: Instant) -> MyState { /* sample sensors */ }
    fn write(&mut self, now: Instant, cmd: &MyCommand) { /* set setpoints */ }
    fn on_command_timeout(&mut self, now: Instant) { /* motors to safe state */ }
}

let mut node = Node::new(MyBoard::new(), NodeConfig::new("zc/rover1/state", "zc/rover1/cmd"));
zc_core::run(&config, "tcp/192.168.1.100:7447", &mut node).await;
```

`read`/`write`/`on_command_timeout` run in the control loop and must not block.
`on_command_timeout` is called when no command arrives for `watchdog` (default
100 ms) and when the zenoh session is lost.

## Run without hardware (virtual device)

```bash
source <ws>/.toolchains/env.sh
cd zcontrol && cargo build -p zc-virtual-device
cargo test                                         # zc-proto, zc-drivers

# terminal 1: router (skip if one already runs on 7447)
ros2 run rmw_zenoh_cpp rmw_zenohd
# terminal 2: device
./target/debug/zc-virtual-device --endpoint tcp/127.0.0.1:7447 --device rover1
# terminal 3: ros2_control (colcon build --packages-select zc_bringup first)
ros2 launch zc_bringup fake_rover.launch.py
# terminal 4: drive
ros2 topic pub -r 20 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.2}}}"
ros2 topic echo /diff_drive_controller/odom
```

## Run on the ESP32-S3

```bash
source <ws>/.toolchains/env.sh
cd zcontrol/zc-boards/esp32s3
WIFI_SSID=<ssid> WIFI_PASSWORD=<pw> ZC_ENDPOINT=tcp/<host-ip>:7447 ZC_DEVICE=rover1 \
  cargo run --release          # builds, flashes and opens the defmt log
ros2 launch zc_bringup fake_rover.launch.py endpoint:=tcp/127.0.0.1:7447 device:=rover1
```

The router must listen on an address the board can reach (`rmw_zenohd` listens
on `tcp/[::]:7447` by default). WiFi power saving is disabled in the firmware.

## Status (2026-10-12)

- Virtual device + plugin + `diff_drive_controller`: tested. Wheels integrate,
  odometry moves (0.2 m/s for 3.5 s → odom x = 0.71 m), about 86 commands/s
  arrive at the device.
- Watchdog: tested. After killing `ros2_control_node` mid-drive, the device
  enters its safe state after 110 ms.
- Reconnect: tested. After 5 router restarts the device reconnects each time,
  with no socket leak.
- ESP32-S3 firmware: 630 KB image (15 % of the app partition), flashed from the
  host with `espflash`. It runs on the board over WiFi (FRITZ!Box, router on
  the laptop at 192.168.178.99:7447) with the plugin and `diff_drive_controller`.
  Driving at 0.2 m/s gives wheel velocity 4.0 rad/s and odom x = 0.71 m after
  about 3.5 s. The wheels stop after `cmd_vel` stops.
- WiFi timing (10 min, 100 Hz, Nagle off): 0 state loss, round trip
  p50 8.9 ms / p99 17.5 ms / max 116 ms (1 round trip > 100 ms). Full table and
  method in `project.md` ("Timing results"). Measure with
  `zc-latency-probe` against firmware built with `--features echo`.
