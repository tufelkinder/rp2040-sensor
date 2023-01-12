# Sensor Pod

## Current status

TODO:
- More docs

## Running with debug probe

For debug probe you can use any DAP probe.
I use an rp pico flashed with rust-dap firmware, specifically [this build](https://raw.githubusercontent.com/9names/binary-bits/main/rust-dap-pico-ramexec-setclock.uf2)

```
# SWD pin connections on the rp pico
swclk = pin 2
swdio = pin 3
```

### Prerequisites
```
$ cargo install cargo-flash
$ cargo install cargo-embed
```

### Procedure
```
# Flash bootloader
$ cargo flash --manifest-path ./boot/Cargo.toml --release --chip RP2040

# Flash app
$ cargo flash --release --chip RP2040

# Watch logs
$ cargo embed --release
```

## Running with UF2 image

To flash the app using the UF2 bootloader, you need the [mergehex](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fug_nrf_cltools%2FUG%2Fcltools%2Fnrf_mergehex.html) utility to merge the application and bootloader into a single file. Use the [elf2uf2](https://github.com/JoNil/elf2uf2-rs) utility to convert the elf file into a .uf2 file that you can copy to the rp2040 USB drive.

```
# Build bootloader
$ cargo build --manifest-path ./boot/Cargo.toml --release

# Build app
$ cargo build --release

# Merge the files
$ mergehex -m ./boot/target/thumbv6m-none-eabi/release/sensor-pod-bootloader ./target/thumbv6m-none-eabi/release/sensor-pod -f ELF -o firmware.elf

# Convert the ELF output to UF2
$ elf2uf2-rs firmware.elf firmware.uf2
```

## UF2 workaround

Seems the above method is not working correctly for some reason that i'm still investigating...

In the meantime, to get hold of a uf2 file you can flash the bootloader and application with a debug probe
(see above instructions) to the device and then use [picotool](https://github.com/raspberrypi/picotool)
to download the program from the device back into a uf2 file on the computer.

```
$ picotool save -r 0x10000000 0x10109000 firmware.uf2
```


## Controller commands

You can send commands to the sensor pods over serial (115200 baud) in the following format.
Note that each message shall end with newline (\n or \r)

```
# Broadcast message to all devices
{"action":"status"}

# ...or
{"id":null,"action":"status"}

# Target a specific device using the "id" parameter
{"id":0,"action":"status"}
```

### Available actions
```
# Get status (firmware version and uptime in seconds)
{"action":"status"}

# Get sensor reading
{"action":"poll"}

# Set heartbeat interval in seconds
{"id":0,"action":{"set_heartbeat":8}}

# Set threshold for activity/inactivity events (1-255)
{"id":0,"action":{"set_activity_threshold":10}}

# Set timeout in seconds (0-255) that motion level must be below threshold to trigger the "inactive" event
{"id":0,"action":{"set_inactivity_time":2}}

# Set threshold for "tap" event (1-255)
{"id":0,"action":{"set_tap_threshold":220}}

# Set duration for "tap" event (1-255, 1.25 ms per step)
{"id":0,"action":{"set_tap_duration":220}}

# Reset configuration to default values
{"id":0,"action":"reset_config"}

# Trigger self test that will shake the sensor and generate activity and inactivity events
{"id":0,"action":"self_test"}

# Reboot device
{"id":0,"action":"reboot"}

# Perform (mock) firmware update
{"id":0,"action":{"dfu_write":{"version":"0.1.1","offset":0,"data":[],"is_last":true}}}

# Commit to updated firmware (or else it will rollback to previous version on reboot)
{"id":0,"action":"dfu_mark_booted"}
```

## Response messages

```
# "poll" response / heatbeat message
{"id":0,"payload":{"data":{"x":5196,"y":1535,"z":14156}}}

# "status" response (firmware version, uptime in seconds)
{"id":0,"payload":{"info":{"version":"0.1.0","uptime":198}}}

# "activity" event was triggered
{"id":0,"payload":{"event":{"type":"activity","data":{"x":1330,"y":15871,"z":12928}}}}

# "inactivity" event was triggered
{"id":0,"payload":{"event":{"type":"inactivity","data":{"x":5222,"y":1483,"z":13926}}}}

# "tap" event was triggered
{"id":0,"payload":{"event":{"type":"tap","data":{"x":16793,"y":-11317,"z":-5248}}}}

# "dfu_write" response (acknowledges a specific chunk was written to memory)
{"id":0,"payload":{"dfu_offset":4096}}
```
