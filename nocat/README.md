# Presentation Remote

A wireless presentation remote built with two Adafruit Feather RP2040 RFM69 boards. The sender scans a 2-key matrix and transmits key events over 915 MHz radio to the receiver, which outputs them as USB HID keyboard arrow keys.

## Hardware

Both boards: **Adafruit Feather RP2040 RFM69** (product 5712)

### Radio Pinout (SPI1)

The Adafruit pinout image is misleading — it labels GPIO 18/19/20 as RFM SPI pins, but they are actually DIO5/DIO3/DIO4. The real SPI bus is SPI1:

| Function | GPIO | Notes |
|----------|------|-------|
| SPI1 SCK | 14 | Hardwired to RFM69 |
| SPI1 MOSI | 15 | Hardwired to RFM69 |
| SPI1 MISO | 8 | Hardwired to RFM69 |
| RFM CS | 16 | Software CS |
| RFM RST | 17 | Active-high reset |
| RFM DIO0 | 21 | Packet-ready interrupt |

### NeoPixel

Built-in WS2812 on **GPIO 4**, driven via PIO0 state machine 0.

---

## Remote (Sender) — `remote/`

**Binary:** `nocat_remote.rs`

Scans a 1×2 NeoKey matrix (COL2ROW diode orientation) and broadcasts key events over RFM69 radio. Runs on battery power with aggressive power saving.

### Pin Assignments

| Function | GPIO | Notes |
|----------|------|-------|
| Key matrix row | 11 | Feather D11 |
| Key matrix col0 | 12 | Feather D12 |
| Key matrix col1 | 13 | Feather D13 |
| Battery ADC | 27 | VBAT/2 via 100K/100K divider |
| Address jumper | 28 | 3.3V = address 3, floating = address 1 |

### Features

- **Radio sleep between key presses** — RFM69 sleeps (~0.1µA) when idle, wakes only to TX/RX (~130mA burst)
- **+20 dBm TX power** with high-power boost registers (PA1+PA2)
- **Battery monitoring** — reads VBAT via ADC every 60 seconds, flashes red + sends radio alert when ≤ 3.2V
- **Boot announcement** — sends `[0xFE, 0xFE]` to receiver on boot (3 retries)
- **Boot status** — flashes green 5 times on power-up
- **NeoPixel signal strength** — blue brightness based on RSSI reply from receiver
- **Key flash** — red (key 0) or green (key 1) flash when receiver confirms key press
- **Address jumper** — GPIO28 pulled to 3.3V selects radio address 3 (for multi-sender setups)
- **USB serial logging** via `log::info!`

### Battery Monitoring Hardware

The Feather RP2040 RFM69 does not have a built-in battery voltage divider on any GPIO. An external divider is required:

```
VBAT (JST pin 2) ---[ 100K ]--- GPIO27 ---[ 100K ]--- GND
```

This gives VBAT/2 at GPIO27. The ADC reads this and calculates: `voltage = adc_value * 3.3 * 2.0 / 4096.0`

### Building

```sh
# With battery monitoring (default — requires external divider on GPIO27)
cargo run --bin nocat_remote

# Without battery monitoring
cargo run --bin nocat_remote --features no_battery_monitor
```

### Radio Protocol

The sender sends broadcast packets with 2-byte payloads:

| Payload | Meaning |
|---------|---------|
| `[key_index, 0/1]` | Key event (key 0 or 1, pressed=1/released=0) |
| `[0xFF, 0xFF]` | Battery low alert |
| `[0xFE, 0xFE]` | Boot announcement |

---

## Base (Receiver) — `base/`

**Binary:** `nocat_base.rs`

Listens for RFM69 radio packets and outputs USB HID keyboard reports. Plugs into the presentation computer via USB.

### Features

- **USB HID keyboard** — outputs configurable keycodes as self-contained taps (keydown + 30ms hold + keyup)
- **USB CDC serial logging** — composite USB device (CDC + HID)
- **Always-on radio RX** — uses Stack/Runner architecture for concurrent TX/RX
- **RSSI reply** — sends `[rssi, key_index]` back to the sender after each received packet
- **Boot status** — flashes green 5 times on power-up
- **Sender boot detection** — flashes blue 5 times when sender boots
- **NeoPixel signal strength** — blue brightness based on received RSSI
- **Key flash** — red (key 0) or green (key 1) flash on key press
- **Battery low alert** — flashes red continuously when sender reports low battery
- **+20 dBm TX power** (for RSSI replies)

### Configurable Keycodes

Edit `KEY_MAP` at the top of `nocat_base.rs`:

```rust
const KEY_MAP: [u8; 2] = [0x50, 0x4F]; // [Left Arrow, Right Arrow]
```

Common HID keycodes:

| Key | Code |
|-----|------|
| Left Arrow | 0x50 |
| Right Arrow | 0x4F |
| Up Arrow | 0x52 |
| Down Arrow | 0x51 |
| Spacebar | 0x2C |
| Escape | 0x29 |
| F5 | 0x3E |
| B (black screen) | 0x05 |
| W (white screen) | 0x1A |

### Building

```sh
cargo run --bin nocat_base
```

---

## Radio Configuration

Both boards use `rfm69-async` with `config::my_defaults`:

- **Frequency:** 915 MHz
- **Modulation:** GFSK, 100 kbps
- **Network ID:** 42
- **Sync word:** `[0x2D, 0x2D]`
- **TX power:** +20 dBm (PA1+PA2 with boost registers)

The `rfm69-async` crate is forked locally at `../../rfm69-async/` with an added `set_tx_power_boost()` method. The fork is referenced via `[patch.crates-io]` in both Cargo.toml files.

## Architecture

Both programs use Embassy's async executor with a task-based architecture:

- **Orchestrator pattern** — shared state, event channels, and signals
- **Dedicated tasks** for keyboard scanning, radio TX/RX, NeoPixel display, and battery monitoring
- **Channels** for inter-task communication (`embassy_sync::channel::Channel`)
- **Signals** for one-shot notifications (`embassy_sync::signal::Signal`)

## Flashing

1. Hold **BOOTSEL** while plugging in USB
2. The board appears as a USB drive
3. Run `cargo run --bin nocat_remote` (sender) or `cargo run --bin nocat_base` (receiver)
4. `elf2uf2-rs` deploys automatically to the mounted drive
