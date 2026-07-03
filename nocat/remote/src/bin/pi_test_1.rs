//! Key matrix scanner + RFM69 radio transmitter
//!
//! Scans a 1x2 NeoKey matrix and broadcasts key events over RFM69 radio
//! to another board, while also logging locally via USB serial.

#![no_std]
#![no_main]

use assign_resources::assign_resources;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
#[cfg(not(feature = "no_battery_monitor"))]
use embassy_rp::adc::{Adc, Channel as AdcChannel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler};
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::gpio::{Flex, Input, Level, Output, Pull};
use embassy_rp::peripherals::{self, PIO0, SPI1, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel as SyncChannel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Timer, with_timeout};
use rfm69_async::registers::OpMode;
use rfm69_async::{Address, Flags, Packet, Rfm69, config};
use smart_leds::RGB8;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// Hardware resource assignment
#[cfg(not(feature = "no_battery_monitor"))]
assign_resources! {
    keyboard: Keyboard {
        // Feather header D11/D12/D13 — free, not used by RFM69
        row: PIN_11,
        col0: PIN_12,
        col1: PIN_13,
    },
    battery: Battery {
        // GPIO27 reads VBAT/2 via external 100K/100K divider
        // (requires external resistors — enable with `battery_monitor` feature)
        adc: ADC,
        pin: PIN_27,
    },
    radio: Radio {
        // Feather RP2040 RFM69 — SPI1 bus per Adafruit board definition
        // SCK=GPIO14, MOSI=GPIO15, MISO=GPIO8
        // CS=GPIO16, RST=GPIO17, DIO0=GPIO21
        sck: PIN_14,
        mosi: PIN_15,
        miso: PIN_8,
        cs: PIN_16,
        reset: PIN_17,
        dio0: PIN_21,
    },
}

// Hardware resource assignment (without battery_monitor)
#[cfg(feature = "no_battery_monitor")]
assign_resources! {
    keyboard: Keyboard {
        row: PIN_11,
        col0: PIN_12,
        col1: PIN_13,
    },
    radio: Radio {
        // Feather RP2040 RFM69 — SPI1 bus per Adafruit board definition
        // SCK=GPIO14, MOSI=GPIO15, MISO=GPIO8
        // CS=GPIO16, RST=GPIO17, DIO0=GPIO21
        sck: PIN_14,
        mosi: PIN_15,
        miso: PIN_8,
        cs: PIN_16,
        reset: PIN_17,
        dio0: PIN_21,
    },
}

// Interrupt bindings
bind_interrupts!(struct Irqs {
        #[cfg(not(feature = "no_battery_monitor"))]
    ADC_IRQ_FIFO => AdcInterruptHandler;
    DMA_IRQ_0 => dma::InterruptHandler<peripherals::DMA_CH0>, dma::InterruptHandler<peripherals::DMA_CH1>, dma::InterruptHandler<peripherals::DMA_CH2>;
    PIO0_IRQ_0 => PioInterruptHandler<peripherals::PIO0>;
});

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

// Radio SPI type aliases
type RadioSpi = SpiDevice<'static, NoopRawMutex, Spi<'static, SPI1, embassy_rp::spi::Async>, Output<'static>>;
type RadioDriver = Rfm69<RadioSpi, Output<'static>, embassy_rp::gpio::Input<'static>, Delay>;

// NeoPixel type alias
type NeoPixel = PioWs2812<'static, PIO0, 0, Grb>;

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

/// Events that worker tasks send to the orchestrator
enum Events {
    KeyPressed(u8, bool), // Key index, pressed state
}

/// Radio-bound packets
enum RadioPacket {
    KeyEvent(u8, bool),
    #[cfg(not(feature = "no_battery_monitor"))]
    BatteryLow,
    #[allow(dead_code)]
    Boot,
}

/// The central state of our system, shared between tasks.
#[derive(Clone)]
struct State {
    key0_pressed: bool,
    key1_pressed: bool,
}

impl State {
    const fn new() -> Self {
        Self {
            key0_pressed: false,
            key1_pressed: false,
        }
    }
}

/// Shared state
static SYSTEM_STATE: Mutex<CriticalSectionRawMutex, State> = Mutex::new(State::new());

/// Channel for events from worker tasks to the orchestrator
static EVENT_CHANNEL: SyncChannel<CriticalSectionRawMutex, Events, 4> = SyncChannel::new();

/// Channel for radio-bound packets
static RADIO_CHANNEL: SyncChannel<CriticalSectionRawMutex, RadioPacket, 4> = SyncChannel::new();

/// Signal for notifying about state changes
static STATE_CHANGED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signal for the latest RSSI reading (dBm) from received radio packets
static RSSI_SIGNAL: Signal<CriticalSectionRawMutex, i16> = Signal::new();

/// Signal for key flash events (key index 0=red, 1=blue)
static KEY_FLASH_SIGNAL: Signal<CriticalSectionRawMutex, u8> = Signal::new();

/// Signal for battery low warning (triggered by battery_monitor task)
#[cfg(not(feature = "no_battery_monitor"))]
static BATTERY_LOW_SIGNAL: Signal<CriticalSectionRawMutex, f32> = Signal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let r = split_resources! {p};

    // Spawn USB logger task
    let driver = Driver::new(p.USB, UsbIrqs);
    spawner.spawn(logger_task(driver).unwrap());

    // --- RFM69 Radio Setup ---
    // Adafruit Feather RP2040 RFM69: SPI1 bus on GPIO14(SCK)/GPIO15(MOSI)/GPIO8(MISO)
    // CS=GPIO16, RST=GPIO17, DIO0=GPIO21
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 8_000_000; // 8 MHz — RFM69 supports up to 10 MHz
    let spi = Spi::new(
        p.SPI1,
        r.radio.sck,
        r.radio.mosi,
        r.radio.miso,
        p.DMA_CH0,
        p.DMA_CH1,
        Irqs,
        spi_cfg,
    );

    let cs_pin = Output::new(r.radio.cs, Level::High);
    let rst_pin = Output::new(r.radio.reset, Level::Low); // RFM69 reset is active-high; LOW = out of reset
    Timer::after_millis(10).await;

    log::info!("RFM69 radio: SPI1 sck=14 mosi=15 miso=8 cs=16 rst=17 dio0=21");

    // Construct SpiDevice for the driver
    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, Spi<'static, SPI1, embassy_rp::spi::Async>>> = StaticCell::new();
    let spi_bus = SPI_BUS.init(Mutex::new(spi));

    let dio0 = Some(embassy_rp::gpio::Input::new(r.radio.dio0, Pull::None));

    let rfm_spi = SpiDevice::new(spi_bus, cs_pin);

    let network_id = 42;
    let frequency = 915_000_000;
    let mut rfm = Rfm69::new(rfm_spi, rst_pin, dio0, Delay);

    log::info!("RFM69 resetting...");
    match rfm.reset().await {
        Ok(()) => log::info!("RFM69 reset OK, version 0x24"),
        Err(e) => {
            log::error!("RFM69 reset/version error: {:?}", e);
            Timer::after(Duration::from_secs(5)).await;
            panic!();
        }
    }

    log::info!("Applying radio config...");
    if let Err(e) = config::my_defaults(&mut rfm, network_id, frequency).await {
        log::error!("Radio config error: {:?}", e);
        Timer::after(Duration::from_secs(5)).await;
        panic!();
    }

    // Set TX power to max: RFM69HCW +20 dBm with high-power boost
    if let Err(e) = rfm.set_tx_power_boost().await {
        log::error!("Set TX power failed: {:?}", e);
    }
    log::info!("TX power set to +20 dBm (PA1+PA2 boost)");

    // Put radio to sleep until a key is pressed (saves ~16mA vs always-RX)
    rfm.set_mode(OpMode::Sleep).await.ok();
    log::info!("RFM69 radio ready, sleeping until key press");

    // ── NeoPixel Setup (GPIO4) ──
    // Built-in NeoPixel on the Feather RP2040 RFM69 — displays signal
    // strength (RSSI) reported back by the receiver.
    static PIO_COMMON: StaticCell<embassy_rp::pio::Common<'static, PIO0>> = StaticCell::new();
    static WS2812_PROGRAM: StaticCell<PioWs2812Program<'static, PIO0>> = StaticCell::new();
    static WS2812: StaticCell<NeoPixel> = StaticCell::new();

    let Pio { common, sm0, .. } = Pio::new(p.PIO0, Irqs);
    let common = PIO_COMMON.init(common);
    let ws2812_program = WS2812_PROGRAM.init(PioWs2812Program::new(common));
    let ws2812 = WS2812.init(PioWs2812::new(common, sm0, p.DMA_CH2, Irqs, p.PIN_4, ws2812_program));
    log::info!("NeoPixel initialized on GPIO4");

    // ── Sender address jumper (GPIO28) ──
    // If GPIO28 is pulled high (jumper to 3.3V), use address 3.
    // If left unconnected (internal pull-down), use default address 1.
    // This allows two senders to coexist without recompiling.
    let addr_jumper = Input::new(p.PIN_28, Pull::Down);
    let own_address = if addr_jumper.is_high() {
        Address::Unicast(3)
    } else {
        Address::Unicast(1)
    };
    log::info!("Sender address: {:?} (GPIO28={})", own_address, addr_jumper.is_high());

    // Send boot announcement to receiver so it flashes blue.
    // Retry a few times with delays in case the receiver isn't listening yet.
    {
        let boot_packet = Packet::new(
            own_address,
            Address::Broadcast,
            Flags::None,
            &[0xFE, 0xFE], // boot marker
        )
        .unwrap();

        match rfm.send(&boot_packet).await {
            Ok(()) => log::info!("Boot announcement sent"),
            Err(e) => log::warn!("Boot announcement failed: {:?}", e),
        }
        rfm.set_mode(OpMode::Sleep).await.ok();
        Timer::after(Duration::from_secs(2)).await;
    }

    // Spawn orchestrator tasks
    spawner.spawn(orchestrate(spawner).unwrap());
    spawner.spawn(consumer(spawner).unwrap());
    spawner.spawn(keyboard_scanner(spawner, r.keyboard).unwrap());

    // Spawn radio task (sleeps between key presses for power saving)
    spawner.spawn(radio_task(rfm, own_address).unwrap());
    spawner.spawn(neopixel_task(ws2812).unwrap());

    // Spawn battery monitor task (only with battery_monitor feature)
    #[cfg(not(feature = "no_battery_monitor"))]
    {
        spawner.spawn(battery_monitor_task(r.battery).unwrap());
    }
}

/// Radio task: sleeps the RFM69 between key presses to save ~16mA.
/// On key press: wakes radio, TX the key event, briefly RX for RSSI reply,
/// then back to sleep. Also handles battery-low alerts.
#[embassy_executor::task]
async fn radio_task(mut rfm: RadioDriver, own_address: Address) {
    let receiver = RADIO_CHANNEL.receiver();

    loop {
        // Radio sleeps while waiting for events (~0.1µA vs ~16mA in RX)
        let packet = receiver.receive().await;
        let payload = match packet {
            RadioPacket::KeyEvent(key, pressed) => [key, if pressed { 1 } else { 0 }],
            #[cfg(not(feature = "no_battery_monitor"))]
            RadioPacket::BatteryLow => [0xFF, 0xFF], // battery-low marker
            RadioPacket::Boot => [0xFE, 0xFE], // boot marker (unused via channel)
        };

        // Wake radio and send
        let tx_packet = Packet::new(own_address, Address::Broadcast, Flags::None, &payload).unwrap();

        #[cfg(not(feature = "no_battery_monitor"))]
        match &packet {
            RadioPacket::KeyEvent(key, pressed) => match rfm.send(&tx_packet).await {
                Ok(()) => log::info!("Radio TX: key={} pressed={}", key, pressed),
                Err(e) => log::warn!("Radio TX failed: {:?}", e),
            },
            RadioPacket::BatteryLow => match rfm.send(&tx_packet).await {
                Ok(()) => log::warn!("Radio TX: battery low alert sent"),
                Err(e) => log::warn!("Radio TX failed: {:?}", e),
            },
            RadioPacket::Boot => {} // handled in main(), never via channel
        }
        #[cfg(feature = "no_battery_monitor")]
        match &packet {
            RadioPacket::KeyEvent(key, pressed) => match rfm.send(&tx_packet).await {
                Ok(()) => log::info!("Radio TX: key={} pressed={}", key, pressed),
                Err(e) => log::warn!("Radio TX failed: {:?}", e),
            },
            RadioPacket::Boot => {} // handled in main(), never via channel
        }

        // Briefly listen for RSSI reply (500ms timeout)
        match with_timeout(Duration::from_millis(500), rfm.recv()).await {
            Ok(Ok(reply)) => {
                if reply.data.len() >= 2 {
                    let rssi = reply.data[0] as i8 as i16;
                    let key_reply = reply.data[1];
                    log::info!("Radio RX: RSSI reply = {} dBm, key={}", rssi, key_reply);
                    RSSI_SIGNAL.signal(rssi);
                    if key_reply <= 1 {
                        KEY_FLASH_SIGNAL.signal(key_reply);
                    }
                }
            }
            Ok(Err(e)) => log::warn!("Radio RX error: {:?}", e),
            Err(_) => {} // no reply within 500ms — radio goes back to sleep
        }

        // Back to sleep
        rfm.set_mode(OpMode::Sleep).await.ok();
    }
}

/// Maps an RSSI value (in dBm) to a green brightness (0–255).
/// RFM69 RSSI typically ranges from ~-120 (very weak) to ~-30 (very strong).
/// We map [-90, -40] dBm → [1, 255], clamping outside that range.
fn rssi_to_brightness(rssi: i16) -> u8 {
    let scaled = ((rssi as i32 + 90) * 255 / 60).max(1).min(255);
    scaled as u8
}

/// Drives the built-in NeoPixel (GPIO4) to show RF signal strength as
/// green brightness, based on RSSI replies from the receiver.
/// Flashes red (key 0) or blue (key 1) briefly when the receiver reports
/// a key press. Green turns off after 1 second with no signal (battery saving).
#[embassy_executor::task]
async fn neopixel_task(ws2812: &'static mut NeoPixel) {
    const IDLE_BRIGHTNESS: u8 = 0; // LED off when no signal (battery saving)
    let mut current_brightness = IDLE_BRIGHTNESS;
    let off = [RGB8 { r: 0, g: 0, b: 0 }];
    ws2812.write_slice(&off).await;

    // Boot status indicator: flash green 5 times
    let boot_green = RGB8 { r: 0, g: 32, b: 0 };
    for _ in 0..5 {
        ws2812.write_slice(&[boot_green]).await;
        Timer::after(Duration::from_millis(150)).await;
        ws2812.write_slice(&off).await;
        Timer::after(Duration::from_millis(150)).await;
    }

    loop {
        // Check for battery low warning first (highest priority)
        #[cfg(not(feature = "no_battery_monitor"))]
        if let Some(voltage) = BATTERY_LOW_SIGNAL.try_take() {
            log::warn!(
                "NeoPixel: battery low warning ({:.2} V), flashing red until cleared",
                voltage
            );
            let red = RGB8 { r: 32, g: 0, b: 0 };
            let key0_color = RGB8 { r: 32, g: 0, b: 0 }; // red (same as battery)
            let key1_color = RGB8 { r: 0, g: 32, b: 0 }; // green

            let mut last_alert = embassy_time::Instant::now();

            // Keep flashing continuously. Stop only after 90s with no
            // battery low alert from the monitor task.
            loop {
                // Check for new battery low alert (non-blocking)
                if BATTERY_LOW_SIGNAL.try_take().is_some() {
                    last_alert = embassy_time::Instant::now();
                }

                // Check for key flash (non-blocking, overrides red for 200ms)
                if let Some(key) = KEY_FLASH_SIGNAL.try_take() {
                    let color = if key == 0 { key0_color } else { key1_color };
                    ws2812.write_slice(&[color]).await;
                    Timer::after(Duration::from_millis(200)).await;
                }

                ws2812.write_slice(&[red]).await;
                Timer::after(Duration::from_millis(300)).await;
                ws2812.write_slice(&off).await;
                Timer::after(Duration::from_millis(300)).await;

                // Stop if no battery low alert for 90s
                if last_alert.elapsed() > Duration::from_secs(90) {
                    log::info!("NeoPixel: battery low cleared");
                    break;
                }
            }

            // Restore previous state
            ws2812
                .write_slice(&[RGB8 {
                    r: 0,
                    g: 0,
                    b: current_brightness,
                }])
                .await;
            continue;
        }

        // Wait for either an RSSI update, a key flash event, or 1s timeout
        match select(
            with_timeout(Duration::from_secs(1), RSSI_SIGNAL.wait()),
            KEY_FLASH_SIGNAL.wait(),
        )
        .await
        {
            Either::First(Ok(rssi)) => {
                let brightness = rssi_to_brightness(rssi);
                if brightness != current_brightness {
                    current_brightness = brightness;
                    ws2812
                        .write_slice(&[RGB8 {
                            r: 0,
                            g: 0,
                            b: brightness,
                        }])
                        .await;
                }
            }
            Either::First(Err(_)) => {
                // No RSSI for 1s — turn off green
                if current_brightness != 0 {
                    current_brightness = 0;
                    ws2812.write_slice(&off).await;
                }
            }
            Either::Second(key) => {
                // Flash red (key 0) or blue (key 1) for 200ms
                let color = match key {
                    0 => RGB8 { r: 32, g: 0, b: 0 },
                    _ => RGB8 { r: 0, g: 32, b: 0 },
                };
                ws2812.write_slice(&[color]).await;
                Timer::after(Duration::from_millis(200)).await;
                // Restore green RSSI display (or off if no recent signal)
                ws2812
                    .write_slice(&[RGB8 {
                        r: 0,
                        g: 0,
                        b: current_brightness,
                    }])
                    .await;
            }
        }
    }
}

/// Task that scans a 1x2 key matrix and reports key events.
/// NeoKey uses COL2ROW diode orientation: drive columns low, read row with pull-up.
#[embassy_executor::task]
async fn keyboard_scanner(_spawner: Spawner, r: Keyboard) {
    let mut row = Flex::new(r.row);
    let mut col0 = Flex::new(r.col0);
    let mut col1 = Flex::new(r.col1);

    let event_sender = EVENT_CHANNEL.sender();
    let radio_sender = RADIO_CHANNEL.sender();
    let mut prev = [false; 2];

    loop {
        Timer::after_millis(10).await;

        // Scan col0: drive col0 low, col1 high-Z, read row with pull-up
        row.set_pull(Pull::Up);
        row.set_as_input();
        col0.set_as_output();
        col0.set_low();
        col1.set_pull(Pull::None);
        col1.set_as_input();
        Timer::after_micros(50).await;
        let pressed0 = !row.is_high();

        // Scan col1: drive col1 low, col0 high-Z, read row with pull-up
        col1.set_as_output();
        col1.set_low();
        col0.set_pull(Pull::None);
        col0.set_as_input();
        Timer::after_micros(50).await;
        let pressed1 = !row.is_high();

        let states = [pressed0, pressed1];
        for (i, &pressed) in states.iter().enumerate() {
            if pressed != prev[i] {
                log::info!("Key {} {}", i, if pressed { "pressed" } else { "released" });
                event_sender.send(Events::KeyPressed(i as u8, pressed)).await;
                radio_sender.send(RadioPacket::KeyEvent(i as u8, pressed)).await;
            }
        }
        prev = states;
    }
}

/// Main task that processes all events and updates system state.
#[embassy_executor::task]
async fn orchestrate(_spawner: Spawner) {
    let receiver = EVENT_CHANNEL.receiver();
    log::info!("Starting up.");

    loop {
        let event = receiver.receive().await;

        {
            let mut state = SYSTEM_STATE.lock().await;

            match event {
                Events::KeyPressed(key, pressed) => match key {
                    0 => state.key0_pressed = pressed,
                    1 => state.key1_pressed = pressed,
                    _ => {}
                },
            }
        }

        STATE_CHANGED.signal(());
    }
}

/// Task that monitors state changes and logs system status.
#[embassy_executor::task]
async fn consumer(_spawner: Spawner) {
    loop {
        STATE_CHANGED.wait().await;
        // State is tracked internally; no logging needed here since
        // keyboard_scanner already logs each key event.
        let _ = SYSTEM_STATE.lock().await;
    }
}

// ─── Battery monitoring (external 100K/100K divider on GPIO27) ───
// Hardware: VBAT ---[100K]--- GPIO27 ---[100K]--- GND
// This gives VBAT/2 at GPIO27.
//
// Checks battery every 60 seconds. If ≤ 3.2V:
// - Signals the local NeoPixel task to flash red
// - Sends a battery-low alert over radio to the receiver
#[cfg(not(feature = "no_battery_monitor"))]
const BATTERY_LOW_THRESHOLD: f32 = 3.2;

#[cfg(not(feature = "no_battery_monitor"))]
const BATTERY_CHECK_INTERVAL: Duration = Duration::from_secs(60);

#[cfg(not(feature = "no_battery_monitor"))]
#[embassy_executor::task]
async fn battery_monitor_task(r: Battery) {
    let mut adc = Adc::new(r.adc, Irqs, AdcConfig::default());
    let mut channel = AdcChannel::new_pin(r.pin, Pull::None);

    log::info!(
        "Battery monitor started on GPIO27 (threshold: {} V)",
        BATTERY_LOW_THRESHOLD
    );

    loop {
        let adc_value = match adc.read(&mut channel).await {
            Ok(v) => v,
            Err(e) => {
                log::warn!("Battery ADC read failed: {:?}", e);
                continue;
            }
        };

        // GPIO27 reads VBAT/2 through the 100K/100K divider.
        // voltage = adc_value * 3.3 * 2.0 / 4096.0
        let voltage = (adc_value as f32) * 3.3 * 2.0 / 4096.0;
        log::info!("Battery: {:.2} V (adc={})", voltage, adc_value);

        if voltage <= BATTERY_LOW_THRESHOLD {
            log::warn!("Battery LOW: {:.2} V ≤ {} V", voltage, BATTERY_LOW_THRESHOLD);
            // Flash local NeoPixel
            BATTERY_LOW_SIGNAL.signal(voltage);
            // Send battery-low alert to receiver over radio
            let radio_sender = RADIO_CHANNEL.sender();
            radio_sender.send(RadioPacket::BatteryLow).await;
        }

        // Wait before next check
        Timer::after(BATTERY_CHECK_INTERVAL).await;
    }
}
