//! RFM69 radio receiver + USB HID keyboard
//!
//! Receives key events from pi_test_1 over RFM69 radio and outputs them
//! as USB HID keyboard reports. Logging via USB CDC serial (composite device).

#![no_std]
#![no_main]

use assign_resources::assign_resources;
use defmt::Format;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::peripherals::{self, PIO0, SPI1, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Channel as SyncChannel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Timer, with_timeout};
use embassy_usb::class::cdc_acm::State as CdcState;
use embassy_usb::class::hid::{
    HidBootProtocol, HidReaderWriter, HidSubclass, State as HidState,
};
use embassy_usb::{Builder, Config as UsbConfig, UsbDevice};
use rfm69_async::{Address, MacTiming, Packet, Rfm69, Runner, Stack, StackResources, Transceiver, TrxError, config};
use smart_leds::RGB8;
use static_cell::StaticCell;
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};
use {defmt_rtt as _, panic_probe as _};

// ─── Configurable keycodes ──────────────────────────────────────────
// USB HID keycodes. See the USB HID Usage Tables specification.
// Left Arrow = 0x50, Right Arrow = 0x4F
// Change this array to map received key indices to different keycodes.
const KEY_MAP: [u8; 2] = [0x50, 0x4F]; // [Left Arrow, Right Arrow]

// ─── Hardware resource assignment ───────────────────────────────────
assign_resources! {
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

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<peripherals::DMA_CH0>, dma::InterruptHandler<peripherals::DMA_CH1>, dma::InterruptHandler<peripherals::DMA_CH2>;
    PIO0_IRQ_0 => PioInterruptHandler<peripherals::PIO0>;
});

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

// ─── Radio type aliases ─────────────────────────────────────────────
type RadioSpi = SpiDevice<'static, NoopRawMutex, Spi<'static, SPI1, embassy_rp::spi::Async>, Output<'static>>;
type RadioDriver = Rfm69<RadioSpi, Output<'static>, embassy_rp::gpio::Input<'static>, Delay>;

/// Radio wrapper that remembers config for recovery on link-down
struct RecoveringRadio {
    rfm: RadioDriver,
    network_id: u8,
    frequency: u32,
}

impl Transceiver for RecoveringRadio {
    async fn send(&mut self, packet: &Packet) -> Result<(), TrxError> {
        self.rfm.send(packet).await.map_err(Into::into)
    }

    async fn recv(&mut self) -> Result<Packet, TrxError> {
        self.rfm.recv().await.map_err(Into::into)
    }

    async fn recover(&mut self) -> Result<(), TrxError> {
        log::warn!("RFM69 recovering from link down");
        config::my_defaults(&mut self.rfm, self.network_id, self.frequency)
            .await
            .map_err(Into::into)
    }
}

// ─── USB type aliases ───────────────────────────────────────────────
type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;
type MyHidWriter = embassy_usb::class::hid::HidWriter<'static, MyUsbDriver, 8>;

// ─── NeoPixel type alias ────────────────────────────────────────────
type NeoPixel = PioWs2812<'static, PIO0, 0, Grb>;

// ─── Events & State ─────────────────────────────────────────────────
enum Events {
    RadioReceived(u8, bool), // key index, pressed
}

/// The central state of our system, shared between tasks.
#[derive(Clone, Format)]
struct State {
    key0_pressed: bool,
    key1_pressed: bool,
    packets_received: u32,
}

impl State {
    const fn new() -> Self {
        Self {
            key0_pressed: false,
            key1_pressed: false,
            packets_received: 0,
        }
    }
}

/// Shared state
static SYSTEM_STATE: Mutex<CriticalSectionRawMutex, State> = Mutex::new(State::new());

/// Channel for events from worker tasks to the orchestrator
static EVENT_CHANNEL: SyncChannel<CriticalSectionRawMutex, Events, 16> = SyncChannel::new();

/// Channel for HID key taps (key index to tap)
static HID_CHANNEL: SyncChannel<CriticalSectionRawMutex, u8, 16> = SyncChannel::new();

/// Signal for notifying about state changes
static STATE_CHANGED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signal for the latest RSSI reading (dBm) from received radio packets
static RSSI_SIGNAL: Signal<CriticalSectionRawMutex, i16> = Signal::new();

// ─── USB device task ────────────────────────────────────────────────
#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}

// ─── Radio runner task ──────────────────────────────────────────────
#[embassy_executor::task]
async fn radio_runner_task(mut runner: Runner<'static, RecoveringRadio>) {
    runner.run().await
}

// ─── Radio RX task ──────────────────────────────────────────────────
/// Listens for incoming radio packets and forwards key events to HID + orchestrator
#[embassy_executor::task]
async fn radio_rx_task(stack: Stack<'static>) {
    let event_sender = EVENT_CHANNEL.sender();
    let hid_sender = HID_CHANNEL.sender();

    loop {
        let packet = stack.recv().await;
        if packet.data.len() >= 2 {
            let key_index = packet.data[0];
            let pressed = packet.data[1] != 0;
            log::info!(
                "Radio RX: key={} pressed={} rssi={:?}",
                key_index,
                pressed,
                packet.rssi
            );
            event_sender.send(Events::RadioReceived(key_index, pressed)).await;
            // Signal RSSI for the NeoPixel display
            if let Some(rssi) = packet.rssi {
                RSSI_SIGNAL.signal(rssi);
            }
            // Only tap on press — release events are ignored for HID output.
            // This makes each keypress self-contained: down + up in one shot,
            // so a lost release packet never leaves a key stuck down.
            if pressed {
                hid_sender.send(key_index).await;
            }
        }
    }
}

// ─── HID keyboard task ──────────────────────────────────────────────
/// Reads key-press events from the channel and sends USB HID keyboard
/// reports as self-contained taps: keydown, brief hold, then keyup.
/// Release events are ignored — each press is a complete down+up cycle
/// so a lost radio packet can never leave a key stuck down.
#[embassy_executor::task]
async fn hid_keyboard_task(mut writer: MyHidWriter) {
    let receiver = HID_CHANNEL.receiver();

    // Duration the key is held before release. 30ms is enough for the OS
    // to register a single keypress without auto-repeat kicking in.
    const TAP_HOLD: Duration = Duration::from_millis(30);

    log::info!("HID keyboard task started, keymap: {:?}", KEY_MAP);

    loop {
        let key_index = receiver.receive().await;

        if (key_index as usize) >= KEY_MAP.len() {
            log::warn!("HID: key index {} out of range", key_index);
            continue;
        }
        let keycode = KEY_MAP[key_index as usize];

        // Keydown report
        let report_down = KeyboardReport {
            keycodes: [keycode, 0, 0, 0, 0, 0],
            leds: 0,
            modifier: 0,
            reserved: 0,
        };
        if let Err(e) = writer.write_serialize(&report_down).await {
            log::warn!("HID keydown failed: {:?}", e);
            continue;
        }

        Timer::after(TAP_HOLD).await;

        // Keyup report
        let report_up = KeyboardReport {
            keycodes: [0, 0, 0, 0, 0, 0],
            leds: 0,
            modifier: 0,
            reserved: 0,
        };
        if let Err(e) = writer.write_serialize(&report_up).await {
            log::warn!("HID keyup failed: {:?}", e);
            continue;
        }

        log::info!("HID tap: key={} keycode=0x{:02x}", key_index, keycode);
    }
}

// ─── NeoPixel RSSI display task ─────────────────────────────────────
/// Maps an RSSI value (in dBm) to a green brightness (0–255).
/// RFM69 RSSI typically ranges from ~-120 (very weak) to ~-30 (very strong).
/// We map [-100, -40] dBm → [1, 255], clamping outside that range.
fn rssi_to_brightness(rssi: i16) -> u8 {
    let scaled = ((rssi as i32 + 100) * 255 / 60).max(1).min(255);
    scaled as u8
}

/// Drives the built-in NeoPixel (GPIO4) to show RF signal strength as
/// green brightness. Updates on each received packet; fades to off if
/// no packets arrive within 3 seconds.
#[embassy_executor::task]
async fn neopixel_task(ws2812: &'static mut NeoPixel) {
    log::info!("NeoPixel RSSI display task started");

    // Default dim green when no signal has been received yet
    const IDLE_BRIGHTNESS: u8 = 8;
    let mut current_brightness = IDLE_BRIGHTNESS;

    // Start with the LED on (dim)
    ws2812.write_slice(&[RGB8 { r: 0, g: IDLE_BRIGHTNESS, b: 0 }]).await;

    loop {
        // Wait up to 2s for a new RSSI reading
        match with_timeout(Duration::from_secs(2), RSSI_SIGNAL.wait()).await {
            Ok(rssi) => {
                let brightness = rssi_to_brightness(rssi);
                if brightness != current_brightness {
                    current_brightness = brightness;
                    log::info!("NeoPixel: rssi={} dBm → green={}", rssi, brightness);
                    ws2812.write_slice(&[RGB8 { r: 0, g: brightness, b: 0 }]).await;
                }
            }
            Err(_) => {
                // No packet in 2s — fade back to idle brightness
                if current_brightness != IDLE_BRIGHTNESS {
                    current_brightness = IDLE_BRIGHTNESS;
                    log::info!("NeoPixel: no signal, fading to idle green={}", IDLE_BRIGHTNESS);
                    ws2812.write_slice(&[RGB8 { r: 0, g: IDLE_BRIGHTNESS, b: 0 }]).await;
                }
            }
        }
    }
}

// ─── Orchestrator task ──────────────────────────────────────────────
#[embassy_executor::task]
async fn orchestrate(_spawner: Spawner) {
    let receiver = EVENT_CHANNEL.receiver();
    log::info!("Orchestrator started.");

    loop {
        let event = receiver.receive().await;

        {
            let mut state = SYSTEM_STATE.lock().await;

            match event {
                Events::RadioReceived(key, pressed) => {
                    state.packets_received += 1;
                    match key {
                        0 => state.key0_pressed = pressed,
                        1 => state.key1_pressed = pressed,
                        _ => {}
                    }
                    log::info!(
                        "Key {} {} (total rx: {})",
                        key,
                        if pressed { "pressed" } else { "released" },
                        state.packets_received
                    );
                }
            }
        }

        STATE_CHANGED.signal(());
    }
}

// ─── Consumer task (logs state changes) ─────────────────────────────
#[embassy_executor::task]
async fn consumer(_spawner: Spawner) {
    loop {
        STATE_CHANGED.wait().await;

        let state = SYSTEM_STATE.lock().await;
        log::info!(
            "State: keys={} {} | rx={}",
            state.key0_pressed,
            state.key1_pressed,
            state.packets_received
        );
    }
}

// ─── Main ───────────────────────────────────────────────────────────
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let r = split_resources! {p};

    // ── USB composite device: CDC (logger) + HID (keyboard) ──
    let driver = Driver::new(p.USB, UsbIrqs);

    let mut usb_config = UsbConfig::new(0xc0de, 0xcafe);
    usb_config.manufacturer = Some("Embassy");
    usb_config.product = Some("RFM69 HID Receiver");
    usb_config.serial_number = Some("pi_test_2");
    usb_config.max_power = 100;
    usb_config.max_packet_size_0 = 64;
    usb_config.composite_with_iads = true;

    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        usb_config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        MSOS_DESCRIPTOR.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    // CDC ACM class for logging
    static CDC_STATE: StaticCell<CdcState> = StaticCell::new();
    let cdc_class = embassy_usb::class::cdc_acm::CdcAcmClass::new(
        &mut builder,
        CDC_STATE.init(CdcState::new()),
        64,
    );

    // HID keyboard class
    static HID_STATE: StaticCell<HidState> = StaticCell::new();
    let hid_config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
        hid_subclass: HidSubclass::Boot,
        hid_boot_protocol: HidBootProtocol::Keyboard,
    };
    let hid = HidReaderWriter::<_, 1, 8>::new(
        &mut builder,
        HID_STATE.init(HidState::new()),
        hid_config,
    );
    let (reader, writer) = hid.split();

    // Build the USB device
    let usb = builder.build();

    // Logger future from the CDC class
    let log_fut = embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, cdc_class);

    // Give USB time to enumerate
    Timer::after_millis(500).await;

    // ── RFM69 Radio Setup ──
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 8_000_000;
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
    let rst_pin = Output::new(r.radio.reset, Level::Low);
    Timer::after_millis(10).await;

    log::info!("RFM69 radio: SPI1 sck=14 mosi=15 miso=8 cs=16 rst=17 dio0=21");

    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, Spi<'static, SPI1, embassy_rp::spi::Async>>> =
        StaticCell::new();
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

    let trx = RecoveringRadio { rfm, network_id, frequency };
    let own_address = Address::Unicast(2); // Receiver address (sender is 1)
    static RESOURCES: StaticCell<StackResources> = StaticCell::new();
    let resources = RESOURCES.init(StackResources::new());
    let (stack, runner) = Stack::new(trx, own_address, resources, MacTiming::default());

    log::info!(
        "RFM69 radio ready, address {:?} freq {} MHz",
        own_address,
        frequency / 1_000_000
    );

    // ── NeoPixel Setup (GPIO4) ──
    // Built-in NeoPixel on the Feather RP2040 RFM69
    // NOTE: PIO Common and the loaded program must stay alive for the
    // lifetime of the state machine — if the program is dropped it gets
    // unloaded from PIO instruction memory and the SM runs garbage.
    // We keep them in static cells so they live forever.
    static PIO_COMMON: StaticCell<embassy_rp::pio::Common<'static, PIO0>> = StaticCell::new();
    static WS2812_PROGRAM: StaticCell<PioWs2812Program<'static, PIO0>> = StaticCell::new();
    static WS2812: StaticCell<NeoPixel> = StaticCell::new();

    let Pio { common, sm0, .. } = Pio::new(p.PIO0, Irqs);
    let common = PIO_COMMON.init(common);
    let ws2812_program = WS2812_PROGRAM.init(PioWs2812Program::new(common));
    let ws2812 = WS2812.init(PioWs2812::new(
        common,
        sm0,
        p.DMA_CH2,
        Irqs,
        p.PIN_4,
        ws2812_program,
    ));

    log::info!("NeoPixel initialized on GPIO4");

    // ── Spawn tasks ──
    spawner.spawn(usb_task(usb).unwrap());
    spawner.spawn(radio_runner_task(runner).unwrap());
    spawner.spawn(radio_rx_task(stack).unwrap());
    spawner.spawn(hid_keyboard_task(writer).unwrap());
    spawner.spawn(orchestrate(spawner).unwrap());
    spawner.spawn(consumer(spawner).unwrap());
    spawner.spawn(neopixel_task(ws2812).unwrap());

    log::info!("All tasks spawned. Receiver ready.");

    // Drop the HID reader — we only need the writer (IN endpoint) for
    // sending keyboard reports. The reader (OUT endpoint) handles host→
    // device reports (e.g. caps-lock LEDs) which we don't need.
    drop(reader);

    // Run the USB logger future (never returns)
    log_fut.await;
}
