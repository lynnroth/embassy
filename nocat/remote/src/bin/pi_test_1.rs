//! Key matrix scanner + RFM69 radio transmitter
//!
//! Scans a 1x2 NeoKey matrix and broadcasts key events over RFM69 radio
//! to another board, while also logging locally via USB serial.

#![no_std]
#![no_main]

use assign_resources::assign_resources;
use defmt::Format;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::adc::{Adc, Channel as AdcChannel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler};
use embassy_rp::bind_interrupts;
use embassy_rp::bootsel::is_bootsel_pressed;
use embassy_rp::dma;
use embassy_rp::gpio::{Flex, Level, Output, Pull};
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
use rfm69_async::{
    Address, Flags, MacTiming, Packet, Rfm69, Runner, Stack, StackResources, Transceiver, TrxError, config,
};
use smart_leds::RGB8;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// Hardware resource assignment
assign_resources! {
    vsys: Vsys {
        adc: ADC,
        pin_29: PIN_29,
    },
    keyboard: Keyboard {
        // Feather header D11/D12/D13 — free, not used by RFM69
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

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

/// Events that worker tasks send to the orchestrator
enum Events {
    VsysVoltage(f32),     // New voltage reading
    KeyPressed(u8, bool), // Key index, pressed state
}

/// Radio-bound packets
enum RadioPacket {
    KeyEvent(u8, bool),
}

/// The central state of our system, shared between tasks.
#[derive(Clone, Format)]
struct State {
    vsys_voltage: f32,
    key0_pressed: bool,
    key1_pressed: bool,
}

#[derive(Debug, Format)]
struct SystemStatus {
    voltage: f32,
}

impl State {
    const fn new() -> Self {
        Self {
            vsys_voltage: 0.0,
            key0_pressed: false,
            key1_pressed: false,
        }
    }

    fn get_system_summary(&self) -> SystemStatus {
        SystemStatus {
            voltage: self.vsys_voltage,
        }
    }
}

/// Shared state
static SYSTEM_STATE: Mutex<CriticalSectionRawMutex, State> = Mutex::new(State::new());

/// Channel for events from worker tasks to the orchestrator
static EVENT_CHANNEL: SyncChannel<CriticalSectionRawMutex, Events, 10> = SyncChannel::new();

/// Channel for radio-bound packets
static RADIO_CHANNEL: SyncChannel<CriticalSectionRawMutex, RadioPacket, 10> = SyncChannel::new();

/// Signal for notifying about state changes
static STATE_CHANGED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signal for the latest RSSI reading (dBm) from received radio packets
static RSSI_SIGNAL: Signal<CriticalSectionRawMutex, i16> = Signal::new();

/// Signal for key flash events (key index 0=red, 1=blue)
static KEY_FLASH_SIGNAL: Signal<CriticalSectionRawMutex, u8> = Signal::new();

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

    let trx = RecoveringRadio {
        rfm,
        network_id,
        frequency,
    };
    let own_address = Address::Unicast(1);
    static RESOURCES: StaticCell<StackResources> = StaticCell::new();
    let resources = RESOURCES.init(StackResources::new());
    let (stack, runner) = Stack::new(trx, own_address, resources, MacTiming::default());

    log::info!(
        "RFM69 radio ready, address {:?} freq {} MHz",
        own_address,
        frequency / 1_000_000
    );

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

    // Spawn orchestrator tasks
    spawner.spawn(orchestrate(spawner).unwrap());
    spawner.spawn(vsys_voltage(spawner, r.vsys).unwrap());
    spawner.spawn(consumer(spawner).unwrap());
    spawner.spawn(bootsel_button(p.BOOTSEL).unwrap());
    spawner.spawn(keyboard_scanner(spawner, r.keyboard).unwrap());

    // Spawn radio tasks
    spawner.spawn(radio_runner_task(runner).unwrap());
    spawner.spawn(radio_tx_task(stack).unwrap());
    spawner.spawn(radio_rx_task(stack).unwrap());
    spawner.spawn(neopixel_task(ws2812).unwrap());
}

/// Drives the RFM69 radio runner (always-listens, arbitrates TX/RX)
#[embassy_executor::task]
async fn radio_runner_task(mut runner: Runner<'static, RecoveringRadio>) {
    runner.run().await;
}

/// Listens for key events and broadcasts them over RFM69
#[embassy_executor::task]
async fn radio_tx_task(stack: Stack<'static>) {
    let receiver = RADIO_CHANNEL.receiver();
    loop {
        let packet = receiver.receive().await;
        let payload = match packet {
            RadioPacket::KeyEvent(key, pressed) => [key, if pressed { 1 } else { 0 }],
        };
        match stack.send(Address::Broadcast, Flags::None, &payload).await {
            Ok(()) => log::info!("Radio TX: key={} pressed={}", payload[0], payload[1] != 0),
            Err(e) => log::warn!("Radio TX failed: {:?}", e),
        }
    }
}

/// Listens for RSSI reply packets from the receiver and signals the NeoPixel task.
/// Reply format: [rssi_byte, key_index] where key_index is 0/1 on press, 0xFF on release.
#[embassy_executor::task]
async fn radio_rx_task(stack: Stack<'static>) {
    loop {
        let packet = stack.recv().await;
        if packet.data.len() >= 2 {
            let rssi = packet.data[0] as i8 as i16;
            let key_reply = packet.data[1];
            log::info!("Radio RX: RSSI reply = {} dBm, key={}", rssi, key_reply);
            RSSI_SIGNAL.signal(rssi);
            // Flash red/blue on key press (0 or 1), skip 0xFF (release)
            if key_reply <= 1 {
                KEY_FLASH_SIGNAL.signal(key_reply);
            }
        } else if packet.data.len() >= 1 {
            // Backward compat: 1-byte reply (RSSI only)
            let rssi = packet.data[0] as i8 as i16;
            log::info!("Radio RX: RSSI reply = {} dBm", rssi);
            RSSI_SIGNAL.signal(rssi);
        }
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
    log::info!("NeoPixel RSSI display task started");

    const IDLE_BRIGHTNESS: u8 = 0; // LED off when no signal (battery saving)
    let mut current_brightness = IDLE_BRIGHTNESS;
    let off = [RGB8 { r: 0, g: 0, b: 0 }];
    ws2812.write_slice(&off).await;

    loop {
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
                    log::info!("NeoPixel: rssi={} dBm → green={}", rssi, brightness);
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
                    log::info!("NeoPixel: no signal, LED off");
                    ws2812.write_slice(&off).await;
                }
            }
            Either::Second(key) => {
                // Flash red (key 0) or blue (key 1) for 200ms
                let color = match key {
                    0 => RGB8 { r: 32, g: 0, b: 0 },
                    _ => RGB8 { r: 0, g: 32, b: 0 },
                };
                log::info!("NeoPixel: key {} flash", key);
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

/// Task that monitors BOOTSEL button and reports via USB serial.
#[embassy_executor::task]
async fn bootsel_button(mut bootsel: Peri<'static, peripherals::BOOTSEL>) {
    let mut previous = false;
    loop {
        Timer::after_micros(10).await;
        let pressed = is_bootsel_pressed(bootsel.reborrow());
        if pressed != previous {
            log::info!("bootsel is now {}", pressed);
        }
        previous = pressed;
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
                Events::VsysVoltage(voltage) => {
                    state.vsys_voltage = voltage;
                    log::info!("Vsys voltage: {}", voltage);
                }
                Events::KeyPressed(key, pressed) => {
                    log::info!("Key {} {}", key, if pressed { "pressed" } else { "released" });
                    match key {
                        0 => state.key0_pressed = pressed,
                        1 => state.key1_pressed = pressed,
                        _ => {}
                    }
                }
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

        let state = SYSTEM_STATE.lock().await;
        log::info!(
            "State update - {:?} | Keys: {} {}",
            state.get_system_summary(),
            state.key0_pressed,
            state.key1_pressed
        );
    }
}

/// Task that reads system voltage through ADC.
#[embassy_executor::task]
pub async fn vsys_voltage(_spawner: Spawner, r: Vsys) {
    let mut adc = Adc::new(r.adc, Irqs, AdcConfig::default());
    let vsys_in = r.pin_29;
    let mut channel = AdcChannel::new_pin(vsys_in, Pull::None);
    let sender = EVENT_CHANNEL.sender();

    loop {
        Timer::after(Duration::from_secs(30)).await;
        let adc_value = adc.read(&mut channel).await.unwrap();
        let voltage = (adc_value as f32) * 3.3 * 3.0 / 4096.0;
        sender.send(Events::VsysVoltage(voltage)).await;
    }
}
