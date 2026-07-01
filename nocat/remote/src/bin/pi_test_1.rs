//! This example demonstrates orchestration between tasks, plus USB serial logging
//! and BOOTSEL button monitoring.
//!
//! It combines the orchestrate_tasks example with USB serial output and bootsel
//! button checking from button_bootsel.

#![no_std]
#![no_main]

use assign_resources::assign_resources;
use defmt::Format;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler as AdcInterruptHandler};
use embassy_rp::bind_interrupts;
use embassy_rp::bootsel::is_bootsel_pressed;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Flex, Pull};
use embassy_rp::peripherals::{self, USB};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::{channel, signal};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

// Hardware resource assignment. See other examples for different ways of doing this.
assign_resources! {
    vsys: Vsys {
        adc: ADC,
        pin_29: PIN_29,
    },
    keyboard: Keyboard {
        row: PIN_11,
        col0: PIN_12,
        col1: PIN_13,
    },
}

// Interrupt binding - required for hardware peripherals like ADC
bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
});

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

/// Events that worker tasks send to the orchestrator
enum Events {
    VsysVoltage(f32),      // New voltage reading
    FirstRandomSeed(u32),  // Random number from 30s timer
    SecondRandomSeed(u32), // Random number from 60s timer
    ThirdRandomSeed(u32),  // Random number from 90s timer
    ResetFirstRandomSeed,  // Signal to reset the first counter
    KeyPressed(u8, bool),  // Key index, pressed state
}

/// Commands that can control task behavior.
/// Currently only used to stop tasks, but could be extended for other controls.
enum Commands {
    /// Signals a task to stop execution
    Stop,
}

/// The central state of our system, shared between tasks.
#[derive(Clone, Format)]
struct State {
    vsys_voltage: f32,
    first_random_seed: u32,
    second_random_seed: u32,
    third_random_seed: u32,
    first_random_seed_task_running: bool,
    times_we_got_first_random_seed: u8,
    maximum_times_we_want_first_random_seed: u8,
    key0_pressed: bool,
    key1_pressed: bool,
}

/// A formatted view of the system status, used for logging. Used for the below `get_system_summary` fn.
#[derive(Debug, Format)]
struct SystemStatus {
    voltage: f32,
}

impl State {
    const fn new() -> Self {
        Self {
            vsys_voltage: 0.0,
            first_random_seed: 0,
            second_random_seed: 0,
            third_random_seed: 0,
            first_random_seed_task_running: false,
            times_we_got_first_random_seed: 0,
            maximum_times_we_want_first_random_seed: 3,
            key0_pressed: false,
            key1_pressed: false,
        }
    }

    /// Returns a formatted summary of power state and voltage.
    /// Shows how to create methods that work with shared state.
    fn get_system_summary(&self) -> SystemStatus {
        SystemStatus {
            voltage: self.vsys_voltage,
        }
    }
}

/// The shared state protected by a mutex
static SYSTEM_STATE: Mutex<CriticalSectionRawMutex, State> = Mutex::new(State::new());

/// Channel for events from worker tasks to the orchestrator
static EVENT_CHANNEL: channel::Channel<CriticalSectionRawMutex, Events, 10> = channel::Channel::new();

/// Signal used to stop the first random number task
static STOP_FIRST_RANDOM_SIGNAL: signal::Signal<CriticalSectionRawMutex, Commands> = signal::Signal::new();

/// Signal for notifying about state changes
static STATE_CHANGED: signal::Signal<CriticalSectionRawMutex, ()> = signal::Signal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let r = split_resources! {p};

    // Spawn USB logger task
    let driver = Driver::new(p.USB, UsbIrqs);
    spawner.spawn(logger_task(driver).unwrap());

    // Spawn orchestrator tasks
    spawner.spawn(orchestrate(spawner).unwrap());
    spawner.spawn(random_60s(spawner).unwrap());
    spawner.spawn(random_90s(spawner).unwrap());
    // `random_30s` is not spawned here, but in the orchestrate task depending on state
    spawner.spawn(vsys_voltage(spawner, r.vsys).unwrap());
    spawner.spawn(consumer(spawner).unwrap());

    // Spawn BOOTSEL button monitor
    spawner.spawn(bootsel_button(p.BOOTSEL).unwrap());

    // Spawn keyboard matrix scanner
    spawner.spawn(keyboard_scanner(spawner, r.keyboard).unwrap());
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

    let sender = EVENT_CHANNEL.sender();
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
                sender.send(Events::KeyPressed(i as u8, pressed)).await;
            }
        }
        prev = states;
    }
}

/// Main task that processes all events and updates system state.
#[embassy_executor::task]
async fn orchestrate(spawner: Spawner) {
    let receiver = EVENT_CHANNEL.receiver();
    log::info!("Starting up.");

    loop {
        // Do nothing until we receive any event
        let event = receiver.receive().await;

        // Scope in which we want to lock the system state. As an alternative we could also call `drop` on the state
        {
            let mut state = SYSTEM_STATE.lock().await;

            match event {
                Events::VsysVoltage(voltage) => {
                    state.vsys_voltage = voltage;
                    log::info!("Vsys voltage: {}", voltage);
                }
                Events::FirstRandomSeed(seed) => {
                    state.first_random_seed = seed;
                    state.times_we_got_first_random_seed += 1;
                    log::info!(
                        "First random seed: {}, and that was iteration {} of receiving this.",
                        seed,
                        &state.times_we_got_first_random_seed
                    );
                }
                Events::SecondRandomSeed(seed) => {
                    state.second_random_seed = seed;
                    log::info!("Second random seed: {}", seed);
                }
                Events::ThirdRandomSeed(seed) => {
                    state.third_random_seed = seed;
                    log::info!("Third random seed: {}", seed);
                }
                Events::ResetFirstRandomSeed => {
                    state.times_we_got_first_random_seed = 0;
                    state.first_random_seed = 0;
                    log::info!("Resetting the first random seed counter");
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

            // Handle task orchestration based on state
            // Just placed as an example here, could be hooked into the event system, put on a timer, ...
            match state.times_we_got_first_random_seed {
                max if max == state.maximum_times_we_want_first_random_seed => {
                    log::info!("Stopping the first random signal task");
                    STOP_FIRST_RANDOM_SIGNAL.signal(Commands::Stop);
                    EVENT_CHANNEL.sender().send(Events::ResetFirstRandomSeed).await;
                }
                0 => {
                    let respawn_first_random_seed_task = !state.first_random_seed_task_running;
                    // Deliberately dropping the Mutex lock here to release it before a lengthy operation
                    drop(state);
                    if respawn_first_random_seed_task {
                        log::info!("(Re)-Starting the first random signal task");
                        spawner.spawn(random_30s(spawner).unwrap());
                    }
                }
                _ => {}
            }
        }

        STATE_CHANGED.signal(());
    }
}

/// Task that monitors state changes and logs system status.
#[embassy_executor::task]
async fn consumer(_spawner: Spawner) {
    loop {
        // Wait for state change notification
        STATE_CHANGED.wait().await;

        let state = SYSTEM_STATE.lock().await;
        log::info!(
            "State update - {:?} | Seeds - First: {} (count: {}/{}, running: {}), Second: {}, Third: {} | Keys: {} {}",
            state.get_system_summary(),
            state.first_random_seed,
            state.times_we_got_first_random_seed,
            state.maximum_times_we_want_first_random_seed,
            state.first_random_seed_task_running,
            state.second_random_seed,
            state.third_random_seed,
            state.key0_pressed,
            state.key1_pressed
        );
    }
}

/// Task that generates random numbers every 30 seconds until stopped.
/// Shows how to handle both timer events and stop signals.
/// As an example of some routine we want to be on or off depending on other needs.
#[embassy_executor::task]
async fn random_30s(_spawner: Spawner) {
    {
        let mut state = SYSTEM_STATE.lock().await;
        state.first_random_seed_task_running = true;
    }

    let mut rng = RoscRng;
    let sender = EVENT_CHANNEL.sender();

    loop {
        // Wait for either 30s timer or stop signal (like select() in Go)
        match select(Timer::after(Duration::from_secs(30)), STOP_FIRST_RANDOM_SIGNAL.wait()).await {
            Either::First(_) => {
                log::info!("30s are up, generating random number");
                let random_number = rng.next_u32();
                sender.send(Events::FirstRandomSeed(random_number)).await;
            }
            Either::Second(_) => {
                log::info!("Received signal to stop, goodbye!");

                let mut state = SYSTEM_STATE.lock().await;
                state.first_random_seed_task_running = false;

                break;
            }
        }
    }
}

/// Task that generates random numbers every 60 seconds. As an example of some routine.
#[embassy_executor::task]
async fn random_60s(_spawner: Spawner) {
    let mut rng = RoscRng;
    let sender = EVENT_CHANNEL.sender();

    loop {
        Timer::after(Duration::from_secs(60)).await;
        let random_number = rng.next_u32();
        sender.send(Events::SecondRandomSeed(random_number)).await;
    }
}

/// Task that generates random numbers every 90 seconds. . As an example of some routine.
#[embassy_executor::task]
async fn random_90s(_spawner: Spawner) {
    let mut rng = RoscRng;
    let sender = EVENT_CHANNEL.sender();

    loop {
        Timer::after(Duration::from_secs(90)).await;
        let random_number = rng.next_u32();
        sender.send(Events::ThirdRandomSeed(random_number)).await;
    }
}

/// Task that reads system voltage through ADC. As an example of some continuous sensor reading.
#[embassy_executor::task]
pub async fn vsys_voltage(_spawner: Spawner, r: Vsys) {
    let mut adc = Adc::new(r.adc, Irqs, Config::default());
    let vsys_in = r.pin_29;
    let mut channel = Channel::new_pin(vsys_in, Pull::None);
    let sender = EVENT_CHANNEL.sender();

    loop {
        Timer::after(Duration::from_secs(30)).await;
        let adc_value = adc.read(&mut channel).await.unwrap();
        let voltage = (adc_value as f32) * 3.3 * 3.0 / 4096.0;
        sender.send(Events::VsysVoltage(voltage)).await;
    }
}
