// This example shows how to read from 2 sensors from a single I2C bus
//
// Core 0 does measurements and communicates via a CHANNEL to Core 1
// Core 1 does display/LED I/O
// Each core has its own I2C bus so they never interfere

// Uses the following I/O devices
// SSD1306 OLED on I2C0 (GPIO4/5)
// BH1750 Light Sensor on I2C1 (GPIO2/3)
// QMP6988 Pressure/Temperature Sensor on I2C1
// LED on Pin_26

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;
use core::u8;

use bh1750::{Resolution, BH1750};
use defmt::*;
use embassy_executor::Executor;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{Async, Config, I2c, InterruptHandler};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::{I2C0, I2C1};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Instant, Timer};
use embedded_hal_bus::i2c;
use embedded_qmp6988::{IirFilter, OverSamplingSetting, Qmp6988, DEFAULT_I2C_ADDRESS};
use ssd1306::mode::DisplayConfig;
use ssd1306::prelude::DisplayRotation;
use ssd1306::size::DisplaySize128x64;
use ssd1306::{I2CDisplayInterface, Ssd1306};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static CHANNEL: Channel<CriticalSectionRawMutex, DisplayMessage, 1> = Channel::new();

enum DisplayMessage {
    LedOn,
    LedOff,
    Pressure(u32),
    Temperature(f32),
    Light(f32),
    Time(u64),
}

bind_interrupts!(struct Irqs {
    I2C0_IRQ => InterruptHandler<I2C0>;
    I2C1_IRQ => InterruptHandler<I2C1>;
});

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    let led = Output::new(p.PIN_26, Level::Low);

    // Set up I2C0 for the SSD1306 OLED Display
    let i2c0 = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, Config::default());

    // Set up I2C1 for the sensors on a shared I2C bus
    let i2c1 = I2c::new_async(p.I2C1, p.PIN_3, p.PIN_2, Irqs, Config::default());

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| unwrap!(spawner.spawn(core1_task(led, i2c0))));
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());

    // Direct I2C
    executor0.run(|spawner: embassy_executor::Spawner| unwrap!(spawner.spawn(core0_task(i2c1))));
}


// Core0 reads sensors and sends the data via CHANNEL

#[embassy_executor::task]
async fn core0_task(i2c: embassy_rp::i2c::I2c<'static, I2C1, Async>) {
    info!("Hello from core 0");

    let i2c_ref_cell = RefCell::new(i2c);

    let mut qmp = Qmp6988::new(
        i2c::RefCellDevice::new(&i2c_ref_cell),
        DEFAULT_I2C_ADDRESS,
        embassy_time::Delay {},
    )
    .unwrap();
    qmp.set_filter(IirFilter::Off).unwrap();
    qmp.set_oversampling_setting(OverSamplingSetting::HighAccuracy)
        .unwrap();

    let mut bh1750 = BH1750::new(
        i2c::RefCellDevice::new(&i2c_ref_cell),
        embassy_time::Delay {},
        false,
    );

    loop {
        // Turn LED on to show we measure
        CHANNEL.send(DisplayMessage::LedOn).await;

        let qmp_measurement = qmp.measure().unwrap();
        let lux = bh1750.get_one_time_measurement(Resolution::High).unwrap();

        CHANNEL.send(DisplayMessage::Light(lux)).await;
        CHANNEL
            .send(DisplayMessage::Pressure(qmp_measurement.pressure as u32))
            .await;
        CHANNEL
            .send(DisplayMessage::Temperature(qmp_measurement.temperature))
            .await;

        let now = Instant::now();
        CHANNEL.send(DisplayMessage::Time(now.as_millis())).await;

        CHANNEL.send(DisplayMessage::LedOff).await;

        Timer::after_millis(1000).await;
    }
}


// This task on core 1 does all I/O and it depend on messages via CHANNEL

const ROW_TEMP: u8 = 2;
const ROW_PRESSURE: u8 = 3;
const ROW_LUX: u8 = 4;
const ROW_TIME: u8 = 5;
const ROW_COUNTER: u8 = 6;

const COL_DATA: u8 = 9;

#[embassy_executor::task]
async fn core1_task(mut led: Output<'static>, i2c: embassy_rp::i2c::I2c<'static, I2C0, Async>) {
    info!("Hello from core 1");

    let interface = I2CDisplayInterface::new(i2c);
    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_terminal_mode();

    let mut counter = 0;

    display.init().unwrap();
    let mut buffer = itoa::Buffer::new();

    display.clear().unwrap();
    let _ = display.write_str("I2C Sensors");
    display.set_position(0, ROW_TEMP).unwrap();
    let _ = display.write_str("Temp:");
    display.set_position(0, ROW_PRESSURE).unwrap();
    let _ = display.write_str("Pressure:");
    display.set_position(0, ROW_LUX).unwrap();
    let _ = display.write_str("Lux:");
    display.set_position(0, ROW_TIME).unwrap();
    let _ = display.write_str("Time:");
    display.set_position(0, ROW_COUNTER).unwrap();
    let _ = display.write_str("Counter:");

    loop {
        match CHANNEL.receive().await {
            DisplayMessage::LedOn => led.set_high(),
            DisplayMessage::LedOff => {
                led.set_low();
                let s: &str = buffer.format(counter);
                display.set_position(COL_DATA, ROW_COUNTER).unwrap();
                let _ = display.write_str(s);
                counter += 1;
            },
            DisplayMessage::Time(t) => {
                let s: &str = buffer.format(t);
                display.set_position(COL_DATA, ROW_TIME).unwrap();
                let _ = display.write_str(s);
            },
            DisplayMessage::Pressure(p) => {
                info!("pressure = {}", p);
                let s: &str = buffer.format(p as u32);
                display.set_position(COL_DATA, ROW_PRESSURE).unwrap();
                let _ = display.write_str(s);
                let _ = display.write_str(" ");
            },
            DisplayMessage::Temperature(t) => {
                info!("Temperature = {}", t);
                let s: &str = buffer.format(t as u16);
                let after_comma_digits = ((t * 100.0) as u16) % 100;
                display.set_position(COL_DATA, ROW_TEMP).unwrap();
                let _ = display.write_str(s);
                let _ = display.write_str(".");
                let s = buffer.format(after_comma_digits);
                let _ = display.write_str(s);
                let _ = display.write_str(" ");
            },
            DisplayMessage::Light(lux) => {
                info!("Light = {}", lux);
                let s = buffer.format(lux as u32);
                display.set_position(COL_DATA, ROW_LUX).unwrap();
                let _ = display.write_str(s);
                let _ = display.write_str("  ");
            },
        }
    }
}
