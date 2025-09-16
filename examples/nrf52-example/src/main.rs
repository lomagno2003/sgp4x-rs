#![no_main]
#![no_std]

use rtt_target::{rprintln, rtt_init_print};

// access to board peripherals:
use nrf52840_hal::{
    self as hal,
    twim::{self, Twim},
    Timer,
};

use sgp41::*;

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    let p = hal::pac::Peripherals::take().unwrap();
    let delay = Timer::new(p.TIMER1);

    let pins_1 = hal::gpio::p1::Parts::new(p.P1);

    let scl = pins_1.p1_15.into_floating_input().degrade();
    let sda = pins_1.p1_13.into_floating_input().degrade();

    let pins = twim::Pins { scl, sda };

    let i2c = Twim::new(p.TWIM1, pins, twim::Frequency::K100);

    let mut sensor = Sgp41::new(i2c, 0x59, delay);
    let mut sample_count = 0u32;

    rprintln!("SGP41 Sensor - VOC and NOx measurements");
    rprintln!("Warming up sensor...");

    loop {
        sample_count += 1;

        // Warm-up period: first 45 samples
        if sample_count <= 45 {
            if let Ok(_) = sensor.measure_voc_index() {
                if sample_count % 10 == 0 {
                    rprintln!("Warm-up: {}/45", sample_count);
                }
            }
        } else {
            // Normal operation: demonstrate different measurement types
            match sample_count % 4 {
                1 => {
                    // Individual VOC measurement
                    if let Ok(voc) = sensor.measure_voc_index() {
                        rprintln!("VOC index: {}", voc);
                    } else {
                        rprintln!("Failed VOC reading");
                    }
                }
                2 => {
                    // Individual NOx measurement
                    if let Ok(nox) = sensor.measure_nox_index() {
                        rprintln!("NOx index: {}", nox);
                    } else {
                        rprintln!("Failed NOx reading");
                    }
                }
                3 => {
                    // Combined measurement (more efficient)
                    if let Ok((voc, nox)) = sensor.measure_indices() {
                        rprintln!("Combined - VOC: {}, NOx: {}", voc, nox);
                    } else {
                        rprintln!("Failed combined reading");
                    }
                }
                0 => {
                    // Raw signals
                    if let Ok((voc_raw, nox_raw)) = sensor.measure_raw_signals() {
                        rprintln!("Raw - VOC: {}, NOx: {}", voc_raw, nox_raw);
                    } else {
                        rprintln!("Failed raw reading");
                    }
                }
                _ => {}
            }
        }
    }
}
