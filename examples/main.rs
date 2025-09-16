use hal::{Delay, I2cdev};
/// Sensirion SGP41 sensor device driver example demonstrating both VOC and NOx measurements.
use linux_embedded_hal as hal;

use std::thread;
use std::time::Duration;

use sgp41::Sgp41;

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();

    let mut sensor = Sgp41::new(dev, 0x59, Delay);

    println!("SGP41 Sensor Example - VOC and NOx Measurements");
    println!("Warming up sensor (first 45 samples)...");

    // Discard the first 45 samples as the algorithm is warming up
    for i in 1..=45 {
        if let Ok(_) = sensor.measure_voc_index() {
            if i % 10 == 0 {
                println!("Warm-up progress: {}/45", i);
            }
        }
        thread::sleep(Duration::from_millis(1000));
    }

    println!("Sensor warmed up. Starting measurements...");

    loop {
        // Example 1: Individual measurements
        match (sensor.measure_voc_index(), sensor.measure_nox_index()) {
            (Ok(voc), Ok(nox)) => {
                println!("VOC index: {}, NOx index: {}", voc, nox);
            }
            _ => {
                println!("Failed to read individual indices");
            }
        }

        thread::sleep(Duration::from_millis(500));

        // Example 2: Combined measurement (more efficient)
        if let Ok((voc, nox)) = sensor.measure_indices() {
            println!("Combined - VOC: {}, NOx: {}", voc, nox);
        } else {
            println!("Failed to read combined indices");
        }

        thread::sleep(Duration::from_millis(500));

        // Example 3: Raw signal measurements
        if let Ok((voc_raw, nox_raw)) = sensor.measure_raw_signals() {
            println!("Raw signals - VOC: {}, NOx: {}", voc_raw, nox_raw);
        } else {
            println!("Failed to read raw signals");
        }

        thread::sleep(Duration::from_millis(500));

        // Example 4: Measurements with temperature and humidity compensation
        let humidity = 50_u16; // 50% RH
        let temperature = 25_i16; // 25°C
        
        if let Ok((voc, nox)) = sensor.measure_indices_with_rht(humidity, temperature) {
            println!("Compensated - VOC: {}, NOx: {} (T: {}°C, RH: {}%)", 
                     voc, nox, temperature, humidity);
        } else {
            println!("Failed to read compensated measurements");
        }

        println!("---");
        thread::sleep(Duration::from_millis(2000));
    }
}
