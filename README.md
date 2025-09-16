# sgp4x
Platform agnostic Rust device driver for Sensirion SGP4x created based on https://crates.io/crates/sgp40. At the moment, it supports both SGP41 and SGP40 (although NOx will fail for SGP40)

[![Build status][workflow-badge]][workflow]
[![Crates.io Version][crates-io-badge]][crates-io]
[![Crates.io Downloads][crates-io-download-badge]][crates-io-download]
![No Std][no-std-badge]

Platform agnostic Rust driver for Sensirion SGP41 gas sensor using the [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.

## Sensirion SGP4X

Sensirion SGP4X is a low-power accurate gas sensor for air quality applications. The sensor uses I²C interface and measures both VOC (*Total Volatile Organic Compounds*) and NOx (*Nitrogen Oxides*) providing comprehensive air quality monitoring capabilities.

Key features:
- **Dual gas measurement**: VOC and NOx indices
- **Temperature/humidity compensation**: Improved accuracy with environmental data
- **Low power consumption**: Optimized for battery-powered applications  
- **I²C interface**: Simple integration with 0x59 address
- **No-std compatible**: Works in embedded environments
- **Gas index algorithms**: Uses official Sensirion algorithms via gas-index-algorithm crate

Datasheet: <https://www.sensirion.com/file/datasheet_sgp41>

## Usage

### Basic VOC and NOx Measurements

```rust
use linux_embedded_hal as hal;
use hal::{Delay, I2cdev};
use sgp41::Sgp41;

let dev = I2cdev::new("/dev/i2c-1").unwrap();
let mut sensor = Sgp41::new(dev, 0x59, Delay);

// Warm up the sensor (first 45 samples should be discarded)
for _ in 0..45 {
    sensor.measure_voc_index().ok();
}

// Individual measurements
let voc_index = sensor.measure_voc_index()?; // 1-500 (100 = average)
let nox_index = sensor.measure_nox_index()?; // 1-500 (1 = average)

// Combined measurement (more efficient)
let (voc, nox) = sensor.measure_indices()?;
```

### Temperature and Humidity Compensation

```rust
let humidity = 50_u16; // 50% RH  
let temperature = 25_i16; // 25°C

// Compensated measurements
let voc = sensor.measure_voc_index_with_rht(humidity, temperature)?;
let nox = sensor.measure_nox_index_with_rht(humidity, temperature)?;

// Combined compensated measurement
let (voc, nox) = sensor.measure_indices_with_rht(humidity, temperature)?;
```

### Raw Signal Access

```rust
// Individual raw signals
let voc_raw = sensor.measure_raw()?;
let nox_raw = sensor.measure_raw_nox()?;

// Combined raw signals
let (voc_raw, nox_raw) = sensor.measure_raw_signals()?;
```

### SGP41-Specific Features

```rust
// Conditioning (recommended for new sensors)
sensor.execute_conditioning()?;

// Conditioning with VOC reading
sensor.execute_conditioning_with_voc_read()?;
```

## Migration from SGP40

The SGP41 driver maintains API compatibility with SGP40 for VOC measurements:

```rust
// SGP40 code
use sgp40::Sgp40;
let mut sensor = Sgp40::new(i2c, 0x59, delay);
let voc = sensor.measure_voc_index()?;

// SGP41 equivalent  
use sgp41::Sgp41;
let mut sensor = Sgp41::new(i2c, 0x59, delay);
let voc = sensor.measure_voc_index()?; // Same API
let nox = sensor.measure_nox_index()?; // New capability
```

## Examples

- [`main.rs`](examples/main.rs) - Linux example showing all measurement types
- [`nrf52-example`](examples/nrf52-example/) - Embedded no-std example for NRF52

## Development status
The crate is feature complete with full VOC and NOx measurement capabilities.


## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT) at your option.


### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

<!-- Badges -->
[workflow]: https://github.com/mjaakkol/sgp40-rs/actions?query=workflow%3ARust
[workflow-badge]: https://img.shields.io/github/workflow/status/mjaakkol/sgp40-rs/Rust/master
[crates-io]: https://crates.io/crates/sgp41
[crates-io-badge]: https://img.shields.io/crates/v/sgp41.svg?maxAge=3600
[crates-io-download]: https://crates.io/crates/sgp41
[crates-io-download-badge]: https://img.shields.io/crates/d/sgp41.svg?maxAge=3600
[no-std-badge]: https://img.shields.io/badge/no__std-yes-blue
