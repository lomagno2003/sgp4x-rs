# SGP41 NRF52 Example

This example demonstrates how to use the SGP41 sensor with an NRF52 microcontroller to measure both VOC (Volatile Organic Compounds) and NOx (Nitrogen Oxides) indices.

## Features

- VOC and NOx index measurements
- Combined measurement for efficiency
- Raw signal readings
- Proper sensor warm-up handling (45 samples)
- No-std embedded environment compatibility

## Hardware Setup

- NRF52-DK (PCA10040) or compatible board
- SGP41 sensor connected via I²C:
  - SCL: P1.15
  - SDA: P1.13
  - I²C address: 0x59

## Set up with `cargo-embed`

Install `cargo-embed` if you don't have it already:

```console
$ cargo install cargo-embed
```

Then just `cd` to the example folder and run

```console
$ cargo embed --target thumbv7em-none-eabihf
```

## Output

The example will output RTT messages showing:
1. Sensor warm-up progress (first 45 samples)
2. Rotating between different measurement types:
   - Individual VOC measurements
   - Individual NOx measurements  
   - Combined VOC+NOx measurements
   - Raw signal readings

