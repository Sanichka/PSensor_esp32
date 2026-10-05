# PSensor ESP32

ESP32 pressure-monitoring firmware that reads pressure from an I2C BMP180 and
an SPI BMP280, validates the readings, smooths them with independent Kalman
filters, and reports status through the serial monitor.

The firmware reports:

- `OK` for valid readings
- `ERR_DISCONNECTED` for `NaN`, zero, or negative readings
- `ERR_OUT_OF_RANGE` for readings outside `300..1100 hPa`
- `WARN_SPIKE` for readings more than `4 hPa` from the current filter estimate

When the SPI BMP280 is unavailable, the firmware generates fallback data based
on the I2C reading and injects a periodic spike for demonstration and testing.

## Hardware and connections

The default board is an ESP32 Dev Module (`esp32dev`).

### I2C BMP180

| BMP180 | ESP32 |
| --- | --- |
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| VCC | 3.3 V |
| GND | GND |

### SPI BMP280

The firmware initializes the VSPI bus with:

| SPI signal | ESP32 |
| --- | --- |
| SCK | GPIO 18 |
| MISO | GPIO 19 |
| MOSI | GPIO 23 |
| CS | GPIO 5 |

The current Wokwi diagram only connects the I2C sensor. Therefore the SPI
device is normally reported as unavailable and the fallback generator is used.

## Software architecture

```text
src/main.cpp
  setup()
    initialize Serial, I2C, and SPI
    detect BMP180 and BMP280
  loop()
    read both sensors or generate SPI fallback data
    process each value
    print raw, filtered, and status data
    compare filtered values for discrepancies

lib/PressureProcessing/
  PressureProcessing.h/.cpp
    KalmanFilter
    SensorData
    processPressure()

test/test_pressure_processing/
  PlatformIO native Unity tests

wokwi.scenario.yaml
  Wokwi serial automation scenario
```

`processPressure()` is shared by production firmware, native tests, and the
Wokwi test harness. This keeps validation and filtering behavior consistent
across all test environments.

## Filter benchmark

The modular filter implementations and empirical benchmark live on the
separate [`benchmark` branch](https://github.com/Sanichka/PSensor_esp32/tree/benchmark).
That branch adds SMA, median, EMA, Kalman, complementary-fusion, and notch
filters behind a common interface, then runs the same deterministic synthetic
dataset through each filter at startup.

The benchmark reports:

- **RAM**: static `sizeof(filter)` for one filter instance
- **Average / Peak**: average and worst-case `update()` latency in microseconds
- **Heap**: whether the update path uses dynamic allocation
- **Sigma**: standard deviation of the filtered 100-sample noisy signal
- **Spike**: output deviation from baseline after a simulated `+15 hPa` impulse
- **90% lag**: update iterations needed to reach 90% of a simulated `+10 hPa`
  step; at 100 Hz, one iteration is approximately 10 ms

The benchmark output is intentionally printed as one filter per block so it
remains readable in narrow serial terminals. Expand the captured run below to
review the results:

<details>
<summary>Expand benchmark results</summary>

```text
================================
         FILTER BENCHMARK
================================
Samples : 100
Baseline: 1000.00 hPa
Step    : +10.00 hPa

[SMA]
  Execution
    RAM      : 60 bytes
    Average  : 13.58 us/update
    Peak     : 35.00 us/update
    Heap     : none
  Signal quality
    Sigma    : 0.0377 hPa
    Spike    : 3.000 hPa
    90% lag  : 5 samples

[Median]
  Execution
    RAM      : 56 bytes
    Average  : 25.19 us/update
    Peak     : 48.00 us/update
    Heap     : none
  Signal quality
    Sigma    : 0.0525 hPa
    Spike    : 0.000 hPa
    90% lag  : 3 samples

[EMA]
  Execution
    RAM      : 16 bytes
    Average  : 9.29 us/update
    Peak     : 44.00 us/update
    Heap     : none
  Signal quality
    Sigma    : 0.0389 hPa
    Spike    : 2.250 hPa
    90% lag  : 15 samples

[Kalman]
  Execution
    RAM      : 28 bytes
    Average  : 14.50 us/update
    Peak     : 52.00 us/update
    Heap     : none
  Signal quality
    Sigma    : 0.0302 hPa
    Spike    : 12.024 hPa
    90% lag  : 3 samples

[Complementary]
  Execution
    RAM      : 28 bytes
    Average  : 10.23 us/update
    Peak     : 32.00 us/update
    Heap     : none
  Signal quality
    Sigma    : 0.1184 hPa
    Spike    : 14.700 hPa
    90% lag  : 1 samples

[Notch]
  Execution
    RAM      : 36 bytes
    Average  : 27.18 us/update
    Peak     : 52.00 us/update
    Heap     : none
  Signal quality
    Sigma    : 0.1213 hPa
    Spike    : 14.348 hPa
    90% lag  : 1 samples
```

</details>

## Dependencies

Dependencies are declared in [platformio.ini](platformio.ini):

- Arduino framework for ESP32
- Adafruit BMP085 Library
- Adafruit BMP280 Library
- Adafruit Unified Sensor and Adafruit BusIO (transitive dependencies)
- Unity, installed by PlatformIO for the native test environment

Install [PlatformIO](https://platformio.org/install) and make sure either
`pio` is on `PATH` or use the full path to the PlatformIO executable.

## Build and run the default firmware

From the project root:

```powershell
pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor -e esp32dev -b 115200
```

The default build artifacts are written to `.pio/build/esp32dev/`.

If `pio` is not on `PATH`, use the PlatformIO virtual environment on Windows:

```powershell
& "$HOME\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
& "$HOME\.platformio\penv\Scripts\platformio.exe" run -e esp32dev -t upload
& "$HOME\.platformio\penv\Scripts\platformio.exe" device monitor -e esp32dev -b 115200
```

The upload command requires an ESP32 connected over USB. If automatic port
detection fails, add `--upload-port COM<n>`.

## PlatformIO tests

The native tests run the pressure-processing logic on the host without
hardware:

```powershell
pio test -e native
```

They cover:

- First valid reading and Kalman initialization
- Subsequent-reading smoothing
- `NaN`, zero, and negative/disconnected values
- Valid lower and upper boundaries
- Out-of-range values
- Spike detection and the exact `4 hPa` threshold
- Filter state preservation after invalid readings
- Disconnect/recovery sequences
- Filter reset

The native test firmware is not uploaded to the ESP32.

## Wokwi tests

The Wokwi environment builds a special firmware with the
`WOKWI_AUTOTEST` compile-time flag. It adds a serial command harness while
leaving the default `esp32dev` firmware unchanged.

Build the Wokwi firmware:

```powershell
pio run -e wokwi
```

Set a Wokwi CLI token for the current PowerShell session:

```powershell
$env:WOKWI_CLI_TOKEN = "your-wokwi-token"
```

Then run the automation scenario:

```powershell
wokwi-cli --scenario wokwi.scenario.yaml
```

The scenario uses the official Wokwi automation steps `wait-serial` and
`write-serial`. It verifies startup detection and then runs the same processing
categories as the native suite:

- First valid reading
- Kalman smoothing
- Disconnected values
- Valid range boundaries
- Out-of-range values
- Spike warning
- Spike threshold
- Filter state preservation
- Disconnect and recovery
- Filter reset
- Continued normal loop output and fallback spike generation

The scenario uses [wokwi.toml](wokwi.toml), which points to the
`.pio/build/wokwi/` firmware and ELF files. The Wokwi CLI token should never be
committed to the repository.

## Continuous integration

[.github/workflows/platformio.yml](.github/workflows/platformio.yml) runs:

```text
pio test -e native
pio run -e esp32dev
pio run -e wokwi
```

The workflow builds the Wokwi test firmware but does not run the cloud Wokwi
scenario unless a Wokwi token and a separate authenticated CI step are
configured.
