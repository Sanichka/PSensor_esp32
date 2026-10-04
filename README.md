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
