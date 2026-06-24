# MPU9250 / MPU6500 IMU Driver

## Purpose

Configures and reads the MPU-9250/MPU-6500 class IMU over SPI, including scaled gyro/accelerometer data, optional magnetometer data, and NVS calibration storage.

## Files

- `MPU9250.h/.cpp`: SPI driver, calibration, scaling, magnetometer bridge, and diagnostics.

## Quick Start

```cpp
#include "MPU9250.h"

MPU9250 imu;

void setup() {
    SPI.begin(5, 19, 18, 33);
    imu.begin();
    imu.loadCalibration();
}

void loop() {
    MPU_SensorData s{};
    if (imu.readScaled(s)) {
        float gx = s.gx_dps;
    }
}
```

## How It Fits Into The Flight Controller

This library lives under `Submodules/IMU` in the main `Test_Quad` firmware
and is built as an Arduino library by adding `Submodules/` to the Arduino
library search path. The main firmware includes it directly from
`RC_FlightController.ino` or from another support module.

The flight controller runs a 400 Hz control loop on ESP32, so this library
should avoid heap allocation, long blocking calls, and unbounded Serial output
inside flight-critical paths. Debug output should use `DebugConfig.h` macros
where available so `VERBOSE_ON=0` builds can compile prints out.

## Data Type Choices

- `MPU_SensorData`: Groups raw scaled IMU axes so a control cycle uses one coherent sample.
- `float` scaled units: Control and estimator code works in g, deg/s, and microtesla, not raw ADC counts.
- `uint8_t` registers: The MPU register map is byte-addressed.
- Calibration structs: Bias and scale factors are kept together and saved/loaded as one unit from NVS.

## Usage Guidance

1. Initialize hardware-facing classes once during `setup()`.
2. Keep update/read calls deterministic when used from a FreeRTOS task.
3. Prefer explicit validity flags over sentinel numeric values.
4. Keep units visible in field names, such as `_dps`, `_g`, `_uT`, `_m`, or `_us`.
5. When adding telemetry fields, update both the packet struct and JSON serializer.

## Example Build Integration

```bash
arduino-cli compile \
  --fqbn esp32:esp32:esp32:UploadSpeed=921600,CPUFreq=240,FlashFreq=80,FlashMode=qio,FlashSize=4M,PartitionScheme=min_spiffs,DebugLevel=none,PSRAM=disabled,LoopCore=1,EventsCore=1,EraseFlash=none,JTAGAdapter=default,ZigbeeMode=default \
  --libraries ./Submodules \
  .
```

For quiet flight builds:

```bash
arduino-cli compile ... --build-property compiler.cpp.extra_flags=-DVERBOSE_ON=0
```


## Integration Notes

In the main flight-controller sketch, this library is included through Arduino's
library search path. When this folder is converted to a git submodule, keep the
folder name stable under `Submodules/` so includes such as `#include "..."`
continue to resolve.

Most examples below are intentionally small. On the real flight controller,
objects are usually constructed globally, initialized once from `setup()`, and
then called from FreeRTOS tasks at deterministic rates.

