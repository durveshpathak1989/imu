# Test Quad IMU Library

`MPU9250` is the SPI driver and calibration store for MPU-9250 and MPU-6500 boards. It returns scaled accel, gyro, and optional AK8963 magnetometer data for the AHRS filters.

## Pin Map

| Sensor signal | ESP32 pin | Notes |
| --- | ---: | --- |
| SCL/SCLK | GPIO 5 | SPI clock |
| AD0/MISO | GPIO 19 | SPI MISO |
| SDA/MOSI | GPIO 18 | SPI MOSI |
| NCS/CS | GPIO 33 | Chip select |
| INT | GPIO 27 | Optional; current firmware can run without it |
| VCC | 3.3V | Use 3.3V logic |
| GND | GND | Common ground |

## Main INO Integration Example

```cpp
#include <SPI.h>
#include "MPU9250.h"

#define PIN_SPI_SCK   5
#define PIN_SPI_MISO  19
#define PIN_SPI_MOSI  18
#define PIN_MPU_CS    33

MPU9250 imu(PIN_MPU_CS);

void setup() {
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_MPU_CS);
    if (!imu.begin()) {
        while (true) delay(1000);
    }
    imu.loadCalibration();
}

void loop() {
    MPU_SensorData s;
    if (imu.readScaled(s)) {
        // s.ax_g/s.ay_g/s.az_g, s.gx_dps/s.gy_dps/s.gz_dps, s.mx_uT...
    }
}
```


## Why These Data Types

`float` keeps scaled sensor units readable and avoids fixed-point mistakes in filter math. Raw register reads are converted once at the driver boundary. `bool` return values let the caller skip control updates when a sample fails. Calibration values are persisted with ESP32 `Preferences` so they survive power cycles.
