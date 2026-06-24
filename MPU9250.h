/**
 * ============================================================
 *  MPU9250.h  —  Driver + Calibration  (v4.0)
 *  Adafruit HUZZAH32 Feather (ESP32-WROOM-32E)
 *  SPI (VSPI) — FreeRTOS compatible
 *  Created By - Durvesh Pathak
 * ============================================================
 *
 *  v3.1 changes vs v3.0:
 *   • _hasMag flag: if AK8963 WHO_AM_I != 0x48 at boot, the
 *     entire mag init and SLV0 burst are skipped. This covers
 *     boards that ship with MPU-6500 (no mag die) and boards
 *     where the internal I2C bus is broken.
 *   • hasMag() public accessor so callers can check at runtime.
 *   • readRaw() skips the EXT_SENS_DATA burst when !_hasMag,
 *     saving 8 SPI bytes per call and removing false DRDY
 *     confusion in the debugger.
 *   • AHRS behaviour unchanged — when _hasMag is false the
 *     Mahony filter runs in 6-DOF mode (accel+gyro only).
 *     Roll and pitch are fully accurate; yaw drifts slowly.
 *
 *  HUZZAH32 Feather wiring:
 *    MPU-9250  →  Feather
 *    VCC       →  3V   (3.3 V ONLY)
 *    GND       →  GND
 *    SCL/SCLK  →  SCK  (GPIO 5)
 *    SDA/MOSI  →  MO   (GPIO 18)
 *    AD0/MISO  →  MI   (GPIO 19)
 *    NCS/CS    →  33   (GPIO 33)
 *    INT       →  27   (GPIO 27)  optional
 * ============================================================
 */

#pragma once

#ifndef MPU9250_H
#define MPU9250_H

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────
//  §1 Register Map — MPU-9250 / MPU-6500 accel + gyro core
// ─────────────────────────────────────────────────────────────

#define MPU_REG_SMPLRT_DIV      0x19  // Sample-rate divider: output rate = gyro rate / (1 + value)
#define MPU_REG_CONFIG          0x1A  // Gyro DLPF config: sets digital low-pass filter bandwidth
#define MPU_REG_GYRO_CONFIG     0x1B  // Gyro full-scale range config: ±250/500/1000/2000 dps
#define MPU_REG_ACCEL_CONFIG    0x1C  // Accel full-scale range config: ±2/4/8/16 g
#define MPU_REG_ACCEL_CONFIG2   0x1D  // Accel DLPF config: sets accelerometer low-pass bandwidth

#define MPU_REG_INT_PIN_CFG     0x37  // Interrupt pin behavior: polarity, latch mode, clear-on-read
#define MPU_REG_INT_ENABLE      0x38  // Interrupt enable: enables data-ready or other interrupt sources
#define MPU_REG_INT_STATUS      0x3A  // Interrupt status: read to check if data-ready interrupt occurred

#define MPU_REG_ACCEL_XOUT_H    0x3B  // Burst-read start: accel XYZ, temperature, gyro XYZ; 14 bytes total

#define MPU_REG_USER_CTRL       0x6A  // User control: enables internal I2C master for AK8963 mag access
#define MPU_REG_PWR_MGMT_1      0x6B  // Power management 1: reset, sleep/wake, clock source selection
#define MPU_REG_PWR_MGMT_2      0x6C  // Power management 2: enable/disable accel and gyro axes

#define MPU_REG_WHO_AM_I        0x75  // Device identity register: read to verify MPU is responding
#define MPU9250_WHO_AM_I_VAL    0x71  // Expected WHO_AM_I value for MPU-9250
#define MPU6500_WHO_AM_I_VAL    0x70  // Expected WHO_AM_I value for MPU-6500; no internal magnetometer


// ─────────────────────────────────────────────────────────────
//  §2 Register Map — AK8963 magnetometer through MPU I2C master
// ─────────────────────────────────────────────────────────────

#define AK8963_ADDR             0x0C  // AK8963 I2C address used by MPU internal I2C master

#define MPU_REG_I2C_MST_CTRL    0x24  // I2C master clock/control for MPU's internal I2C bus
#define MPU_REG_I2C_SLV0_ADDR   0x25  // I2C slave 0 address: used for automatic AK8963 reads
#define MPU_REG_I2C_SLV0_REG    0x26  // I2C slave 0 register: AK8963 register address to read from
#define MPU_REG_I2C_SLV0_CTRL   0x27  // I2C slave 0 control: enable read and set number of bytes

#define MPU_REG_I2C_SLV4_ADDR   0x31  // I2C slave 4 address: used for one-shot AK8963 writes
#define MPU_REG_I2C_SLV4_REG    0x32  // I2C slave 4 register: AK8963 register address to write to
#define MPU_REG_I2C_SLV4_DO     0x33  // I2C slave 4 data-out: byte value to write to AK8963
#define MPU_REG_I2C_SLV4_CTRL   0x34  // I2C slave 4 control: starts one-shot write transaction

#define MPU_REG_I2C_MST_STATUS  0x36  // I2C master status: checks SLV4 done, errors, arbitration lost
#define MPU_REG_EXT_SENS_DATA   0x49  // External sensor data start: AK8963 mag bytes appear here

#define AK_REG_WIA              0x00  // AK8963 identity register: read to verify magnetometer is present
#define AK_REG_ST1              0x02  // AK8963 status 1: data-ready bit
#define AK_REG_HXL              0x03  // AK8963 magnetic data start: HXL,HXH,HYL,HYH,HZL,HZH
#define AK_REG_ST2              0x09  // AK8963 status 2: overflow bit; must be read to release data latch

#define AK_REG_CNTL1            0x0A  // AK8963 control 1: mode select, power-down, fuse ROM, continuous mode
#define AK_REG_CNTL2            0x0B  // AK8963 control 2: soft reset
#define AK_REG_ASAX             0x10  // AK8963 fuse ROM sensitivity adjustment start: ASAX, ASAY, ASAZ
#define AK_WIA_VAL              0x48  // Expected AK8963 WIA identity value


// ─────────────────────────────────────────────────────────────
//  §3  Scale constants
// ─────────────────────────────────────────────────────────────
#define GYRO_SCALE_500DPS   (500.0f  / 32768.0f)
#define ACCEL_SCALE_8G      (8.0f    / 32768.0f)
#define MAG_SCALE_16BIT     (4912.0f / 32760.0f)
#define DEG2RAD             (3.14159265358979f / 180.0f)
#define RAD2DEG             (180.0f / 3.14159265358979f)

// ─────────────────────────────────────────────────────────────
//  §4  SPI speed constants
// ─────────────────────────────────────────────────────────────
#define MPU_SPI_SLOW        1000000UL   //  1 MHz — config writes
#define MPU_SPI_FAST        20000000UL  // 20 MHz — data reads
#define MPU_MAG_PERIOD_MS   10          //mag to be read ever 100hz 10ms

// ─────────────────────────────────────────────────────────────
//  §5  Data structures
// ─────────────────────────────────────────────────────────────

struct MPU_RawData {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
    int16_t temp;
    bool    magOk;
};

struct MPU_SensorData {
    float ax_g,  ay_g,  az_g;
    float gx_dps, gy_dps, gz_dps;
    float mx_uT, my_uT, mz_uT;
    float temp_c;
    uint32_t ts_ms;
    bool magOk;
};

struct MPU_CalData {
    float gx_b, gy_b, gz_b;
    float ax_b, ay_b, az_b;
    float ax_s, ay_s, az_s;
    float mx_b, my_b, mz_b;
    float mx_s, my_s, mz_s;
    float mag_asa_x, mag_asa_y, mag_asa_z;
    bool  valid;
};

// ─────────────────────────────────────────────────────────────
//  §6  MPU9250 class
// ─────────────────────────────────────────────────────────────
class MPU9250 {
public:
    explicit MPU9250(uint8_t csPin, SPIClass& spi = SPI);

    // ── Initialisation ──────────────────────────────────────
    bool begin();

    // ── Diagnostics ─────────────────────────────────────────
    bool    isConnected();
    bool    isMagConnected();   ///< true once first valid mag sample received
    bool    hasMag() const;     ///< true if AK8963 was detected at boot
    uint8_t whoAmI();
    float   readTemperature();

    // ── Data reads ──────────────────────────────────────────
    bool readRaw(MPU_RawData& out);
    bool readScaled(MPU_SensorData& out);

    // ── Calibration ─────────────────────────────────────────
    MPU_CalData cal;

    void sampleAvg(int N, MPU_SensorData& out);
    void calibrateGyro();
    void calibrateAccel();
    void calibrateMag(uint32_t durationMs = 20000);
    void diagMag();
    void saveCalibration(const char* ns = "imu_cal");
    bool loadCalibration(const char* ns = "imu_cal");
    void eraseCalibration(const char* ns = "imu_cal");
    void printCalibration();

private:
    
    uint8_t   _cs;
    SPIClass& _spi;

    // true if AK8963 was detected at boot
    bool _hasMag = false;

    // Last valid magnetometer sample
    float _last_mx = 0.0f;
    float _last_my = 0.0f;
    float _last_mz = 0.0f;
    bool  _magValid = false;
    uint32_t _lastMagReadMs = 0;

    void    _writeReg(uint8_t reg, uint8_t val);
    uint8_t _readReg(uint8_t reg);
    void    _burstRead(uint8_t reg, uint8_t* buf, uint8_t len);

    void    _akWrite(uint8_t akReg, uint8_t val);
    uint8_t _akReadByte(uint8_t akReg);
    void    _akBurstRead(uint8_t akReg, uint8_t* buf, uint8_t len);
    void    _akSetupContinuous();

    bool    _initMPU();
    void    _readMagASA();
};

#endif // MPU9250_H
