/**
 * ============================================================
 *  MPU9250.cpp  —  Driver + Calibration  (v4.0 no Mahony)
 *  Adafruit HUZZAH32 Feather (ESP32-WROOM-32E)
 *  SPI (VSPI) — FreeRTOS compatible
 *  Created By - Durvesh Pathak
 * ============================================================
 *
 *  v3.2 change:
 *   • Mahony AHRS removed from MPU9250 driver.
 *   • MPU9250 now only handles sensor acquisition, scaling,
 *     calibration, NVS storage, and diagnostics.
 *   • Attitude estimation should live in a separate MahonyAHRS
 *     / MadgwickAHRS / AttitudeEKF library.
 * ============================================================
 */

#include "MPU9250.h"
#include "DebugConfig.h"
#include <Preferences.h>

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
MPU9250::MPU9250(uint8_t csPin, SPIClass& spi)
    : _cs(csPin), _spi(spi)
{
    memset(&cal, 0, sizeof(cal));

    // Safe default scale values
    cal.ax_s = cal.ay_s = cal.az_s = 1.0f;
    cal.mx_s = cal.my_s = cal.mz_s = 1.0f;
    cal.mag_asa_x = cal.mag_asa_y = cal.mag_asa_z = 1.0f;
}

// ─────────────────────────────────────────────────────────────
//  SPI helpers
// ─────────────────────────────────────────────────────────────
void MPU9250::_writeReg(uint8_t reg, uint8_t val) {
    _spi.beginTransaction(SPISettings(MPU_SPI_SLOW, MSBFIRST, SPI_MODE3));
    digitalWrite(_cs, LOW);
    _spi.transfer(reg & 0x7F);     // bit7 = 0 → write
    _spi.transfer(val);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

uint8_t MPU9250::_readReg(uint8_t reg) {
    _spi.beginTransaction(SPISettings(MPU_SPI_FAST, MSBFIRST, SPI_MODE3));
    digitalWrite(_cs, LOW);
    _spi.transfer(reg | 0x80);     // bit7 = 1 → read
    uint8_t val = _spi.transfer(0x00);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
    return val;
}

void MPU9250::_burstRead(uint8_t reg, uint8_t* buf, uint8_t len) {
    _spi.beginTransaction(SPISettings(MPU_SPI_FAST, MSBFIRST, SPI_MODE3));
    digitalWrite(_cs, LOW);
    _spi.transfer(reg | 0x80);     // bit7 = 1 → read
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = _spi.transfer(0x00);
    }
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

// ─────────────────────────────────────────────────────────────
//  AK8963 helpers — only valid when internal I2C master is enabled
// ─────────────────────────────────────────────────────────────
void MPU9250::_akWrite(uint8_t akReg, uint8_t val) {
    // SLV4 is used for one-shot writes to AK8963
    _writeReg(MPU_REG_I2C_SLV4_ADDR, AK8963_ADDR);   // write mode
    _writeReg(MPU_REG_I2C_SLV4_REG,  akReg);
    _writeReg(MPU_REG_I2C_SLV4_DO,   val);
    _writeReg(MPU_REG_I2C_SLV4_CTRL, 0x80);          // enable one-shot

    uint32_t t = millis();
    while (!((_readReg(MPU_REG_I2C_MST_STATUS) >> 6) & 0x01)) {
        if (millis() - t > 10) break;
        delayMicroseconds(100);
    }
    delay(1);
}

uint8_t MPU9250::_akReadByte(uint8_t akReg) {
    _writeReg(MPU_REG_I2C_SLV0_ADDR, AK8963_ADDR | 0x80); // read mode
    _writeReg(MPU_REG_I2C_SLV0_REG,  akReg);
    _writeReg(MPU_REG_I2C_SLV0_CTRL, 0x81);               // enable, 1 byte
    delay(10);
    return _readReg(MPU_REG_EXT_SENS_DATA);
}

void MPU9250::_akBurstRead(uint8_t akReg, uint8_t* buf, uint8_t len) {
    _writeReg(MPU_REG_I2C_SLV0_ADDR, AK8963_ADDR | 0x80); // read mode
    _writeReg(MPU_REG_I2C_SLV0_REG,  akReg);
    _writeReg(MPU_REG_I2C_SLV0_CTRL, 0x80 | (len & 0x0F));
    delay(10);
    _burstRead(MPU_REG_EXT_SENS_DATA, buf, len);
}

void MPU9250::_akSetupContinuous() {
    // Auto-read 8 bytes: ST1 + HXL/HXH + HYL/HYH + HZL/HZH + ST2
    _writeReg(MPU_REG_I2C_SLV0_ADDR, AK8963_ADDR | 0x80);
    _writeReg(MPU_REG_I2C_SLV0_REG,  AK_REG_ST1);
    _writeReg(MPU_REG_I2C_SLV0_CTRL, 0x88);   // EN=1, LEN=8
}

void MPU9250::_readMagASA() {
    _akWrite(AK_REG_CNTL1, 0x00);   // power down
    delay(10);

    _akWrite(AK_REG_CNTL1, 0x0F);   // fuse ROM access mode
    delay(10);

    uint8_t asa[3];
    _akBurstRead(AK_REG_ASAX, asa, 3);

    cal.mag_asa_x = ((float)asa[0] - 128.0f) / 256.0f + 1.0f;
    cal.mag_asa_y = ((float)asa[1] - 128.0f) / 256.0f + 1.0f;
    cal.mag_asa_z = ((float)asa[2] - 128.0f) / 256.0f + 1.0f;

    DBG_PRINTF("[MPU9250] Mag ASA: X=%.4f Y=%.4f Z=%.4f\n",
                  cal.mag_asa_x, cal.mag_asa_y, cal.mag_asa_z);

    _akWrite(AK_REG_CNTL1, 0x00);   // power down before next mode change
    delay(10);
}

// ─────────────────────────────────────────────────────────────
//  MPU core init — accel + gyro
// ─────────────────────────────────────────────────────────────
bool MPU9250::_initMPU() {
    _writeReg(MPU_REG_PWR_MGMT_1, 0x80);   // hard reset
    delay(100);

    _writeReg(MPU_REG_PWR_MGMT_1, 0x01);   // wake, gyro clock
    delay(10);

    _writeReg(MPU_REG_PWR_MGMT_2, 0x00);   // all accel/gyro axes enabled
    delay(10);

    _writeReg(MPU_REG_CONFIG,        0x03); // gyro DLPF ≈ 41 Hz
    _writeReg(MPU_REG_SMPLRT_DIV,    0x00); // sample rate = 1 kHz
    _writeReg(MPU_REG_GYRO_CONFIG,   0x08); // ±500 dps
    _writeReg(MPU_REG_ACCEL_CONFIG,  0x10); // ±8 g
    _writeReg(MPU_REG_ACCEL_CONFIG2, 0x03); // accel DLPF ≈ 41 Hz

    _writeReg(MPU_REG_INT_PIN_CFG,   0x30); // interrupt pin config
    _writeReg(MPU_REG_INT_ENABLE,    0x01); // data-ready interrupt enable

    return true;
}

// ─────────────────────────────────────────────────────────────
//  begin()
// ─────────────────────────────────────────────────────────────
bool MPU9250::begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    delay(10);

    _hasMag = false;
    _magValid = false;

    uint8_t who = _readReg(MPU_REG_WHO_AM_I);

    if (who == MPU6500_WHO_AM_I_VAL) {
        DBG_PRINTLN(F("[MPU9250] Detected MPU-6500 (WHO_AM_I=0x70)."));
    } else if (who == MPU9250_WHO_AM_I_VAL) {
        DBG_PRINTF("[MPU9250] WHO_AM_I=0x%02X OK\n", who);
    } else {
        DBG_PRINTF("[MPU9250] WHO_AM_I=0x%02X — expected 0x71 or 0x70. Check wiring!\n", who);
        return false;
    }

    _initMPU();
    delay(10);

    // Enable internal I2C master so MPU can talk to AK8963
    _writeReg(MPU_REG_USER_CTRL,    0x20); // I2C_MST_EN=1
    delay(10);
    _writeReg(MPU_REG_I2C_MST_CTRL, 0x0D); // 400 kHz
    delay(10);

    uint8_t akWho = _akReadByte(AK_REG_WIA);

    if (akWho == AK_WIA_VAL) {
        DBG_PRINTF("[MPU9250] AK8963 WHO_AM_I=0x%02X OK — magnetometer active\n", akWho);
        _hasMag = true;

        _readMagASA();

        _akWrite(AK_REG_CNTL1, 0x16);  // continuous mode 2, 16-bit, 100 Hz
        delay(10);

        _akSetupContinuous();
        delay(50);
    } else {
        DBG_PRINTF("[MPU9250] AK8963 not found (WHO_AM_I=0x%02X). Running 6-DOF.\n", akWho);
        _hasMag = false;
        _magValid = false;

        // Disable internal I2C master if there is no mag to read
        _writeReg(MPU_REG_USER_CTRL, 0x00);
    }

    return true;
}

// ─────────────────────────────────────────────────────────────
//  Diagnostics
// ─────────────────────────────────────────────────────────────
bool MPU9250::isConnected() {
    uint8_t who = _readReg(MPU_REG_WHO_AM_I);
    return (who == MPU9250_WHO_AM_I_VAL) || (who == MPU6500_WHO_AM_I_VAL);
}

bool MPU9250::isMagConnected() {
    // True only after at least one valid mag sample has been received
    return _magValid;
}

bool MPU9250::hasMag() const {
    // True if AK8963 was detected at boot
    return _hasMag;
}

uint8_t MPU9250::whoAmI() {
    return _readReg(MPU_REG_WHO_AM_I);
}

float MPU9250::readTemperature() {
    uint8_t buf[2];
    _burstRead(0x41, buf, 2);   // TEMP_OUT_H / TEMP_OUT_L
    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    return (float)raw / 333.87f + 21.0f;
}

// ─────────────────────────────────────────────────────────────
//  readRaw()
// ─────────────────────────────────────────────────────────────
bool MPU9250::readRaw(MPU_RawData& out) {
    uint8_t buf[14];
    _burstRead(MPU_REG_ACCEL_XOUT_H, buf, 14);

    out.ax   = (int16_t)((buf[0]  << 8) | buf[1]);
    out.ay   = (int16_t)((buf[2]  << 8) | buf[3]);
    out.az   = (int16_t)((buf[4]  << 8) | buf[5]);
    out.temp = (int16_t)((buf[6]  << 8) | buf[7]);
    out.gx   = (int16_t)((buf[8]  << 8) | buf[9]);
    out.gy   = (int16_t)((buf[10] << 8) | buf[11]);
    out.gz   = (int16_t)((buf[12] << 8) | buf[13]);


    // ── Magnetometer read: non-blocking, decimated to ~100 Hz ──
    // Accel/gyro may run much faster than AK8963 mag.
    // Do not block waiting for mag data. If not time, report no fresh mag.
    out.magOk = false;
    out.mx = 0;
    out.my = 0;
    out.mz = 0;

    if (_hasMag) {
    uint32_t nowMs = millis();

    if ((nowMs - _lastMagReadMs) >= MPU_MAG_PERIOD_MS) {
        _lastMagReadMs = nowMs;

        uint8_t mag[8];
        _burstRead(MPU_REG_EXT_SENS_DATA, mag, 8);

        bool dataReady = (mag[0] & 0x01);
        bool overflow  = (mag[7] & 0x08);

        out.magOk = dataReady && !overflow;

        if (out.magOk) {
            out.mx = (int16_t)((mag[2] << 8) | mag[1]);
            out.my = (int16_t)((mag[4] << 8) | mag[3]);
            out.mz = (int16_t)((mag[6] << 8) | mag[5]);
        }
    }
} 

    else {
        out.magOk = false;
        out.mx = out.my = out.mz = 0;
    }

    return true;
}

// ─────────────────────────────────────────────────────────────
//  readScaled()
// ─────────────────────────────────────────────────────────────
bool MPU9250::readScaled(MPU_SensorData& out) {
    MPU_RawData raw;
    if (!readRaw(raw)) return false;

    out.ax_g = ((float)raw.ax * ACCEL_SCALE_8G - cal.ax_b) * cal.ax_s;
    out.ay_g = ((float)raw.ay * ACCEL_SCALE_8G - cal.ay_b) * cal.ay_s;
    out.az_g = ((float)raw.az * ACCEL_SCALE_8G - cal.az_b) * cal.az_s;

    out.gx_dps = (float)raw.gx * GYRO_SCALE_500DPS - cal.gx_b;
    out.gy_dps = (float)raw.gy * GYRO_SCALE_500DPS - cal.gy_b;
    out.gz_dps = (float)raw.gz * GYRO_SCALE_500DPS - cal.gz_b;

    if (_hasMag && raw.magOk) {
        out.magOk = true;
        float mx_raw = (float)raw.mx * MAG_SCALE_16BIT * cal.mag_asa_x;
        float my_raw = (float)raw.my * MAG_SCALE_16BIT * cal.mag_asa_y;
        float mz_raw = (float)raw.mz * MAG_SCALE_16BIT * cal.mag_asa_z;

        out.mx_uT = (mx_raw - cal.mx_b) * cal.mx_s;
        out.my_uT = (my_raw - cal.my_b) * cal.my_s;
        out.mz_uT = (mz_raw - cal.mz_b) * cal.mz_s;

        _last_mx = out.mx_uT;
        _last_my = out.my_uT;
        _last_mz = out.mz_uT;
        _magValid = true;

    } else {
        out.magOk = false;
        out.mx_uT = _last_mx;
        out.my_uT = _last_my;
        out.mz_uT = _last_mz;
        
    }

    out.temp_c = (float)raw.temp / 333.87f + 21.0f;
    out.ts_ms  = millis();

    return true;
}

// ─────────────────────────────────────────────────────────────
//  sampleAvg() — read N raw samples, return average scaled values
// ─────────────────────────────────────────────────────────────
void MPU9250::sampleAvg(int N, MPU_SensorData& out) {
    double ax = 0, ay = 0, az = 0;
    double gx = 0, gy = 0, gz = 0;
    uint8_t buf[14];

    for (int i = 0; i < N; i++) {
        _burstRead(MPU_REG_ACCEL_XOUT_H, buf, 14);

        ax += (int16_t)((buf[0]  << 8) | buf[1]);
        ay += (int16_t)((buf[2]  << 8) | buf[3]);
        az += (int16_t)((buf[4]  << 8) | buf[5]);
        gx += (int16_t)((buf[8]  << 8) | buf[9]);
        gy += (int16_t)((buf[10] << 8) | buf[11]);
        gz += (int16_t)((buf[12] << 8) | buf[13]);

        delay(2);
    }

    out.ax_g   = (float)(ax / N) * ACCEL_SCALE_8G;
    out.ay_g   = (float)(ay / N) * ACCEL_SCALE_8G;
    out.az_g   = (float)(az / N) * ACCEL_SCALE_8G;
    out.gx_dps = (float)(gx / N) * GYRO_SCALE_500DPS;
    out.gy_dps = (float)(gy / N) * GYRO_SCALE_500DPS;
    out.gz_dps = (float)(gz / N) * GYRO_SCALE_500DPS;
}

// ─────────────────────────────────────────────────────────────
//  Calibration: Gyro
// ─────────────────────────────────────────────────────────────
void MPU9250::calibrateGyro() {
    DBG_PRINTLN(F("\n[CAL] Gyro — place drone flat and still."));
    DBG_PRINTLN(F("[CAL] Press ENTER to start..."));
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read();

    DBG_PRINTLN(F("[CAL] Sampling 1000 readings (2 s)..."));

    MPU_SensorData avg;
    sampleAvg(1000, avg);

    cal.gx_b = avg.gx_dps;
    cal.gy_b = avg.gy_dps;
    cal.gz_b = avg.gz_dps;

    DBG_PRINTF("[CAL] Gyro bias: X=%+.4f  Y=%+.4f  Z=%+.4f  °/s\n",
                  cal.gx_b, cal.gy_b, cal.gz_b);
}

// ─────────────────────────────────────────────────────────────
//  Calibration: Accel
// ─────────────────────────────────────────────────────────────
void MPU9250::calibrateAccel() {
    const char* labels[6] = {
        "+X axis UP  (right side up)",
        "-X axis UP  (left side up)",
        "+Y axis UP  (nose up)",
        "-Y axis UP  (nose down)",
        "+Z axis UP  (flat, top up)",
        "-Z axis UP  (inverted)"
    };

    float readings[6][3];

    DBG_PRINTLN(F("\n[CAL] Accel 6-position calibration."));
    DBG_PRINTLN(F("[CAL] For each orientation, place drone then press ENTER.\n"));

    for (int pos = 0; pos < 6; pos++) {
        DBG_PRINTF("[CAL] Position %d/6: %s\n[CAL] Press ENTER when ready...\n",
                      pos + 1, labels[pos]);

        while (!Serial.available()) delay(10);
        while (Serial.available()) Serial.read();

        MPU_SensorData avg;
        sampleAvg(500, avg);

        readings[pos][0] = avg.ax_g;
        readings[pos][1] = avg.ay_g;
        readings[pos][2] = avg.az_g;

        DBG_PRINTF("[CAL]  Got: ax=%+.4f  ay=%+.4f  az=%+.4f\n",
                      avg.ax_g, avg.ay_g, avg.az_g);
    }

    cal.ax_b = (readings[0][0] + readings[1][0]) / 2.0f;
    cal.ay_b = (readings[2][1] + readings[3][1]) / 2.0f;
    cal.az_b = (readings[4][2] + readings[5][2]) / 2.0f;

    float hrX = (readings[0][0] - readings[1][0]) / 2.0f;
    float hrY = (readings[2][1] - readings[3][1]) / 2.0f;
    float hrZ = (readings[4][2] - readings[5][2]) / 2.0f;

    cal.ax_s = (fabsf(hrX) > 0.01f) ? 1.0f / hrX : 1.0f;
    cal.ay_s = (fabsf(hrY) > 0.01f) ? 1.0f / hrY : 1.0f;
    cal.az_s = (fabsf(hrZ) > 0.01f) ? 1.0f / hrZ : 1.0f;

    DBG_PRINTF("[CAL] Accel bias: X=%+.4f  Y=%+.4f  Z=%+.4f  g\n",
                  cal.ax_b, cal.ay_b, cal.az_b);
    DBG_PRINTF("[CAL] Accel scale: X=%+.4f  Y=%+.4f  Z=%+.4f\n",
                  cal.ax_s, cal.ay_s, cal.az_s);
}

// ─────────────────────────────────────────────────────────────
//  Calibration: Mag
// ─────────────────────────────────────────────────────────────
void MPU9250::calibrateMag(uint32_t durationMs) {
    if (!_hasMag) {
        DBG_PRINTLN(F("[CAL] No AK8963 magnetometer detected. Skipping mag calibration."));
        return;
    }

    DBG_PRINTLN(F("\n[CAL] Mag calibration — rotate slowly through ALL axes."));
    DBG_PRINTF("[CAL] Duration: %lu s. Press ENTER to start...\n", durationMs / 1000);

    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read();

    float xmin =  1e9f, ymin =  1e9f, zmin =  1e9f;
    float xmax = -1e9f, ymax = -1e9f, zmax = -1e9f;

    uint32_t end_ms = millis() + durationMs;
    uint32_t lastPrint = 0;

    while (millis() < end_ms) {
        uint8_t mag[8];
        _burstRead(MPU_REG_EXT_SENS_DATA, mag, 8);

        bool ok = (mag[0] & 0x01) && !(mag[7] & 0x08);

        if (ok) {
            float mx = (int16_t)((mag[2] << 8) | mag[1]) * MAG_SCALE_16BIT * cal.mag_asa_x;
            float my = (int16_t)((mag[4] << 8) | mag[3]) * MAG_SCALE_16BIT * cal.mag_asa_y;
            float mz = (int16_t)((mag[6] << 8) | mag[5]) * MAG_SCALE_16BIT * cal.mag_asa_z;

            if (mx < xmin) xmin = mx;
            if (mx > xmax) xmax = mx;
            if (my < ymin) ymin = my;
            if (my > ymax) ymax = my;
            if (mz < zmin) zmin = mz;
            if (mz > zmax) zmax = mz;
        }

        if (millis() - lastPrint > 2000) {
            DBG_PRINTF("[CAL] %lu s remaining...\n", (end_ms - millis()) / 1000);
            lastPrint = millis();
        }

        delay(10);
    }

    cal.mx_b = (xmax + xmin) / 2.0f;
    cal.my_b = (ymax + ymin) / 2.0f;
    cal.mz_b = (zmax + zmin) / 2.0f;

    float avg = ((xmax - xmin) + (ymax - ymin) + (zmax - zmin)) / 3.0f;
    float rx = xmax - xmin;
    float ry = ymax - ymin;
    float rz = zmax - zmin;

    cal.mx_s = (rx > 0.1f) ? avg / rx : 1.0f;
    cal.my_s = (ry > 0.1f) ? avg / ry : 1.0f;
    cal.mz_s = (rz > 0.1f) ? avg / rz : 1.0f;

    DBG_PRINTF("[CAL] Mag bias:  X=%+.2f  Y=%+.2f  Z=%+.2f  µT\n",
                  cal.mx_b, cal.my_b, cal.mz_b);
    DBG_PRINTF("[CAL] Mag scale: X=%+.4f  Y=%+.4f  Z=%+.4f\n",
                  cal.mx_s, cal.my_s, cal.mz_s);
}

// ─────────────────────────────────────────────────────────────
//  NVS: save / load / erase
// ─────────────────────────────────────────────────────────────
void MPU9250::saveCalibration(const char* ns) {
    Preferences p;
    p.begin(ns, false);

    p.putFloat("gx_b", cal.gx_b);
    p.putFloat("gy_b", cal.gy_b);
    p.putFloat("gz_b", cal.gz_b);

    p.putFloat("ax_b", cal.ax_b);
    p.putFloat("ay_b", cal.ay_b);
    p.putFloat("az_b", cal.az_b);

    p.putFloat("ax_s", cal.ax_s);
    p.putFloat("ay_s", cal.ay_s);
    p.putFloat("az_s", cal.az_s);

    p.putFloat("mx_b", cal.mx_b);
    p.putFloat("my_b", cal.my_b);
    p.putFloat("mz_b", cal.mz_b);

    p.putFloat("mx_s", cal.mx_s);
    p.putFloat("my_s", cal.my_s);
    p.putFloat("mz_s", cal.mz_s);

    p.putBool("valid", true);
    p.end();

    cal.valid = true;
    DBG_PRINTLN(F("[NVS] Calibration saved."));
}

bool MPU9250::loadCalibration(const char* ns) {
    Preferences p;
    p.begin(ns, true);

    bool valid = p.getBool("valid", false);

    if (valid) {
        cal.gx_b = p.getFloat("gx_b", 0.0f);
        cal.gy_b = p.getFloat("gy_b", 0.0f);
        cal.gz_b = p.getFloat("gz_b", 0.0f);

        cal.ax_b = p.getFloat("ax_b", 0.0f);
        cal.ay_b = p.getFloat("ay_b", 0.0f);
        cal.az_b = p.getFloat("az_b", 0.0f);

        cal.ax_s = p.getFloat("ax_s", 1.0f);
        cal.ay_s = p.getFloat("ay_s", 1.0f);
        cal.az_s = p.getFloat("az_s", 1.0f);

        cal.mx_b = p.getFloat("mx_b", 0.0f);
        cal.my_b = p.getFloat("my_b", 0.0f);
        cal.mz_b = p.getFloat("mz_b", 0.0f);

        cal.mx_s = p.getFloat("mx_s", 1.0f);
        cal.my_s = p.getFloat("my_s", 1.0f);
        cal.mz_s = p.getFloat("mz_s", 1.0f);

        cal.valid = true;
    }

    p.end();
    return valid;
}

void MPU9250::eraseCalibration(const char* ns) {
    Preferences p;
    p.begin(ns, false);
    p.clear();
    p.end();

    memset(&cal, 0, sizeof(cal));
    cal.ax_s = cal.ay_s = cal.az_s = 1.0f;
    cal.mx_s = cal.my_s = cal.mz_s = 1.0f;
    cal.mag_asa_x = cal.mag_asa_y = cal.mag_asa_z = 1.0f;

    DBG_PRINTLN(F("[NVS] Calibration erased."));
}

void MPU9250::printCalibration() {
    DBG_PRINTLN(F("\n--- MPU9250 Calibration ---"));
    DBG_PRINTF("  Gyro bias:   X=%+.4f  Y=%+.4f  Z=%+.4f  °/s\n", cal.gx_b, cal.gy_b, cal.gz_b);
    DBG_PRINTF("  Accel bias:  X=%+.4f  Y=%+.4f  Z=%+.4f  g\n",   cal.ax_b, cal.ay_b, cal.az_b);
    DBG_PRINTF("  Accel scale: X=%+.4f  Y=%+.4f  Z=%+.4f\n",      cal.ax_s, cal.ay_s, cal.az_s);
    DBG_PRINTF("  Mag bias:    X=%+.2f  Y=%+.2f  Z=%+.2f  µT\n",  cal.mx_b, cal.my_b, cal.mz_b);
    DBG_PRINTF("  Mag scale:   X=%+.4f  Y=%+.4f  Z=%+.4f\n",      cal.mx_s, cal.my_s, cal.mz_s);
    DBG_PRINTF("  Mag ASA:     X=%.4f  Y=%.4f  Z=%.4f\n",         cal.mag_asa_x, cal.mag_asa_y, cal.mag_asa_z);
    DBG_PRINTF("  Valid: %s\n", cal.valid ? "YES" : "NO");
}

// ─────────────────────────────────────────────────────────────
//  diagMag()
// ─────────────────────────────────────────────────────────────
void MPU9250::diagMag() {
    if (!_hasMag) {
        DBG_PRINTLN(F("[DIAG] No AK8963 magnetometer detected at boot."));
        return;
    }

    DBG_PRINTLN(F("\n[DIAG] Raw EXT_SENS_DATA (10 samples):"));
    DBG_PRINTLN(F("  [  ST1  ] [  HXL  HXH  HYL  HYH  HZL  HZH  ] [  ST2  ]"));

    for (int i = 0; i < 10; i++) {
        uint8_t mag[8];
        _burstRead(MPU_REG_EXT_SENS_DATA, mag, 8);

        bool drdy = (mag[0] & 0x01);
        bool hofl = (mag[7] & 0x08);

        int16_t rx = (int16_t)((mag[2] << 8) | mag[1]);
        int16_t ry = (int16_t)((mag[4] << 8) | mag[3]);
        int16_t rz = (int16_t)((mag[6] << 8) | mag[5]);

        float mx = rx * MAG_SCALE_16BIT * cal.mag_asa_x;
        float my = ry * MAG_SCALE_16BIT * cal.mag_asa_y;
        float mz = rz * MAG_SCALE_16BIT * cal.mag_asa_z;

        DBG_PRINTF("  [0x%02X %s] [%6d %6d %6d]  ST2=0x%02X %s  → %.1f %.1f %.1f µT\n",
                      mag[0], drdy ? "DRDY" : "    ",
                      rx, ry, rz,
                      mag[7], hofl ? "HOFL" : "    ",
                      mx, my, mz);

        delay(15);
    }

    DBG_PRINTLN(F("\n[DIAG] SLV0 config registers:"));
    DBG_PRINTF("  SLV0_ADDR(0x25) = 0x%02X (should be 0x8C = AK8963|READ)\n", _readReg(MPU_REG_I2C_SLV0_ADDR));
    DBG_PRINTF("  SLV0_REG (0x26) = 0x%02X (should be 0x02 = ST1)\n",       _readReg(MPU_REG_I2C_SLV0_REG));
    DBG_PRINTF("  SLV0_CTRL(0x27) = 0x%02X (should be 0x88 = EN|8bytes)\n", _readReg(MPU_REG_I2C_SLV0_CTRL));
    DBG_PRINTF("  USER_CTRL(0x6A) = 0x%02X (bit5=I2C_MST_EN should be 1)\n", _readReg(MPU_REG_USER_CTRL));
    DBG_PRINTF("  I2C_MST_CTRL(0x24) = 0x%02X\n",                            _readReg(MPU_REG_I2C_MST_CTRL));
    DBG_PRINTF("  I2C_MST_STATUS(0x36) = 0x%02X\n",                          _readReg(MPU_REG_I2C_MST_STATUS));
}
