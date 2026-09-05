/**
 * @file ist8310.h
 * @brief Minimal I2C driver for the IST8310 3-axis magnetometer, used as the
 *        onboard compass on the Holybro M10 GPS module (SDA/SCL pins).
 *
 * Register map and default config values follow the commonly published
 * IST8310 driver implementations (PX4/Betaflight). Mounting orientation and
 * hard/soft-iron calibration are NOT applied here — this driver only exposes
 * raw calibrated-scale readings in micro-Tesla.
 */
#pragma once

#include <Arduino.h>
#include "I2CDev.h"

#define IST8310_ADDRESS      0x0E  // default 7-bit address on Holybro M10 GPS module

#define IST8310_REG_WAI      0x00
#define IST8310_WAI_VALUE    0x10

#define IST8310_REG_STAT1    0x02
#define IST8310_STAT1_DRDY   0x01

#define IST8310_REG_DATA_X_L 0x03

#define IST8310_REG_CNTL1    0x0A
#define IST8310_CNTL1_STANDBY 0x00
#define IST8310_CNTL1_SINGLE  0x01

#define IST8310_REG_CNTL2    0x0B
#define IST8310_CNTL2_SRST   0x01

#define IST8310_REG_AVGCNTL  0x41
#define IST8310_REG_PDCNTL   0x42

#define IST8310_UT_PER_LSB   0.3f  // micro-Tesla per LSB

class IST8310 {
   public:
    IST8310(uint8_t address = IST8310_ADDRESS) : addr(address), i2c(_I2C2) {}

    bool init() {
        i2c.init(400000);

        i2c.writeByte(addr, IST8310_REG_CNTL2, IST8310_CNTL2_SRST);
        delay(10);

        uint8_t whoami = 0;
        i2c.readByte(addr, IST8310_REG_WAI, &whoami);
        if (whoami != IST8310_WAI_VALUE) {
            connected = false;
            return false;
        }

        i2c.writeByte(addr, IST8310_REG_AVGCNTL, 0x24);  // 16x sample averaging (typical)
        i2c.writeByte(addr, IST8310_REG_PDCNTL, 0xC0);   // recommended pulse duration

        connected = true;
        startMeasurement();
        return true;
    }

    // Poll periodically (e.g. 20-50 Hz). Reads the previous measurement when
    // ready and immediately triggers the next single-shot measurement.
    bool update() {
        if (!connected) return false;

        uint8_t stat1 = 0;
        i2c.readByte(addr, IST8310_REG_STAT1, &stat1);
        if (!(stat1 & IST8310_STAT1_DRDY)) return false;

        uint8_t raw[6];
        i2c.readBytes(addr, IST8310_REG_DATA_X_L, 6, raw);

        int16_t raw_x = (int16_t)((raw[1] << 8) | raw[0]);
        int16_t raw_y = (int16_t)((raw[3] << 8) | raw[2]);
        int16_t raw_z = (int16_t)((raw[5] << 8) | raw[4]);

        mag_x_ut = raw_x * IST8310_UT_PER_LSB;
        mag_y_ut = raw_y * IST8310_UT_PER_LSB;
        mag_z_ut = raw_z * IST8310_UT_PER_LSB;

        startMeasurement();
        return true;
    }

    bool isConnected() const { return connected; }
    float getMagX() const { return mag_x_ut; }
    float getMagY() const { return mag_y_ut; }
    float getMagZ() const { return mag_z_ut; }

   private:
    uint8_t addr;
    I2C &i2c;
    bool connected = false;
    float mag_x_ut = 0, mag_y_ut = 0, mag_z_ut = 0;

    void startMeasurement() {
        i2c.writeByte(addr, IST8310_REG_CNTL1, IST8310_CNTL1_SINGLE);
    }
};
