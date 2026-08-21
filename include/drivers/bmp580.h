#pragma once

#include <Arduino.h>
#include "I2CDev.h"

// Alamat I2C BMP580 / BMP581
#define BMP580_I2C_ADDR_PRIM    (0x47) // SDO = VDDIO / High (Default)
#define BMP580_I2C_ADDR_SEC     (0x46) // SDO = GND / Low

// Register Definitions
#define BMP580_REG_CHIP_ID      (0x01) // Expected: 0x50 (BMP580) atau 0x51 (BMP581)
#define BMP580_REG_STATUS       (0x27)
#define BMP580_REG_PRESS_DATA_0 (0x1D) // XLSB
#define BMP580_REG_PRESS_DATA_1 (0x1E) // LSB
#define BMP580_REG_PRESS_DATA_2 (0x1F) // MSB
#define BMP580_REG_TEMP_DATA_0  (0x20) // XLSB
#define BMP580_REG_TEMP_DATA_1  (0x21) // LSB
#define BMP580_REG_TEMP_DATA_2  (0x22) // MSB
#define BMP580_REG_OSR_CONFIG   (0x36)
#define BMP580_REG_ODR_CONFIG   (0x37)
#define BMP580_REG_DSP_CONFIG   (0x30)
#define BMP580_REG_DSP_IIR      (0x31)
#define BMP580_REG_CMD          (0x7E)

// Command Definitions
#define BMP580_CMD_SOFT_RESET   (0xB6)

// Chip IDs
#define BMP580_CHIP_ID_VAL      (0x50)
#define BMP581_CHIP_ID_VAL      (0x51)

class BMP580_Driver {
public:
    uint8_t i2c_addr = BMP580_I2C_ADDR_PRIM;
    bool is_connected = false;

    volatile float raw_pressure = 0.0f;    // Pascal (Pa)
    volatile float raw_temperature = 0.0f; // Celsius (°C)

    bool init(uint8_t addr = BMP580_I2C_ADDR_PRIM) {
        i2c_addr = addr;
        uint8_t chip_id = 0;

        // Cek alamat primer (0x47), fallback ke sekunder (0x46) jika gagal
        _I2C1.readByte(i2c_addr, BMP580_REG_CHIP_ID, &chip_id);
        if (chip_id != BMP580_CHIP_ID_VAL && chip_id != BMP581_CHIP_ID_VAL) {
            uint8_t alt_addr = (i2c_addr == BMP580_I2C_ADDR_PRIM) ? BMP580_I2C_ADDR_SEC : BMP580_I2C_ADDR_PRIM;
            _I2C1.readByte(alt_addr, BMP580_REG_CHIP_ID, &chip_id);
            if (chip_id == BMP580_CHIP_ID_VAL || chip_id == BMP581_CHIP_ID_VAL) {
                i2c_addr = alt_addr;
            } else {
                is_connected = false;
                return false;
            }
        }

        // 1. Soft Reset
        _I2C1.writeByte(i2c_addr, BMP580_REG_CMD, BMP580_CMD_SOFT_RESET);
        delay(20);

        // 2. Konfigurasi OSR (Oversampling):
        // OSR_T = 8x (011b = 3), OSR_P = 32x (101b = 5 << 3 = 0x28), PRESS_EN = 1 (1 << 6 = 0x40)
        // Nilai: 0x40 | 0x28 | 0x03 = 0x6B
        _I2C1.writeByte(i2c_addr, BMP580_REG_OSR_CONFIG, 0x6B);

        // 3. Konfigurasi ODR (Output Data Rate & Mode):
        // Power Mode = Normal (01b = 0x01), ODR = 50Hz (01111b = 15 << 2 = 60 / 0x3C)
        // Nilai: 0x3C | 0x01 = 0x3D
        _I2C1.writeByte(i2c_addr, BMP580_REG_ODR_CONFIG, 0x3D);

        // 4. Konfigurasi IIR Filter (Coeff 3):
        _I2C1.writeByte(i2c_addr, BMP580_REG_DSP_IIR, 0x12);
        delay(10);

        is_connected = true;
        return true;
    }

    void read(float &press_out, float &temp_out) {
        uint8_t buffer[6] = {0};
        _I2C1.readBytes(i2c_addr, BMP580_REG_PRESS_DATA_0, 6, buffer);

        // Raw Pressure (24-bit unsigned, LSB first -> buffer[0]=XLSB, buffer[1]=LSB, buffer[2]=MSB)
        // Output Pascal = raw / 64.0
        uint32_t press_raw = ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[0];
        if (press_raw != 0 && press_raw != 0x7FFFFF) {
            raw_pressure = (float)press_raw / 64.0f;
            press_out = raw_pressure;
        }

        // Raw Temperature (24-bit signed -> buffer[3]=XLSB, buffer[4]=LSB, buffer[5]=MSB)
        // Output °C = raw / 65536.0
        int32_t temp_raw = (int32_t)(((uint32_t)buffer[5] << 16) | ((uint32_t)buffer[4] << 8) | (uint32_t)buffer[3]);
        if (temp_raw != 0 && temp_raw != 0x7FFFFF) {
            if (temp_raw & 0x800000) {
                temp_raw |= 0xFF000000; // Sign extend jika negatif
            }
            raw_temperature = (float)temp_raw / 65536.0f;
            temp_out = raw_temperature;
        }
    }
};

inline BMP580_Driver bmp580;
