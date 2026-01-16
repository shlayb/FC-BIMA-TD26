#pragma once

#include <Arduino.h>

#include "I2CDev.h"
// #include "TeensyThreads.h"

#define MS5611_ADDRESS (0x77)

#define MS5611_CMD_ADC_READ (0x00)
#define MS5611_CMD_RESET (0x1E)
#define MS5611_CMD_CONV_D1 (0x40)
#define MS5611_CMD_CONV_D2 (0x50)
#define MS5611_CMD_READ_PROM (0xA2)

typedef enum {
    MS5611_ULTRA_HIGH_RES = 0x08,
    MS5611_HIGH_RES = 0x06,
    MS5611_STANDARD = 0x04,
    MS5611_LOW_POWER = 0x02,
    MS5611_ULTRA_LOW_POWER = 0x00
} osr_t;

uint16_t fc[6];
uint8_t ct;
uint8_t uosr;
int32_t TEMP2;
int64_t OFF2, SENS2;
volatile uint32_t raw_pressure, raw_temperature;

void setOversampling(osr_t osr = MS5611_ULTRA_HIGH_RES) {
    switch (osr) {
        case MS5611_ULTRA_LOW_POWER:
            ct = 1;
            break;
        case MS5611_LOW_POWER:
            ct = 2;
            break;
        case MS5611_STANDARD:
            ct = 3;
            break;
        case MS5611_HIGH_RES:
            ct = 5;
            break;
        case MS5611_ULTRA_HIGH_RES:
            ct = 10;
            break;
    }
    uosr = osr;
}

void reset() {
    _I2C1.sendByte(MS5611_ADDRESS, MS5611_CMD_RESET);
}

void readPROM() {
    for (uint8_t offset = 0; offset < 6; offset++) {
        _I2C1.readWord(MS5611_ADDRESS, MS5611_CMD_READ_PROM + (offset * 2), &fc[offset]);
    }
}

uint32_t readRawPressure() {
    _I2C1.sendByte(MS5611_ADDRESS, MS5611_CMD_CONV_D1 + uosr);
    // threads.delay(ct);
    delay(ct);
    uint32_t raw_press;
    _I2C1.readWords(MS5611_ADDRESS, MS5611_CMD_ADC_READ, 3, &raw_press);
    return raw_press;
}

uint32_t readRawTemperature() {
    _I2C1.sendByte(MS5611_ADDRESS, MS5611_CMD_CONV_D2 + uosr);
    // threads.delay(ct);
    delay(ct);
    uint32_t raw_temp;
    _I2C1.readWords(MS5611_ADDRESS, MS5611_CMD_ADC_READ, 3, &raw_temp);
    return raw_temp;
}

void readRawBaro() {
    raw_pressure = readRawPressure();
    raw_temperature = readRawTemperature();
}