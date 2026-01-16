#pragma once

#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"

#include "Drivers/MS5611.h"
#include "Filter.h"
#include <Filter/Filter.h>

// Virtual Temperature Calculator (NOAA):
// https://www.weather.gov/epz/wxcalc_virtualtemperature


LowPassFilterFloat lpfbaro;
DerivativeFilterFloat_Size7 _climb_rate_filter;

KalmanFilter kalman;

class Barometer{
public:
    bool init_baro();
    void thread_baro();
    void update_baro();
    float altitude;
    float get_climb_rate();
    float getEAS2TAS();

    void readPressure(bool compensation);
    void readTemperature(bool compensation);
    
private:
    double sea_level;
    double pressure;
    double temperature;

    float source = 1013.25;
    float ground_pressure;   // Pa
    float ground_altitude;   // Pa
    float virtual_temp = 0;  // Kelvin
    float _last_altitude_EAS2TAS;
    float _last_alt;
    float _EAS2TAS;
    float highest_altitude = 0;
    bool is_reach_approach = false;

    const float ALT_DROP_THRESHOLD = 0.001;

    void determineGroundPress(uint8_t n);
    void hypsometricEquation(float current_press);
    void calcAltitude(double press, double reference_press);
    void calcSeaLevel(double press, double relative_alt);
    bool reached_approach_alt(float current_alt);
};

void Barometer::readPressure(bool compensation) {
    uint32_t D1 = raw_pressure;
    uint32_t D2 = raw_temperature;

    int32_t dT = D2 - (uint32_t)fc[4] * 256;
    int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
    int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

    if (compensation) {
        int32_t TEMP = 2000 + ((int64_t)dT * fc[5]) / 8388608;

        OFF2 = 0;
        SENS2 = 0;

        if (TEMP < 2000) {
            OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
            SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
        }

        if (TEMP < -1500) {
            OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
        }

        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    pressure = (D1 * SENS / 2097152 - OFF) / 32768;
}

void Barometer::readTemperature(bool compensation) {
    uint32_t D2 = raw_temperature;
    int32_t dT = D2 - (uint32_t)fc[4] * 256;
    int32_t TEMP = 2000 + ((int64_t)dT * fc[5]) / 8388608;

    TEMP2 = 0;

    if (compensation) {
        if (TEMP < 2000) {
            TEMP2 = (dT * dT) / pow(2, 31);
        }
    }

    TEMP = TEMP - TEMP2;
    temperature = ((double)TEMP / 100);
}

void Barometer::determineGroundPress(uint8_t n) {
    uint8_t i;

    // Perform a few readings to 'flush' bad data from the sensor
    for (i = 0; i < 5; i++) {
        readRawBaro();
        readPressure(false);
        // vTaskDelay(pdMS_TO_TICKS(100));
        delay(100);
    }
    // Perform 'n' readings and determine the average pressure
    Serial.println("Computing ground-level pressure...");
    ground_pressure = 0.0f;
    for (i = 0; i < n; i++) {
        readRawBaro();
        readPressure(false);
        ground_pressure += pressure;
        // vTaskDelay(pdMS_TO_TICKS(100));
        delay(100);
    }
    ground_pressure /= (float)n;
}

// https://en.wikipedia.org/wiki/Hypsometric_equation
void Barometer::hypsometricEquation(float current_press) {
    float R = 287.0f;                                                            // Gas const. [J / kg.K]
    float g = 9.81f;                                                             // Grav. accel. [m/s/s]
    altitude = ((R * virtual_temp) / g) * log(ground_pressure / current_press);  // [m]
}

void Barometer::calcAltitude(double press, double reference_press) {
    //* HYPESOMETRIC FORMULA 1
    // altitude = ((powf(source / ((float)pressure / 100.0), 0.19022256) - 1.0) * ((float)temperature / 100 + 273.15)) / 0.0065;

    //* FORMULA
    altitude = (44330.0f * (1.0f - pow((double)pressure / (double)reference_press, 0.1902949f)));
}

void Barometer::calcSeaLevel(double press, double relative_alt) {
    sea_level = ((double)press / pow(1.0f - ((double)relative_alt / 44330.0f), 5.255f));
}

// validation : https://keisan.casio.com/exec/system/1224575267
float Barometer::getEAS2TAS() {
    if ((fabsf(altitude - _last_altitude_EAS2TAS) < 25.0f) && _EAS2TAS != 0) {
        // not enough change to require re-calculating
        return _EAS2TAS;
    }
    float tempK = temperature + C_TO_KELVIN - ISA_LAPSE_RATE * altitude;
    const float eas2tas_squared = SSL_AIR_DENSITY / ((float)pressure / (ISA_GAS_CONSTANT * tempK));
    if (eas2tas_squared < 0) {
        return 1.0;
    }
    _EAS2TAS = sqrtf(eas2tas_squared);
    _last_altitude_EAS2TAS = altitude;
    return _EAS2TAS;
}

float Barometer::get_climb_rate() {
    _climb_rate_filter.update(altitude, millis());
    return _climb_rate_filter.slope() * 1.0e3f;
}

bool Barometer::reached_approach_alt(float current_alt) {
    if (current_alt > highest_altitude) {
        highest_altitude = current_alt;
    }

    float altitude_drop = highest_altitude - current_alt;
    if (!is_reach_approach && altitude_drop >= ALT_DROP_THRESHOLD) {
        is_reach_approach = true;
        return true;
    }

    return false;
}

bool Barometer::init_baro() {
    _I2C1.init();
    kalman.initializeKalman(5, 0.12, 0.8);
    reset();
    setOversampling();
    // vTaskDelay(pdMS_TO_TICKS(100));
    delay(100);
    readPROM();
    determineGroundPress(15);

    calcAltitude(pressure, ground_pressure);
    ground_altitude = altitude;
    calcSeaLevel(pressure, ground_altitude);

    return true;
}


void Barometer::update_baro() {
    readRawBaro();
    readTemperature(true);
    readPressure(true);

    calcAltitude(pressure, ground_pressure);
    calcSeaLevel(pressure, altitude);

    // lpfbaro.apply(altitude, 0.02f); 
    altitude = kalman.kalmanFilter(altitude);
}

