#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP5xx.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Filter.h"
#include <Filter/Filter.h>

// Virtual Temperature Calculator (NOAA):
// https://www.weather.gov/epz/wxcalc_virtualtemperature

#define BMP581_I2C_ADDR_SEC  (0x46) // Default SDO = GND
#define BMP581_I2C_ADDR_PRIM (0x47) // SDO = VDDIO
#define SEALEVELPRESSURE_HPA (1013.25f)

LowPassFilterFloat lpfbaro;
DerivativeFilterFloat_Size7 _climb_rate_filter;
KalmanFilter kalman;

class Barometer {
public:
    bool init_baro();
    void thread_baro();
    void update_baro();
    float altitude = 0.0f;
    float get_climb_rate();
    float getEAS2TAS();

    void readPressure(bool compensation = true);
    void readTemperature(bool compensation = true);
    void determineGroundPress(uint8_t n);
    
    double get_pressure() const { return pressure; }           // Satuan: Pascal (Pa)
    double get_temperature() const { return temperature; }     // Satuan: Celsius (°C)
    float get_ground_pressure() const { return ground_pressure; } // Satuan: Pascal (Pa)
    
private:
    Adafruit_BMP5xx bmp;
    double sea_level = 101325.0;
    double pressure = 101325.0;        // Pa
    double temperature = 25.0;         // °C

    float source = 1013.25;
    float ground_pressure = 101325.0f; // Pa
    float ground_altitude = 0.0f;
    float virtual_temp = 0;            // Kelvin
    float _last_altitude_EAS2TAS = 0;
    float _last_alt = 0;
    float _EAS2TAS = 1.0f;
    float highest_altitude = 0;
    bool is_reach_approach = false;

    const float ALT_DROP_THRESHOLD = 0.001;

    void hypsometricEquation(float current_press);
    void calcAltitude(double press, double reference_press);
    void calcSeaLevel(double press, double relative_alt);
    bool reached_approach_alt(float current_alt);
};

void Barometer::readPressure(bool compensation) {
    if (bmp.performReading()) {
        pressure = bmp.pressure * 100.0; // hPa -> Pa
        temperature = bmp.temperature;
    }
}

void Barometer::readTemperature(bool compensation) {
    if (bmp.performReading()) {
        pressure = bmp.pressure * 100.0; // hPa -> Pa
        temperature = bmp.temperature;
    }
}

void Barometer::determineGroundPress(uint8_t n) {
    uint8_t i;

    // Flush a few initial readings
    for (i = 0; i < 5; i++) {
        readPressure(false);
        delay(20);
    }
    
    Serial.println("Computing ground-level pressure (BMP581)...");
    float sum_press = 0.0f;
    uint8_t valid_cnt = 0;
    for (i = 0; i < n; i++) {
        if (bmp.performReading()) {
            sum_press += (bmp.pressure * 100.0f); // Pa
            valid_cnt++;
        }
        delay(20);
    }
    if (valid_cnt > 0) {
        ground_pressure = sum_press / (float)valid_cnt;
        pressure = ground_pressure;
        temperature = bmp.temperature;
    }
    Serial.printf("[BARO] Ground pressure set to: %.2f hPa\n", ground_pressure / 100.0f);
}

// https://en.wikipedia.org/wiki/Hypsometric_equation
void Barometer::hypsometricEquation(float current_press) {
    float R = 287.0f;                                                            // Gas const. [J / kg.K]
    float g = 9.81f;                                                             // Grav. accel. [m/s/s]
    altitude = ((R * virtual_temp) / g) * log(ground_pressure / current_press);  // [m]
}

void Barometer::calcAltitude(double press, double reference_press) {
    if (reference_press > 0.0) {
        altitude = (44330.0f * (1.0f - pow((double)press / (double)reference_press, 0.1902949f)));
    }
}

void Barometer::calcSeaLevel(double press, double relative_alt) {
    sea_level = ((double)press / pow(1.0f - ((double)relative_alt / 44330.0f), 5.255f));
}

// validation : https://keisan.casio.com/exec/system/1224575267
float Barometer::getEAS2TAS() {
    if ((fabsf(altitude - _last_altitude_EAS2TAS) < 25.0f) && _EAS2TAS != 0) {
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

// Utility: return 0 if value is NaN/Inf, otherwise return value
static inline float baro_safe_value(float v) {
    if (isnan(v) || isinf(v)) return 0.0f;
    return v;
}

bool Barometer::init_baro() {
    Wire1.begin();
    Wire1.setClock(400000);
    kalman.initializeKalman(5, 0.1, 1);
    
    bool ok = bmp.begin(BMP581_I2C_ADDR_SEC, &Wire1); // Coba alamat 0x46
    if (!ok) {
        Serial.println("BMP581 init on 0x46 failed, trying 0x47...");
        delay(50);
        ok = bmp.begin(BMP581_I2C_ADDR_PRIM, &Wire1); // Coba alamat 0x47
        if (!ok) {
            Serial.println("[ERROR] BMP581 sensor NOT detected!");
            return false;
        }
    }
    Serial.println("[OK] BMP581 sensor initialized successfully.");

    /* --- Konfigurasi Filter & Oversampling --- */
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_32X);
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
    bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);

    delay(50);
    determineGroundPress(15);

    calcAltitude(pressure, ground_pressure);
    ground_altitude = altitude;
    calcSeaLevel(pressure, ground_altitude);

    return true;
}

void Barometer::update_baro() {
    if (bmp.performReading()) {
        pressure = bmp.pressure * 100.0; // hPa -> Pa
        temperature = bmp.temperature;

        if (ground_pressure > 0.0f) {
            calcAltitude(pressure, ground_pressure);
            calcSeaLevel(pressure, altitude);

            // lpfbaro.apply(altitude, 0.02f); 
            altitude = kalman.kalmanFilter(altitude);
            altitude = baro_safe_value(altitude);
        }
    }
}

extern Barometer baro;
inline bool zeroAltitudePreflight() {
    baro.determineGroundPress(15);
    return true;
}