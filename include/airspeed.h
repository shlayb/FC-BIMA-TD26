#pragma once

#include <Arduino.h>
#include <TeensyThreads.h>
#include <Wire.h>
#include <stdint.h>
#include <Filter.h>

// MS4525DO sensor characteristic (14-bit ADC, 10%-90% range for -1 to +1 PSI differential)
const uint8_t MS4525DAddress = 0x28;
const float MS4525FullScaleRange = 1.0f;  // 1 PSI differential
const float MS4525ZeroCounts = 8192.0f;   // 50% counts for 0 differential pressure
const float MS4525ScaleFactor = 6553.6f;  // 80% span / 2 = 13107.2 / 2 = 6553.6 counts per PSI

KalmanFilter kalmanFilter;

class Airspeed {
public:
    Airspeed() : press_psi(0), psi_offset(0), v_ms(0), v_kmh(0), prspsi(0), init_press(20.0f), stats(0xFF) {}

    void initAirspeed();
    void updateAirspeed();
    void airspeedThread();
    void calibrateAirspeed();

    float getVelocityMS() const { return v_ms; }
    float getVelocityKMH() const { return v_kmh; }
    float getPressurePSI() const { return press_psi; }

    float press_psi, psi_offset, v_ms, v_kmh, prspsi;
private:
    byte stats;
    volatile uint16_t press_data, temp_data;
    float init_press;

    byte getRawAirspeed();
};

// Mendapatkan data mentah dari sensor via I2C Wire2
byte Airspeed::getRawAirspeed() {
    uint8_t bytesReceived = Wire2.requestFrom(MS4525DAddress, (uint8_t)4);
    if (bytesReceived < 4) {
        return 0xFF; // I2C read error
    }

    byte press_h = Wire2.read();
    byte press_l = Wire2.read();
    byte temp_h  = Wire2.read();
    byte temp_l  = Wire2.read();

    byte status = (press_h >> 6) & 0x03;
    press_h = press_h & 0x3f;
    press_data = (((uint16_t)press_h) << 8) | press_l;

    temp_l = (temp_l >> 5);
    temp_data = (((uint16_t)temp_h) << 3) | temp_l;

    return status;
}

// Kalibrasi offset nol sensor pitot tube
void Airspeed::calibrateAirspeed() {
    float sum_psi = 0.0f;
    int valid_samples = 0;

    for (int i = 0; i < 50; i++) {
        byte s = getRawAirspeed();
        if (s != 0xFF) {
            float raw_psi = ((static_cast<float>(press_data) - MS4525ZeroCounts) / MS4525ScaleFactor) * MS4525FullScaleRange;
            sum_psi += raw_psi;
            valid_samples++;
        }
        delay(10);
    }

    if (valid_samples > 0) {
        psi_offset = sum_psi / static_cast<float>(valid_samples);
        Serial.print("[AIRSPEED] Calibrated offset PSI: ");
        Serial.print(psi_offset, 6);
        Serial.print(" from ");
        Serial.print(valid_samples);
        Serial.println(" samples");
    } else {
        Serial.println("[AIRSPEED] Calibration FAILED - No I2C response from MS4525D (Wire2)");
    }
}

// Inisialisasi sensor kecepatan angin
void Airspeed::initAirspeed() {
    Wire2.begin();
    delay(50);
    calibrateAirspeed();
    kalmanFilter.initializeKalman(4, 0.3, 0.1);
    randomSeed(A15);
}

// Memperbarui data kecepatan angin
void Airspeed::updateAirspeed() {
    stats = getRawAirspeed();
    if (stats == 0xFF) {
        // Sensor tidak merespons di I2C Wire2
        return;
    }

    float raw_psi = ((static_cast<float>(press_data) - MS4525ZeroCounts) / MS4525ScaleFactor) * MS4525FullScaleRange;
    float diff_psi = raw_psi - psi_offset;

    // Tekanan diferensial positif menandakan aliran udara maju
    if (diff_psi > 0.0f) {
        press_psi = diff_psi;
        v_ms = sqrt((press_psi * 13789.5144f) / 1.225f);
    } else {
        press_psi = 0.0f;
        v_ms = 0.0f;
    }

    // Filter jika diperlukan: v_ms = kalmanFilter.kalmanFilter(v_ms);
    v_kmh = v_ms * 3.6f;
    prspsi = press_psi * 13789.5144f;
}


