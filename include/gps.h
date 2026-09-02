/**
 * @file gps.h
 * @brief  Provides functionality to interface with a GPS module using the TinyGPSPlus library.
 */
#pragma once

#include <AP_Math.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

#include "Barometer.h"

Barometer baro;

// Enumerasi Status GPS sesuai standar MAVLink GPS_FIX_TYPE
enum StatusGPS {
    GPS_NO_GPS = 0, // Tidak ada data GPS mengalir
    GPS_NO_FIX = 1, // GPS terdeteksi tapi belum lock satelit (searching)
    GPS_FIX_2D = 2, // 2D Fix
    GPS_FIX_3D = 3  // 3D Fix
};

// Kelas GPS
class Gepees {
   public:
    Gepees(uint8_t rxPin, uint8_t txPin, uint32_t baudRate)
        : gps_status(GPS_NO_FIX), latitude(0), longitude(0), hdop(0), satellites(0), speed_mps(0), velocity(), age(0), gps(), ss(rxPin, txPin), baudRate(baudRate), alt_m(0), alt_cm(0), speed_kmph(0), course(0), prev_alt(0), prev_time(0) {}

    void initGPS();    // Inisialisasi GPS
    void updateGPS();  // Memperbarui data GPS

    int line = 0;

    StatusGPS gps_status;
    float latitude;
    float longitude;
    float hdop;
    float satellites;
    float speed_mps;
    Vector3f velocity;
    uint32_t age;
    TinyGPSPlus gps;
    SoftwareSerial ss;
    uint32_t baudRate;
    float alt_m;
    int32_t alt_cm;
    float speed_kmph;
    float course;

    float getLatitude() const { return latitude; }
    float getLongitude() const { return longitude; }
    float getAltitudeM() const { return alt_m; }
    int32_t getAltitudeCM() const { return alt_cm; }
    float getHDOP() const { return hdop; }
    float getSatellites() const { return satellites; }
    float getSpeedMPS() const { return speed_mps; }
    float getSpeedKMPH() const { return speed_kmph; }
    float getCourse() const { return course; }
    Vector3f getVelocity() const { return velocity; }
    StatusGPS getStatus() const { return gps_status; }

   private:
    float prev_alt;
    uint32_t prev_time;

    void parseLocation();      // Parsing data lokasi
    void parseSpeed();         // Parsing data kecepatan
    void parseSatDOP();        // Parsing HDOP dan satelit
    void calculateVelocity();  // Menghitung kecepatan VELNED
    void checkStatus();        // Mengecek status GPS
};

void Gepees::parseLocation() {
    if (gps.location.isValid() || gps.location.isUpdated()) {
        latitude = (float)gps.location.lat();
        longitude = (float)gps.location.lng();
    }

    if (gps.course.isValid() || gps.course.isUpdated()) {
        course = (float)gps.course.deg();
    }
}

void Gepees::parseSpeed() {
    if (gps.speed.isValid() || gps.speed.isUpdated()) {
        age = gps.speed.age();
        speed_kmph = (float)gps.speed.kmph();
        speed_mps = (float)gps.speed.mps();
    }
}

void Gepees::parseSatDOP() {
    if (gps.hdop.isValid() || gps.hdop.isUpdated()) {
        hdop = (float)gps.hdop.hdop();
    }

    if (gps.satellites.isValid() || gps.satellites.isUpdated()) {
        satellites = (float)gps.satellites.value();
    }
}

void Gepees::calculateVelocity() {
    if (gps.speed.isUpdated()) {
        float horizontal_speed = (float)gps.speed.mps();
        uint32_t curr_time = millis();
        float time_interval = (curr_time - prev_time) / 1000.0;  // Dalam detik
        prev_time = curr_time;

        velocity.x = horizontal_speed * cos(radians(course));
        velocity.y = horizontal_speed * sin(radians(course));
        velocity.z = (baro.altitude - prev_alt) / time_interval;
        prev_alt = baro.altitude;
    }
}

void Gepees::checkStatus() {
    if (gps.charsProcessed() < 10) {
        gps_status = GPS_NO_GPS;
        return;
    }

    if (!gps.location.isValid() || satellites < 4) {
        gps_status = GPS_NO_FIX;
    } else if (satellites >= 6 && hdop < 2.5f) {
        gps_status = GPS_FIX_3D;
    } else {
        gps_status = GPS_FIX_2D;
    }
}

void Gepees::initGPS() {
    ss.begin(baudRate);
    Serial.println(TinyGPSPlus::libraryVersion());
}

void Gepees::updateGPS() {
    // Serial.print(" gps ss:");
    // Serial.println(ss.available());
    // if (ss.available() > 0) {
    //     if (gps.encode(ss.read())) {
    //         parseLocation();
    //         parseSpeed();
    //         parseSatDOP();
    //         calculateVelocity();
    //         checkStatus();
    //         line++;
    //     }
    // }
    // for(int i = 0; i < 50; i++){
    //     if(ss.available() > 0){
    //         if (gps.encode(ss.read())) {
    //             parseLocation();
    //             parseSpeed();
    //             parseSatDOP();
    //             calculateVelocity();
    //             checkStatus();
    //             line++;
    //         }
    //     }
    // }
    while(ss.available() > 0){
        // baro.update_baro();
        if(ss.available() > 0){
            if (gps.encode(ss.read())) {
                parseLocation();
                parseSpeed();
                parseSatDOP();
                calculateVelocity();
                checkStatus();
                line++;
            }
        }
    };
}
