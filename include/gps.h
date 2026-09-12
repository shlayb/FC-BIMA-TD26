/**
 * @file gps.h
 * @brief  Provides functionality to interface with a GPS module using native UBX
 *         protocol (SparkFun u-blox GNSS Arduino Library), reading NAV-PVT directly.
 */
#pragma once

#include <AP_Math.h>
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

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
// Holybro M10: pin 28/29 Teensy 4.1 adalah RX7/TX7 (hardware UART, Serial7),
// jadi dipakai langsung sebagai HardwareSerial (bukan SoftwareSerial) supaya
// tidak drop byte di baud rate tinggi. Protokol asli modul ini UBX (binary),
// jadi dibaca langsung lewat NAV-PVT (bukan parsing teks NMEA) — checksum
// UBX (Fletcher 16-bit) jauh lebih kuat dari checksum NMEA (XOR 1 byte).
class Gepees {
   public:
    Gepees(HardwareSerialIMXRT &serial, uint32_t baudRate)
        : gps_status(GPS_NO_GPS), latitude(0), longitude(0), hdop(0), satellites(0), speed_mps(0), velocity(), age(0), gpsSerial(serial), baudRate(baudRate), alt_m(0), alt_cm(0), speed_kmph(0), course(0) {}

    void initGPS();    // Inisialisasi GPS
    void updateGPS();  // Memperbarui data GPS

    int line = 0;

    // Info diagnostik — dibaca oleh Mavlink.h buat ditampilkan sebagai
    // STATUSTEXT ke GCS (jangan pakai Serial.print, USB serial dipakai buat
    // MAVLink jadi tidak kebaca sebagai teks biasa di GCS).
    bool ubx_handshake_ok = false;
    uint32_t ubxPacketsOk = 0;   // jumlah NAV-PVT valid yang sudah diterima
    uint8_t fixType = 0;        // fixType mentah dari modul (0=no,2=2D,3=3D,4=GNSS+DR)

    StatusGPS gps_status;
    float latitude;
    float longitude;
    float hdop;
    float satellites;
    float speed_mps;
    Vector3f velocity;
    uint32_t age;
    HardwareSerialIMXRT &gpsSerial;
    uint32_t baudRate;
    float alt_m;
    int32_t alt_cm;
    float speed_kmph;
    float course;
    SFE_UBLOX_GNSS gnss;

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
    uint32_t lastFixMillis = 0;
    uint8_t candidateIndex = 0;
    uint32_t lastAttemptMillis = 0;

    void checkStatus();      // Mengecek status GPS dari fixType/satellites/hdop
    void tryAutoDetect();    // Satu langkah percobaan baud/handshake UBX, non-blocking terhadap task lain
};

void Gepees::checkStatus() {
    if (ubxPacketsOk == 0) {
        gps_status = GPS_NO_GPS;
        return;
    }

    if (fixType < 2 || satellites < 4) {
        gps_status = GPS_NO_FIX;
    } else if (fixType >= 3 && satellites >= 6 && hdop < 2.5f) {
        gps_status = GPS_FIX_3D;
    } else {
        gps_status = GPS_FIX_2D;
    }
}

void Gepees::initGPS() {
    // Default buffer RX HardwareSerial Teensy cuma 64 byte. Di 115200 baud itu
    // penuh dalam ~5.5ms — kalau task FreeRTOS berprioritas lebih tinggi
    // (IMU/Control/Mavlink/Radio) menunda task GPS lebih lama dari itu,
    // sebagian byte GPS keburu ke-drop di level hardware. ArduPilot di
    // Pixhawk aman dari ini karena UART RX-nya lewat DMA + buffer besar.
    // Kita tiru dengan kasih Serial7 buffer RX tambahan.
    static uint8_t gpsRxExtra[512];
    gpsSerial.addMemoryForRead(gpsRxExtra, sizeof(gpsRxExtra));

    // PENTING: auto-baud/handshake UBX TIDAK dilakukan di sini lagi.
    // initGPS() dipanggil synchronous di setup() SEBELUM FreeRTOS scheduler
    // jalan penuh — kalau di sini kita blocking beberapa detik (nyoba 6
    // kandidat baud), itu menunda SELURUH sistem (buzzer startup, heartbeat
    // Mavlink, dll) ikut telat/kacau. Auto-baud sekarang jalan bertahap di
    // updateGPS() (dipanggil dari GPS task sendiri), supaya yang ketunda
    // cuma task GPS itu — task lain tetap jalan normal berkat preemptive
    // scheduling FreeRTOS.
    gpsSerial.begin(baudRate);
}

void Gepees::tryAutoDetect() {
    static const uint32_t candidates[] = {230400};
    static const uint8_t numCandidates = sizeof(candidates) / sizeof(candidates[0]);
    static const uint32_t ATTEMPT_INTERVAL_MS = 1000;  // jeda 1s antar percobaan baud

    uint32_t now = millis();
    if (now - lastAttemptMillis < ATTEMPT_INTERVAL_MS) return;
    lastAttemptMillis = now;

    uint32_t candidate = candidates[candidateIndex];
    gpsSerial.begin(candidate);
    vTaskDelay(pdMS_TO_TICKS(10));
    while (gpsSerial.available()) gpsSerial.read();

    if (gnss.begin(gpsSerial, 50)) {
        baudRate = candidate;
        ubx_handshake_ok = true;
        gnss.setUART1Output(COM_TYPE_UBX);  // matikan NMEA, murni UBX
        gnss.setAutoPVT(true);              // NAV-PVT mengalir otomatis di nav rate modul
    } else {
        candidateIndex = (candidateIndex + 1) % numCandidates;
    }
}

void Gepees::updateGPS() {
    if (!ubx_handshake_ok) {
        tryAutoDetect();
        return;
    }

    // checkUblox() non-blocking: proses byte yang sudah masuk buffer,
    // tidak menunggu. getPVT() dengan autoPVT aktif juga non-blocking —
    // cuma balikin true kalau ada data NAV-PVT baru sejak panggilan terakhir.
    gnss.checkUblox();

    if (!gnss.getPVT()) {
        if (lastFixMillis > 0) age = millis() - lastFixMillis;
        return;
    }

    ubxPacketsOk++;
    fixType = gnss.getFixType();
    satellites = (float)gnss.getSIV();
    hdop = (float)gnss.getPDOP() / 100.0f;  // pDOP dari modul, dipakai sebagai proxy hdop
    latitude = (float)gnss.getLatitude() / 1e7f;
    longitude = (float)gnss.getLongitude() / 1e7f;

    int32_t altMM = gnss.getAltitudeMSL();
    alt_m = altMM / 1000.0f;
    alt_cm = altMM / 10;

    int32_t gSpeedMMs = gnss.getGroundSpeed();
    speed_mps = gSpeedMMs / 1000.0f;
    speed_kmph = speed_mps * 3.6f;
    course = (float)gnss.getHeading() / 1e5f;

    velocity.x = gnss.getNedNorthVel() / 1000.0f;
    velocity.y = gnss.getNedEastVel() / 1000.0f;
    velocity.z = gnss.getNedDownVel() / 1000.0f;

    lastFixMillis = millis();
    age = 0;
    line++;

    checkStatus();
}
