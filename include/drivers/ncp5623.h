#include <Wire.h>
#include <gps.h>
#include <AHRS.h>

#define NCP5623_ADDR 0x39

void ncp5623_set_color(uint8_t r, uint8_t g, uint8_t b) {
    // Driver NCP5623 menerima skala intensitas 0 - 31 (5-bit)
    Wire2.beginTransmission(NCP5623_ADDR);
    Wire2.write(0x20 | (15 & 0x1F)); // Set arus global (dimming)
    Wire2.endTransmission();

    // Set intensitas Red (Register 0x40)
    Wire2.beginTransmission(NCP5623_ADDR);
    Wire2.write(0x40 | (r & 0x1F));
    Wire2.endTransmission();

    // Set intensitas Green (Register 0x60)
    Wire2.beginTransmission(NCP5623_ADDR);
    Wire2.write(0x60 | (g & 0x1F));
    Wire2.endTransmission();

    // Set intensitas Blue (Register 0x80)
    Wire2.beginTransmission(NCP5623_ADDR);
    Wire2.write(0x80 | (b & 0x1F));
    Wire2.endTransmission();
}

void update_gps_led() {
    switch (gepees.getStatus()) {
        case GPS_NO_GPS:
        case GPS_NO_FIX:
            // Merah (R=31, G=0, B=0)
            ncp5623_set_color(31, 0, 0);
            break;
        case GPS_FIX_2D:
            // Kuning/Oranye (R=31, G=15, B=0)
            ncp5623_set_color(31, 15, 0);
            break;
        case GPS_FIX_3D:
            // Hijau (R=0, G=31, B=0) atau Biru (R=0, G=0, B=31)
            ncp5623_set_color(0, 31, 0);
            break;
    }
}