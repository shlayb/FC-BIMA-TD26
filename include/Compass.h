/**
 * @file Compass.h
 * @brief Wraps the IST8310 compass built into the Holybro M10 GPS module,
 *        wired via I2C (SDA/SCL -> Teensy 4.1 pin 25/24, Wire2).
 *
 * This is kept independent from AHRS's heading source (BNO055). It is
 * exposed for logging/telemetry and future fusion, not auto-applied to
 * imu.heading, since PCB mounting offset and hard/soft-iron calibration
 * have not been determined yet.
 */
#pragma once

#include <Arduino.h>
#include "drivers/ist8310.h"

class Compass {
   public:
    Compass() : sensor() {}

    void init_compass() {
        connected = sensor.init();
    }

    void update_compass() {
        if (sensor.update()) {
            mag_x = sensor.getMagX();
            mag_y = sensor.getMagY();
            mag_z = sensor.getMagZ();
            heading_deg = atan2f(mag_y, mag_x) * 180.0f / PI;
            if (heading_deg < 0) heading_deg += 360.0f;
        }
    }

    bool isConnected() const { return connected; }
    float getMagX() const { return mag_x; }
    float getMagY() const { return mag_y; }
    float getMagZ() const { return mag_z; }
    float getHeadingDeg() const { return heading_deg; }

   private:
    IST8310 sensor;
    bool connected = false;
    float mag_x = 0, mag_y = 0, mag_z = 0;
    float heading_deg = 0;
};
