#ifndef GPS_TELEM_H
#define GPS_TELEM_H

#include "gps.h" 
#include "Barometer.h"

class GPSTelemetry {
   public:
    void send_gps_to_rpi(); 
};

void GPSTelemetry::send_gps_to_rpi() {
    Serial1.print("LAT:");
    Serial1.print(gepees.latitude, 6);
    Serial1.print(", LNG:");
    Serial1.print(gepees.longitude, 6);
    Serial1.print(", Alt:");
    Serial1.print(baro.altitude);
    Serial1.print(", Roll:");
    Serial1.print(imu.roll);
    Serial1.print(", Pitch:");
    Serial1.print(imu.pitch);
    Serial1.print(", Yaw:");
    Serial1.print(imu.yaw);
    Serial1.print(", Head:");
    Serial1.print(imu.heading);
    Serial1.print(", WP:");
    Serial1.print(flag_wp);
    Serial1.print(", Mode:");
    Serial1.print(modeCode4(current));
    Serial1.println();
}

extern GPSTelemetry gps_telem;
extern Gepees gepees;
extern Barometer baro;

#endif
