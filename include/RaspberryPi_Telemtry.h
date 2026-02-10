#ifndef GPS_TELEM_H
#define GPS_TELEM_H

#include "gps.h" 
#include "Barometer.h"

class GPSTelemetry {
   public:
    void send_gps_to_rpi(); 
};

void GPSTelemetry::send_gps_to_rpi() {
    Serial7.print("LAT:");
    Serial7.print(gepees.latitude, 6);
    Serial7.print(", LNG:");
    Serial7.print(gepees.longitude, 6);
    Serial7.print(", Alt:");
    Serial7.print(baro.altitude);
    Serial7.print(", Roll:");
    Serial7.print(imu.roll);
    Serial7.print(", Pitch:");
    Serial7.print(imu.pitch);
    Serial7.print(", Yaw:");
    Serial7.print(imu.yaw);
    Serial7.print(", Head:");
    Serial7.print(imu.heading);
    Serial7.print(", WP:");
    Serial7.print(flag_wp);
    Serial7.print(", Mode:");
    Serial7.print(modeCode4(current));
    Serial7.println();
}

extern GPSTelemetry gps_telem;
extern Gepees gepees;
extern Barometer baro;

#endif
