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
    Serial7.print(" LNG:");
    Serial7.print(gepees.longitude, 6);
    Serial7.print(" Altitude:");
    Serial7.print(baro.altitude);
    Serial7.println();
}

extern GPSTelemetry gps_telem;
extern Gepees gepees;
extern Barometer baro;

#endif
