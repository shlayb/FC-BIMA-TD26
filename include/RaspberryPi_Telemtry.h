#ifndef GPS_TELEM_H
#define GPS_TELEM_H

#include "gps.h" 

class GPSTelemetry {
   public:
    void send_gps_to_rpi(); 
};

void GPSTelemetry::send_gps_to_rpi() {
    Serial3.print("LAT:");
    Serial3.print(gepees.latitude, 6);
    Serial3.print(" LNG:");
    Serial3.print(gepees.longitude, 6);
    Serial3.println();
}

extern GPSTelemetry gps_telem;
extern Gepees gepees;

#endif
