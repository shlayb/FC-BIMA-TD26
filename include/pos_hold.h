#include <cmath>

// Radius bumi dalam sentimeter (R = 6.371 km = 6371000 meter = 637100000 cm)
const double EARTH_RADIUS_CM = 637100000.0;

// Fungsi Haversine: menghitung jarak antara 2 titik GPS dalam cm
double haversineDistanceCM(double lat1, double lon1, double lat2, double lon2) {
    // Konversi derajat ke radian
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    lon2 = lon2 * M_PI / 180.0;

    // Haversine formula
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;

    double a = pow(sin(dLat / 2), 2) +
               cos(lat1) * cos(lat2) * pow(sin(dLon / 2), 2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    double distance_cm = EARTH_RADIUS_CM * c;
    return distance_cm;
} 

double calculateHeading(double currentHeading, double lat1, double lon1, double lat2, double lon2) {
    // Konversi ke radian
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    double deltaLon = (lon2 - lon1) * M_PI / 180.0;

    // Hitung bearing ke target
    double x = sin(deltaLon) * cos(lat2);
    double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(deltaLon);
    double initial_bearing = atan2(x, y);
    double targetHeading = fmod((initial_bearing * 180.0 / M_PI) + 360.0, 360.0);

    // Hitung delta heading [-180, 180]
    double delta = fmod((targetHeading - currentHeading + 180.0), 360.0) - 180.0;
    return delta;
}
