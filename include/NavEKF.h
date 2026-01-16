#pragma once

#include "TinyEKF.h"
#include <AP_Math.h>
#include <definitions.h>
#include <location.h>
#include <Locations.h>
#include "gps.h"

class nav_ekf : public TinyEKF {
public:
    nav_ekf();

    void get_pos_ned(struct Locations &loc);
    Vector3f get_vel_ned();

    void get_originLL(struct Locations &loc) {
        loc = _origin;
    }

    void set_origin(const struct Locations &loc);

    void update(Vector2f pos, Vector2f vel);

    void estimate_pos_and_vel(const struct Locations &loc, Vector2f vel);

protected:
    float T;
    uint32_t _last_update_ms;
    void model(double fx[NstaEKF2], double F[NstaEKF2][NstaEKF2], double hx[MobsEKF2], double H[MobsEKF2][NstaEKF2]);
    Locations _origin;
};

/*
 * ini adalah nav_ekf - versi 2
*/

nav_ekf::nav_ekf() // pada pembuatan objek, nilai P,R, Q dan nilai awal diatur
{
    // We approximate the process noise using a small constant
    setQ(0, 0, .0001f); //matriks q adalah matriks covariansi proses nois yang berhubungan dengan niai marik skovariansi state ( eorr estmatin P)
    setQ(1, 1, .0001f); //nilai nois state pada mat C di isi pada diagonal utama matriks, (0.0), (1.1), (2.2)
    setQ(2, 2, .0001f);
    setQ(3, 3, .0001f);

    // Same for measurement noise
    setR(0, 0, .0001f); // matriks R adalah matrik untuk error in easurement ->digunakan untuk menentukan kalman gain
    setR(1, 1, .0001f); // ada 3 sensor yang digunakan jadi,ukuran matriks r (sensor x sensor) = 3x3
    setR(2, 2, .0001f); // nilai diset sekecil2nya
    setR(3, 3, .0001f);

    // pengisian nilai prediction error covariance
    setP(0, 0, .0001f); // pos x - next , data dimasukan dari hasil pengambilan data kemaren
    setP(1, 1, .0001f); // vel x
    setP(2, 2, .0001f); // pos y
    setP(3, 3, .0001f); // vel y

    // initial position
    setX(0, 0); // posisi x
    setX(2, 0);  // posisi y

    // initial velocity
    setX(1, 0.f); // velocity x
    setX(3, 0.f); // velocity y

    T = .0f;
    _last_update_ms = 0;
}

//fungsi untuk mengembalikan nilai output x,y
void nav_ekf::get_pos_ned(struct Locations &loc)
{
    loc.lat = _origin.lat + (getX(0) / LATLON_TO_CM);
    loc.lng = _origin.lng + (getX(2) / (LATLON_TO_CM * _origin.longitude_scale()));
}

//fungsi untuk mengembalikan nilai output vel x,y
Vector3f nav_ekf::get_vel_ned()
{
    return {getX(1), getX(3), 0};
}

// fungsi untuk update estimator
void nav_ekf::update(Vector2f pos, Vector2f vel)
{
    uint32_t now = millis();
    T = (now - _last_update_ms) * 0.001f;
    if (T > 0.1f) {
        // give small initial delta time
        T = 0.1f;
    }
    vel *= 100.0f;
    _last_update_ms = now;                              // ubah vector kecepatan dalam cm/s
    double z[4] = {pos.x, vel.x, pos.y, vel.y}; // membuat container local untuk input ekf

    step(z);
}

// fungsi estimasi dengan parameter speed dan long lat
void nav_ekf::estimate_pos_and_vel(const struct Locations &loc, Vector2f vel)
{
    Vector2f pos;
    pos.x = (loc.lat - _origin.lat) * LATLON_TO_CM; // container posisis NED
    pos.y = (loc.lng - _origin.lng) * LATLON_TO_CM * _origin.longitude_scale();

    // update EKF estimation
    update(pos, vel);
}

void nav_ekf::set_origin(const struct Locations &loc)
{
    _origin = loc;
    double x = _origin.lat * LATLON_TO_CM;
    double y = _origin.lng * LATLON_TO_CM * _origin.longitude_scale();
    setX(0, x);
    setX(2, y);
}

void nav_ekf::model(double fx[NstaEKF2], double F[NstaEKF2][NstaEKF2], double hx[MobsEKF2], double H[MobsEKF2][NstaEKF2])
{
    // Process model is f(x) - model kinematik
    fx[0] = x[0] + x[1] * T; // untuk state posisi x
    fx[1] = x[1];            // untuk state vx
    fx[2] = x[2] + x[3] * T; // untuk state posisi Y
    fx[3] = x[3];            // untuk state vy

    // So process model Jacobian is identity matrix - matrix A
    F[0][0] = 1.0f;
    F[0][1] = T;
    F[1][1] = 1.0f;
    F[2][2] = 1.0f;
    F[2][3] = T;
    F[3][3] = 1.0f;

    // Measurement function simplifies the relationship between state and sensor readings for convenience.
    hx[0] = x[0]; // nilai sensor 1 (posisi x - cm) - sudah sama dengan state
    hx[1] = x[1]; // nilai sensor 2 vx (kecepatan - cm/s) - sudah sama dengan state
    hx[2] = x[2];
    hx[3] = x[3];

    // Jacobian of measurement function
    H[0][0] = 1.0f; // position x from previous state
    H[1][1] = 1.0f; // velocity x from previous state
    H[2][2] = 1.0f;
    H[3][3] = 1.0f;
}