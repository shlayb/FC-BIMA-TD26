/**
 * @file AHRS.h
 * @brief Provides functionality for sensor fusion and state estimation using an Extended Kalman Filter (EKF).
 *
 * This code integrates data from various sensors like IMU, GPS, barometer, and airspeed sensor to estimate the vehicle's position, velocity, and attitude. It includes functions for drift correction, wind speed estimation, and calculation of angle of attack (AOA) and sideslip angle (SSA).
 *
 *! Key features:
 * - Initializes and updates IMU, GPS, barometer, and airspeed sensor
 * - Implements an Extended Kalman Filter for state estimation
 * - Performs drift correction using GPS and airspeed data
 * - Estimates wind speed based on fuselage direction and velocity changes
 * - Calculates AOA and SSA based on velocity and wind vectors
 * - Provides access to estimated position, velocity, acceleration, and rotation matrix.
 *
 *! Dependencies:
 * - Locations.h
 * - Arduino.h
 * - AP_Math.h
 * - Barometer.h
 * - NavEKF.h
 * - airspeed.h
 * - bno055.h
 * - gps.h
 */

#pragma once

#include <AP_Math.h>
#include <Arduino.h>
#include <Locations.h>

#include "Barometer.h"
#include "NavEKF.h"
#include "airspeed.h"
#include "bno055.h"
#include "gps.h"




Gepees gepees(0, 1, 38400);
Airspeed arspd;
IMU imu;

class AHRS {
   public:
    Vector3f accel_gravity;
    Vector3f accel_ef;
    Vector3f accel_bf;
    Matrix3f dcm_matrix;
    float groundspeed;
    bool is_airspeed_sensor_enabled;

    AHRS();
    void init_ahrs();
    void update_ahrs();
    bool haveGPS() const;
    const Matrix3f &get_rotation_body_to_ned() const;
    Vector3f get_velocity_ned() const;
    bool get_position(Locations &loc);
    void setHome(Locations &loc);
    bool airspeedEstimate(float *airspeed_ret) const;
    void estimateGroundspeedVelocity(float sin_h, float cos_h);
    void estimatePositionEKF();
    void driftCorrection();
    void estimate_windspeed();
    void updateAOASSA();
    void get_relative_position_D_home(float &posD) const;
    Vector3f get_accel_ef() const;
    Vector3f get_accel_bf() const;
    void threadAHRS();
    
   private:
    bool is_gps_lock;
    nav_ekf ekf;
    Locations home_loc;
    Vector3f velocity_vector;
    Vector3f last_velocity;
    float airspeed;
    float last_groundspeed;
    float lp;
    float hp;
    int32_t last_latitude;
    int32_t last_longitude;
    uint32_t last_aoa_update;
    float AOA;
    float SSA;
    Vector3f last_fuse;
    uint32_t last_wind_time;
    Vector3f windspeed;
    float windspeed_x;
    float last_airspeed;

    Vector3f getNormAccel();
    Matrix3f getRotationMatFromQuat() const;

};

AHRS::AHRS()
    : accel_gravity(0.0, 0.0, -9.80665f),
      accel_ef(),
      accel_bf(),
      dcm_matrix(),
      groundspeed(0.0f),
      is_airspeed_sensor_enabled(false),
      is_gps_lock(false),
      ekf(),
      home_loc(),
      velocity_vector(),
      last_velocity(),
      airspeed(0.0f),
      last_groundspeed(0.0f),
      lp(0.0f),
      hp(0.0f),
      last_latitude(0),
      last_longitude(0),
      last_aoa_update(0),
      AOA(0.0f),
      SSA(0.0f),
      last_fuse(),
      last_wind_time(0),
      windspeed(),
      windspeed_x(0.0f),
      last_airspeed(0.0f) {}

bool AHRS::haveGPS() const {
    return gepees.gps_status > GPS_NO_FIX;
}

Vector3f AHRS::get_accel_bf() const {
    return accel_bf;
}

Vector3f AHRS::get_accel_ef() const {
    return accel_ef;
}

const Matrix3f &AHRS::get_rotation_body_to_ned() const {
    return dcm_matrix;
}

Vector3f AHRS::get_velocity_ned() const {
    return velocity_vector;
}

bool AHRS::get_position(Locations &loc) {
    ekf.get_pos_ned(loc);
    loc.lat = gepees.latitude * 10000000;
    loc.lng = gepees.longitude * 10000000;
    loc.alt = baro.altitude * 100;
    return true;
}

void AHRS::setHome(Locations &loc) {
    if (gepees.gps_status >= GPS_FIX_3D) {
        home_loc.lat = loc.lat = gepees.latitude * 10000000;
        home_loc.lng = loc.lng = gepees.longitude * 10000000;
        home_loc.alt = loc.alt = baro.altitude * 100;

        ekf.set_origin(home_loc);
        ekf.set_origin(loc);
    }
}

bool AHRS::airspeedEstimate(float *airspeed_ret) const {
    if (haveGPS()) {
        *airspeed_ret = last_airspeed;
        return true;
    }
    return false;
}

void AHRS::estimateGroundspeedVelocity(float sin_h, float cos_h) {
    const float dt = 0.1f;
    const float beta = 0.1f;
    const float alpha = 1.0f - beta;
    const Vector3f acc_bf = getNormAccel();

    lp = gepees.speed_mps * beta + alpha * lp;
    hp = ((groundspeed - last_groundspeed) + acc_bf.x * cos_h * dt) + alpha * hp;
    groundspeed = (hp + lp) * 10000;

    if (groundspeed < 0.0f) {
        groundspeed = 0.0f;
    }

    if (is_gps_lock) {
        last_groundspeed = gepees.speed_mps;
        last_velocity = gepees.velocity;
    } else {
        last_groundspeed = groundspeed;
        last_velocity = velocity_vector;
    }

    velocity_vector.x = groundspeed * cos_h;
    velocity_vector.y = groundspeed * sin_h;
    velocity_vector.z = 0;  //! need implementation climb rate
}

void AHRS::estimatePositionEKF() {
    if (haveGPS() && home_loc.lat != 0 && home_loc.lng != 0) {
        Locations loc;
        loc.lat = gepees.latitude * 10000000;
        loc.lng = gepees.longitude * 10000000;
        Vector2f vel(velocity_vector.x, velocity_vector.y);
        ekf.estimate_pos_and_vel(loc, vel);
        last_latitude = loc.lat;
        last_longitude = loc.lng;
    } else if (home_loc.lat != 0 && home_loc.lng != 0) {
        Locations last_loc(
            last_latitude,
            last_longitude,
            0,
            Locations::AltFrame::ABOVE_HOME);
        Vector2f vel(velocity_vector.x, velocity_vector.y);
        ekf.estimate_pos_and_vel(last_loc, vel);
    }
}

void AHRS::driftCorrection() {
    //* helper
    float sin_heading = sinf(radians(imu.heading));
    float cos_heading = cosf(radians(imu.heading));
    bool is_gps_fix = gepees.age > 0 && gepees.age < 3000;

    if (is_gps_fix || gepees.gps_status >= GPS_FIX_3D || gepees.gps.speed.isUpdated()) {
        groundspeed = gepees.speed_mps;  // ini dicek lagi, kalau terbang jangan (* 1000)
        velocity_vector = gepees.velocity;
        is_gps_lock = true;
    } else if (haveGPS()) {
        estimateGroundspeedVelocity(sin_heading, cos_heading);

        if (is_airspeed_sensor_enabled) {
            airspeed = arspd.v_ms;
        } else {
            airspeed = groundspeed - windspeed_x;
        }
        last_airspeed = MAX(airspeed, 0);
        is_gps_lock = false;
    } else {
        groundspeed = 0.0f;
        is_gps_lock = false;
    }

    estimatePositionEKF();
}

void AHRS::estimate_windspeed() {
    // is_airspeed_enable = false;
    Vector3f velocity = velocity_vector;

    // this is based on the wind speed estimation code from MatrixPilot by
    // Bill Premerlani. Adaption for ArduPilot by Jon Challinger
    const Vector3f fuselage_direction = dcm_matrix.colx();
    const Vector3f fuselage_direction_diff = fuselage_direction - last_fuse;
    const uint32_t now = millis();

    if (now - last_wind_time > 1000) {
        last_wind_time = now;
        last_fuse = fuselage_direction;
        last_velocity = velocity;
        return;
    }

    float diff_length = fuselage_direction_diff.length();
    if (diff_length > 0.2f) {
        float V;
        const Vector3f velocity_diff = velocity - last_velocity;
        V = velocity_diff.length() / diff_length;

        const Vector3f fuselage_direction_sum = fuselage_direction + last_fuse;
        const Vector3f velocity_sum = velocity + last_velocity;
        last_fuse = fuselage_direction;
        last_velocity = velocity;

        const float theta = atan2f(velocity_diff.y, velocity_diff.x) - atan2f(fuselage_direction_diff.y, fuselage_direction_diff.x);
        const float sintheta = sinf(theta);
        const float costheta = cosf(theta);

        Vector3f wind = Vector3f();
        wind.x = velocity_sum.x - V * (costheta * fuselage_direction_sum.x - sintheta * fuselage_direction_sum.y);
        wind.y = velocity_sum.y - V * (sintheta * fuselage_direction_sum.x + costheta * fuselage_direction_sum.y);
        wind.z = velocity_sum.z - V * fuselage_direction_sum.z;
        wind *= 0.5f;

        if (wind.length() < windspeed.length() + 20) {
            windspeed = windspeed * 0.95f + wind * 0.05f;
        }

        last_wind_time = now;
        float wind_x = windspeed.x * windspeed.x + windspeed.y * windspeed.y;
        windspeed_x = sqrt(wind_x);
    }
}

void AHRS::updateAOASSA() {
    const uint32_t now = millis();
    if (now - last_aoa_update < 50) {
        return;
    }
    last_aoa_update = now;

    Vector3f aoa_velocity, aoa_wind;
    aoa_velocity = gepees.velocity;
    if (aoa_velocity.length() == 0) {
        return;
    }
    aoa_wind = windspeed;

    // Rotate vectors to the body frame and calculate velocity and wind
    const Matrix3f &rot = getRotationMatFromQuat();
    aoa_velocity = rot.mul_transpose(aoa_velocity);
    aoa_wind = rot.mul_transpose(aoa_wind);

    // calculate relative velocity in body coordinates
    aoa_velocity = aoa_velocity - aoa_wind;
    const float vel_len = aoa_velocity.length();

    if (vel_len < 2.0f) {
        AOA = 0;
        SSA = 0;
        return;
    }

    if (aoa_velocity.x > 0) {
        AOA = degrees(atanf(aoa_velocity.z / aoa_velocity.x));
    } else {
        AOA = 0;
    }

    SSA = degrees(safe_asin(aoa_velocity.y / vel_len));
}

Matrix3f AHRS::getRotationMatFromQuat() const {
    Quaternion q_math;
    Matrix3f rot_matrix;

    q_math.from_euler(imu.rad_roll, imu.rad_pitch, imu.rad_yaw);
    q_math.rotation_matrix(rot_matrix);
    return rot_matrix;
}

Vector3f AHRS::getNormAccel() {
    accel_bf = accel_ef - accel_gravity;
    return accel_bf;
}

void AHRS::get_relative_position_D_home(float &posD) const {
    posD = -baro.altitude;
}

void AHRS::init_ahrs() {
    imu.init_imu();
    gepees.initGPS();
    arspd.initAirspeed();
    baro.init_baro();
}

void AHRS::update_ahrs() {
    dcm_matrix = getRotationMatFromQuat();
    accel_ef = dcm_matrix * imu._accel;

    driftCorrection();
    updateAOASSA();
}

AHRS ahrs;

// void AHRS::threadAHRS() {
//     threads.addThread([]() { IMU_thread(); }, 2);
//     threads.addThread([]() { BARO_thread(); }, 1);
//     threads.addThread([]() { GPS_thread(); });
//     threads.addThread([]() { ARSPD_thread(); }, 1);
// }