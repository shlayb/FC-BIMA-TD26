/**
 * @file bno055.h
 * @brief  Provides functions and data structures for interacting with a BNO055 IMU sensor using TeensyThreads.
 *
 * This code defines representation unit settings, data structures for storing sensor readings,
 * and functions to initialize, update and process the raw sensor data into usable formats like degrees and radians. It utilizes TeensyThreads for concurrent execution of the IMU update process.
 *
 *! Key features:
 *  - Handles conversion of raw sensor data from the BNO055 IMU
 *  - Extracts Euler angles (roll, pitch, heading), gyroscope data and accelerometer data
 *  - Converts raw data to degrees and radians
 *  - Provides functions to invert heading and calculate yaw
 *  - Includes necessary header files and data structures to interface with the BNO055 sensor
 *  - Implements a dedicated thread for continuous IMU updates
 *
 *! Dependencies:
 *  - Arduino.h
 *  - AP_Math.h
 *  - BNO055_support.h
 *  - TeensyThreads.h
 *  - I2CDev.h
 */

#pragma once

#include <Arduino.h>
#include <AP_Math.h>
#include <BNO055_support.h>
#include <TeensyThreads.h>
#include "I2CDev.h"

// Representation Unit Settings
#define ACCEL_TO_MS2        100
#define ACCEL_TO_MG         1
#define MAG_TO_MIKROTESLA   16
#define GYRO_TO_DPS         16
#define GYRO_TO_RPS         900
#define EULER_TO_DEG        16
#define EULER_TO_RAD        900
#define QUAT_UNIT_LESS      16384
#define GRAVITY_TO_MS2      100
#define GRAVITY_TO_MG       1
#define TEMP_TO_CELC        1
#define TEMP_TO_FAHR        2


class IMU{
    public:
        Vector3f _accel;
        Vector3f _linear_acc;
        float roll, pitch, heading, yaw;
        float rad_roll, rad_pitch, rad_yaw;
        float last_roll, last_pitch, last_yaw;
        float delta_roll, delta_pitch, delta_yaw;
        float gyro_x, gyro_y, gyro_z;
        int pitch_mode;

        void init_imu();
        void update_imu();
        void check_imu_calibration();
        void imu_thread();
        void imu_task();

    private:
        struct bno055_t bno_data;              
        struct bno055_euler bno_euler;     
        struct bno055_gyro bno_gyro;      
        struct bno055_accel bno_accel;
        struct bno055_quaternion bno_quat;
        struct bno055_linear_accel bno_linear;

        unsigned char sys_calib_status = 0;
        unsigned char mag_calib_status = 0;
        unsigned long last_time = 0;

        float invert_heading(float raw_h);
        float calc_yaw(float real_h);
        void extract_euler();
        void extract_gyroscope();
        void extract_accelerometer();
        void extract_linear_accelerometer();
};

void IMU::check_imu_calibration() {
    while (mag_calib_status != 3 || sys_calib_status != 3) {
        if ((millis() - last_time) > 200) {
            bno055_get_magcalib_status(&mag_calib_status);
            bno055_get_syscalib_status(&sys_calib_status);
            Serial2.print("Time Stamp: ");
            Serial2.println(last_time);
            Serial2.print("Magnetometer Calibration Status: ");
            Serial2.println(mag_calib_status);
            Serial2.print("System Calibration Status: ");
            Serial2.println(sys_calib_status);
            last_time = millis();
        }
    }
}

float IMU::invert_heading(float raw_h) {
    float real_h = raw_h - 180.0;
    if (real_h < 0) real_h += 360;
    return real_h;
}

float IMU::calc_yaw(float real_h) {
    return real_h > 180 ? real_h - 360.0f : real_h;
}

void IMU::extract_euler() {
    bno055_read_euler_hrp(&bno_euler);
    
    //? Convert to degree
    roll = (float)bno_euler.r / EULER_TO_DEG;
    pitch = ((float)bno_euler.p / EULER_TO_DEG);

    // if (pitch_mode == 1) {
    //     pitch = (float)bno_euler.p / EULER_TO_DEG;
    // } else if (pitch_mode == 2 ) {
    //     pitch = ((float)bno_euler.p / EULER_TO_DEG) - 90;
    // }

    heading = invert_heading((float)bno_euler.h / EULER_TO_DEG);
    yaw = calc_yaw(heading);

    //? Convert to radians
    rad_roll = (float)bno_euler.r / EULER_TO_RAD;
    rad_pitch = (float)bno_euler.p / EULER_TO_RAD;
    rad_yaw = radians(yaw);
}

void IMU::extract_gyroscope() {
    bno055_read_gyro_xyz(&bno_gyro);
    gyro_x = ((float)bno_gyro.x / GYRO_TO_DPS);
    gyro_y = -1 * ((float)bno_gyro.y / GYRO_TO_DPS);
    gyro_z = ((float)bno_gyro.z / GYRO_TO_DPS);
}

void IMU::extract_accelerometer() {
    bno055_read_accel_xyz(&bno_accel);
    _accel.x = (float)bno_accel.x;
    _accel.y = (float)bno_accel.y;
    _accel.z = (float)bno_accel.z;
}

void IMU::extract_linear_accelerometer() {
    bno055_read_linear_accel_xyz(&bno_linear);
    _linear_acc.x = (float)bno_linear.x / ACCEL_TO_MS2;
    _linear_acc.y = (float)bno_linear.y / ACCEL_TO_MS2;
    _linear_acc.z = (float)bno_linear.z / ACCEL_TO_MS2;
}

void IMU::init_imu() {
    _I2C.init();
    BNO_Init(&bno_data);
    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    delay(1 << 9);
}

void IMU::update_imu() {
    extract_linear_accelerometer();
    extract_euler();
    extract_gyroscope();
    extract_accelerometer();

    delta_roll = abs(roll - last_roll);
    delta_pitch = abs(pitch - last_pitch);
    delta_yaw = abs(yaw - last_yaw);

    last_roll = roll;
    last_pitch = pitch;
    last_yaw = yaw;
}