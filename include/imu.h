#pragma once

#include <Arduino.h>
#include <TeensyThreads.h>

#include "drivers/mpu6050.h"

Scheduler imu_runner;

//? For Quaternion function
#define kp_def (2.0f * 0.5f)  // 2 * proportional gain
#define ki_def (2.0f * 0.0f)  // 2 * integral gain
volatile float two_kp = kp_def;
volatile float two_ki = ki_def;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                     // quaternion of sensor frame relative to auxiliary frame
volatile float integral_fbx = 0.0f, integral_fby = 0.0f, integral_fbz = 0.0f;  // integral error terms scaled by Ki

float rollm, pitchm, yawm;
float sample_freq = 0.0f;                    // integration interval for both filter schemes
uint32_t last_update = 0, first_update = 0;  // used to calculate integration interval
uint32_t now = 0;
float self_test[6];

void mahonyFilter(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az);
        ax /= norm;
        ay /= norm;
        az /= norm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (two_ki > 0.0f) {
            integral_fbx += two_ki * halfex * (1.0f / sample_freq);  // integral error scaled by Ki
            integral_fby += two_ki * halfey * (1.0f / sample_freq);
            integral_fbz += two_ki * halfez * (1.0f / sample_freq);
            gx += integral_fbx;  // apply integral feedback
            gy += integral_fby;
            gz += integral_fbz;
        } else {
            integral_fbx = 0.0f;  // prevent integral windup
            integral_fby = 0.0f;
            integral_fbz = 0.0f;
        }

        // Apply proportional feedback
        gx += two_kp * halfex;
        gy += two_kp * halfey;
        gz += two_kp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sample_freq));  // pre-multiply common factors
    gy *= (0.5f * (1.0f / sample_freq));
    gz *= (0.5f * (1.0f / sample_freq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

void calcRollPitchYawm() {
    yawm = -atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.29577951;
    pitchm = asin(2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;
    rollm = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.29577951;
}

void initMPU(bool is_calibrate) {
    _I2C.init();
    //* MPU6050 Initializing & Reset
    _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);  // set to zero (wakes up the MPU-6050)

    //* MPU6050 DLPF(Digital Low Pass Filter)
    _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x00);  // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz

    //! MPU6050 Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) for DMP
    _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00);  // Default is 1KHz // example 0x04 is 200Hz

    //* MPU6050 Gyroscope Configuration Setting
    _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x18);  // FS_SEL=3
    //* MPU6050 Accelerometer Configuration Setting
    _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x10);  // AFS_SEL=2

    //* MPU6050 Clock Type
    if (is_calibrate) {
        _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x03);  // Selection Clock 'PLL with Z axis gyroscope reference'
        calibrateIMU();
    } else {
        _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);  // Selection Clock 'PLL with X axis gyroscope reference'
    }
}

void initializeIMU(bool calib) {
    bool cal = calib;
    if (cal) {
        initMPU(cal);
    } else {
        performSelfTestIMU(self_test);
        initMPU(cal);
    }
}

void updateIMU() {
    readRawIMU();
    updateQuaternion();

    now = micros();
    sample_freq = (1000000.0f / (now - last_update));  // set integration time by time elapsed since last filter update
    last_update = now;

    // mahonyFilter(gyro_x, gyrs, gzrs, axg, ayg, azg);
    calcRollPitchYawm();
}

void threadIMU() {
    while (1) {
        updateIMU();
        threads.yield();
    }
}