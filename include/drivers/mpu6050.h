#pragma once

#include <Arduino.h>
#include <math.h>

#include "I2CDev.h"
#include "MPU6050Reg.h"

//? GAIN & CONSTANT
#define MPU6050_AXOFFSET 251
#define MPU6050_AYOFFSET 12
#define MPU6050_AZOFFSET -58
#define MPU6050_GXOFFSET -105
#define MPU6050_GYOFFSET 375
#define MPU6050_GZOFFSET -25
#define MPU6050_AXGAIN 4096.0  // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
#define MPU6050_AYGAIN 4096.0  // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
#define MPU6050_AZGAIN 4096.0  // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
#define MPU6050_GXGAIN 16.384  // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000
#define MPU6050_GYGAIN 16.384  // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000
#define MPU6050_GZGAIN 16.384  // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000

//? For Get & Update Data
volatile int16_t acx, acy, acz, tmp, gyx, gyy, gyz;
int32_t cal_acx, cal_acy, cal_acz, cal_tmp, cal_gyx, cal_gyy, cal_gyz;
float axg, ayg, azg, gxrsm, gyrsm, gzrsm;

void readRawIMU(bool is_calibrate = false) {
    uint8_t ori_data[14];  // original data of accelerometer and gyro
    _I2C.readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14, &ori_data[0]);

    acx = ori_data[0] << 8 | ori_data[1];  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    acy = ori_data[2] << 8 | ori_data[3];  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    if (is_calibrate) {
        acz = (ori_data[4] << 8 | ori_data[5]) - 4096;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    } else {
        acz = ori_data[4] << 8 | ori_data[5];  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    }
    tmp = ori_data[6] << 8 | ori_data[7];    // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gyx = ori_data[8] << 8 | ori_data[9];    // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gyy = ori_data[10] << 8 | ori_data[11];  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gyz = ori_data[12] << 8 | ori_data[13];  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void calibrateIMU() {
    int16_t count = 2000;
    for (int16_t i = 0; i < count; i++) {
        if (i % 250 == 0) Serial.println("Calibrating.....");
        readRawIMU(true);
        delay(10);
        // Sum data
        cal_acx += acx;
        cal_acy += acy;
        cal_acz += acz;
        cal_gyx += gyx;
        cal_gyy += gyy;
        cal_gyz += gyz;
    }

    cal_acx /= count;
    cal_acy /= count;
    cal_acz /= count;
    cal_gyx /= count;
    cal_gyy /= count;
    cal_gyz /= count;

    while (1) {
        Serial.print("acx: ");
        Serial.print(cal_acx);
        Serial.print("  acy: ");
        Serial.print(cal_acy);
        Serial.print("  acz: ");
        Serial.print(cal_acz);
        Serial.print("  gyx: ");
        Serial.print(cal_gyx);
        Serial.print("  gyy: ");
        Serial.print(cal_gyy);
        Serial.print("  gyz: ");
        Serial.println(cal_gyz);
        delay(5000);
    }
}

void updateQuaternion() {
    axg = (float)(acx - MPU6050_AXOFFSET) / MPU6050_AXGAIN;
    ayg = (float)(acy - MPU6050_AYOFFSET) / MPU6050_AYGAIN;
    azg = (float)(acz - MPU6050_AZOFFSET) / MPU6050_AZGAIN;
    gyro_x = (float)(gyx - MPU6050_GXOFFSET) / MPU6050_GXGAIN * 0.0174532925;  // degree to radians
    gyro_y = (float)(gyy - MPU6050_GYOFFSET) / MPU6050_GYGAIN * 0.0174532925;  // degree to radians
    gyro_z = (float)(gyz - MPU6050_GZOFFSET) / MPU6050_GZGAIN * 0.0174532925;  // degree to radians
    //? Degree to Radians Pi / 18
}

//* Accelerometer and gyroscope self test, check calibration wrt factory settings
//! Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
void performSelfTestIMU(float* destination) {
    uint8_t raw_data[4];
    uint8_t selfTest[6];
    float factory_trim[6];

    //* Configure the accelerometer for self-test
    _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0xF0);        // Enable self test on all three axes and set accelerometer range to +/- 8 g
    _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0xE0);         // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    delay(250);                                                                    // Delay a while to let the device execute the self-test
    _I2C.readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_X, &raw_data[0]);  // X-axis self-test results
    _I2C.readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Y, &raw_data[1]);  // Y-axis self-test results
    _I2C.readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Z, &raw_data[2]);  // Z-axis self-test results
    _I2C.readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_A, &raw_data[3]);  // Mixed-axis self-test results
    //* Extract the acceleration test results first
    selfTest[0] = (raw_data[0] >> 3) | (raw_data[3] & 0x30) >> 4;  // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (raw_data[1] >> 3) | (raw_data[3] & 0x0C) >> 2;  // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (raw_data[2] >> 3) | (raw_data[3] & 0x03);       // ZA_TEST result is a five-bit unsigned integer
    //* Extract the gyration test results first
    selfTest[3] = raw_data[0] & 0x1F;  // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = raw_data[1] & 0x1F;  // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = raw_data[2] & 0x1F;  // ZG_TEST result is a five-bit unsigned integer
    //* Process results to allow final comparison with factory set values
    factory_trim[0] = (4096.0 * 0.34) * (pow((0.92 / 0.34), (((float)selfTest[0] - 1.0) / 30.0)));  // FT[Xa] factory trim calculation
    factory_trim[1] = (4096.0 * 0.34) * (pow((0.92 / 0.34), (((float)selfTest[1] - 1.0) / 30.0)));  // FT[Ya] factory trim calculation
    factory_trim[2] = (4096.0 * 0.34) * (pow((0.92 / 0.34), (((float)selfTest[2] - 1.0) / 30.0)));  // FT[Za] factory trim calculation
    factory_trim[3] = (25.0 * 131.0) * (pow(1.046, ((float)selfTest[3] - 1.0)));                    // FT[Xg] factory trim calculation
    factory_trim[4] = (-25.0 * 131.0) * (pow(1.046, ((float)selfTest[4] - 1.0)));                   // FT[Yg] factory trim calculation
    factory_trim[5] = (25.0 * 131.0) * (pow(1.046, ((float)selfTest[5] - 1.0)));                    // FT[Zg] factory trim calculation

    //  Output self-test results and factory trim calculation if desired
    //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
    //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
    //  Serial.println(factory_trim[0]); Serial.println(factory_trim[1]); Serial.println(factory_trim[2]);
    //  Serial.println(factory_trim[3]); Serial.println(factory_trim[4]); Serial.println(factory_trim[5]);

    //* Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        destination[i] = 100.0 + 100.0 * ((float)selfTest[i] - factory_trim[i]) / factory_trim[i];  // Report percent differences
    }
}