#ifndef COPTER_CONTROL_H
#define COPTER_CONTROL_H
#include <Arduino.h>

#include "AHRS.h"
#include "ControlModes.h"
#include "Copter_Config.h"
#include "Radio.h"
#include "bno055.h"
#include "pos_hold.h"
#include "TimingUtils.h"

// ===================== FRAME MIX (punyamu) =====================
// F450 frame konfigurasi x
// const double A_invers[4][4] = {
//   {292600,  1300300,  1300300,  6283300},
//   {292600,  1300300, -1300300, -6283300},
//   {292600, -1300300, -1300300,  6283300},
//   {292600, -1300300,  1300300, -6283300}
// };

//WAHANA TD 2025
const double A_invers[4][4] = {{1131000, 2381000, 2381000, 1166822},
                               {1131000, 2381000, -2381000, -1166822},
                               {1131000, -2381000, -2381000, 1166822},
                               {1131000, -2381000, 2381000, -1166822}};  

// const double A_invers[4][4] = {
//   {1,  20,  1,  1},
//   {1,  20, -1, -1},
//   {1, -20, -1,  1},
//   {1, -20,  1, -1}
// };

// ===================== STATE & PARAMETER (punyamu) =====================
float heading_target;
float yaw_error, yaw_ref, yaw_offset;
float error_roll, error_pitch;
unsigned long last_t;
float i_yaw_acc, i_alt_acc;
float error_heading;
float p_yaw, d_yaw, i_yaw;
float u1, u2, u3, u4;
float w1, w2, w3, w4;
float roll_int, pitch_int;            // (tetap ada, boleh 0 saat awal—kini tidak krusial)
unsigned long tnow, tbefore;
unsigned long calc_time, last_calc_time;
float delta_calc_time;
float alt_ref, heading_now, last_alt, last_heading, alt_now;
float alt_target, z_velocity;
float roll_cmd, pitch_cmd, yaw_cmd;
float min_roll  = -15.0f;
float max_roll  =  15.0f;
float min_pitch = -35.0f;
float max_pitch =  35.0f;
float min_yaw   = -35.0f;
float max_yaw   =  35.0f;
int   PID_max_roll  =  400;
int   PID_max_pitch =  400;
int   PID_max_yaw   =  400;
int   PID_min_roll  = -400;
int   PID_min_pitch = -400;
int   PID_min_yaw   = -400;
float omega2[4];
float lat_pos, lon_pos;
double roll_pos, pitch_pos;
bool alt_hold_on = false;
const int YAW_DEADZONE_RC = 15;
bool pilot_has_yaw_input = false;
float trim_roll = 0.0f;  // 4.8f;//= 2.71;//3.78; //-40 //(+) bales kanan, (-) bales kiri 5 -2
float trim_pitch = 0.0f;  // 10.0f;// 2.40;//2.62; //-30 //25 //22.4 -15.5 -10 -5 // (+) bales maju, (-)bales mundur // 3.5
float trim_yaw = 0.00f;  //0.004 dg subtrim remote 50% // 0.006 //(+) bales kanan, (-) bales kiri -0.349 -0.360

float tesdata;

float tesgz,tesgx;

// ===================== GAINS =====================
struct gains {
    float k_alt        = 6.50f;
    float k_z_velocity = 3.0f;
    float k_z_vel      = 1.0f;
    float k_pos        = 1.0f;
    int16_t roll_rmt, pitch_rmt, yaw_rmt;

    // Stabilize (angle -> rate), P-only (
    float stab_roll_P  = 4.5f;
    float stab_pitch_P = 4.0f;
    float stab_yaw_P   = 5.0f;

    // Rate PID + Imax
    float rate_roll_P    = 5.00f;
    float rate_roll_I    = 0.01f;
    float rate_roll_D    = 0.05f;
    float rate_roll_IMAX = 500.0f;

    float rate_pitch_P    = 5.0f;
    float rate_pitch_I    = 0.01f;
    float rate_pitch_D    = 0.05f;
    float rate_pitch_IMAX = 500.0f;

    float rate_yaw_P      = 30.00f; //20.0f;
    float rate_yaw_I      = 0.01f; //0.01f;
    float rate_yaw_D      = 0.05f; //0.05f;
    float rate_yaw_IMAX   = 8.0f;

    // batas rate target (deg/s)
    float max_rate_r = 220.0f;
    float max_rate_p = 220.0f;
    float max_rate_y = 150.0f;
};
gains gain;

// ===================== RATE LOOP STATE =====================
float roll_rate_sp  = 0.0f;
float pitch_rate_sp = 0.0f;
float yaw_rate_sp   = 0.0f;

float i_rate_roll = 0.0f;
float i_rate_pitch = 0.0f;
float i_rate_yaw = 0.0f;

static float gy_prev = 0.0f, gx_prev = 0.0f, gz_prev = 0.0f;
static uint32_t last_us_rate = 0;

// ===================== INTEGRATOR ANGLE (tetap ada) =====================
void copter_getIntegral(int16_t ch_thr, float roll, float pitch, float yaw) {
    tbefore = tnow;
    tnow = millis();
    float tdelta = (tnow - tbefore) / 1000.0f;

    roll_int  += 0.0f * error_roll  * tdelta; // set 0 agar tidak double-control (outer loop P-only)
    pitch_int += 0.0f * error_pitch * tdelta; // (biarkan 0 dulu; kalau ingin, baru aktifkan kecil)

    roll_int  = constrain(roll_int,  -15.0f, 15.0f);
    pitch_int = constrain(pitch_int, -15.0f, 15.0f);
    i_yaw     = constrain(i_yaw,     -20.0f, 20.0f);

    if (ch_thr < 1100) { roll_int = pitch_int = i_yaw = 0.0f; }
}

// ===================== POS HOLD ERROR =====================
void Error_poshold(double distance, double z){
    // distance cm, z deg (delta heading ke target)
    double e_north = distance * cos(z * DEG_TO_RAD);
    double e_east  = distance * sin(z * DEG_TO_RAD);

    e_north *= gain.k_pos;
    e_east  *= gain.k_pos;

    pitch_pos = constrain(e_north/100.0, -5.0, 5.0);
    roll_pos  = constrain(e_east /100.0, -5.0, 5.0);
}

// ===================== CONTROLLER =====================
void copter_ControlFSFB(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr,
                        float roll, float pitch, float yaw,
                        float gy, float gx, float gz)
{
    // ===== time step untuk rate PID =====
    uint32_t now_us = micros();
    float dt = (last_us_rate==0) ? 0.0f : (now_us - last_us_rate) * 1e-6f;
    if (dt <= 0.0f || dt > 0.2f) dt = 0.0f;  // lock I/D jika timing jelek
    last_us_rate = now_us;
    tesdata = last_us_rate;

    // KALAU MAU CEK GYRO YANG DIBALIK
    // gx = -gx;
    gz = -gz; 
    // tesgx = gx;
    // tesgz = gz;

    // ===== altitude (aktifkan pembacaan agar alt_now valid) =====
    if (alt_hold) {
        if (ch_thr > 1600 && ch_thr < 1800) {
            if (!alt_hold_on) { alt_ref = baro.altitude; alt_hold_on = true; }
            alt_now    = baro.altitude;
            alt_target = alt_now - alt_ref;
            z_velocity = baro.get_climb_rate() / 100.0f;
        } else {
            alt_hold_on = false;
            z_velocity = 0;
        }
    } else {
        alt_hold_on = false;
        z_velocity = 0;
    }

    // ===== POS HOLD (punyamu) =====
    if (pos_hold) {
        if (ch_r > 1490 && ch_r < 1510 && ch_p > 1490 && ch_p < 1510){
            static bool pos_locked = false;
            if(!pos_locked) {
                lat_pos = gepees.latitude;
                lon_pos = gepees.longitude;
                pos_locked = true;
            }
            double distance = haversineDistanceCM(gepees.latitude, gepees.longitude, lat_pos, lon_pos);
            double z = calculateHeading(imu.heading, gepees.latitude, gepees.longitude, lat_pos, lon_pos);
            Error_poshold(distance, z);
        } else {
            static bool pos_locked = true;
            if(pos_locked) {
                pos_locked = false;
                roll_pos = 0.0; pitch_pos = 0.0;
                // Serial.println("Position UNLOCKED - Manual control");
            }
        }
    } else {
        roll_pos = 0.0; pitch_pos = 0.0;
    }

    // ===== RC → angle command =====
    // roll_cmd  = 1.3f*(map(ch_r - 1500, min_roll_corr,  max_roll_corr,  min_roll,  max_roll));
    // pitch_cmd = 1.1f*(map(ch_p - 1500, min_pitch_corr, max_pitch_corr, min_pitch, max_pitch));
    // yaw_cmd   = 1.3f*(map(ch_y - 1500, min_yaw_corr,   max_yaw_corr,   min_yaw,   max_yaw));

    // tambahkan koreksi pos-hold
    roll_cmd  += roll_pos;
    pitch_cmd += pitch_pos;

    // ===== error sudut (cmd - meas) =====
    const float e_roll  = (-roll_cmd  + trim_roll)  - roll;
    const float e_pitch = (pitch_cmd + trim_pitch) - (-pitch); // sesuai definisimu

    // ===== Stabilize (angle→rate) P-only =====
    roll_rate_sp  = constrain(e_roll  * gain.stab_roll_P,  -gain.max_rate_r,  gain.max_rate_r);
    pitch_rate_sp = constrain(e_pitch * gain.stab_pitch_P, -gain.max_rate_p,  gain.max_rate_p);

    // Yaw: rate dari stick atau heading-hold
    // const bool yaw_stick = (abs(ch_y - 1500) > YAW_DEADZONE_RC);
    // static float heading_target_deg = 0.0f;
    // if (yaw_stick) {
    //     heading_target_deg = yaw; // update target ketika pilot gerak yaw
    //     yaw_rate_sp = constrain(map(ch_y, 1000, 2000, -gain.max_rate_y, gain.max_rate_y),
    //                         -gain.max_rate_y, gain.max_rate_y);
    // } else {
    //     float e_yaw = heading_target_deg - yaw;
    //     if (e_yaw > 180.0f)  e_yaw -= 360.0f;
    //     if (e_yaw < -180.0f) e_yaw += 360.0f;
    //     yaw_rate_sp = constrain(e_yaw * gain.stab_yaw_P, -gain.max_rate_y, gain.max_rate_y);
    // }

    //yaw manual
    // Yaw: rate langsung dari stick, tanpa heading-hold
    const bool yaw_stick = (abs(ch_y - 1500) > YAW_DEADZONE_RC);
        if (yaw_stick) {
            yaw_rate_sp = constrain(
                map(ch_y, 1000, 2000, -gain.max_rate_y, gain.max_rate_y),
                -gain.max_rate_y, gain.max_rate_y
            );
        } else {
            yaw_rate_sp = 0.0f; // stick netral → yaw diam, tidak heading lock
        }


    // ===== Rate PID (P + I + D_on_meas) =====
    const float er = (roll_rate_sp  - gy);   // NOTE: pakai gy untuk roll
    const float ep = (pitch_rate_sp - gx);   // gx untuk pitch
    const float ey = (yaw_rate_sp   - gz);   // gz untuk yaw

    const float dgy = (dt>0) ? (gy - gy_prev)/dt : 0.0f; gy_prev = gy;
    const float dgx = (dt>0) ? (gx - gx_prev)/dt : 0.0f; gx_prev = gx;
    const float dgz = (dt>0) ? (gz - gz_prev)/dt : 0.0f; gz_prev = gz;

    const bool allow_I = (dt>0) && (ch_thr >= 1100);
    if (allow_I){
        i_rate_roll  += gain.rate_roll_I  * er * dt;
        i_rate_pitch += gain.rate_pitch_I * ep * dt;
        i_rate_yaw   += gain.rate_yaw_I   * ey * dt;

        i_rate_roll  = constrain(i_rate_roll,  -gain.rate_roll_IMAX,  gain.rate_roll_IMAX);
        i_rate_pitch = constrain(i_rate_pitch, -gain.rate_pitch_IMAX, gain.rate_pitch_IMAX);
        i_rate_yaw   = constrain(i_rate_yaw,   -gain.rate_yaw_IMAX,   gain.rate_yaw_IMAX);
    } else {
        i_rate_roll = i_rate_pitch = i_rate_yaw = 0.0f;
    }

    // keluaran inner-loop (skala tetap seperti punyamu)
    u2 = ( gain.rate_roll_P  * er + i_rate_roll  - gain.rate_roll_D  * dgy ) / 10'000'000.0f; // roll via gy
    u3 = ( gain.rate_pitch_P * ep + i_rate_pitch - gain.rate_pitch_D * dgx ) / 10'000'000.0f; // pitch via gx
    u4 = ( gain.rate_yaw_P   * ey + i_rate_yaw   - gain.rate_yaw_D   * dgz )/ 10'000'000.0f; // yaw

    // ===== Throttle/altitude (punyamu) =====
    u1 = (-gain.k_alt * (alt_target/1.000f) + (-gain.k_z_velocity*(z_velocity)/100.0f)) / 10'000'000.0f; //0.0f;

    // ===== MIXER =====
    omega2[0] = (A_invers[0][0] * u1 + A_invers[0][1] * u2 + A_invers[0][2] * u3 + A_invers[0][3] * u4);
    omega2[1] = (A_invers[1][0] * u1 + A_invers[1][1] * u2 + A_invers[1][2] * u3 + A_invers[1][3] * u4);
    omega2[2] = (A_invers[2][0] * u1 + A_invers[2][1] * u2 + A_invers[2][2] * u3 + A_invers[2][3] * u4);
    omega2[3] = (A_invers[3][0] * u1 + A_invers[3][1] * u2 + A_invers[3][2] * u3 + A_invers[3][3] * u4);

    last_heading = imu.heading;
}
#endif