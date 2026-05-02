#ifndef FW_CONTROL_FBWA_H
#define FW_CONTROL_FBWA_H
#include <Arduino.h>
#include "Radio.h"
#include "AHRS.h"

struct FW_gain
{
        // Rate PID + Imax
  float rate_roll_P    = 10.00f; //2.50f;
  float rate_roll_I    = 0.00f; //0.01f;
  float rate_roll_D    = 0.00f; //0.13f;
  float rate_roll_IMAX = 500.0f;

  //fix
  float rate_pitch_P    = 10.00f; //2.50f;
  float rate_pitch_I    = 0.00f; //0.01f;
  float rate_pitch_D    = 0.20f; //0.13f;
  float rate_pitch_IMAX = 500.0f;

  float rate_yaw_P      = 0.00f; //20.0f;
  float rate_yaw_I      = 0.00f; //0.01f;
  float rate_yaw_D      = 0.00f; //1.1581
  float rate_yaw_IMAX   = 8.00f;

  float rate_thr_P      = 35.00f; //20.0f;
  float rate_thr_I      = 0.00f; //0.01f;
  float rate_thr_D      = 0.00f; //1.1581
};

class FW_CONTROL {
  public:
    signed char selector[4][3] = {
        { 1,  0,  0},   // aileron right
        { 1,  0,  0},   // aileron left 
        { 0,  1,  0},   // elevator //trainer +, wahana -
        { 0,  0,  1}    // rudder
    };
    void updateFBWA_FW(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr,
                        float roll, float pitch, float yaw,
                        float gy, float gx, float gz);
    void updateAUTO_FW(float roll, float pitch, float yaw,
                        float gy, float gx, float gz);
    FW_gain fw_gain;
    int servos[4];
    int auto_throttle;
    int get_auto_throttle();

  private:
  void getIntegral(int16_t ch_thr, float roll, float pitch, float yaw);
  // float heading_target;
  // float error_roll, error_pitch;
  // unsigned long last_t;
  // float p_yaw, d_yaw, i_yaw;
  float u_roll, u_pitch, u_yaw;
  unsigned long calc_time, last_calc_time;
  unsigned long tnow = 0;
  unsigned long tbefore = 0;
  float tdelta = 0;
  float delta_calc_time;
  float roll_cmd, pitch_cmd, yaw_cmd;
  float roll_int, pitch_int;
  float min_roll  = -35.0f; //-15.0f;
  float max_roll  =  35.0f; //15.0f;
  float min_pitch = -35.0f;
  float max_pitch =  35.0f;
  int min_roll_corr  = -512;
  int max_roll_corr  =  512;
  int min_pitch_corr = -512;
  int max_pitch_corr =  512;
  float trim_roll = 0.0f;
  float trim_pitch = 0.0f;
  float trim_yaw = 0.0f;
  float demanded_pitch = 0.0f;
  float demanded_pitch_min = -20.0f;
  float demanded_pitch_max = 20.0f;
  float demanded_roll_min = -40.0f;
  float demanded_roll_max = 40.0f;
  float auto_throttle_min = 1200.0f;
  float auto_throttle_max = 1700.0f;
};

void FW_CONTROL::updateFBWA_FW(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr,
                        float roll, float pitch, float yaw,
                        float gy, float gx, float gz) {
    // last_calc_time = calc_time;
    // calc_time = micros();
    // delta_calc_time = (calc_time - last_calc_time) / 1000000.0f;
    getIntegral(ch_thr, roll, pitch, yaw);
    
    // kode kontrol FBWA untuk fixed wing di sini  
    roll_cmd  = 1.3f*(map(ch_r - 1500, min_roll_corr,  max_roll_corr,  min_roll,  max_roll));
    pitch_cmd = 1.1f*(map(ch_p - 1500, min_pitch_corr, max_pitch_corr, min_pitch, max_pitch));
    // yaw_cmd   = 1.0f*(map(ch_y - 1500, min_roll_corr,  max_roll_corr,  min_roll,  max_roll));
    // Serial.print("Roll :");
    // Serial.print(roll);
    // Serial.print(" |Pitch:");
    // Serial.print(pitch);

    u_roll = ((-fw_gain.rate_roll_P * (-roll + roll_int - (roll_cmd + trim_roll))) + (-fw_gain.rate_roll_D * (gy)));
    // Serial.print(" |u_roll:");
    // Serial.print(u_roll);
    u_pitch = ((fw_gain.rate_pitch_P * (-pitch + pitch_int - (-pitch_cmd + trim_pitch))) + (-fw_gain.rate_pitch_D * (gx)));
    // Serial.print(" |u_pitch:");
    // Serial.print(u_pitch);

    servos[0] = (selector[0][0] * u_roll + selector[0][1] * u_pitch + selector[0][2] * u_yaw); // aileron right
    servos[1] = (selector[1][0] * u_roll + selector[1][1] * u_pitch + selector[1][2] * u_yaw); // aileron left
    servos[2] = (selector[2][0] * u_roll + selector[2][1] * u_pitch + selector[2][2] * u_yaw); // elevator
    servos[3] = (selector[3][0] * u_roll + selector[3][1] * u_pitch + selector[3][2] * u_yaw); // rudder

    servos[0] = constrain(servos[0], -400, 400) ;
    servos[1] = constrain(servos[1], -400, 400) ;
    servos[2] = constrain(servos[2], -400, 400) ;
    servos[3] = constrain(servos[3], -400, 400) ;

    // Serial.print(" |ail_L:");
    // Serial.print(servos[1]);
    // Serial.print(" |ail_R:");
    // Serial.print(servos[0]);
    // Serial.print(" |ele:");
    // Serial.print(servos[2]);
    // Serial.print(" |rud:");
    // Serial.println(servos[3]);

}

void FW_CONTROL::getIntegral(int16_t ch_thr, float roll, float pitch, float yaw) {
    tbefore = tnow;
    tnow = millis();
    tdelta = (tnow - tbefore) / 1000.0f;
    roll_int += fw_gain.rate_roll_I * roll * tdelta;
    pitch_int += fw_gain.rate_pitch_I * pitch * tdelta;
    roll_int = constrain(roll_int, -15, 15);
    pitch_int = constrain(pitch_int, -15, 15);
    // Reset the integrators
    if (ch_thr < 1100) {
        roll_int = 0.0f;
        pitch_int = 0.0f;
}
}

void FW_CONTROL::updateAUTO_FW(float roll, float pitch, float yaw, float gy, float gx, float gz) {
    int ch_thr = get_auto_throttle();
    getIntegral(ch_thr, roll, pitch, yaw);
    demanded_pitch = constrain(get_pitch_demand(), demanded_pitch_min, demanded_pitch_max);
    
    roll_cmd  = 1.3f*(constrain(nav_roll_cd()/100, demanded_roll_min, demanded_roll_max));
    pitch_cmd = 1.1f*(demanded_pitch);
    u_roll = ((-fw_gain.rate_roll_P * (-roll + roll_int - (roll_cmd + trim_roll))) + (-fw_gain.rate_roll_D * (gy)));
    u_pitch = ((fw_gain.rate_pitch_P * (-pitch + pitch_int - (-pitch_cmd + trim_pitch))) + (-fw_gain.rate_pitch_D * (gx)));


    servos[0] = (selector[0][0] * u_roll + selector[0][1] * u_pitch + selector[0][2] * u_yaw); // aileron right
    servos[1] = (selector[1][0] * u_roll + selector[1][1] * u_pitch + selector[1][2] * u_yaw); // aileron left
    servos[2] = (selector[2][0] * u_roll + selector[2][1] * u_pitch + selector[2][2] * u_yaw); // elevator
    servos[3] = (selector[3][0] * u_roll + selector[3][1] * u_pitch + selector[3][2] * u_yaw); // rudder

    servos[0] = constrain(servos[0], -400, 400) ;
    servos[1] = constrain(servos[1], -400, 400) ;
    servos[2] = constrain(servos[2], -400, 400) ;
    servos[3] = constrain(servos[3], -400, 400) ;
}

int FW_CONTROL::get_auto_throttle() {
  auto_throttle = fw_gain.rate_thr_P * auto_state.distance_next_wp ; // Implementasi PID throttle jika diperlukan
  auto_throttle = constrain(auto_throttle, auto_throttle_min, auto_throttle_max);
  return auto_throttle;
}


FW_CONTROL fw_control;
#endif