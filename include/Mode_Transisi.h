#pragma once
#include "Arduino.h"
#include "Radio.h"
#include "Mode_setup.h"
#include "Attitude.h"
#include "Actuator.h"
#include "flywing_control.h"
#include "Copter_Control.h"

// ================== MIXER PUSHER & QUAD ==================
int16_t PUSHER_PWM_MIN = 988;   // idle/off
int16_t PUSHER_PWM_MAX = 988; //1800  // cruise power
int16_t HOVER_PWM_MIN = 988;   // idle/off
int16_t HOVER_PWM_MAX = 988;

float cek_alpha_pusher;
float cek_alpha_hover;

int PUSHER_PWM, QUAD_PWM_THR;

static inline float _clampf(float x, float a, float b){
  if (x < a) return a; if (x > b) return b; return x;
}

void vtol_pusher_set_mix(float alpha){
  alpha = _clampf(alpha, 0.0f, 1.0f);
  PUSHER_PWM = (int)(PUSHER_PWM_MIN + (PUSHER_PWM_MAX - PUSHER_PWM_MIN) * alpha);
}

void vtol_quads_set_mix(float alpha){
  alpha = _clampf(alpha, 0.0f, 1.0f);
  QUAD_PWM_THR = (int)(HOVER_PWM_MIN + (HOVER_PWM_MAX - HOVER_PWM_MIN) * alpha);
}

// void update_max_pwm_from_remote() {
//     // Get max throttle position from last few seconds
//     static int max_throttle = 988;
//     max_throttle = max(max_throttle, ch_throttle);
    
//     // Slowly decay max_throttle value over time
//     if (millis() % 1000 == 0) { // Check every second
//         max_throttle = max(988, max_throttle - 10);
//     }
    
//     // Update max PWM values with some headroom
//     PUSHER_PWM_MAX = constrain(max_throttle + 100, 988, 1900);
//     HOVER_PWM_MAX = constrain(max_throttle + 100, 1000, 1900);
// }

// ================== STATE TRANSISI ==================
bool          is_transitioning   = false;
bool          trans_to_fw        = false;
unsigned long trans_start_ms     = 0;
float         alpha              = 0.0f;
float         alpha_hover        ;
float         alpha_pusher       ;     // 0..1 progress ramp
const float   RAMP_TIME          = 1.0f;     // s

// latch speed sederhana
bool          latched_speed_ok   = false;
// speed_reach_ms: tidak dipakai (dead variable) — timeout pakai trans_start_ms

// ================== KONFIG CEPAT ==================
static const bool  USE_AIRSPEED       = false;
static const float SPEED_MIN_MPS      = 16.0f;    // 12 UNTUK AIRSPEED // syarat minimal dianggap "cukup"
static const float SPEED_RELAX_MPS    = 10.0f;    // hysteresis sederhana
static const float ALPHA_BEFORE_SPEED = 0.60f;    // batasi pusher sebelum speed ok
static const float ALPHA_BEFORE_SPEED_HOVER = 0.45f;  // hover will maintain 60% power (1.0 - 0.40)
static const float ALPHA_BEFORE_SPEED_PUSHER = 0.85f; // pusher will reach 80% power
static const float RAMP_TIME_PUSHER_FW = 2.0f;    // Faster ramp for pusher
static const float RAMP_TIME_HOVER_FW = 5.0f;     // Slower ramp for hover
static const float RAMP_TIME_PUSHER_MC = 1.0f;    // Faster ramp for pusher
static const float RAMP_TIME_HOVER_MC = 1.0f;     // Slower ramp for hover
static const float FORCE_TRANSITION_SEC = 7.0f;   // force transisi ke FBWA setelah N detik

// ================== API MULAI TRANSISI ==================
void start_transition_to_fw(){
  is_transitioning   = true;
  trans_to_fw        = true;
  HOVER_PWM_MAX = ch_throttle;
  PUSHER_PWM_MAX = ch_throttle;
  trans_start_ms     = millis();
  alpha_hover            = 0.0f;
  alpha_pusher           = 0.0f;
  latched_speed_ok   = false;
  // Serial.println("Mulai transisi MC -> FW");
}

void start_transition_to_mc(){
  is_transitioning   = true;
  trans_to_fw        = false;
  PUSHER_PWM_MAX = ch_throttle;
  HOVER_PWM_MAX = ch_throttle;
  trans_start_ms     = millis();
  alpha_hover            = 0.0f;
  alpha_pusher           = 0.0f;
  // Serial.println("Mulai transisi FW -> MC");
}

// ================== KONTROL SELAMA TRANSISI ==================
void phase_transisi() {
  alt_hold   = true;   // tahan ketinggian
  mode_hover = true;

  // Quad
  min_roll = -35.0f;
  max_roll = 35.0f;
  copter_ControlFSFB(ch_roll, ch_pitch, ch_yaw, ch_throttle,
                     imu.roll, imu.pitch, imu.yaw,
                     imu.gyro_y, imu.gyro_x, imu.gyro_z);
  copter_calcOutput(QUAD_PWM_THR);
  motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);

  // Flywing
    fw_control.updateFBWA_FW(ch_roll, ch_pitch, ch_yaw, ch_throttle,
                          imu.roll, imu.pitch, imu.yaw,
                          imu.gyro_y, imu.gyro_x, imu.gyro_z);
    fw_servos_out_fbwa();
    plane_motors_out();

  // Pusher pakai mixer
  plane_motors_out_auto(PUSHER_PWM);

  mode_phase = 2;
  current    = ModeId::TRNS;
}

// ================== UPDATE TRANSISI (SIMPLE + FAILSAFE) ==================
void update_transition(){
  if(!is_transitioning) return;

  // ---- variabel FORCE (failsafe) lokal-static ----
  static bool     forcing = false;          // sedang ramp paksa ke tujuan?
  static bool     force_target_fw = false;  // true→paksa ke FW, false→paksa ke MC
  static uint32_t force_ms = 0;
  static uint32_t last_seen_start_ms = 0;
  const  float    FORCE_RAMP_SEC = 0.6f;    // ramp cepat saat fail-safe

  // update_max_pwm_from_remote();
  // reset force saat transisi baru dimulai
  if (last_seen_start_ms != trans_start_ms){
    last_seen_start_ms = trans_start_ms;
    forcing = false;
  }

  // ---- cek FAILSAFE dari switch ----
  // - Jika sedang MC->FW dan pilot balik ke 1 (MC) → paksa kembali MC
  // - Jika sedang FW->MC dan pilot balik ke 2 (FW) → paksa kembali FW
  if (!forcing){
    if (trans_to_fw && mode_now == 1){
      forcing = true; force_target_fw = false; force_ms = millis();
      // Serial.println("[FS] Paksa batal: kembali ke MC");
    } else if (!trans_to_fw && mode_now == 2){
      forcing = true; force_target_fw = true;  force_ms = millis();
      // Serial.println("[FS] Paksa batal: kembali ke FW");
    }
  }

  // jalankan kontrol gabungan (tetap setiap frame)
  phase_transisi();

  // ---- MODE FORCE: ramp singkat menuju target ----
  if (forcing){
    float tau = _clampf((millis() - force_ms) * 0.001f / FORCE_RAMP_SEC, 0.0f, 1.0f);
    if (force_target_fw){
      // paksa ke FW
      vtol_quads_set_mix(1.0f - tau);
      vtol_pusher_set_mix(tau);
      alpha = tau;
      if (tau >= 1.0f){
        forcing = false; is_transitioning = false;
        modes.setMode(ModeId::FBWA); active_mode = ModeId::FBWA;
        // Serial.println("[FS] Selesai paksa → FBWA");
      }
    } else {
      // paksa ke MC
      vtol_quads_set_mix(tau);
      vtol_pusher_set_mix(1.0f - tau);
      alpha = tau;
      if (tau >= 1.0f){
        forcing = false; is_transitioning = false;
        modes.setMode(ModeId::QHOV); active_mode = ModeId::QHOV;
        // Serial.println("[FS] Selesai paksa → COPT");
      }
    }
    return; // frame ini cukup
  }

  float t = (millis() - trans_start_ms) * 0.001f;

  if (trans_to_fw){
    // latch speed sederhana (opsional pakai groundspeed kalau tidak ada airspeed)
  float prog_pusher_fw = _clampf(t / RAMP_TIME_PUSHER_FW, 0.0f, 1.0f);
  float prog_hover_fw = _clampf(t / RAMP_TIME_HOVER_FW, 0.0f, 1.0f);

    const float v_now = ahrs.groundspeed;
    if (!latched_speed_ok && v_now >= SPEED_MIN_MPS){
      latched_speed_ok = true;
      // speed_reach_ms = millis();
    }
    if (latched_speed_ok && v_now < SPEED_RELAX_MPS){
      latched_speed_ok = false; // lepas latch bila turun jauh (hysteresis ringan)
    }

    float alpha_pusher = prog_pusher_fw;
    float alpha_hover =  prog_hover_fw;

    if (!latched_speed_ok && alpha_pusher > ALPHA_BEFORE_SPEED_PUSHER) {
        alpha_pusher = ALPHA_BEFORE_SPEED_PUSHER;
    }

    // Keep hover at minimum 60% before reaching speed
    if (!latched_speed_ok && alpha_hover > ALPHA_BEFORE_SPEED_HOVER) {
        alpha_hover = ALPHA_BEFORE_SPEED_HOVER;
    }

    // alpha = alpha_pusher; // For progress tracking

    float mix_hover = 1.0f - alpha_hover;

    vtol_quads_set_mix(mix_hover);  // 60% minimum hover power
    vtol_pusher_set_mix(alpha_pusher);       // 80% maximum pusher power
    // kode sebelumnya
    // vtol_quads_set_mix(1.0f - alpha);
    // vtol_pusher_set_mix(alpha);
    cek_alpha_pusher = alpha_pusher;
    cek_alpha_hover = mix_hover;


    // Kondisi 1: ramp selesai normal (speed threshold terpenuhi)
    // Kondisi 2: force transition — sudah >= FORCE_TRANSITION_SEC detik
    bool force_timeout = (t >= FORCE_TRANSITION_SEC);

    if ((alpha_hover >= 1.0f && alpha_pusher >= 1.0f) || force_timeout){
      is_transitioning = false;
      PUSHER_PWM = PUSHER_PWM_MIN;
      QUAD_PWM_THR = HOVER_PWM_MIN;
      m1_pwm = m2_pwm = m3_pwm = m4_pwm = HOVER_PWM_MIN;
      motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
      plane_motors_out_auto(PUSHER_PWM);
      modes.setMode(ModeId::FBWA); active_mode = ModeId::FBWA;
      // if (force_timeout) Serial.println("[FORCE] Timeout 5s → FBWA");
      // else               Serial.println("Selesai: masuk FW");
    }
  } else {
    // float mix_hover = alpha_hover;
    // float mix_push  = 1.0f - prog_pusher;
    // float tau = _clampf((millis() - force_ms) * 0.001f / FORCE_RAMP_SEC, 0.0f, 1.0f);
    // FW -> MC: ramp biasa
    float prog_pusher_mc = _clampf(t / RAMP_TIME_PUSHER_MC, 0.0f, 1.0f);
    float prog_hover_mc = _clampf(t / RAMP_TIME_HOVER_MC, 0.0f, 1.0f);
    alpha_hover = prog_hover_mc;
    alpha_pusher = prog_pusher_mc;
    float mix_pusher = 1.0f - alpha_pusher;

    vtol_quads_set_mix(alpha_hover);
    vtol_pusher_set_mix(mix_pusher);
    // kode sebelumnya
    // vtol_quads_set_mix(alpha);
    // vtol_pusher_set_mix(1.0f - alpha);

    if (alpha_hover >= 1.0f && alpha_pusher >= 1.0f){
      is_transitioning = false;
      PUSHER_PWM = PUSHER_PWM_MIN;
      QUAD_PWM_THR = HOVER_PWM_MIN;
      // m1_pwm = m2_pwm = m3_pwm = m4_pwm = HOVER_PWM_MIN;
      // motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm); 
      plane_motors_out_auto(PUSHER_PWM);
      modes.setMode(ModeId::COPT); active_mode = ModeId::COPT;
      // Serial.println("Selesai: masuk MC");
    }

          // paksa ke MC
    // vtol_quads_set_mix(tau);
    // vtol_pusher_set_mix(1.0f - tau);
    // alpha = tau;
    // if (tau >= 1.0f){
    // forcing = false; is_transitioning = false;
    // modes.setMode(ModeId::COPT); active_mode = ModeId::COPT;
    // Serial.println("[FS] Selesai paksa → COPT");
  }
}
