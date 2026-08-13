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
int16_t PUSHER_PWM_MAX = 2012;  // pusher max = full throttle
int16_t HOVER_PWM_MIN = 988;   // idle/off
int16_t HOVER_PWM_MAX = 988;

float cek_alpha_pusher;
float cek_alpha_hover;

// ================== RAMP PUSHER dari REMOTE ==================
// Saat masuk transisi, pusher ramp dari 0 ke nilai remote
// sehingga tidak langsung loncat ke throttle pilot
static float pusher_ramp_alpha = 0.0f;  // 0..1, ramp dari 0 ke target remote
static const float PUSHER_RAMP_TIME = 2.0f;  // detik untuk ramp dari idle ke nilai remote

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
  // VTOL motor: latch dari throttle terakhir remote, ramp turun otomatis
  HOVER_PWM_MAX      = ch_throttle;
  // Pusher: max = 2012 (full range), pilot kontrol via remote, ramp dari 0
  PUSHER_PWM_MAX     = 2012;
  PUSHER_PWM_MIN     = 988;
  pusher_ramp_alpha  = 0.0f;  // reset ramp pusher ke 0
  trans_start_ms     = millis();
  alpha_hover            = 0.0f;
  alpha_pusher           = 0.0f;
  latched_speed_ok   = false;
  // Serial.println("Mulai transisi MC -> FW");
}

void start_transition_to_mc(){
  is_transitioning   = true;
  trans_to_fw        = false;
  // VTOL motor: latch dari throttle remote saat ini
  HOVER_PWM_MAX      = ch_throttle;
  // Pusher: ramp turun dari nilai saat ini ke 988
  PUSHER_PWM_MAX     = 2012;
  PUSHER_PWM_MIN     = 988;
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
    // JANGAN plane_motors_out() di sini — pusher dikendalikan penuh dari update_transition()
    // plane_motors_out() akan tulis ch_throttle langsung ke pusher tanpa kendali

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
  static int      force_pusher_start = 988; // latch PUSHER_PWM saat mulai force ke MC
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
      force_pusher_start = PUSHER_PWM;  // latch nilai pusher saat ini
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
      // paksa ke MC: naikkan hover, ramp turun pusher dari nilai saat masuk force
      vtol_quads_set_mix(tau);
      // Ramp pusher dari force_pusher_start (nilai saat force dimulai) → 988
      int pusher_ramp_down = (int)(force_pusher_start + (988 - force_pusher_start) * tau);
      PUSHER_PWM = constrain(pusher_ramp_down, 988, 2012);
      plane_motors_out_auto(PUSHER_PWM);
      alpha = tau;
      if (tau >= 1.0f){
        forcing = false; is_transitioning = false;
        PUSHER_PWM = 988;                         // pastikan pusher mati
        plane_motors_out_auto(988);               // kirim 988 ke ESC pusher
        modes.setMode(ModeId::QHOV); active_mode = ModeId::QHOV;
        // Serial.println("[FS] Selesai paksa → QHOV");
      }
    }
    return; // frame ini cukup
  }

  float t = (millis() - trans_start_ms) * 0.001f;

  if (trans_to_fw){
    // ---- VTOL motor: latch throttle awal, ramp turun otomatis ----
    float prog_hover_fw = _clampf(t / RAMP_TIME_HOVER_FW, 0.0f, 1.0f);

    // ---- Pusher: ramp dari 0, lalu ikuti remote (ch_throttle) ----
    // Ramp alpha naik dari 0 ke 1 dalam PUSHER_RAMP_TIME detik
    pusher_ramp_alpha = _clampf(t / PUSHER_RAMP_TIME, 0.0f, 1.0f);

    // Nilai target pusher dari remote: map ch_throttle (988..2012) -> 0..1
    float remote_pusher_alpha = _clampf(
        (ch_throttle - 988.0f) / (2012.0f - 988.0f), 0.0f, 1.0f);

    // Pusher PWM = ramp * nilai remote → dari 988 naik smooth ke setpoint pilot
    float alpha_pusher_target = pusher_ramp_alpha * remote_pusher_alpha;
    PUSHER_PWM = (int)(988.0f + (2012.0f - 988.0f) * alpha_pusher_target);
    PUSHER_PWM = constrain(PUSHER_PWM, 988, 2012);

    // ---- Hover: latch awal, ramp turun ----
    float alpha_hover = prog_hover_fw;

    // Speed latch (opsional, pakai groundspeed)
    const float v_now = ahrs.groundspeed;
    if (!latched_speed_ok && v_now >= SPEED_MIN_MPS){
      latched_speed_ok = true;
    }
    if (latched_speed_ok && v_now < SPEED_RELAX_MPS){
      latched_speed_ok = false;
    }

    // Batasi hover ramp sebelum speed tercapai
    if (!latched_speed_ok && alpha_hover > ALPHA_BEFORE_SPEED_HOVER) {
        alpha_hover = ALPHA_BEFORE_SPEED_HOVER;
    }

    float mix_hover = 1.0f - alpha_hover;
    vtol_quads_set_mix(mix_hover);

    // Output pusher langsung (sudah dihitung di atas, bypass vtol_pusher_set_mix)
    plane_motors_out_auto(PUSHER_PWM);

    cek_alpha_pusher = alpha_pusher_target;
    cek_alpha_hover  = mix_hover;

    // Kondisi selesai: hover ramp habis atau timeout
    bool force_timeout = (t >= FORCE_TRANSITION_SEC);
    if ((alpha_hover >= 1.0f) || force_timeout){
      is_transitioning = false;
      QUAD_PWM_THR = HOVER_PWM_MIN;
      m1_pwm = m2_pwm = m3_pwm = m4_pwm = HOVER_PWM_MIN;
      motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
      plane_motors_out_auto(PUSHER_PWM);
      modes.setMode(ModeId::FBWA); active_mode = ModeId::FBWA;
      // if (force_timeout) Serial.println("[FORCE] Timeout -> FBWA");
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
