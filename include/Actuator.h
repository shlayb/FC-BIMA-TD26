// TUNINGAN WAHANA UTAMA LOMBA REGIONAL
#ifndef ACTUATOR_H
#define ACTUATOR_H
#include <Arduino.h>
#include <Servo.h>
#include "FW_ControlModes.h"
#include "Copter_Control.h"
#include "FW_control.h"
// #include "copter_control_nyt.h"
#include "flywing_control.h"
#include "TECS.h"

// //Pin FC LAMA 
// #define SERVO_AIL_L 2   // 2 6 pwm 1 = pin 2
// #define SERVO_ELE 3     // 3 7
// #define THROTTLE 4      // 4 8
// #define SERVO_RUD_L 5   // 5 9
// #define SERVO_AIL_R 6   // 6 10
// #define SERVO_RUD_R 9  // 9 9
// #define MOTOR_1_PIN  10  
// #define MOTOR_2_PIN  11
// #define MOTOR_3_PIN  12
// #define MOTOR_4_PIN  14

//PIN FC BARU (Dibuat variabel float agar dapat dikonfigurasi via Params / EEPROM)
// 13 DAN 33 TERBALIK
// 36 DAN 37 TERBALIK
float SERVO_RUD_L   = 2;   // pin 2  - MTR 1  (CH_MTR_1)
float SERVO_RUD_R   = 13;  // pin 13 - MTR 11 (CH_MTR_11)
float SERVO_ELE     = 37;  // pin 37 - MTR 2  (CH_MTR_2)
float SERVO_AIL_L   = 33;   // pin 6  - MTR 3  (CH_MTR_3)
float SERVO_AIL_R   = 6;  // pin 33 - MTR 5  (CH_MTR_5)
float THROTTLE      = 10;  // pin 10 - MTR 4  (CH_MTR_4)
float SERVO_PAYLOAD = 36;  // pin 36 - MTR 6  (CH_MTR_6)
float MOTOR_1_PIN   = 3;   // pin 3  - MTR 7  (Tidak dipakai)
float MOTOR_2_PIN   = 5;   // pin 5  - MTR 8  (Tidak dipakai) - FC pin DICEK LAGI
float MOTOR_3_PIN   = 9;   // pin 9  - MTR 9  (Tidak dipakai)
float MOTOR_4_PIN   = 11;  // pin 11 - MTR 10 (Tidak dipakai)

// #define MTR_1_PIN 9  // MTR 9 (Throttle Kanan)
// #define MTR_2_PIN 2  // MTR 1  (Throttle Kiri) - Changed from pin 6 to pin 2 (PWM capable, no conflicts)

Servo motor1, motor2, motor3, motor4;
Servo aileron_L; 
Servo aileron_R;
Servo elevator;
Servo rudder_L;
Servo rudder_R;
Servo pusher;
Servo payload;
Servo mtr_1; 
Servo mtr_2; 

// Forward declaration untuk dual motor throttle
// void dual_motor_throttle_out(int pwm_throttle);
// void dual_motor_throttle_out_yaw(int pwm_base, float yaw_diff);

// Payload drop timing variables
uint32_t payload_drop_start_time = 0;  // Waktu mulai drop
const uint32_t PAYLOAD_DROP_DURATION = 500; // Durasi drop dalam milliseconds (0.5 detik)
bool payload_dropping = false;  // Flag untuk tracking status drop

float u_roll, u_pitch, u_yaw, u_yaw_cmd;
float m1_pwm, m2_pwm, m3_pwm, m4_pwm;
float servo_L, servo_R;
int servo_yaw;
float high_out = 90;
int basePWM = 1500;

/*NOTE: remote yang dipakai tahun lalu punya koreksi sebesar 12
jadi semua PWM yang defaultnya 1000 (min) dan 2000 (max) ditambah dan dikurangi dgn 12
tapi untuk nilai median PWM nya tetep 1500
cara tahu koreksinya gimana? pilot yang bisa ngerasain.
kalo remote masih bagus. stick/dongle seret ga lecek buat mainan harusnya gausa koreksi/koreksinya dikit bgt*/
uint16_t servo_max = 2012;
uint16_t servo_min = 988;
uint16_t pwm_ail_L;
uint16_t pwm_ail_R;
uint16_t pwm_ele;
uint16_t pwm_rud;
uint16_t pwm_rud_L;
uint16_t pwm_rud_R;
uint16_t throttle;

/*NOTE: decimal value generated from display eq serial plotter (based on remote type)
link excel/spreadsheet:
*/

// scale pwm to -45 until 45 (servo, angle)
float scaleToAngle(uint16_t ch) {
    float scaledOutput = 0.0879 * ch - 131.46;
    return scaledOutput;
}

// scale pwm to 0 until 100 (throttle, percent)
float scaleToPercent(uint16_t ch) {
    float scaledOutput = 0.0976 * ch - 96.067;
    return scaledOutput;
}

float scaleToPercent_reversed(uint16_t ch) {
    // float scaledpercent = 10.24 * ch - 988.0;
    // return scaledpercent;

    float scaled = (ch + 100) * (2012 - 988) / 200 + 988;
    return scaled;
}

// kebalikan scale to angle
uint16_t angleToPwm(float angle /*, uint16_t servo_trim*/) {
    if (isnan(angle)) return 1500;
    int servo_out = (int)(11.378f * angle) + 1500;
    return (uint16_t)constrain(servo_out, 988, 2012);
}

// ignore (karena dulu di reversenya dr sinyal keluaran kendali)
uint16_t angleToPwm_reversed(float angle /*, uint16_t servo_trim*/) {
    uint16_t servo_out;
    servo_out = (11.378 * (-angle)) + 1500;
    return servo_out;
}

void servo_idle() {
    servo_pwm_ail_L = 0;
    servo_pwm_ail_R = 0;
    servo_pwm_ele = 0;
    servo_pwm_rud_L = 0;
    servo_pwm_rud_R = 0;
}

extern float model_uav;

void init_actuator() {
    pinMode((int)SERVO_AIL_L, OUTPUT);
    pinMode((int)SERVO_ELE, OUTPUT);
    pinMode((int)SERVO_RUD_L, OUTPUT);
    pinMode((int)SERVO_AIL_R, OUTPUT);
    pinMode((int)SERVO_RUD_R, OUTPUT);
    pinMode((int)THROTTLE, OUTPUT);
    pinMode((int)SERVO_PAYLOAD, OUTPUT);

    aileron_L.attach((int)SERVO_AIL_L);
    elevator.attach((int)SERVO_ELE);
    rudder_L.attach((int)SERVO_RUD_L);
    aileron_R.attach((int)SERVO_AIL_R);
    rudder_R.attach((int)SERVO_RUD_R);
    payload.attach((int)SERVO_PAYLOAD);
    pusher.attach((int)THROTTLE);

    // Langsung center ke 1500us saat boot agar servo tidak nyangkut/mentok
    aileron_L.writeMicroseconds(1500);
    aileron_R.writeMicroseconds(1500);
    elevator.writeMicroseconds(1500);
    rudder_L.writeMicroseconds(1500);
    rudder_R.writeMicroseconds(1500);
    pusher.writeMicroseconds(988);
    payload.writeMicroseconds(1000); // default hold
    servo_pwm_payload = 1000;        // make sure the variable is initialized

    int model = (int)(model_uav + 0.5f);
    if (model == 1) { // MODEL_UAV_VTOL
        pinMode((int)MOTOR_1_PIN, OUTPUT);
        pinMode((int)MOTOR_2_PIN, OUTPUT);
        pinMode((int)MOTOR_3_PIN, OUTPUT);
        pinMode((int)MOTOR_4_PIN, OUTPUT);

        motor1.attach((int)MOTOR_1_PIN);
        motor2.attach((int)MOTOR_2_PIN);
        motor3.attach((int)MOTOR_3_PIN);
        motor4.attach((int)MOTOR_4_PIN);

        motor1.writeMicroseconds(988);
        motor2.writeMicroseconds(988);
        motor3.writeMicroseconds(988);
        motor4.writeMicroseconds(988);
        Serial.println("VTOL Motors initialized");
    }

    Serial.println("[ACTUATOR] Pin Mapping Active:");
    Serial.printf("  MTR 3 (AIL_L)   : Pin %d\n", (int)SERVO_AIL_L);
    Serial.printf("  MTR 2 (ELE)     : Pin %d\n", (int)SERVO_ELE);
    Serial.printf("  MTR 1 (RUD_L)   : Pin %d\n", (int)SERVO_RUD_L);
    Serial.printf("  MTR 5 (AIL_R)   : Pin %d\n", (int)SERVO_AIL_R);
    Serial.printf("  MTR 11 (RUD_R)  : Pin %d\n", (int)SERVO_RUD_R);
    Serial.printf("  MTR 4 (THROTTLE): Pin %d\n", (int)THROTTLE);
    Serial.printf("  MTR 6 (PAYLOAD) : Pin %d\n", (int)SERVO_PAYLOAD);
    Serial.println("Actuator setup complete");
}

void motor_loop(int pwm1, int pwm2, int pwm3, int pwm4) {
    motor1.writeMicroseconds(pwm1);
    motor2.writeMicroseconds(pwm2);
    motor3.writeMicroseconds(pwm3);
    motor4.writeMicroseconds(pwm4);

    // motor1.writeMicroseconds(ch_throttle);
    // motor2.writeMicroseconds(ch_throttle);
    // motor3.writeMicroseconds(ch_throttle);
    // motor4.writeMicroseconds(ch_throttle);
}

void payload_loop() {
    // Update payload release position based on a dedicated switch (ch_vehicle_mode)
    // - ch_vehicle_mode > 1500: drop
    // - ch_vehicle_mode <= 1500: hold
    if (!arming) {
        servo_pwm_payload = 1000; // safe position while disarmed
    } else {
        if (ch_vehicle_mode > 1500) {
            servo_pwm_payload = 2000; // drop
        } else {
            servo_pwm_payload = 1000; // hold
        }
    }

    // Ensure the servo command is within a valid PWM range
    servo_pwm_payload = constrain(servo_pwm_payload, 988, 2012);
    payload.writeMicroseconds(servo_pwm_payload);
}

// Auto payload loop - triggered by "DROP" message from Raspberry Pi via MAVLink
void autopayload_loop() {
    // Update payload release position based on MAVLink command from Raspberry Pi
    // - payload_drop_command = true: drop payload
    // - payload_drop_command = false: hold payload
    if (!arming) {
        servo_pwm_payload = 1000; // safe position while disarmed
        payload_drop_command = false; // reset command jika disarmed
    } else {
        if (payload_drop_command) {
            servo_pwm_payload = 2000; // drop
            payload_drop_command = false; // reset command after triggering
            Serial.println("AUTOPAYLOAD: Payload dropped via MAVLink command");
        } else {
            servo_pwm_payload = 1000; // hold
        }
    }
    servo_pwm_payload = constrain(servo_pwm_payload, 988, 2012);
    payload.writeMicroseconds(servo_pwm_payload);
}

// UNIFIED PAYLOAD CONTROL - manual and auto 
void payload_control_unified() {
    uint32_t current_time = millis();
    
    if (!arming) {
        // Saat disarmed, set ke posisi aman
        servo_pwm_payload = 1000; // safe position while disarmed
        payload_drop_command = false; // reset command jika disarmed
        payload_dropping = false;
        payload_drop_start_time = 0;
    } else {
        // Prioritas 1: Cek perintah AUTO dari Raspberry Pi (MAVLink)
        if (payload_drop_command) {
            // Mulai proses drop
            if (!payload_dropping) {
                payload_dropping = true;
                payload_drop_start_time = current_time;
                servo_pwm_payload = 2000; // drop
                // Serial.println("[PAYLOAD] AUTO DROP initiated via MAVLink");
            }
            
            // Cek apakah durasi drop sudah selesai
            if (current_time - payload_drop_start_time >= PAYLOAD_DROP_DURATION) {
                payload_drop_command = false; // reset command setelah delay
                payload_dropping = false;
                // Serial.println("[PAYLOAD] AUTO DROP completed");
            } else {
                // Masih dalam proses drop, pertahankan posisi 2000
                servo_pwm_payload = 2000;
            }
        } 
        // Prioritas 2: Gunakan kontrol MANUAL dari remote jika tidak ada auto command
        else if (ch_vehicle_mode > 1500) {
            servo_pwm_payload = 3000; // drop
            payload_dropping = false;  // reset flag untuk auto
            // Serial.println("[PAYLOAD] MANUAL DROP via Remote");
        } else {
            servo_pwm_payload = 1000; // hold
            payload_dropping = false;  // reset flag untuk auto
        }
    }

    // Pastikan command berada dalam range PWM yang valid
    servo_pwm_payload = constrain(servo_pwm_payload, 988, 2012);
    
    // WRITE KE SERVO HANYA SEKALI (bukan dua kali seperti sebelumnya)
    payload.writeMicroseconds(servo_pwm_payload);
}

void copter_calcOutput(int16_t ch_thr)
{
  // 1. Base throttle limit
  int thr_base = constrain(ch_thr, 988, 1850);
  throttle = thr_base;

  int control1 = (int)(omega2[0]*M_CONST);
  int control2 = (int)(omega2[1]*M_CONST);
  int control3 = (int)(omega2[2]*M_CONST);
  int control4 = (int)(omega2[3]*M_CONST);

  control1 = constrain(control1, -350, 350);
  control2 = constrain(control2, -350, 350);
  control3 = constrain(control3, -350, 350);
  control4 = constrain(control4, -350, 350);
  
  if(arming){
    // 2. Dynamic Mixer Desaturation:
    int raw1 = control1 + thr_base;
    int raw2 = control2 + thr_base;
    int raw3 = control3 + thr_base;
    int raw4 = control4 + thr_base;

    int max_raw = max(max(raw1, raw2), max(raw3, raw4));
    if (max_raw > PWM_MAX) {
        int excess = max_raw - PWM_MAX;
        thr_base -= excess;
    }

    // Under-saturation prevention
    int min_raw = min(min(control1 + thr_base, control2 + thr_base), min(control3 + thr_base, control4 + thr_base));
    if (min_raw < PWM_MIN && ch_thr > 1100) {
        int deficit = PWM_MIN - min_raw;
        thr_base += deficit;
    }

    m1_pwm = constrain(control1 + thr_base, PWM_MIN, PWM_MAX);
    m2_pwm = constrain(control2 + thr_base, PWM_MIN, PWM_MAX);
    m3_pwm = constrain(control3 + thr_base, PWM_MIN, PWM_MAX);
    m4_pwm = constrain(control4 + thr_base, PWM_MIN, PWM_MAX);
  }
  else{
    m1_pwm = 988;
    m2_pwm = 988;
    m3_pwm = 988;
    m4_pwm = 988;
  }
    
    Serial.print(ch_thr); Serial.print("| ");
    Serial.print(m1_pwm); Serial.print("| ");
    Serial.print(m2_pwm); Serial.print("| ");
    Serial.print(m3_pwm); Serial.print("| ");
    Serial.println(m4_pwm);
}

void servos_out_manual() {
    uint16_t safe_roll  = (ch_roll >= 900 && ch_roll <= 2100) ? ch_roll : 1500;
    uint16_t safe_pitch = (ch_pitch >= 900 && ch_pitch <= 2100) ? ch_pitch : 1500;
    uint16_t safe_yaw   = (ch_yaw >= 900 && ch_yaw <= 2100) ? ch_yaw : 1500;

    // aileron_L.writeMicroseconds(3000 - safe_roll);  // 45 = trim wahana FW
    // aileron_R.writeMicroseconds(3000 - safe_roll);  // TODO: besok 24 feb di 0 in dulu
    aileron_L.writeMicroseconds(safe_roll);
    aileron_R.writeMicroseconds(safe_roll);
    elevator.writeMicroseconds(safe_pitch);
    rudder_L.writeMicroseconds(safe_yaw);
    rudder_R.writeMicroseconds(safe_yaw);
}

void fw_servos_out_fbwa() {
    // Hitung output servo dari kontrol pitch, roll, yaw
    // pwm_ail_R = constrain(1500 - fw_control.servos[0], 988, 2012);
    // pwm_ail_L = constrain(1500 - fw_control.servos[1], 988, 2012);       
    pwm_ail_R = constrain(fw_control.servos[0] + 1500, 988, 2012);
    pwm_ail_L = constrain(fw_control.servos[1] + 1500, 988, 2012);       
    pwm_ele   = constrain(fw_control.servos[2] + 1500, 988, 2012);
    pwm_rud   = constrain(fw_control.servos[3] + 1500, 988, 2012);

    // Output ke servo
    aileron_L.writeMicroseconds(pwm_ail_L);
    aileron_R.writeMicroseconds(pwm_ail_R);
    elevator.writeMicroseconds(pwm_ele);
    rudder_L.writeMicroseconds(pwm_rud);
    rudder_R.writeMicroseconds(pwm_rud);
}

/////////////for flywing////////////////////////

// NYT
void copter_nyt_calcOutput(int16_t ch_thr)
{
  int thr_base = constrain(ch_thr, 988, 1850);
  throttle = thr_base;

  int control1 = (int)(omega2[0]*M_CONST);
  int control2 = (int)(omega2[1]*M_CONST);
  int control3 = (int)(omega2[2]*M_CONST);
  int control4 = (int)(omega2[3]*M_CONST);
  
  servo_yaw = int(u4 * M_CONST * 600000000); // untuk yaw

  control1 = constrain(control1, -350, 350);
  control2 = constrain(control2, -350, 350);
  control3 = constrain(control3, -350, 350);
  control4 = constrain(control4, -350, 350);
  servo_yaw = constrain(servo_yaw, -512, 512);
  
  if(arming){
    int raw1 = control1 + thr_base;
    int raw2 = control2 + thr_base;
    int raw3 = control3 + thr_base;
    int raw4 = control4 + thr_base;

    int max_raw = max(max(raw1, raw2), max(raw3, raw4));
    if (max_raw > PWM_MAX) {
        int excess = max_raw - PWM_MAX;
        thr_base -= excess;
    }

    m1_pwm = constrain(control1 + thr_base, PWM_MIN, PWM_MAX);
    m2_pwm = constrain(control2 + thr_base, PWM_MIN, PWM_MAX);
    m3_pwm = constrain(control3 + thr_base, PWM_MIN, PWM_MAX);
    m4_pwm = constrain(control4 + thr_base, PWM_MIN, PWM_MAX);
  }
  else{
    m1_pwm = 988;
    m2_pwm = 988;
    m3_pwm = 988;
    m4_pwm = 988;
  }

  // Untuk yaw
  servo_L = basePWM - servo_yaw; // 1500 + 512
  servo_R = basePWM - servo_yaw; // 1500 - 512
}

void motor_loop_nyt(int pwm1, int pwm2, int pwm3, int pwm4, int pwm_yaw_l = 0, int pwm_yaw_r = 0) {
    // motor1.writeMicroseconds(pwm1);
    // motor2.writeMicroseconds(pwm2);
    // motor3.writeMicroseconds(pwm3);
    // motor4.writeMicroseconds(pwm4);
    aileron_L.writeMicroseconds(pwm_yaw_l); // untuk yaw kiri
    aileron_R.writeMicroseconds(pwm_yaw_r); // untuk yaw kanan

    // motor1.writeMicroseconds(ch_throttle);
    // motor2.writeMicroseconds(ch_throttle);
    // motor3.writeMicroseconds(ch_throttle);
    // motor4.writeMicroseconds(ch_throttle);
}


//tambah pwm untuk yaw
void servos_out_manual_flywing() {
    uint16_t safe_roll  = (ch_roll >= 900 && ch_roll <= 2100) ? ch_roll : 1500;
    uint16_t safe_pitch = (ch_pitch >= 900 && ch_pitch <= 2100) ? ch_pitch : 1500;

    int pwm_l = constrain(abs(basePWM - ((safe_pitch)+(safe_roll))), 988, 2012);
    int pwm_r = constrain(abs(basePWM - ((safe_pitch)-(safe_roll))), 988, 2012);

    // Output elevon (pitch + roll)
    aileron_L.writeMicroseconds(pwm_l);
    aileron_R.writeMicroseconds(pwm_r);

    // Throttle + yaw untuk motor kanan & kiri
    // float throttle_pwm = ch_throttle;
    // // float yaw_diff = ch_yaw * 0.5f;
    // float yaw_diff = (ch_yaw - 1500) * 0.5f; // untuk yaw di 1500 (0) di tengah
    // m1_pwm = constrain(throttle_pwm + yaw_diff, PWM_MIN, PWM_MAX);
    // m3_pwm = constrain(throttle_pwm - yaw_diff, PWM_MIN, PWM_MAX);

    // // Matikan motor depan & belakang
    // m2_pwm = 988;
    // m4_pwm = 988;

    // if (arming) {
    //     motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
    // } else {
    //     motor_loop(988, 988, 988, 988);
    // }
}


void servos_out_fbwa_flywing() {
    // Hitung output elevon dari kontrol pitch dan roll
    pwm_ail_L = angleToPwm_reversed(u_roll);
    pwm_ail_R = angleToPwm_reversed(u_roll);
    pwm_ele   = angleToPwm(u_pitch);

    int pwm_l = constrain(abs(basePWM - ((pwm_ele)+(pwm_ail_L))), 988, 2012);
    int pwm_r = constrain(abs(basePWM - ((pwm_ele)-(pwm_ail_R))), 988, 2012);

    // Output ke elevon (pitch + roll)
    aileron_L.writeMicroseconds(pwm_l);
    aileron_R.writeMicroseconds(pwm_r);

    // Throttle dari autopilot / TECS
    float throttle_pwm = ch_throttle; // sudah dalam bentuk PWM

    // Yaw control via thrust differential
    float yaw_diff = u_yaw_cmd * 5.0f;  // Gain yaw bisa diubah sesuai tuning

    m1_pwm = constrain(throttle_pwm + yaw_diff, PWM_MIN, PWM_MAX);  // motor kanan
    m3_pwm = constrain(throttle_pwm - yaw_diff, PWM_MIN, PWM_MAX);  // motor kiri

    // Motor depan dan belakang dimatikan (cruise mode)
    m2_pwm = 988;
    m4_pwm = 988;

    if (arming) {
        motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
    } else {
        motor_loop(988, 988, 988, 988);
    }
}



// void servos_out_manual_flywing() {
//     aileron_L.writeMicroseconds (abs(basePWM - ((ch_pitch)+(ch_roll))));  // 45 = trim wahana FW
//     aileron_R.writeMicroseconds (abs(basePWM - ((ch_pitch)-(ch_roll))));  // TODO: besok 24 feb di 0 in dulu
//     rudder_L.writeMicroseconds(ch_yaw);
//     rudder_R.writeMicroseconds(ch_yaw);
// }

// void servos_out_fbwa_flywing() {
//     pwm_ail_L = angleToPwm(u_roll);
//     pwm_ail_R = angleToPwm(u_roll);
//     pwm_ele = angleToPwm(u_pitch);
//     pwm_rud_L = angleToPwm(u_yaw_cmd);
//     pwm_rud_R = angleToPwm(u_yaw_cmd);

//     aileron_L.writeMicroseconds(abs(basePWM - ((pwm_ele)+(pwm_ail_L)))); // TODO di 0 in dulu
//     aileron_R.writeMicroseconds(abs(basePWM - ((pwm_ele)-(pwm_ail_R)))); 
//     rudder_L.writeMicroseconds(pwm_rud_L);
//     rudder_R.writeMicroseconds(pwm_rud_R);
// }
//////////////////////////////////////////////////////

void servos_out_fbwa() {
    pwm_ail_L = angleToPwm_reversed(u_roll);
    pwm_ail_R = angleToPwm_reversed(u_roll);
    pwm_ele = angleToPwm(u_pitch);
    pwm_rud_L = angleToPwm(u_yaw_cmd);
    pwm_rud_R = angleToPwm(u_yaw_cmd);

    aileron_L.writeMicroseconds((pwm_ail_L));  // TODO di 0 in dulu
    aileron_R.writeMicroseconds((pwm_ail_R));
    elevator.writeMicroseconds((pwm_ele));
    rudder_L.writeMicroseconds(pwm_rud_L);
    rudder_R.writeMicroseconds(pwm_rud_R);
}



/*servo FW mode FBWA + auto*/
void servos_out_fbwa_auto()
{
    pwm_ail_L = angleToPwm(u_roll);
    pwm_ail_R = angleToPwm(u_roll);
    pwm_ele = angleToPwm(u_pitch);
    pwm_rud_L = angleToPwm(u_yaw_cmd);
    pwm_rud_R = angleToPwm(u_yaw_cmd);

    aileron_L.writeMicroseconds((pwm_ail_L));
    aileron_R.writeMicroseconds((pwm_ail_R));
    elevator.writeMicroseconds((pwm_ele-42));
    rudder_L.writeMicroseconds(pwm_rud_L);
    rudder_R.writeMicroseconds(pwm_rud_R);
}

float PercenttoPWM(float Percent){
   Percent = map(Percent, 0, 100, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM); 
   return Percent;
}

void plane_motors_out() {
    motor_pwm = ch_throttle;
    if (mode_fbwa_plane) {                                                     // mode_fbwa
        motor_pwm = constrain(motor_pwm, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);  // pwmnya gabisa maks 100 persen krn diatur sesuai min dan max throttle
    }
    if (arming) {
        pusher.writeMicroseconds(motor_pwm);  // ch_throttle
    } else if (!arming) {
        pusher.writeMicroseconds(MIN_THROTTLE_PWM);
    }
}

void dual_motor_throttle_out(int pwm_throttle) { //GA KEPAKE
    pwm_throttle = constrain(pwm_throttle, 988, 2012);
    if (arming) {
        mtr_1.writeMicroseconds(pwm_throttle);  // Kanan
        mtr_2.writeMicroseconds(pwm_throttle);  // Kiri (thrust sama)
    } else {
        mtr_1.writeMicroseconds(988);
        mtr_2.writeMicroseconds(988);
    }
}

// Jika ingin kontrol yaw via thrust differential:
void dual_motor_throttle_out_yaw() {
    // motor_pwm = ch_throttle;
    if (mode_now == 3) {
        motor_pwm = fw_control.get_auto_throttle(); // Dapatkan throttle dari kontrol otomatis
    } else {
        motor_pwm = ch_throttle; // Gunakan throttle manual saat belum di fase 3
    }
    float yaw_diff = (ch_yaw - 1500) * 0.3f;
    int pwm_right = constrain(motor_pwm + yaw_diff, 988, 2012);
    int pwm_left = constrain(motor_pwm - yaw_diff, 988, 2012);
    
    if (arming) {
        mtr_1.writeMicroseconds(pwm_right);  // Kanan
        mtr_2.writeMicroseconds(pwm_left);   // Kiri
    } else {
        mtr_1.writeMicroseconds(988);
        mtr_2.writeMicroseconds(988);
    }
}
void plane_motors_out_auto(float pwm = -1.0f) {
    if (pwm >= 0.0f) {
        motor_pwm = pwm;
    } else {
        motor_pwm = fw_control.get_auto_throttle();
    }
    motor_pwm = constrain(motor_pwm, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);
    if (arming) {
        pusher.writeMicroseconds(motor_pwm);  // Dapatkan throttle dari kontrol otomatis
    } else {
        pusher.writeMicroseconds(MIN_THROTTLE_PWM);
    }
}

int32_t commanded_throttle; 
/** calc throttle  */
void calc_throttle() {
    // commanded_throttle = PercenttoPWM(get_throttle_demand());
    // commanded_throttle = scaleToPercent_reversed(commanded_throttle);
    // commanded_throttle = constrain(commanded_throttle, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);
    
    // //dual_motor_throttle_out(commanded_throttle);  
    // // Ganti dari pusher.writeMicroseconds()
    // // Yaw differential dari autopilot
    // float yaw_diff = u_yaw_cmd * 100.0f;
    // dual_motor_throttle_out_yaw(commanded_throttle, yaw_diff);
}

void flywing_calcout()
{
  int control2 = (int)(flywingcontrol.servos[0]*M_CONST);
  int control3 = (int)(flywingcontrol.servos[1]*M_CONST);

  servo_R = constrain(control2, -400, 400);
  servo_L = constrain(control3, -400, 400);
}

void flywing_servos_out(int pwm_L, int pwm_R){
    aileron_L.writeMicroseconds(pwm_L + basePWM);
    aileron_R.writeMicroseconds(pwm_R + basePWM);

    // // Throttle + yaw untuk motor kanan & kiri
    float throttle_pwm = ch_throttle;
    // float yaw_diff = ch_yaw * 0.5f;
    float yaw_diff = ((ch_yaw - 1500) * 0.5f); // untuk yaw di 1500 (0) di tengah
    m1_pwm = constrain(throttle_pwm - yaw_diff, PWM_MIN, PWM_MAX);
    m2_pwm = constrain(throttle_pwm + yaw_diff, PWM_MIN, PWM_MAX);

    // Matikan motor depan & belakang
    // m2_pwm = 988;
    // m4_pwm = 988;

    if (arming) {
        motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
    } else {
        motor_loop(988, 988, 988, 988);
    }
}

void motorCopterOff(int pwm1, int pwm2, int pwm3, int pwm4) {
    motor1.writeMicroseconds(pwm1);
    motor2.writeMicroseconds(pwm2);
    motor3.writeMicroseconds(pwm3);
    motor4.writeMicroseconds(pwm4);
    pusher.writeMicroseconds(988);
}

#endif