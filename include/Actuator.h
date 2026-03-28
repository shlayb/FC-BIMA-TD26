// TUNINGAN WAHANA UTAMA LOMBA REGIONAL
#ifndef ACTUATOR_H
#define ACTUATOR_H
#include <Arduino.h>
#include <Servo.h>
#include "FW_ControlModes.h"
#include "Copter_Control.h"
#include "FW_control_fbwa.h"
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

//PIN FC BARU
#define SERVO_AIL_L 2  //MTR 1
#define SERVO_ELE 5    //MTR 8
#define THROTTLE 10  //MTR 4
#define SERVO_RUD_L 4 //MTR 2
#define SERVO_AIL_R 12 //MTR 5
#define SERVO_RUD_R 6 //MTR 3
#define SERVO_PAYLOAD 9 //MTR 9
#define MOTOR_1_PIN 14  
#define MOTOR_2_PIN 20 //FC pin 8
#define MOTOR_3_PIN 33  //FC pin 9
#define MOTOR_4_PIN 13 //FC pin 10
//Pin belum dipake =>> mtr 11 = Pin 13

Servo motor1, motor2, motor3, motor4;
Servo aileron_L; 
Servo aileron_R;
Servo elevator;
Servo rudder_L;
Servo rudder_R;
Servo pusher;
Servo payload;

// Payload drop timing variables
uint32_t payload_drop_start_time = 0;  // Waktu mulai drop
const uint32_t PAYLOAD_DROP_DURATION = 500; // Durasi drop dalam milliseconds (0.5 detik)
bool payload_dropping = false;  // Flag untuk tracking status drop

FW_FWBA fw_fbwa;

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
    uint16_t servo_out;
    servo_out = (11.378 * angle) + 1500;
    return servo_out;
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

void init_actuator() {
    pinMode(SERVO_AIL_L, OUTPUT);
    pinMode(SERVO_ELE, OUTPUT);
    pinMode(SERVO_RUD_L, OUTPUT);
    pinMode(SERVO_AIL_R, OUTPUT);
    pinMode(SERVO_RUD_R, OUTPUT);
    pinMode(THROTTLE, OUTPUT);
    pinMode(MOTOR_1_PIN, OUTPUT);
    pinMode(MOTOR_2_PIN, OUTPUT);
    pinMode(MOTOR_3_PIN, OUTPUT);
    pinMode(MOTOR_4_PIN, OUTPUT);
    pinMode(SERVO_PAYLOAD, OUTPUT);

    aileron_L.attach(SERVO_AIL_L);
    elevator.attach(SERVO_ELE);
    rudder_L.attach(SERVO_RUD_L);
    aileron_R.attach(SERVO_AIL_R);
    rudder_R.attach(SERVO_RUD_R);
    payload.attach(SERVO_PAYLOAD);
    
    pusher.attach(THROTTLE);

    motor1.attach(MOTOR_1_PIN);
    motor2.attach(MOTOR_2_PIN);
    motor3.attach(MOTOR_3_PIN);
    motor4.attach(MOTOR_4_PIN);

    motor1.writeMicroseconds(988);
    motor2.writeMicroseconds(988);
    motor3.writeMicroseconds(988);
    motor4.writeMicroseconds(988);
    pusher.writeMicroseconds(988);
    payload.writeMicroseconds(1000); // default hold
    servo_pwm_payload = 1000;        // make sure the variable is initialized

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

    // Ensure the servo command is within a valid PWM range
    servo_pwm_payload = constrain(servo_pwm_payload, 988, 2012);
    payload.writeMicroseconds(servo_pwm_payload);
}

// ============================================================================
// UNIFIED PAYLOAD CONTROL - Menggabungkan manual dan auto tanpa konflik
// ============================================================================
// Dengan delay untuk memastikan servo sempat memegang posisi drop
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
            servo_pwm_payload = 2000; // drop
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
  throttle = ch_thr;
  int control1 = (int)(omega2[0]*M_CONST);
  int control2 = (int)(omega2[1]*M_CONST);
  int control3 = (int)(omega2[2]*M_CONST);
  int control4 = (int)(omega2[3]*M_CONST);

  control1 = constrain(control1, -400, 400);
  control2 = constrain(control2, -400, 400);
  control3 = constrain(control3, -400, 400);
  control4 = constrain(control4, -400, 400);
  
  if(arming){
    m1_pwm = control1 + throttle ;
    m2_pwm = control2 + throttle ;
    m3_pwm = control3 + throttle ;
    m4_pwm = control4 + throttle ;
  }
  else{
    m1_pwm = 988;
    m2_pwm = 988;
    m3_pwm = 988;
    m4_pwm = 988;
  }
    m1_pwm = constrain(m1_pwm, PWM_MIN, PWM_MAX);
    m2_pwm = constrain(m2_pwm, PWM_MIN, PWM_MAX);
    m3_pwm = constrain(m3_pwm, PWM_MIN, PWM_MAX);
    m4_pwm = constrain(m4_pwm, PWM_MIN, PWM_MAX);
}

void servos_out_manual() {
    aileron_L.writeMicroseconds((ch_roll));  // 45 = trim wahana FW
    aileron_R.writeMicroseconds((ch_roll));  // TODO: besok 24 feb di 0 in dulu
    elevator.writeMicroseconds((ch_pitch));
    rudder_L.writeMicroseconds(ch_yaw);
    rudder_R.writeMicroseconds(ch_yaw);
}

void fw_servos_out_fbwa() {
    // Hitung output servo dari kontrol pitch, roll, yaw
    pwm_ail_L = fw_fbwa.servos[0] + 1500;
    pwm_ail_R = fw_fbwa.servos[1] + 1500;       
    pwm_ele   = fw_fbwa.servos[2] + 1500;
    pwm_rud   = fw_fbwa.servos[3] + 1500;

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
  throttle = ch_thr;
  int control1 = (int)(omega2[0]*M_CONST);
  int control2 = (int)(omega2[1]*M_CONST);
  int control3 = (int)(omega2[2]*M_CONST);
  int control4 = (int)(omega2[3]*M_CONST);
  
  servo_yaw = int(u4 * M_CONST * 600000000); // untuk yaw

  control1 = constrain(control1, -400, 400);
  control2 = constrain(control2, -400, 400);
  control3 = constrain(control3, -400, 400);
  control4 = constrain(control4, -400, 400);
  servo_yaw = constrain(servo_yaw, -512, 512);
  
  if(arming){
    m1_pwm = control1 + throttle ;
    m2_pwm = control2 + throttle ;
    m3_pwm = control3 + throttle ;
    m4_pwm = control4 + throttle ;
  }
  else{
    m1_pwm = 988;
    m2_pwm = 988;
    m3_pwm = 988;
    m4_pwm = 988;
  }
    m1_pwm = constrain(m1_pwm, PWM_MIN, PWM_MAX);
    m2_pwm = constrain(m2_pwm, PWM_MIN, PWM_MAX);
    m3_pwm = constrain(m3_pwm, PWM_MIN, PWM_MAX);
    m4_pwm = constrain(m4_pwm, PWM_MIN, PWM_MAX);
    // Untuk yaw
    servo_L = basePWM - servo_yaw; // 1500 + 512
    servo_R = basePWM - servo_yaw; // 1500 - 512
}

void motor_loop_nyt(int pwm1, int pwm2, int pwm3, int pwm4, int pwm_yaw_l = 0, int pwm_yaw_r = 0) {
    motor1.writeMicroseconds(pwm1);
    motor2.writeMicroseconds(pwm2);
    motor3.writeMicroseconds(pwm3);
    motor4.writeMicroseconds(pwm4);
    aileron_L.writeMicroseconds(pwm_yaw_l); // untuk yaw kiri
    aileron_R.writeMicroseconds(pwm_yaw_r); // untuk yaw kanan

    // motor1.writeMicroseconds(ch_throttle);
    // motor2.writeMicroseconds(ch_throttle);
    // motor3.writeMicroseconds(ch_throttle);
    // motor4.writeMicroseconds(ch_throttle);
}


//tambah pwm untuk yaw
void servos_out_manual_flywing() {
    // Output elevon (pitch + roll)
    aileron_L.writeMicroseconds(abs(basePWM - ((ch_pitch)+(ch_roll))));
    aileron_R.writeMicroseconds(abs(basePWM - ((ch_pitch)-(ch_roll))));

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
    pwm_ail_L = angleToPwm(u_roll);
    pwm_ail_R = angleToPwm(u_roll);
    pwm_ele   = angleToPwm(u_pitch);

    // Output ke elevon (pitch + roll)
    aileron_L.writeMicroseconds(abs(basePWM - ((pwm_ele)+(pwm_ail_L))));
    aileron_R.writeMicroseconds(abs(basePWM - ((pwm_ele)-(pwm_ail_R))));

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
    pwm_ail_L = angleToPwm(u_roll);
    pwm_ail_R = angleToPwm(u_roll);
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
   Percent = map(Percent, 0, 100, 988, 2012); 
   return Percent;
}

void plane_motors_out() {
    motor_pwm = ch_throttle;
    if (mode_fbwa_plane) {                                                     // mode_fbwa
        motor_pwm = constrain(motor_pwm, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);  // pwmnya gabisa maks 100 persen krn diatur sesuai min dan max throttle
    }
    if (arming) {
        pusher.writeMicroseconds(motor_pwm);  // ch_throttle (jgn pake ini dulu)
    } else if (!arming) {
        pusher.writeMicroseconds(988);
    }
}

void plane_motors_out_auto(float pwm) {
    motor_pwm = pwm;
    motor_pwm = constrain(motor_pwm, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);
    if (arming) {
        pusher.writeMicroseconds(motor_pwm);
    } else if (!arming) {
        pusher.writeMicroseconds(988);
    }
}

int32_t commanded_throttle; 
/** calc throttle  */
void calc_throttle()
{
    /** check if throttle is disabled */
    commanded_throttle = PercenttoPWM(get_throttle_demand());
    commanded_throttle = scaleToPercent_reversed(commanded_throttle);
    commanded_throttle = constrain(commanded_throttle, MIN_THROTTLE_PWM, MAX_THROTTLE_PWM);
    if (arming) {
        pusher.writeMicroseconds(commanded_throttle);
    } else if (!arming) {
        pusher.writeMicroseconds(988);
    }
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
    // float throttle_pwm = ch_throttle;
    // // float yaw_diff = ch_yaw * 0.5f;
    // float yaw_diff = ((ch_yaw - 1500) * 0.5f); // untuk yaw di 1500 (0) di tengah
    // m1_pwm = constrain(throttle_pwm - yaw_diff, PWM_MIN, PWM_MAX);
    // m3_pwm = constrain(throttle_pwm + yaw_diff, PWM_MIN, PWM_MAX);

    // // Matikan motor depan & belakang
    // m2_pwm = 988;
    // m4_pwm = 988;

    // if (arming) {
    //     motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
    // } else {
    //     motor_loop(988, 988, 988, 988);
    // }
}

void motorCopterOff(int pwm1, int pwm2, int pwm3, int pwm4) {
    motor1.writeMicroseconds(pwm1);
    motor2.writeMicroseconds(pwm2);
    motor3.writeMicroseconds(pwm3);
    motor4.writeMicroseconds(pwm4);
    // pusher.writeMicroseconds(988);
}

#endif