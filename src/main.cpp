// #include <../lib/TeensyThreads/TeensyThreads.h>
#include <AHRS.h>
#include <Actuator.h>
#include <Arduino.h>
#include <buzzer.h>
#include <Fuzzy_FW_Pitch.h>
#include <Fuzzy_FW_Roll.h>
#include <Radio.h>
#include <Telemetry.h>
#include <Transition.h>
#include <takeoff.h>
#include <RaspberryPi_Telemtry.h>
#include <Mavlink.h>
#include <WP_EEPROM.h>
// #include <TimingUtils.h>
#include "FreeRTOS.h"    
#include "task.h"
#include "mode_setup.h"
#include "Mode_Manager.h"

 
#define SERIAL_USB Serial
#define SERIAL_UART Serial2
#define SERIAL_RASPI Serial7
#define CALIBRATE_MOTORS false
// pilih salah satu: 1 = fixedwing, 0 = flywing
#define MODEL_UAV_FIXEDWING 1

#define IMU_time 5
#define BARO_time 10
#define GPS_time 10
#define AIRSPEED_time 50
#define PRINT_time 50
#define CONTROL_time 5
#define FWRoll_time 20
#define RADIO_time 10
#define TELEM_time 10
#define MAVLINK_time 10
#define RASPI_time 100
#define BUZZER_time 100
#define VOLTAGE_time 100

uint32_t loop_timer = 0, debug_timer = 0;
uint32_t time_now;
uint32_t dt;
uint32_t time_last;

Telemetry telem;
GPSTelemetry rasprint;
MavlinkHandler mavHandler;

TaskHandle_t Task_IMU;
TaskHandle_t Task_BARO;
TaskHandle_t Task_GPS;
TaskHandle_t Task_Print;
TaskHandle_t Task_Raspi;
TaskHandle_t Task_Telem;
TaskHandle_t Task_Mavlink;
TaskHandle_t Task_Buzzer;
TaskHandle_t Task_Control;
TaskHandle_t Task_Voltage;

int line = 0;
int milisawal = 0;
uint32_t lastSecondPrint = 0;
void printUSB();

void updateIMU(void *pvParameters){ 
    for(;;){
        imu.update_imu();
        vTaskDelay(pdMS_TO_TICKS(IMU_time)); // Reduced delay for more frequent updates
    }
};

void updateBARO(void *pvParameters){
    for(;;){
        baro.update_baro();
        vTaskDelay(pdMS_TO_TICKS(BARO_time)); // Reduced delay for more frequent updates
    }
};

void GPS(void *pvParameters){
    for(;;){
        gepees.updateGPS();
        vTaskDelay(pdMS_TO_TICKS(GPS_time)); // Reduced delay for more frequent updates
    }
};

void Print_task(void *pvParameters){
    for(;;){
        printUSB();  // Original print function
        // aileron_L.writeMicroseconds();
        vTaskDelay(pdMS_TO_TICKS(PRINT_time)); // Reduced delay for more frequent updates
    }
};

void Print_To_Raspi(void *pvParameters){
    for(;;){
        rasprint.send_gps_to_rpi();
        vTaskDelay(pdMS_TO_TICKS(RASPI_time)); // Reduced delay for more frequent updates
    }
};

void Print_telem(void *pvParameters){
    for(;;){ 
        telem.pid_gain_tuning();
        // telem.send_vehicle_attitude_data();
        vTaskDelay(pdMS_TO_TICKS(TELEM_time)); // Reduced delay for more frequent updates
    }
};

void mavlink_print(void *pvParameters){
    for(;;){
        // Send Heartbeat every 1000ms (1 second)
        // mavSend(&mavHeartbeatMsg);
        // handlePortUSB();
        // handlePortTLM();
        // handlePortRPI();
        mavHandler.handlePorts();
        mavHandler.mavlinkTask();
        vTaskDelay(pdMS_TO_TICKS(MAVLINK_time)); // Reduced delay for more frequent updates
    }
};

void airspeed(void *pvParameters){
    for(;;){
        arspd.updateAirspeed();
        vTaskDelay(pdMS_TO_TICKS(AIRSPEED_time)); // Reduced delay for more frequent updates
    }
};

void radio(void *pvParameters){
    for(;;){
        remote_loop();
        vTaskDelay(pdMS_TO_TICKS(RADIO_time)); // Reduced delay for more frequent updates
    }
};

void voltage(void *pvParameters){
    for(;;){
        update_voltage();
        vTaskDelay(pdMS_TO_TICKS(VOLTAGE_time)); // Reduced delay for more frequent updates
    }
};

void controlThd(void *pvParameters){
    // pilih model yang ingin dipakai
    // Fixedwing
    #if MODEL_UAV_FIXEDWING
    modes.registerMode(&mMANU_FW);
    modes.registerMode(&mFBWA_FW);
    modes.registerMode(&mAUTO_FW);
    // Flywing
    #else
    modes.registerMode(&mMANU);
    modes.registerMode(&mFBWA);
    modes.registerMode(&mAUTO);
    #endif
    // Qopter
    modes.registerMode(&mQHOV);
    modes.registerMode(&mCOPT);
    for(;;){
    setup_mode();
    vTaskDelay(pdMS_TO_TICKS(CONTROL_time)); // Reduced delay for more frequent updates
  }
    
};

void fwRoll(void *pvParameters){
    for(;;){
        if (mode_now == 3) {
            test_fuzzy_roll();
        }
        vTaskDelay(pdMS_TO_TICKS(FWRoll_time)); // Reduced delay for more frequent updates
    }
};

void buzzerTask(void *pvParameters) {
    for(;;) {
        if (arming) {
            buzzer.isPlayingInitMusic = false;
            buzzer.update(arming);    
        } else {
            buzzer.initWithMusic();
            if (!buzzer.isPlayingInitMusic) {
                buzzer.update(arming); 
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void printUSB() {
    // if(Serial.available()){
    //     char c = Serial.read();
    //     if(c=='z') mode_now = 1;
    //     if(c=='x') mode_now = 2;
    //     if(c=='c') mode_now = 3;
    //     if(c=='v') mode_now = 4;
    // }
    // Serial.print(" Ele : ");
    // Serial.print(ch_roll);
    // Serial.print(" Millis : ");
    // Serial.print(getMillis());

    // Serial.print(" Millis : ");
    // Serial.print(getSeconds());
    // tambahan
    //  Serial.print("t: ");
    //  Serial.print(millis());
    // Serial.print(" sp: ");
    // Serial.print(_linear_acc.x);
    // Serial.print(" sp1: ");
    // Serial.print(_linear_acc.y);
    // Serial.print(" fout: ");
    // Serial.print(outroll);
    // Serial.print(" droll: ");
    // Serial.print(delta_roll);
    // Serial.print("| arm:");
    // Serial.print(arming);
    // //asli
    // Serial.print(" ch_roll:");
    // Serial.print(ch_roll);
    // Serial.print(" ch_pitch:");
    // Serial.print(ch_pitch);
    // Serial.print(" h_thr:");
    // Serial.print(ch_throttle);
    // Serial.print(" ch_yaw:");
    // Serial.print(ch_yaw);
    Serial.print("Arm:");
    Serial.print(arming);
    Serial.print(" |r:");
    Serial.print(imu.roll);
    Serial.print(" |p:");
    Serial.print(imu.pitch);
    Serial.print(" |y:");
    Serial.print(imu.yaw);
    Serial.print(" |Mode: ");
    Serial.print(modeCode4(current));
    Serial.print(" |dist:");
    Serial.print(auto_state.distance_next_wp/*current_loc.get_distance(next_WP_loc)*/);
    // Serial.print(" |MHOV: ");
    // Serial.print(QUAD_PWM_THR);
    // Serial.print(" |MPUS: ");
    // Serial.print(PUSHER_PWM);
    // Serial.print(" |AP: ");
    // Serial.print(cek_alpha_pusher);
    // Serial.print(" |AQ: ");
    // Serial.print(cek_alpha_hover);
    // Serial.print(" |ch_thr:");
    // Serial.print(ch_throttle);
    Serial.print(" |hdop:");
    Serial.print(gepees.hdop);
    Serial.print(" |sat:");
    Serial.print(gepees.satellites);
    Serial.print(" |gspd:");
    Serial.print(ahrs.groundspeed);
    Serial.print(" |head:");
    Serial.print(imu.heading);
    Serial.print(" |alt = ");
    Serial.print(baro.altitude);
    // Serial.print(" |ch_roll:");
    // Serial.print(ch_roll);
    // Serial.print(" |ch_pitch:");
    // Serial.print(ch_pitch);
    // Serial.print(" |ch_thr:");
    // Serial.print(ch_throttle);
    // Serial.print(" |ch_yaw:");
    // Serial.print(ch_yaw);
    // Serial.print(" MHOV: ");
    // Serial.print(QUAD_PWM_THR);
    // Serial.print (" mp:");
    // Serial.print (motor_pwm);
    // Serial.print(" MPUS: ");
    // Serial.print(PUSHER_PWM);
    // Serial.print (" m1: ");
    // Serial.print (m1_pwm);
    // Serial.print (" m2: ");
    // Serial.print (m2_pwm);
    // Serial.print (" m3: ");
    // Serial.print (m3_pwm);
    // Serial.print (" m4: ");
    // Serial.print (m4_pwm);
    // Serial.print(" |GPD = ");
    // Serial.print(get_pitch_demand());
    // Serial.print(" |GTD = ");
    // Serial.print(get_throttle_demand());
    // Serial.print(" |GRD: ");
    // Serial.print(nav_roll_cd());
    // Serial.print(" |dis_wp = ");
    // Serial.print(current_loc.get_distance(next_WP_loc));
    // Serial.print(" |flwp = ");
    // Serial.print(flag_wp);
    // Serial.print(" |PA = ");
    // Serial.print(nav_roll_deg);
    // Serial.print(" |PE = ");
    // Serial.print(nav_pitch_deg);
    // Serial.print(" r: ");
    // Serial.print(imu.roll);
    // Serial.print(" ");
    // Serial.print(" p: ");
    // Serial.print(imu.pitch);
    // Serial.print(" y: ");
    // Serial.print(imu.yaw);
    // // Serial.print(" DT: ");
    // // Serial.print(tesdata,10);
    // Serial.print(" Mode: ");
    // Serial.print(modeCode4(current));
    // Serial.print(" ");
    // Serial.print(alpha);

    // Serial.print("  gx: ");
    // Serial.print(imu.gyro_x);
    // Serial.print("  gy: ");
    // Serial.print(imu.gyro_y);
    // Serial.print("  gz: ");
    // Serial.print(imu.gyro_z);
    // Serial.print("  tesgx: ");
    // Serial.print(tesgx);
    // Serial.print("  tesgz: ");
    // Serial.print(tesgz);
    // Serial.print(" ");
    // Serial.print("yaw = ");
    // Serial.print(imu.yaw);
    // Serial.print(" servo_R: ");
    // Serial.print(servo_R);
    // Serial.print(" servo_L: ");
    // Serial.print(servo_L);
    // Serial.print(" U2: ");
    // Serial.print(u2);
    // Serial.print(" U3: ");
    // Serial.print(u3);
    // Serial.print(" arming:");
    // Serial.print(arming);
    // Serial.print(" Mode:");
    // Serial.print(mode_now);

    // Serial.print(" ");
    // Serial.print("alt = ");
    // Serial.print(baro.altitude);
    // Serial.print(" ");
    // Serial.print("Task: printUSB running at ");
    // Serial.println(millis());
    // Serial.print(" press_psi = ");
    // Serial.print(arspd.press_psi);
    Serial.print(" v_ms = ");
    Serial.print(arspd.v_ms);
    // Serial.print(" prspsi = ");
    // Serial.print(arspd.prspsi);
    // Serial.print(" gz = ");
    // Serial.print(gzrs);
    // Serial.print("vvel = ");
    // Serial.print(getClimbRate());
    // Serial.print(" alt = ");
    // Serial.print(baro.altitude);
    // Serial.print(" lat = ");
    // Serial.print(gepees.latitude, 7);
    // Serial.print(" lon = ");
    // Serial.print(gepees.longitude, 7);
    // Serial.print(" hdop = ");
    // Serial.print(gepees.hdop);
    // Serial.print(" heading = ");
    // Serial.print(imu.heading);
    // Serial.print(" sat = ");
    // Serial.print(gepees.satellites);
    // Serial.print(" speed = ");
    // Serial.print(gepees.line);
    // Serial.print(" headc = ");
    // Serial.print(head_card);
    // Serial.print(" gsp = ");
    // Serial.print(groundspeed);
    // Serial.print(" sat = ");
    // Serial.print(sattelites);
    // Serial.print(" age = ");
    // Serial.print(age);
    Serial.print(" flwp = ");
    Serial.print(flag_wp);
    // Serial.print(" PA = ");
    // Serial.print(nav_roll_deg);
    // Serial.print(" PE = ");
    // Serial.print(nav_pitch_deg);
    // Serial.print(" PT = ");
    // Serial.print(motor_pwm);
    // Serial.print(" heading = ");
    // Serial.print(heading);
    // Serial.print(" GPD = ");
    // Serial.print(get_pitch_demand());
    // Serial.print(" GTD = ");
    // Serial.print(get_throttle_demand());
    // Serial.print(" GRD: ");
    // Serial.print(nav_roll_cd());
    // Serial.print(" dis_wp = ");
    // Serial.print(current_loc.get_distance(next_WP_loc));
    // Serial.print(" get_VXdot() ");
    // Serial.print(_accel.x);
    // Serial.print ("  delay : ");
    // Serial.print (millis());
    // Serial.print (" m1: ");
    // Serial.print (m1_pwm);
    // Serial.print (" m2: ");
    // Serial.print (m2_pwm);
    // Serial.print (" m3: ");
    // Serial.print (m3_pwm);
    // Serial.print (" m4: ");
    // Serial.print (m4_pwm);
    // Serial.print (" omega1: ");
    // Serial.print (omega2[0]);
    // Serial.print (" omega2: ");
    // Serial.print (omega2[1]);
    // Serial.print (" omega3: ");
    // Serial.print (omega2[2]);
    // Serial.print (" omega4: ");
    // Serial.print (omega2[3]);


    // Serial.print("ch trothle = ");
    // Serial.print(ch_throttle);
    // Serial.print("ch yaw = ");
    // Serial.print(ch_yaw);

    // Serial.print(" phase = ");
    // if (mode_now == 1)
    // {
    //   Serial.print("1");
    // }
    // else if (mode_now == 2)
    // {
    //   Serial.print("2");
    // }
    // else if (mode_now == 3)
    // {
    //   Serial.print("3");
    // }
    // else if (mode_now == 4)
    // {
    //   Serial.print("4");
    // }
    // if(line == 100){
    //     Serial.println("DAH 100 LINEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
    //     Serial.print(" TOTAL WAKTU :    ");
    //     Serial.println(getMillis() - milisawal);
    // }

    Serial.println("");
    line++;
}

void calibrateMotor() {
    pusher.writeMicroseconds(2000);
    delay(3000);
    pusher.writeMicroseconds(1000);
    delay(1000);
}

void wp_setup() {
    // take off wp lomba
    // _takeoff[0].lat = -79171550;
    // _takeoff[0].lng = 1105655770;

    // take off sumberarum lomba
    //  _takeoff[0].lat = -77677220;
    //  _takeoff[0].lng = 1102333770;

    // WP mirip lomba di sumberarum
    //  add_waypoint(-77682400, 1102339700, 90 * M_TO_CM, 1);  // wp1 
    //  add_waypoint(-77688900, 1102348500, 90 * M_TO_CM, 2);  // wp2 
    //  add_waypoint(-77684400, 1102357500, 90 * M_TO_CM, 3);  // wp3
    //  add_waypoint(-77670000, 1102358800, 90 * M_TO_CM, 4);  // wp4
    //  add_waypoint(-77663700, 1102351400, 90 * M_TO_CM, 5);  // wp5
    //  add_waypoint(-77669100, 1102339700, 90 * M_TO_CM, 6);  // wp6

    //WP SIMULASI 1 SUMBER ARUM
    add_waypoint(0, 0, 69 * M_TO_CM, 0);
    add_waypoint(-77671400, 1102340100, 90 * M_TO_CM, 1);  // wp1 
    add_waypoint(-77679300, 1102340400, 100 * M_TO_CM, 2);  // wp2 
    add_waypoint(-77683800, 1102345800, 110 * M_TO_CM, 3);  // wp3
    add_waypoint(-77683400, 1102354100, 110 * M_TO_CM, 4);  // wp4
    add_waypoint(-77658400, 1102364200, 110 * M_TO_CM, 5);  // wp5
    add_waypoint(-77652000, 1102359400, 110 * M_TO_CM, 6);  // wp6
    add_waypoint(-77650400, 1102348200, 100 * M_TO_CM, 7);  // wp7
    add_waypoint(-77660000, 1102341800, 90 * M_TO_CM, 8);  // wp8
    add_waypoint(-77672500, 1102336300, 90 * M_TO_CM, 9);  // wp9
    
    // ======= WAYPOINT AKTIF (Terintegrasi dengan Mission Planner) =======
    // Format: add_waypoint(lat*1e7, lng*1e7, altitude_CM, index)
    // Waypoint ini akan otomatis muncul di Mission Planner (READ WPs)
    // dan bisa dioverwrite dari Mission Planner (WRITE WPs)
    
    // add_waypoint(0, 0, 69 * M_TO_CM, 0); // home
    // add_waypoint(-77738260, 1103784420, 2 * M_TO_CM, 1);  // WP 1: ~2m altitude
    // add_waypoint(-77732600, 1103786320, 2 * M_TO_CM, 2);  // WP 2: ~2m altitude
    // add_waypoint(-77746550, 1103782450, 2 * M_TO_CM, 3);  // WP 3: ~2m altitude
    // =====================================================================
    
    // test waypoint D13 yang mirip di hanggar Sumberarum
    // add_waypoint(0, 0, 69 * M_TO_CM, 0);
    //  add_waypoint(-77666500, 1102331000, 90 * M_TO_CM, 1);
    //  add_waypoint(-77671400, 1102334100, 90 * M_TO_CM, 2);
    //  add_waypoint(-77741106, 1102336200, 2 * M_TO_CM, 3);    
}

void setup() {
    milisawal = getMillis();
    modes.setMode(ModeId::MANU);
    active_mode = ModeId::MANU;
    current    = ModeId::MANU;
    
    SERIAL_USB.begin(115200);
    SERIAL_UART.begin(57600);
    SERIAL_RASPI.begin(57600); //harus sama mengirim barudrate nya
    
    delay(100);  // Wait for serial init
    
    // Try to load waypoints from EEPROM first
    Serial.println("\\n=== Waypoint Loading ===");
    if (loadWaypointsFromEEPROM()) {
        Serial.println("Using waypoints from EEPROM");
    } else {
        Serial.println("No EEPROM data, loading hardcoded waypoints");
        wp_setup();  // Fallback to hardcoded waypoints
    }
    
    Serial.printf("Setup: %d waypoints loaded (MAVLink integrated)\\n", wp_sum);
    printEEPROMStatus();
    Serial.println("========================\\n");
    
    Serial.print("Starting setup");
    Serial.println(" " + String(millis()));
    pinMode(30, OUTPUT); // 29 nabrak sama RXTX7 
    // MODE = initMODE->mode();
    // modes.setMode(ModeId::COPT);

    buzzer.initWithMusic();
    init_actuator();
    ahrs.init_ahrs();
    remote_setup();
    setup_fuzzy_roll();
    init_voltage();
    buzzer.init(); // Inisialisasi buzzer

    if (CALIBRATE_MOTORS) {
        calibrateMotor();
    }
    loop_timer = micros();

    // xTaskCreate(Print_To_Raspi, "PrintRaspi", 4096, NULL, 3, &Task_Raspi);
    // xTaskCreate(Print_telem, "Telem", 2048, NULL, 2, &Task_Telem);
    xTaskCreate(mavlink_print, "Mavlink", 2048, NULL, 2, &Task_Mavlink);
    // xTaskCreate(Print_task, "Print", 4096, NULL, 3, &Task_Print);
    xTaskCreate(radio, "Radio", 4096, NULL, 3, &Task_Print);
    xTaskCreate(airspeed, "Airspeed", 4096, NULL, 2, &Task_Print);
    xTaskCreate(controlThd, "Control", 4096, NULL, 2, &Task_Control);
    xTaskCreate(fwRoll, "FWRoll", 4096, NULL, 3, &Task_Print);
    xTaskCreate(updateIMU, "IMU", 2048, NULL, 3, &Task_IMU);
    xTaskCreate(GPS, "GPS", 2048, NULL, 3, &Task_GPS);
    xTaskCreate(updateBARO, "BARO", 2048, NULL, 3, &Task_BARO);
    xTaskCreate(buzzerTask, "Buzzer", 2048, NULL, 3, &Task_Buzzer);
    xTaskCreate(voltage, "Voltage", 2048, NULL, 2, &Task_Voltage);
    vTaskStartScheduler(); // Start FreeRTOS scheduler
}

void loop() {
    // uint64_t temps = micros();
    // update_voltage();
    // Transition_sequence_manual();
    // printUSB();
    // timestamps = micros() - temps;

    //Hidupkan kalo pake Scheduler
    // printUSB();
}
