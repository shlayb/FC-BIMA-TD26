// #include <../lib/TeensyThreads/TeensyThreads.h>
#include <AHRS.h>
#include <Actuator.h>
#include <Arduino.h>
#include <Fuzzy_FW_Pitch.h>
#include <Fuzzy_FW_Roll.h>
#include <Radio.h>
#include <Telemetry.h>
#include <Transition.h>
#include <takeoff.h>
#include <RaspberryPi_Telemtry.h>
// #include "FreeRTOS.h"    
// #include "task.h"

#define SERIAL_USB Serial
#define SERIAL_UART Serial2
#define SERIAL_RASPI Serial3
#define CALIBRATE_MOTORS false

#define IMU_time 5
#define BARO_time 10
#define GPS_time 10
#define AIRSPEED_time 50
#define PRINT_time 50
#define CONTROL_time 5
#define FWRoll_time 20
#define RADIO_time 10
#define TELEM_time 10
#define RASPI_time 10


uint32_t loop_timer = 0, debug_timer = 0;
uint32_t time_now;
uint32_t dt;
uint32_t time_last;

Telemetry telem;
GPSTelemetry rasprint;

TaskHandle_t Task_IMU;
TaskHandle_t Task_BARO;
TaskHandle_t Task_GPS;
TaskHandle_t Task_Print;
TaskHandle_t Task_Raspi;
TaskHandle_t Task_Telem;


int line = 0;
int milisawal = 0;
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
        printUSB();
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
        vTaskDelay(pdMS_TO_TICKS(TELEM_time)); // Reduced delay for more frequent updates
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

void controlThd(void *pvParameters){
    for(;;){
        Transition_sequence_manual();
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

void printUSB() {
    Serial.print(" Ele : ");
    Serial.print(ch_roll);
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

    Serial.print(" roll = ");
    Serial.print(imu.roll);
    Serial.print(" ");
    Serial.print("pitch = ");
    Serial.print(imu.pitch);
    Serial.print(" ");
    Serial.print("yaw = ");
    Serial.print(imu.yaw);
    // Serial.print(" ");
    // Serial.print("Task: printUSB running at ");
    // Serial.println(millis());
    // Serial.print(" press_psi = ");
    // Serial.print(arspd.press_psi);
    // Serial.print(" v_ms = ");
    // Serial.print(arspd.v_ms);
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
    // Serial.print(" flwp = ");
    // Serial.print(flag_wp);
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
    Serial.print (" copter out 1 ");
    Serial.print (m1_pwm);
    // Serial.print (" copter out 2 ");
    // Serial.print (m2_pwm);
    Serial.print (" copter out 3 ");
    Serial.print (m3_pwm);
    // Serial.print (" copter out 4 ");
    // Serial.print (m4_pwm);

    Serial.print("ch trothle = ");
    Serial.print(ch_throttle);
    Serial.print("ch yaw = ");
    Serial.print(ch_yaw);

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
    if(line == 100){
        Serial.println("DAH 100 LINEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
        Serial.print(" TOTAL WAKTU :    ");
        Serial.println(millis() - milisawal);
    }
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

    // WP lomba
    // add_waypoint(-79177330, 1105657110, 46 * M_TO_CM, 1);  // wp1
    // add_waypoint(-79179220, 1105671770, 66 * M_TO_CM, 2);  // wp2
    // add_waypoint(-79174550, 1105677330, 66 * M_TO_CM, 3);  // wp3
    // add_waypoint(-79169550, 1105675440, 66 * M_TO_CM, 4);  // wp4
    // add_waypoint(-79168110, 1105655990, 66 * M_TO_CM, 5);  // wp5
    
    // // take off wp lomba
    // _takeoff[0].lat = -79174330;
    // _takeoff[0].lng = 1105665220;

    // // WP mirip lomba
    // add_waypoint(-79180220, 1105668220, 46 * M_TO_CM, 1);  // wp1
    // add_waypoint(-79182440, 1105685880, 76 * M_TO_CM, 2);  // wp2
    // add_waypoint(-79176000, 1105690440, 76 * M_TO_CM, 3);  // wp3
    // add_waypoint(-79170440, 1105686990, 76 * M_TO_CM, 4);  // wp4
    // add_waypoint(-79169110, 1105665550, 76 * M_TO_CM, 5);  // wp5

    // _takeoff[0].lat = -77677770;
    // _takeoff[0].lng = 1102334770;

    // _takeoff[0].lat = -77682550;
    // _takeoff[0].lng = 1102332660;

    // tes takeoff
    //  _takeoff[0].lat = -77738229;
    //  _takeoff[0].lng = 1103784488;

    // take off gading
    //  _takeoff[0].lat = -79169220;
    //  _takeoff[0].lng = 1105675330;

    // take off sumberarum lomba
    //  _takeoff[0].lat = -77677220;
    //  _takeoff[0].lng = 1102333770;

    // WP mirip lomba di sumberarum
    //  add_waypoint(-77684000, 1102335770, 68 * M_TO_CM, 1);  // wp1 36
    //  add_waypoint(-77688330, 1102349880, 68 * M_TO_CM, 2);  // wp2 46
    //  add_waypoint(-77683880, 1102355440, 68 * M_TO_CM, 3);  // wp3
    //  add_waypoint(-77677550, 1102353000, 68 * M_TO_CM, 4);  // wp4
    //  add_waypoint(-77673110, 1102335110, 68 * M_TO_CM, 5);  // wp5

    // Tes WP terbang Gading pertama
    //  add_waypoint(-79171550, 1105685110, 99 * M_TO_CM, 1);  // wp1
    //  add_waypoint(-79175330, 1105688220, 99 * M_TO_CM, 2);  // wp2
    //  add_waypoint(-79179000, 1105684440, 99 * M_TO_CM, 3);  // wp3
    //  add_waypoint(-79176000, 1105661880, 99 * M_TO_CM, 4);  // wp4
    //  add_waypoint(-79171880, 1105658990, 99 * M_TO_CM, 5);  // wp5
    //  add_waypoint(-79168990, 1105663110, 99 * M_TO_CM, 6);  // wp6

    // cek WP RTF Sumberarum
    //  add_waypoint(-77732550, 1103785910, 80,1);  // Pusat studi wanita (1)
    //  add_waypoint(-77738590, 1103784510, 80,2);  // GAMAFORCE (2)
    //  add_waypoint(-77754770, 1103779970, 80,3);  // Pusat Studi pancasila (3)
    //  add_waypoint(-77682500, 1102335000, 100);  // (1) (to home: 150 m)
    //  add_waypoint(-77684000, 1102341900, 100);  // (2) (to home: 190 m)
    //  add_waypoint(-77674700, 1102346000, 100);  //(3)  (to home: 140 m)
    //  add_waypoint(-77670200, 1102337200, 100);  // (4) (to home: 30  m)

    //tes wp D13
    add_waypoint(-77738260, 1103784420, 100 * M_TO_CM, 1);
    add_waypoint(-77732600, 1103786320, 100 * M_TO_CM, 2);
    add_waypoint(-77746550, 1103782450, 100 * M_TO_CM, 3);

    // test waypoint D13 yang mirip di hanggar Sumberarum
    //  add_waypoint(-77750618, 1103786786, 100 * M_TO_CM, 1);
    //  add_waypoint(-77751521, 1103795260, 100 * M_TO_CM, 2);
    //  add_waypoint(-77741106, 1103798538, 100 * M_TO_CM, 3);
    //  add_waypoint(-77740989, 1103787970, 100 * M_TO_CM, 4);

    // test wp belok
    //  add_waypoint(-77738590, 1103784510, 100); // GAMAFORCE (1)
    //  add_waypoint(-77730381, 1103780394, 100); // pertigaan (2)

    // // TERBANG 20 Juli: auto lurus timur lapangan sumberarum
    // add_waypoint(-77675330, 1102338660, 100); // (WP1)
    // add_waypoint(-77667770, 1102341000, 100); // (WP2, 88m dari 1)
    // add_waypoint(-77657880, 1102344220, 100); // (WP3 buat exit, 120m dari 2)

    // auto diagonal sumberarum
    //  add_waypoint(-77668330, 1102332990, 100); // (WP1)
    //  add_waypoint(-77672880,  1102335110, 100); // (WP2, 88m dari 1)
    //  add_waypoint(-77677770,  1102336880, 100); // (WP3 buat exit, 120m dari 2)

    // TERBANG 22 Juli: auto circular 2
    //  add_waypoint(-77680990, 1102339550, 99 * M_TO_CM, 1); //wp1
    //  add_waypoint(-77679000, 1102345660, 99 * M_TO_CM, 2); //wp2
    //  add_waypoint(-77674000, 1102349330, 99 * M_TO_CM, 3); //wp3
    //  add_waypoint(-77666880, 1102349330, 99 * M_TO_CM, 4); //wp4
    //  add_waypoint(-77666550, 1102340770, 99 * M_TO_CM, 5); //wp5

    // TERBANG 1 agustus: auto circular coba pake home wp
    //  add_waypoint(-77670720, 1102335004, 80, 1); //wp1
    //  add_waypoint(-77675550, 1102338220, 90, 1); //wp1
    //  add_waypoint(-77678770, 1102344990, 90, 2); //wp2
    //  add_waypoint(-77674000, 1102349330, 90, 3); //wp3
    //  add_waypoint(-77666880, 1102349330, 90, 4); //wp4
    //  add_waypoint(-77666550, 1102340770, 90, 5); //wp5

    // TERBANG 26 Juli : WP angka 8
    //  add_waypoint(-77671220, 1102339660, 80, 1); // (WP1)
    //  add_waypoint(-77675660, 1102344990, 80, 2); // (WP2)
    //  add_waypoint(-77680660, 1102343440, 80, 3); // (WP3)
    //  add_waypoint(-77680330, 1102338110, 80, 4); // (WP4)
    //  add_waypoint(-77675550, 1102335550, 80, 5); // (WP5)
    //  add_waypoint(-77666330, 1102348770, 80, 6); // (WP6)
    //  add_waypoint(-77661660, 1102346110, 80, 7); // (WP7)
    //  add_waypoint(-77662440, 1102340660, 80, 8); // (WP8)
    //  add_waypoint(-77666110, 1102339440, 80, 9); // (WP9)

    // TERBANG 4 agustus: auto garis lurus
    //  add_waypoint(-77678660, 1102338220, 80, 1); //wp1
    //  add_waypoint(-77661990, 1102343550, 80, 2); //wp1
    //  add_waypoint(-77674000, 1102336000, 80, 3); //wp2

    // TERBANG 4 agustus: auto angka 8 (5 wp)
    //  add_waypoint(-77676770, 1102338110, 80, 1); //wp1
    //  add_waypoint(-77675550, 1102346660, 80, 2); //wp1
    //  add_waypoint(-77669990, 1102342770, 80, 3); //wp2
    //  add_waypoint(-77664220, 1102340000, 80, 4); //wp1
    //  add_waypoint(-77662990, 1102348220, 80, 5); //wp2

    // TERBANG 25 Agustus: WP lurus 7 titik (lanjutan auto takeoff)
    //  add_waypoint(-77682220, 1102338220, 59 * M_TO_CM, 1);  // wp1
    //  add_waypoint(-77678770, 1102343550, 59 * M_TO_CM, 2);  // wp2
    //  add_waypoint(-77658330, 1102351000, 59 * M_TO_CM, 3);  // wp3
    //  add_waypoint(-77651880, 1102348330, 99 * M_TO_CM, 4);  // wp4
    //  add_waypoint(-77654440, 1102342880, 99 * M_TO_CM, 5);  // wp5
    //  add_waypoint(-77666550, 1102339550, 99 * M_TO_CM, 6);  // wp6
}

void setup() {
    // // threads.setSliceMicros(10);
    // milisawal = millis();
    // Serial.print("Starting setup : ");
    // Serial.println(millis());
    wp_setup();
    SERIAL_USB.begin(115200);
    SERIAL_UART.begin(57600);
    SERIAL_RASPI.begin(115200);
    Serial.print("Starting setup");
    Serial.println(" " + String(millis()));
    pinMode(29, OUTPUT);
    init_actuator();
    ahrs.init_ahrs();
    remote_setup();
    setup_fuzzy_roll();
    init_voltage();

    xTaskCreate(Print_To_Raspi, "PrintRaspi", 4096, NULL, 3, &Task_Raspi);
    // xTaskCreate(Print_telem, "Telem", 2048, NULL, 2, &Task_Telem);
    xTaskCreate(Print_task, "Print", 4096, NULL, 3, &Task_Print);
    xTaskCreate(radio, "Radio", 4096, NULL, 3, &Task_Print);
    xTaskCreate(airspeed, "Airspeed", 4096, NULL, 2, &Task_Print);
    xTaskCreate(controlThd, "Control", 4096, NULL, 3, &Task_Print);
    xTaskCreate(fwRoll, "FWRoll", 4096, NULL, 3, &Task_Print);
    xTaskCreate(updateIMU, "IMU", 2048, NULL, 3, &Task_IMU);
    xTaskCreate(GPS, "GPS", 2048, NULL, 3, &Task_GPS);
    xTaskCreate(updateBARO, "BARO", 2048, NULL, 3, &Task_BARO);
    vTaskStartScheduler(); // Start FreeRTOS scheduler

    if (CALIBRATE_MOTORS) {
        calibrateMotor();
    }
    MODE = initMODE->mode();
    loop_timer = micros();
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
