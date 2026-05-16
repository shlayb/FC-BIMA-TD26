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
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"       // ← TAMBAH: untuk mutex
#include "Mode_setup.h"
#include "Mode_Manager.h"
#include "Params.h"
#include <WP_EEPROM.h>
 
// ============================================================
//  SERIAL ALIAS
// ============================================================
#define SERIAL_USB   Serial
#define SERIAL_UART  Serial2
#define SERIAL_RASPI Serial7
 
// ============================================================
//  KONFIGURASI MODEL & TIMING (ms)
// ============================================================
#define CALIBRATE_MOTORS    false
  
 
#define IMU_time      5     // 200 Hz
#define BARO_time     10    // 100 Hz
#define GPS_time      10    // 100 Hz
#define AIRSPEED_time 50    //  20 Hz
#define PRINT_time    50    //  20 Hz
#define CONTROL_time  5     // 200 Hz  — sinkron dengan IMU
#define FWRoll_time   20    //  50 Hz
#define RADIO_time    10    // 100 Hz
#define TELEM_time    50    //  20 Hz
#define MAVLINK_time  2    // 100 Hz  — real-time
#define RASPI_time    100   //  10 Hz
#define BUZZER_time   10    // 100 Hz
#define VOLTAGE_time  100   //  10 Hz
 
// ============================================================
//  VARIABEL GLOBAL
// ============================================================
uint32_t loop_timer = 0;
 
Telemetry     telem;
GPSTelemetry  rasprint;
MavlinkHandler mavHandler;
 
int    mode_phase  = 0;
int    Mode        = 0;
ModeId current     ;
ModeManager modes;
ModeId active_mode ;
int    mode_now    = 0;
int    prev_mode   = 0;
 
int      line      = 0;
int      milisawal = 0;
float model_uav = MODEL_UAV_FIXEDWING;
 
// ============================================================
//  MUTEX — proteksi shared data antar task
// ============================================================
SemaphoreHandle_t xIMUMutex;    // guard imu.* yang dibaca Task_Control
SemaphoreHandle_t xAHRSMutex;  // guard ahrs.* (dcm, velocity, dll.)
SemaphoreHandle_t xSerialMutex; // guard Serial USB agar tidak tabrakan
 
// ============================================================
//  TASK HANDLES — FIX: setiap task punya handle-nya sendiri
// ============================================================
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
TaskHandle_t Task_FWRoll;   // ← FIX: dulunya salah pakai &Task_Print
TaskHandle_t Task_Radio;    // ← FIX: dulunya salah pakai &Task_Print
TaskHandle_t Task_Airspeed; // ← FIX: dulunya salah pakai &Task_Print
 
// ============================================================
//  DMA BUFFER SERIAL
// ============================================================
DMAMEM static uint8_t dma_rx2[4096];
DMAMEM static uint8_t dma_tx2[4096];
DMAMEM static uint8_t dma_rx7[2048];
DMAMEM static uint8_t dma_tx7[4096];
 
// ============================================================
//  FORWARD DECLARATION
// ============================================================
void printUSB();
void calibrateMotor();
void wp_setup();

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

void mavlink_print(void *pvParameters) {
    TickType_t xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(MAVLINK_time);
    for (;;) {
        mavHandler.handlePorts();
        mavHandler.mavlinkTask();

        // Kalau task overrun, reset xLastWake agar tidak catch-up
        if ((xTaskGetTickCount() - xLastWake) >= xPeriod) {
            xLastWake = xTaskGetTickCount();  // ← reset, skip catch-up
        }
        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

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
    // Fixedwing VTOL mode set
    int model = (int)(model_uav + 0.5f);

    if (model == MODEL_UAV_FIXEDWING) {
        modes.registerMode(&mMANU_FW);
        modes.registerMode(&mFBWA_FW);
        modes.registerMode(&mGUIDED_FW);
        modes.registerMode(&mAUTO_FW);

    } else if (model == MODEL_UAV_VTOL) {
        modes.registerMode(&mMANU);
        modes.registerMode(&mFBWA);
        modes.registerMode(&mAUTO);
        modes.registerMode(&mQHOV);
        modes.registerMode(&mCOPT);
    }
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
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'z') mode_now = 1;
        if (c == 'x') mode_now = 2;
        if (c == 'c') mode_now = 3;
        if (c == 'v') mode_now = 4;
    }
 
    Serial.print("Arm:");      Serial.print(arming);    
    // Serial.print(" |r:");      Serial.print(imu.roll);
    // Serial.print(" |p:");      Serial.print(imu.pitch);
    // Serial.print(" |y:");      Serial.print(imu.yaw);
    Serial.print(" |Mode: ");  Serial.print(modeCode4(current));
    Serial.print(" |dist:");   Serial.print(auto_state.distance_next_wp);
    // Serial.print(" |ch_roll:"); Serial.print(ch_roll);
    // Serial.print(" |ch_pitch:");Serial.print(ch_pitch);
    // Serial.print(" |ch_thr:"); Serial.print(ch_throttle);
    // Serial.print(" |ch_yaw:"); Serial.print(ch_yaw);
    Serial.print(" |lat:");     Serial.print(gepees.latitude,  7);
    Serial.print(" |lon:");     Serial.print(gepees.longitude, 7);
    Serial.print(" |hdop:");    Serial.print(gepees.hdop);
    Serial.print(" |flwp:");    Serial.print(flag_wp);
    Serial.print(" |auto_thr:"); Serial.print(fw_control.get_auto_throttle());
    Serial.println();
    line++;
}

void calibrateMotor() {
    mtr_1.writeMicroseconds(2000);  // Motor kanan
    mtr_2.writeMicroseconds(2000);  // Motor kiri
    delay(3000);
    mtr_1.writeMicroseconds(1000);
    mtr_2.writeMicroseconds(1000);
    delay(1000);
}

void wp_setup() {
    add_waypoint(0, 0, 69 * M_TO_CM, 0); // home
    add_waypoint(-77738260, 1103784420, 0.2 * M_TO_CM, 1);  // WP 1: ~2m altitude
    add_waypoint(-77732600, 1103786320, 0.2 * M_TO_CM, 2);  // WP 2: ~2m altitude
    add_waypoint(-77746550, 1103782450, 0.2 * M_TO_CM, 3);  // WP 3: ~2m altitude
}

void setup() {
    milisawal = getMillis();
 
    modes.setMode(ModeId::MANU);
    active_mode = ModeId::MANU;
    current     = ModeId::MANU;
 
    // --- Serial ---
    SERIAL_USB.begin(115200);
    SERIAL_UART.begin(460800);
    SERIAL_UART.addMemoryForRead(dma_rx2,  sizeof(dma_rx2));
    SERIAL_UART.addMemoryForWrite(dma_tx2, sizeof(dma_tx2));
    SERIAL_RASPI.begin(57600);
    SERIAL_RASPI.addMemoryForRead(dma_rx7,  sizeof(dma_rx7));
    SERIAL_RASPI.addMemoryForWrite(dma_tx7, sizeof(dma_tx7));
    delay(100);
 
    // --- Waypoint ---
    Serial.println("\n=== Waypoint Loading ===");
    initParams();
    loadParamsFromEEPROM();
    loadWaypointsFromEEPROM();
    initParams();
    if (!loadWaypointsFromEEPROM()){
        wp_setup();
    }
    printEEPROMStatus();
    Serial.println("========================\n");
 
    Serial.print("Starting setup ");
    Serial.println(millis());
    pinMode(30, OUTPUT);
 
    // --- Init hardware ---
    buzzer.initWithMusic();
    init_actuator();
    ahrs.init_ahrs();
    remote_setup();
    setup_fuzzy_roll();
    init_voltage();
    buzzer.init();
 
    if (CALIBRATE_MOTORS) calibrateMotor();
 
    // --- Buat mutex SEBELUM task dibuat ---
    xIMUMutex    = xSemaphoreCreateMutex();
    xAHRSMutex   = xSemaphoreCreateMutex();
    xSerialMutex = xSemaphoreCreateMutex();
 
    loop_timer = micros();
 
    // ============================================================
    //  TASK CREATION
    //  Prioritas  : semakin tinggi angka = semakin didahulukan
    //  Stack size : dalam WORD (4 byte), bukan byte
    //
    //  FIX: setiap xTaskCreate menggunakan handle yang benar
    //  FIX: stack size disesuaikan dengan beban masing-masing task
    // ============================================================
 
    //                 fungsi          nama        stack  param  prio           handle
    xTaskCreate(updateIMU,    "IMU",     2048, NULL,  6,  &Task_IMU);     // 200 Hz — tertinggi
    xTaskCreate(controlThd,   "Control", 8192, NULL,  5,  &Task_Control); // 200 Hz — logic kompleks
    xTaskCreate(mavlink_print,"Mavlink", 8192, NULL,  5,  &Task_Mavlink); // 100 Hz real-time
    // xTaskCreate(radio,        "Radio",   4096, NULL,  5,  &Task_Radio);   // 100 Hz
    xTaskCreate(GPS,          "GPS",     4096, NULL,  4,  &Task_GPS);     // 100 Hz — string parsing
    xTaskCreate(updateBARO,   "BARO",    2048, NULL,  3,  &Task_BARO);    // 100 Hz
    // xTaskCreate(fwRoll,       "FWRoll",  4096, NULL,  3,  &Task_FWRoll);  // 50 Hz
    // xTaskCreate(airspeed,     "Airspeed",2048, NULL,  3,  &Task_Airspeed);// 20 Hz
    // xTaskCreate(voltage,      "Voltage", 2048, NULL,  2,  &Task_Voltage); // 10 Hz
    xTaskCreate(buzzerTask,   "Buzzer",  2048, NULL,  3,  &Task_Buzzer);  // 100 Hz
   //  xTaskCreate(Print_task,   "Print",   4096, NULL,  2,  &Task_Print);   // 20 Hz — terendah
    // xTaskCreate(Print_To_Raspi,"Raspi", 4096, NULL,  1,  &Task_Raspi);
    // xTaskCreate(Print_telem,   "Telem", 4096, NULL,  1,  &Task_Telem);
 
    vTaskStartScheduler();
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