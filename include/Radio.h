#ifndef RADIO_H
#define RADIO_H
#include <Arduino.h>
#include <sbus.h>

bfs::SbusRx sbus_rx(&Serial8);
// bfs: class dr library, Serial8 dari teensy

bfs::SbusData data;

bool signal_lost = false;
uint16_t ch_roll = 1500, ch_pitch = 1500, ch_throttle = 988, ch_yaw = 1500;
uint16_t ch_mode = 1000, ch_mode_backup = 1000, ch_vehicle_mode = 1000;
uint16_t ch5 = 1000, ch6 = 1000, ch7 = 1000, ch8 = 1000;
bool arming = false;
bool rpi_mode_lock = false;

// enum MODE
// {
//     PHASE_1 = 0,
//     PHASE_2 = 1,
//     PHASE_3 = 2,
// };

bool mode_fbwa = false;
bool alt_hold = false;
bool mode_hover = false;
bool pos_hold = false;
bool mode_manual = false;
bool mode_fbwa_plane = false;
bool mode_fbwb = false;
bool transition_phase1 = false;
bool transition_phase2 = false;
bool transition_phase3 = false;
bool mode_vtol = false;
bool mode_safety = false;
bool mode_vtol_plane = false;
extern int mode_now, prev_mode;

float outputScaler(uint16_t ch)
// scaling data dr remote biar bs masuk ke perhitungan kendali, konversi ke bentuk sudut
{
    float scaledOutput = 0.002 * ch - 3;  // pokoknya gmnpun rumusnya yg penting pwm 1000 tuh 0 pwm 2000 tuh 1
    // pake excel jg, konversi dr 0 hingga 1 dikali headmax (sudut max roll)
    // misal ch nya 2000, hasilnya ttp aja 1 jd angka ini udh bener
    return scaledOutput;
}

void remote_setup() {
    sbus_rx.Begin();

    // Default safe initial values
    ch_roll = 1500;
    ch_pitch = 1500;
    ch_throttle = 988;
    ch_yaw = 1500;
    ch_mode = 1000;
    ch_mode_backup = 1000;
    arming = false;
    signal_lost = true;

    // Cek apakah remote sudah terhubung saat booting (non-blocking)
    if (sbus_rx.Read()) {
        data = sbus_rx.data();
        if (!data.failsafe && !data.lost_frame && data.ch[0] > 50) {
            arming = data.ch[4] > 1500 ? 1 : 0;
            signal_lost = false;
        }
    }

    Serial.println("Remote setup complete");
}

//remote satria
void remote_loop() {
    if (sbus_rx.Read()) {
        data = sbus_rx.data();

        // Validasi frame SBUS: abaikan jika failsafe, lost frame, atau channel 0 belum ada sinyal
        if (data.failsafe || data.lost_frame || data.ch[0] < 50) {
            signal_lost = true;
            return;
        }

        signal_lost = false;

        //remot juang
        ch_roll = data.ch[0] * 0.6267f + 877.91f;
        ch_pitch = data.ch[1] * 0.6251f + 878.83f;
        ch_throttle = data.ch[2] * 0.6274f + 879.25f;
        ch_yaw = data.ch[3] * 0.6304f + 876.19f;

        ch_roll = constrain(ch_roll, 988, 2012);
        ch_pitch = constrain(ch_pitch, 988, 2012);
        ch_throttle = constrain(ch_throttle, 988, 2012);
        ch_yaw = constrain(ch_yaw, 988, 2012);

        arming = data.ch[4] > 1500 ? 1 : 0;

        ch_mode = data.ch[5] * 0.6267f + 877.91f;
        ch_mode_backup = data.ch[6] * 0.6267f + 877.91f;
        ch_vehicle_mode = data.ch[7] * 0.6267f + 877.91f;

        ch_mode = constrain(ch_mode, 988, 2012);
        ch_mode_backup = constrain(ch_mode_backup, 988, 2012);
        ch_vehicle_mode = constrain(ch_vehicle_mode, 988, 2012);

        //Scaling to pwm
        // ch_roll = data.ch[0] * 0.6248f + 880.54f;
        // ch_pitch = data.ch[1] * 0.6248f + 880.54f;
        // ch_throttle = data.ch[2] * 0.6248f + 880.54f;
        // ch_yaw = data.ch[3] * 0.6248f + 880.54f;

        // ch_roll = constrain(ch_roll, 988, 2012);
        // ch_pitch = constrain(ch_pitch, 988, 2012);
        // ch_throttle = constrain(ch_throttle, 988, 2012);
        // ch_yaw = constrain(ch_yaw, 988, 2012);

        // arming = data.ch[4] > 1500 ? 1 : 0;
        // signal_lost = data.lost_frame;

        // ch_mode = data.ch[5] * 0.6248f + 880.54f;
        // ch_mode_backup = data.ch[6] * 0.6248f + 880.54f;
        // ch_vehicle_mode = data.ch[7] * 0.6248f + 880.544f;
        // misal nambah ch ya sama aja nilainya

//remote mbak vivo & husen
        // ch_roll = data.ch[0] * 0.6251f + 881.74f;
        // ch_pitch = data.ch[1] * 0.6251f + 881.74f;
        // ch_throttle = data.ch[2] * 0.6251f + 881.74f;
        // ch_yaw = data.ch[3] * 0.6251f + 881.74f;

        // ch_roll = constrain(ch_roll, 988, 2012);
        // ch_pitch = constrain(ch_pitch, 988, 2012);
        // ch_throttle = constrain(ch_throttle, 988, 2012);
        // ch_yaw = constrain(ch_yaw, 988, 2012);

        // arming = data.ch[4] > 1500 ? 1 : 0;
        // signal_lost = data.lost_frame;

        // ch_mode = data.ch[5] * 0.6251f + 881.74f;
        // ch_mode_backup = data.ch[6] * 0.6251f + 881.74f;
        // ch_vehicle_mode = data.ch[7] * 0.6251f + 881.74f;

        // buat karakterisasi keluaran minimal dan maksimal dr remote ke min dan max pwm
        // kalo remote 16 channel, remote bakal urut tu ngeprint nya, abis 16 ch ganti baris
        // buat tau tiap channel buat apa tinggal gerak2in aja stick remotenya
        // misal min dr remote: 172, 400, 988
        // min pwm  988, 1500, 2012 default 1000 2000
        // di scatter, harusnya linear, trendline>linear?>display eq
        // x adlh sinyal masukan remote yg kebaca
        // y nya pwm

        // program read ch remote:
        // for (int i = 0; i < 16; i++) {
        //     Serial.print(data.ch[i]);
        //     Serial.print(" ");
        // }
        // Serial.println("");

        // manual:
    //     if (ch_mode > 1000 && ch_mode <= 1512 && ch_mode_backup >= 1512) {
    //         mode_now = 4;  // Special mode 4
    //     } else if (ch_mode >= 1512 && ch_mode_backup >= 1512) {
    //         mode_now = 5;  // Special mode 5
    //     } else {
    // // Only check ch_mode_backup modes if not in special modes
        // if (ch_mode < 1000 && ch_mode_backup <= 1000) {
        //     mode_now = 1;
        // } else if (ch_mode < 1000 && ch_mode_backup > 1000 && ch_mode_backup <= 1512) {
        //     mode_now = 2;
        // } else if (ch_mode < 1000 && ch_mode_backup >= 1512) {
        //     mode_now = 3;
        // }

        // Mode selection with edge-detection + auto-mode lock (ArduPilot style)
        // Switch 1 (ch_mode / Ch6) memilih Bank mode:
        //   Low  (< 1500) : Bank 1 (Regular: 1=MANU, 2=FBWA, 3=AUTO)
        //   High (>= 1500): Bank 2 (Special: 4=GUIDED, 5=RTL, 6=LOITER)
        // Switch 2 (ch_mode_backup / Ch7, 3-pos switch):
        //   Low  (< 1300)       : Posisi 1 (MANU / GUIDED)
        //   Mid  (1300 .. 1700) : Posisi 2 (FBWA / RTL)
        //   High (>= 1700)      : Posisi 3 (AUTO / LOITER)
        int target_radio_mode = 0;
        if (ch_mode < 1300) {  // Bank 1: Regular modes (1, 2, 3)
            if (ch_mode_backup < 1300) {
                target_radio_mode = 1;  // Mode 1: MANU
            } else if (ch_mode_backup < 1700) {
                target_radio_mode = 2;  // Mode 2: FBWA
            } else {
                target_radio_mode = 3;  // Mode 3: AUTO
            }
        } else {  // Bank 2: Special modes (4, 5, 6)
            if (ch_mode_backup < 1300) {
                target_radio_mode = 4;  // Mode 4: GUIDED
            } else if (ch_mode_backup < 1700) {
                target_radio_mode = 5;  // Mode 5: RTL
            } else {
                target_radio_mode = 6;  // Mode 6: LOITER
            }
        }

        static int last_radio_mode = -1;
        if (last_radio_mode == -1) {
            // Inisialisasi awal saat pertama kali remote terbaca
            last_radio_mode = target_radio_mode;
            mode_now = target_radio_mode;
        } else if (target_radio_mode != last_radio_mode) {
            // Pilot secara fisik memindahkan switch remote -> override mode
            last_radio_mode = target_radio_mode;
            mode_now = target_radio_mode;
        }
    }
}
#endif
