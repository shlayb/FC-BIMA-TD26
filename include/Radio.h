#ifndef RADIO_H
#define RADIO_H
#include <Arduino.h>
#include <sbus.h>

bfs::SbusRx sbus_rx(&Serial8);
// bfs: class dr library, Serial8 dari teensy

bfs::SbusData data;

bool signal_lost = false;
int16_t ch_roll, ch_pitch, ch_throttle, ch_yaw, ch_mode, ch_mode_backup, ch_vehicle_mode;
bool arming;

// enum MODE
// {
//     PHASE_1 = 0,
//     PHASE_2 = 1,
//     PHASE_3 = 2,
// };

bool alt_hold = false;
bool mode_fbwa = false;
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
int mode_now, prev_mode;

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

    if (sbus_rx.Read()) {
        data = sbus_rx.data();

        arming = data.ch[4] > 1500 ? 1 : 0;
        signal_lost = data.lost_frame;

        if (arming) {
            Serial.println("Please disarm the remote for safety");
            while (arming) {
                if (sbus_rx.Read()) {
                    data = sbus_rx.data();
                    arming = data.ch[4] > 1500 ? 1 : 0;
                    signal_lost = data.lost_frame;
                }
            }
        }

        if (signal_lost) {
            Serial.println("Signal lost, please reconnect");
            while (signal_lost) {
                if (sbus_rx.Read()) {
                    data = sbus_rx.data();
                    arming = data.ch[4] > 1500 ? 1 : 0;
                    signal_lost = data.lost_frame;
                }
            }
        }
    }

    Serial.println("Remote setup complete");
}

//remote satria
void remote_loop() {
    if (sbus_rx.Read()) {
        data = sbus_rx.data();

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
        signal_lost = data.lost_frame;

        ch_mode = data.ch[5] * 0.6267f + 877.91f;
        ch_mode_backup = data.ch[6] * 0.6267f + 877.91f;
        ch_vehicle_mode = data.ch[7] * 0.6267f + 877.91f;

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
        if (ch_mode_backup <= 1000) {
            mode_now = 1;
        } else if (ch_mode_backup > 1000 && ch_mode_backup <= 1512) {
            mode_now = 2;
        } else {
            mode_now = 3;
        }
    }
}
#endif
