#ifndef TELEM_H
#define TELEM_H
#include "Attitude.h"
#include <Protobuf/Protobuf.h>
#include "Transition.h"
#include "airspeed.h"
#include "gps.h"
#include "Voltage.h"
#include "flywing_control.h"
#include "TimingUtils.h"
// #include "mode_setup.h"
#include "fuzzy_fw_roll.h"
// #include "Mode_Transisi.h"

// #include <Scheduler/scheduler.h>
class Telemetry {
   public:
    // send telemetry log
    void print_log_fw_telem();          // serial print to putty with telemetry module
    void tuning_gain_pid();             // for tuning using keyboard
    void send_vehicle_attitude_data();  // send vehicle attitude data to gcs using protobuf
    void receive_command_from_gcs();    // receive command from gcs
    void print_wp();                    // for serial print waypoint
    void calibrate_compass();           // to check compass calibration value
    void raw_imu();                     // to check imu calibration value
    void pid_gain_tuning();             // new telemetry tuning program, tested using putty
    void send_telemetry();              // send telemetry data to gcs
};

/**
 * @brief program to modify gain value
 * @param prev_gain refers to the gain value to be replaced
 * @param new_gain are the new gain that replaces/increments prev_gain
 */

void modifygain(float &prev_gain, float new_gain) {
    prev_gain += new_gain;
}

/**
 * @brief function to temporarily manipulate gain based on GCS input
 * @return void, but chages the selected gain value temporarily
 * @details read serial input and use switch case based selector
 * @note better used with arduino IDE serial monitor
 * @note make sure to input ONLY ONE char
 *
 */

int linee = 0;
int milisawall = 0;
void Telemetry::pid_gain_tuning() {
    if (Serial2.available()) {
        char selector = Serial2.read();

        if (true) {
            switch (selector) {
                case 'Q':
                case 'q':
                    modifygain(r_gain.roll_P, 0.01);
                    break;
                case 'A':
                case 'a':
                    modifygain(r_gain.roll_P, -0.01);
                    break;
                case 'W':
                case 'w':
                    modifygain(r_gain.roll_D, 0.01);
                    break;
                case 'S':
                case 's':
                    modifygain(r_gain.roll_D, -0.01);
                    break;
                case 'E':
                case 'e':
                    modifygain(p_gain.pitch_P, 0.01);
                    break;
                case 'D':
                case 'd':
                    modifygain(p_gain.pitch_P, -0.01);
                    break;
                case 'R':
                case 'r':
                    modifygain(p_gain.pitch_D, 0.01);
                    break;
                case 'F':
                case 'f':
                    modifygain(p_gain.pitch_D, -0.01);
                    break;
                default:
                    break;
            }
        }
    }
    Serial2.print("1)PR: ");
    Serial2.print(r_gain.roll_P);
    Serial2.print(" | 2)DR: ");
    Serial2.print(r_gain.roll_D);
    Serial2.print(" | 3)PP: ");
    Serial2.print(p_gain.pitch_P);
    Serial2.print(" | 4)PD: ");
    Serial2.print(p_gain.pitch_D);

    // Serial2.print("YP: ");
    // Serial2.print(gain.rate_yaw_P);
    // Serial2.print(" |YD: ");
    // Serial2.print(gain.rate_yaw_D);
    // Serial2.print(" |YI: ");
    // Serial2.print(gain.rate_yaw_I);
    // Serial2.print(" |PD: ");
    // Serial2.print(flywingcontrol.gain_flywing.k_pitch_rate);
    Serial2.print(" |Arm: ");
    Serial2.print(arming);
    Serial2.print(" |R: ");
    Serial2.print(imu.roll);
    Serial2.print(" |P: ");
    Serial2.print(imu.pitch);
    Serial2.print(" |Y: ");
    Serial2.print(imu.yaw);
    Serial2.print(" |head:");
    Serial2.print(imu.heading);
    Serial2.print(" |alt = ");
    Serial2.print(baro.altitude);
    // Serial2.print(" |Mode: ");
    // Serial2.print(modeCode4(current));
    Serial2.print(" |Mode: ");
    Serial2.print(mode_now);
    Serial2.print(" |ch_roll: ");
    Serial2.print(ch_roll);
    Serial2.print(" |ch_pitch: ");
    Serial2.print(ch_pitch);
    Serial2.print(" |ch_yaw: ");
    Serial2.print(ch_yaw);
    Serial2.print(" |ch_thr: ");
    Serial2.print(ch_throttle);
    // Serial2.print(" |gx:");
    // Serial2.print(imu.gyro_x);
    // Serial2.print(" |gy:");
    // Serial2.print(imu.gyro_y);
    // Serial2.print(" |gz:");
    // Serial2.print(imu.gyro_z);
    // Serial2.print(" |hdop:");
    // Serial2.print(gepees.hdop);
    // Serial2.print(" |sat:");
    // Serial2.print(gepees.satellites);
    // Serial2.print(" |gspd:");
    // Serial2.print(ahrs.groundspeed);

    // Serial2.print(" |aspd = ");
    // Serial2.print(arspd.v_ms);
    // Serial2.print(" |GPD = ");
    // Serial2.print(get_pitch_demand());
    // Serial2.print(" |GTD = ");
    // Serial2.print(get_throttle_demand());
    // Serial2.print(" |GRD: ");
    // Serial2.print(nav_roll_cd());
    // Serial2.print(" |dis_wp = ");
    // Serial2.print(current_loc.get_distance(next_WP_loc));
    // Serial2.print(" |flwp = ");
    // Serial2.print(display_wp());
    // Serial2.print(" |PA = ");
    // Serial2.print(nav_roll_deg);
    // Serial2.print(" |PE = ");
    // Serial2.print(nav_pitch_deg);
    // Serial2.print(" |arm:");
    // Serial2.print(arming);
    // Serial2.print(" |phase:");
    // Serial2.print(phase_now);
    // Serial2.print(" |alt:");
    // Serial2.print(baro.altitude);
    // Serial2.print(" wpsum:");
    // Serial2.print(wp_sum);
    // Serial2.print(" wp:");
    // Serial2.print(flag_wp);
    // Serial2.print(" dist:");
    // Serial2.print(auto_state.distance_next_wp/*current_loc.get_distance(next_WP_loc)*/);
    // Serial2.print(" | r:");
    // Serial2.print(imu.roll);
    // Serial2.print(" | p:");
    // Serial2.print(imu.pitch);
    // Serial2.print(" p_rad:");
    // Serial2.print(rad_pitch);
    // Serial2.print(" | y:");
    // Serial2.print(imu.yaw);
    // Serial2.print(" |hdop:");
    // Serial2.print(gepees.hdop);
    // Serial2.print(" |sat:");
    // Serial2.print(gepees.satellites);
    // Serial2.print(" mode: ");
    // Serial2.print(modeCode4(current));
    // Serial2.print(" MHOV: ");
    // Serial2.print(QUAD_PWM_THR);
    // Serial2.print (" mp:");
    // Serial2.print (motor_pwm);
    // Serial2.print(" MPUS: ");
    // Serial2.print(PUSHER_PWM);
    // Serial2.print(" dR:");
    // Serial2.print(imu.delta_roll);
    // Serial2.print(" fzout:");
    // Serial2.print(outroll);

    // Serial2.print(" |ch_thr:");
    // Serial2.print(ch_throttle);
   
    // Serial2.print(" navroll:");
    // Serial2.print(nav_roll_deg);
    // Serial2.print(" navptch ");
    // Serial2.print(nav_pitch_deg);
    // Serial2.print(" at:");
    // Serial2.print(get_throttle_demand());
    // Serial2.print(" bearing_error:");
    // Serial2.print(_bearing_error);
    // Serial2.print(" NRD:");
    // Serial2.print(nav_roll_deg);
    // Serial2.print(" PAL:");
    // Serial2.print(pwm_ail_L);
    // Serial2.print(" PAR:");
    // Serial2.print(pwm_ail_R);
    // Serial2.print(" PEL:");
    // Serial2.print(pwm_ele);
    // Serial2.print(" PTH:");
    // Serial2.print(motor_pwm);
    // Serial2.print(" |head:");
    // Serial2.print(imu.heading);
    // Serial2.print(" get_VXdot:");
    // Serial2.print(get_VXdot());
    // Serial2.print(" lacc:");
    // Serial2.print(_linear_acc.y);
    // Serial2.print(" p:");
    // Serial2.print(positive_counter);
    // Serial2.print(" n:");
    // Serial2.print(negative_counter);
    // Serial2.print(" |lat:");
    // Serial2.print(gepees.latitude, 6);
    // Serial2.print(" |lng:");
    // Serial2.print(gepees.longitude, 6);
    // Serial2.print(" aspd:");
    // Serial2.print(arspd.v_ms);

    // Serial2.print (" m1: ");
    // Serial2.print (m1_pwm);
    // Serial2.print (" m2: ");
    // Serial2.print (m2_pwm);
    // Serial2.print (" m3: ");
    // Serial2.print (m3_pwm);
    // Serial2.print (" m4: ");
    // Serial2.print (m4_pwm);
    // Serial2.print(" | gspd:");
    // Serial2.print(ahrs.groundspeed);
    // Serial2.print(" |millis:");
    // Serial2.print(getMillis());
    // Serial2.print(" gspd_l1:");
    // Serial2.print(grspd);
    // Serial2.println(groundspeed);
    // Serial2.print(" sp:");
    // Serial2.print(scaleToAngle(ch_roll));
    // Serial2.print(" bearing :");
    // Serial2.print(nav_bearing_cd()); 
    // Serial2.print(" batt:");
    // Serial2.print(batt_v);
    // Serial2.print(" tim:");    

    // Serial2.print(" | 6)DP: ");
    // Serial2.print(gain.k_pitch_rate);

    // Serial2.print(" | 7)PY: ");
    // Serial2.print(gain.k_yaw);
    // Serial2.print(" | 8)IY: ");
    // Serial2.print(gain.k_i_yaw);
    // Serial2.print(" | 9)DY: ");
    // Serial2.print(gain.k_yaw_rate);

    // Serial2.print("|   Roll: ");
    // Serial2.print(0);
    // Serial2.print(" pitch: ");
    // Serial2.print(0);
    // Serial2.print(" Yaw: ");
    // if(linee == 100){
    //     Serial2.println("DAH 100 LINEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
    //     Serial2.print(" TOTAL WAKTU :    ");
    //     Serial2.println(millis() - milisawall);
    // }
    // linee++;
    Serial2.println(" ");
}

void Telemetry::print_log_fw_telem() {
    String desc[29];
    float data[29];

    desc[0] = "";
    desc[1] = " phse:";
    desc[2] = " r:";
    desc[3] = " p:";
    desc[4] = " y:";
    desc[5] = " alt:";

    desc[6] = " tim:";
    desc[7] = " sp:";
    desc[8] = " droll:";
    desc[9] = " hdop:";
    desc[10] = " fout:";

    desc[11] = " gsp:";
    desc[12] = " sat:";
    desc[13] = " wp:";
    desc[14] = " NRD:";
    desc[15] = " PEL:";
    desc[16] = " PTH:";

    desc[17] = " head:";
    desc[18] = " dist:";
    desc[19] = " lng:";
    desc[20] = " lat:";
    desc[21] = " ar:";
    desc[22] = " ap:";
    desc[23] = " at:";

    data[0] = arming;
    data[1] = mode_phase;
    data[2] = imu.roll;
    data[3] = imu.pitch;
    data[4] = imu.yaw;        // heading;
    data[5] = baro.altitude;

    data[6] = millis();    // altitude
    data[7] = scaleToAngle(ch_roll);
    data[8] = imu.delta_roll;
    data[9] = gepees.hdop;
    data[10] = outroll;  // getClimbRate();

    data[11] = ahrs.groundspeed;
    data[12] = gepees.satellites;
    data[13] = flag_wp;
    data[14] = nav_roll_deg;
    data[15] = ch_pitch;
    data[16] = motor_pwm;

    data[17] = imu.heading;
    data[18] = current_loc.get_distance(next_WP_loc);
    data[19] = gepees.longitude;
    data[20] = gepees.latitude;
    data[21] = nav_roll_deg;
    data[22] = get_pitch_demand();
    data[23] = get_throttle_demand();
    

    for (int i = 0; i < 24; i++) {
        if (i < 19) {
            Serial2.print(desc[i]);
            Serial2.print(data[i]);
        } else {
            Serial2.print(desc[i]);
            Serial2.print(data[i], 6);
        }
    }
    // Serial2.print(MODE);
    Serial2.println();
}
// void Telemetry::tuning_gain_pid()
// {
//   uint8_t key;
//   while (Serial1.available()){
//     key = Serial1.read();
//     switch (key){
//     case 'Q' :  /// add
//         controls.set_gain_k_roll(&controls.gain.k_roll); ///add
//         break;
//     case 'A' :
//         controls.set_gain_k_gyro_roll(&controls.gain.k_roll_rate);
//         break;
//     case 'Z' :
//         controls.set_gain_k_i_roll(&controls.gain.k_i_roll);
//         break;
//     case 'W' :
//         controls.set_gain_k_pitch(&controls.gain.k_pitch);
//         break;
//     case 'S' :
//         controls.set_gain_k_gyro_pitch(&controls.gain.k_pitch_rate);
//         break;
//     case 'X' :
//         controls.set_gain_k_i_pitch(&controls.gain.k_i_pitch);
//         break;
//     case 'E' :
//         controls.set_gain_k_yaw(&controls.gain.k_yaw);
//         break;
//     case 'D' :
//         controls.set_gain_k_gyro_yaw(&controls.g;ain.k_yaw_rate);
//         break;
//     case 'R' :
//         controls.set_gain_k_alt(&controls.gain.k_alt);
//         break;
//     case 'F' :
//         controls.set_gain_k_z_velocity(&controls.gain.k_z_vel);
//         break;
//     case 'q' : ///subtract
//         controls.min_set_gain_k_roll(&controls.gain.k_roll);  /// subtract
//         break;
//     case 'a' :
//         controls.min_set_gain_k_gyro_roll(&controls.gain.k_roll_rate);
//         break;
//     case 'z' :
//         controls.min_set_gain_k_i_roll(&controls.gain.k_i_roll);
//         break;
//     case 'w' :
//         controls.min_set_gain_k_pitch(&controls.gain.k_pitch);
//         break;
//     case 's' :
//         controls.min_set_gain_k_gyro_pitch(&controls.gain.k_pitch_rate);
//         break;
//     case 'x' :
//         controls.min_set_gain_k_i_pitch(&controls.gain.k_i_pitch);
//         break;
//     case 'e' :
//         controls.min_set_gain_k_yaw(&controls.gain.k_yaw);
//         break;
//     case 'd' :
//         controls.min_set_gain_k_gyro_yaw(&controls.gain.k_yaw_rate);
//         break;
//     case 'r' :
//         controls.min_set_gain_k_alt(&controls.gain.k_alt);
//         break;
//     case 'f' :
//         controls.min_set_gain_k_z_velocity(&controls.gain.k_z_vel);
//         break;
//     default:
//         break;
//         };
//     };
// }

void Telemetry::send_vehicle_attitude_data() {
    // complete this parameter below cuy !
    gcs.SendFWBuffer(
                    next_WP_loc.alt, 
					0, //----velocity_target, 
					current_loc.get_distance(next_WP_loc),
					imu.roll, 
					imu.pitch, 
					imu.heading,
					baro.altitude, 
					arspd.v_ms, //----airspeed 
					ahrs.groundspeed,
					gepees.latitude,
					gepees.longitude, 
					display_batt(), //percentage
					MODE, //MODE
					arming, 
					mode_phase,
					imu.delta_roll, //aileron
					0,  // elevator
					0, // rudder
					display_wp(), // jumlah wp
				    nav_roll_deg,//---roll_demand, 
					nav_pitch_deg, //calc_nav_pitch, //---pitch_demand,
				  	0, //--yaw demand
					next_WP_loc.alt, //-- alt_demand, 
					get_throttle_demand(), //---- aspd_demamd,
					outroll,
					0, //--pitch_control,
					0, //---yaw_control,
					0, //--throttle_control,
					scaleToAngle(ch_roll), //-- roll_radio,
					ch_pitch,  //--pitch_radio,
					ch_yaw,  //--yaw_radio,
					ch_throttle, /// throttle_radio,
					_crosstrack_error, //--Xtrack_err,  
					gepees.hdop, 
					gepees.satellites, 
					auto_state.next_turn_angle, 
				  	0, ///bearing to next wp 
					acceptance_distance_m,
					0, //lateral_acceleration(),  
					0,  //--climbRate, //! need implementation
                    pwm_ail_R, //--ail_as_servo, 
					pwm_ele, //--ele_as_servo, 
					pwm_rud_R, //--rud_as_servo, 
					ch_throttle,  //--mtr_as_servo, 
				  	HEAD_MAX);
    }       
    
// void Telemetry::print_wp(){
//     // Serial.print("TAKEOFF(%d)\r\n",fixwing.takeoff_sum);
//     // for(int i=0;i<fixwing.takeoff_sum;i++){
//     //     Serial.print("%d lat=%d lng=%d\r\n",
//     //     i,
//     //     fixwing.takeoff[i].lat,
//     //     fixwing.takeoff[i].lng
//     //     );
//     // }
//     // Serial.print("WAYPOINT(%d)\r\n",fixwing.wp_sum);
//     // for(int i=0;i<fixwing.wp_sum;i++){
//     //     Serial.print("%d lat=%d lng=%d\r\n",
//     //     i,
//     //     fixwing.waypoint[i].lat,
//     //     fixwing.waypoint[i].lng
//     //     );
//     // }
//     // Serial.print("LANDING(%d)\r\n",fixwing.land_sum);
//     // for(int i=0;i<fixwing.land_sum;i++){
//     //     Serial.print("%d lat=%d lng=%d\r\n",
//     //     i,
//     //     fixwing.landing[i].lat,
//     //     fixwing.landing[i].lng
//     //     );
//     // }
// }
// void Telemetry::calibrate_compass(){
//     // telemprint("%.3f %.3f %.3f\r\n",
//     // ahrs()._compass.getMag().x,
//     // ahrs()._compass.getMag().y,
//     // ahrs()._compass.getMag().z
//     // );
//     // dbg("%.3f %.3f %.3f\r\n",
//     // ahrs()._compass.getMag().x,
//     // ahrs()._compass.getMag().y,
//     // ahrs()._compass.getMag().z
//     // );
// }
// void Telemetry::raw_imu(){
//     // dbg("r=%.3f p=%.3f gyro =%.3f %.3f accel=%.3f %.3f\r\n",
//     // ahrs().roll,
//     // ahrs().pitch,
//     // ahrs().get_gyro().x,
//     // ahrs().get_gyro().y,
//     // ahrs().get_accel().x,
//     // ahrs().get_accel().y
//     // );
// }
extern Telemetry telem;
// extern ModeId current;
#endif