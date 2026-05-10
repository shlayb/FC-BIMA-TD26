#pragma once
#include "Arduino.h"
#include "AHRS.h"
#include "Radio.h"
#include "Mode_setup.h"
#include "Mode_Flywing.h"
#include "Mode_Copter.h"
#include "Mode_Fixedwing.h"
#include "Mode_Transisi.h"



void setup_mode(){
  // contoh input pilot
  // if(Serial.available()){
  //   char c = Serial.read();
  //   if(c=='z') mode_now = 1;
  //   if(c=='x') mode_now = 2;
  //   if(c=='c') mode_now = 3;
  //   if(c=='v') mode_now = 4;
  // }
  ahrs.update_ahrs();
  nav_gps();
  modes.update();

  // Keep payload release updated each control cycle
  

  if (is_transitioning){
    update_transition();        
          // fase transxxxisi mengontrol aktuator sendiri
    return;                           // selesai frame ini
  }

  ModeId want = active_mode; // default: tetap di mode sekarang
  //  FIX: RC channels only support 3 modes
  // GUIDED only via GCS/MAVLink (mode_now == 4)
  if (mode_now == 1) want = ModeId::MANU;
  else if (mode_now == 2) want = ModeId::FBWA;
  else if (mode_now == 3) want = ModeId::AUTO;
  else if (mode_now == 4) want = ModeId::GUIDED;  //  GCS command only

  // transisi full mode
    // if(mode_now == 1 && active_mode == ModeId::FBWA && active_mode != ModeId::AUTO){
    //     modes.suspend(); 
    //     start_transition_to_mc(); // FW -> MC
    // }
    // else if(mode_now == 2 && active_mode != ModeId::FBWA && active_mode != ModeId::AUTO){
    //     modes.suspend(); 
    //     start_transition_to_fw(); // MC -> FW
    // }
    // else if(mode_now == 1 && active_mode != ModeId::FBWA){
    //   modes.setMode(ModeId::COPT);
    //   active_mode = ModeId::COPT;
    //   current    = ModeId::COPT;
    // }

    // else if (mode_now == 3) want = ModeId::MANU;
    // else if (mode_now == 4) want = ModeId::AUTO;
    // else {
    //   if (want != active_mode){
    //   // mode berubah
    //   if (modes.setMode(want)){
    //   active_mode = want;
    //   current    = want;
    //     }
    //   }
    // }

    // //full FixedWIng
    if(mode_now == 1){
    modes.setMode(ModeId::MANU);
    active_mode = ModeId::MANU;
    }
    else if(mode_now == 2){
    modes.setMode(ModeId::FBWA);
    active_mode = ModeId::FBWA;
    }
    else if(mode_now == 3){
    modes.setMode(ModeId::GUIDED);
    active_mode = ModeId::GUIDED;
    }
    else if(mode_now == 4){
    modes.setMode(ModeId::AUTO);
    active_mode = ModeId::AUTO;
    }
    
    //full flywing
    // if(mode_now == 1){
    // modes.setMode(ModeId::MANU);
    // active_mode = ModeId::MANU;
    // }
    // else if(mode_now == 2){
    // modes.setMode(ModeId::FBWA);
    // active_mode = ModeId::FBWA;
    // }
    // else if(mode_now == 3){
    // modes.setMode(ModeId::AUTO);
    // active_mode = ModeId::AUTO;
    // }

}
