#pragma once
#include "Arduino.h"
#include "AHRS.h"
#include "radio.h"
#include "mode_setup.h"
#include "Mode_Flywing.h"
#include "Mode_Copter.h"
#include "Mode_Fixedwing.h"
#include "Mode_Transisi.h"



void setup_mode(){
  // contoh input pilot
  // if(Serial.available()){
  //   char c = Serial.read();
  //   if(c=='1') mode_now = 1;
  //   if(c=='2') mode_now = 2;
  //   if(c=='3') mode_now = 3;
  //   if(c=='4') mode_now = 4;
  // }
  ahrs.update_ahrs();
  nav_gps();
  modes.update();

  

  if (is_transitioning){
    update_transition();        
          // fase transisi mengontrol aktuator sendiri
    return;                           // selesai frame ini
  }

  ModeId want = active_mode; // default: tetap di mode sekarang
  if (mode_now == 1) want = ModeId::COPT;
  else if (mode_now == 2) want = ModeId::FBWA;
  else if (mode_now == 3) want = ModeId::AUTO;
  else if (mode_now == 4) want = ModeId::MANU;

  // transisi full mode
    if(mode_now == 1 && active_mode == ModeId::FBWA && active_mode != ModeId::AUTO){
        modes.suspend(); 
        start_transition_to_mc(); // FW -> MC
    }
    else if(mode_now == 2 && active_mode != ModeId::FBWA && active_mode != ModeId::AUTO){
        modes.suspend(); 
        start_transition_to_fw(); // MC -> FW
    }
    else if(mode_now == 1 && active_mode != ModeId::FBWA){
      modes.setMode(ModeId::COPT);
      active_mode = ModeId::COPT;
      current    = ModeId::COPT;
    }

    // else if (mode_now == 3) want = ModeId::MANU;
    // else if (mode_now == 4) want = ModeId::AUTO;
    else {
      if (want != active_mode){
      // mode berubah
      if (modes.setMode(want)){
      active_mode = want;
      current    = want;
        }
      }
    }

    // //full copter
    // if(mode_now == 1){
    // // modes.setMode(ModeId::COPT);
    // active_mode = ModeId::COPT;
    // }
    // else if(mode_now == 2){
    // modes.setMode(ModeId::QHOV);
    // active_mode = ModeId::QHOV;
    // }
    
    // //full flywing
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
