#pragma once

#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

#define BUZZER_PIN 35
#define BUZZER_DELAY_MS 100
#define BUZZER_FREQ 4000
#define BEEP_DURATION 100

#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976

#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7 2794 
#define NOTE_FS7 2960 
#define NOTE_G7 3136

class Buzzer {
public:
    void init();
    void update(bool armed);
    void beep();
    void stop();
    void loop();
    void soundBuzzer();
    void playHappyBirthday();
    void playNote(int freq, int duration);
    void initWithMusic();
    
    bool buzzer_state = false;
    unsigned long last_buzz_time = 0;
    bool is_buzzing = false;
    unsigned long last_beep = 0;
    bool is_beeping = false;
    
    bool isPlayingInitMusic = false;
private:
    bool initialized = false;
    uint8_t currentNote = 0;
    unsigned long lastNoteTime = 0;
    bool waiting_long_pause = false;
    uint8_t beep_count = 0;
    unsigned long beep_timer = 0;
    unsigned long long_pause_start = 0;
    unsigned long transition_timer = 0;
    bool in_transition = false;  
};

Buzzer buzzer;

void Buzzer::init() {
    if (!initialized) {
        pinMode(BUZZER_PIN, OUTPUT);
        Serial.println("Buzzer initialized");
        
        // Play Happy Birthday tune on first initialization
        // playHappyBirthday();
        
        initialized = true;
    }
}

void Buzzer::initWithMusic() {
    if (!initialized) {
        pinMode(BUZZER_PIN, OUTPUT);
        Serial.println("Buzzer initialized");
        initialized = true;
        isPlayingInitMusic = true;
        currentNote = 0;
        lastNoteTime = millis();
    }
    
    if (isPlayingInitMusic) {
        unsigned long currentTime = millis();
        // // Happy Birthday notes array
        // const uint16_t notes[] = {
        //     NOTE_C6, NOTE_C6, NOTE_D6, NOTE_C6, NOTE_F6, NOTE_E6,
        //     NOTE_C6, NOTE_C6, NOTE_D6, NOTE_C6, NOTE_G6, NOTE_F6,
        //     NOTE_C6, NOTE_C6, NOTE_C7, NOTE_A6, NOTE_F6, NOTE_E6, NOTE_D6,
        //     NOTE_AS6, NOTE_AS6, NOTE_A6, NOTE_F6, NOTE_G6, NOTE_F6
        // };
        // // Corresponding durations
        // const uint16_t durations[] = {
        //     400, 400, 800, 800, 800, 1000,
        //     400, 400, 800, 800, 800,800,
        //     400, 400, 800, 800, 800,800, 800,
        //     400, 400, 800, 800, 800, 800
        // };

        // Mario Bros Theme
        const int notes[] = {
        NOTE_E7, NOTE_E7, NOTE_E7, NOTE_C7, NOTE_E7, NOTE_G7, NOTE_G6, NOTE_G6
         };

        const int durations[] = {
        175, 175, 175, 175, 175, 350, 350, 350
        };
        
        const float staccatoFactor = 0.7;  
        int totalNotes = sizeof(notes) / sizeof(notes[0]);

        if (currentNote < totalNotes) {
            if (currentTime - lastNoteTime >= durations[currentNote]) {
                int playDuration = durations[currentNote] * staccatoFactor;
                tone(BUZZER_PIN, notes[currentNote], playDuration);
                lastNoteTime = currentTime;
                currentNote++;
                }

            } else {
                noTone(BUZZER_PIN);
                isPlayingInitMusic = false;
                in_transition = true;
                transition_timer = millis(); 
                last_buzz_time = millis();
                is_buzzing = false;
                waiting_long_pause = false;
                beep_count = 0;
                beep_timer = millis();
                long_pause_start = millis();
            }
        }
    }

void Buzzer::loop() {
    unsigned long current_time = millis();
    
    if (current_time - last_beep >= BUZZER_DELAY_MS) {
        if (!is_beeping) {
            beep();
        } else {
            stop();
        }
        is_beeping = !is_beeping;
        last_beep = current_time;
    }
}

void Buzzer::update(bool armed) {
    const unsigned long LONG_PAUSE = 2000;  // jeda
    const unsigned long BEEP_INTERVAL = 100;  // interval antar beep
    const unsigned long TRANSITION_DELAY = 2000;

    unsigned long current_time = millis();

    if (in_transition) {
        if (current_time - transition_timer >= TRANSITION_DELAY) {
            in_transition = false;
        }
        return; 
    }
    
    if (!armed) {
        if (waiting_long_pause) {
            if (current_time - long_pause_start >= LONG_PAUSE) {
                waiting_long_pause = false;
                beep_count = 0;
            }
            return;
        }

        if (!is_buzzing) {
            if (current_time - beep_timer >= BEEP_INTERVAL) {  //interval beep
                tone(BUZZER_PIN, BUZZER_FREQ, BEEP_DURATION);
                beep_timer = current_time;
                beep_count++;
                is_buzzing = true;

                if (beep_count >= 3) {
                    waiting_long_pause = true;
                    long_pause_start = current_time;
                }
            }
        } else {
            if (current_time - beep_timer >= BEEP_DURATION) {
                noTone(BUZZER_PIN);
                is_buzzing = false;
            }
        }
    } else {
        noTone(BUZZER_PIN);
        is_buzzing = false;
        beep_count = 0;
        waiting_long_pause = false;
    }
}

void Buzzer::beep() {
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, BUZZER_FREQ, BEEP_DURATION); 
    delay(200);                
    tone(BUZZER_PIN, BUZZER_FREQ, BEEP_DURATION);  
    delay(200);                 
  }
  delay(500);
}

void Buzzer::soundBuzzer() {
    analogWrite(BUZZER_PIN, HIGH);
    buzzer_state = true;
}

void Buzzer::stop() {
    analogWrite(BUZZER_PIN, LOW);
    buzzer_state = false;
}

// ==== Tambahan untuk musik ====
void Buzzer::playNote(int freq, int duration) {
    if (freq == 0) {
        noTone(BUZZER_PIN);
    } else {
        tone(BUZZER_PIN, freq, duration);
    }
    delay(duration * 1.3); 
}


#endif // BUZZER_H