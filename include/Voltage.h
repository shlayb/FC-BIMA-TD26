#pragma once

#include <Arduino.h>

const int numReadings = 10;

int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int average = 0;            // the average

double raw_batt;
float batt_v = 0;
int32_t perc_batt;
uint64_t prevs_time;

int inputPin = A13;

void init_voltage() {
  // initialize serial communication with computer:
  // Serial.begin(9600);
  // initialize all the readings to 0:
  pinMode(inputPin, INPUT);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void read_voltage() {
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(inputPin);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits

  raw_batt = map(average, 0, 1023, 0, 16800);
  batt_v = (raw_batt / 1000.0f) - 0.4;
  perc_batt = map(batt_v, 14.4, 16.4, 0, 100);
}

void update_voltage() {
    uint64_t currs_time = millis();
    if (currs_time - prevs_time >= 50) {
        read_voltage();
        prevs_time = currs_time;
    }
}


int32_t display_batt() {
  if (perc_batt < 0) {
    return 87;
  }
  return perc_batt;
}