#pragma once
#include <FW_controller.h>
#include <FW_ControlModes.h>
#include <FW_pitch_controller.h>
#include <AHRS.h>
#include <Radio.h>
#include <Fuzzy.h>
#include <Telemetry.h>
// #include <Transition.h>

Fuzzy *fuzzypitch = new Fuzzy();

float deltapitch;
float currentpitch = 0.0f;
float prevpitch = 0.0f;
float outputpitch;

void test_fuzzy_pitch() {
    currentpitch = abs(nav_pitch_deg - imu.pitch);
    // set the input
    fuzzypitch->setInput(1, currentpitch);
    fuzzypitch->setInput(2, deltapitch);
    // running the fuzzification
    fuzzypitch->fuzzify();
    // running the defuzzification
    outputpitch = fuzzypitch->defuzzify(1);
    // printing something gain K default = 2.09
    float pitch = 1.57 + outputpitch;
    float gyro_pitch = 0.2f; //get_gain_K_gyro();
    set_gain_pitch(pitch, gyro_pitch);
    deltapitch = abs(currentpitch - prevpitch);
    prevpitch = currentpitch;
    Serial.println("enter fuzzy mode!");
}

void setup_fuzzy_pitch() {
// Instantiating a FuzzyInput object
    FuzzyInput *errorpitch_fuzy = new FuzzyInput(1);
    FuzzySet *small = new FuzzySet(0.8, 10.4, 10.4, 20);
    errorpitch_fuzy->addFuzzySet(small);
    FuzzySet *safe = new FuzzySet(20, 35, 35, 50);
    errorpitch_fuzy->addFuzzySet(safe);
    FuzzySet *big = new FuzzySet(40, 47.5, 47.5, 55);
    errorpitch_fuzy->addFuzzySet(big);
    fuzzypitch->addFuzzyInput(errorpitch_fuzy);

// Instantiating a FuzzyInput object
    FuzzyInput *errorpitchDelta_fuzy = new FuzzyInput(2);
    FuzzySet *smallDelta = new FuzzySet(1, 1.5, 2, 2.5);
    errorpitchDelta_fuzy->addFuzzySet(smallDelta);
    FuzzySet *safeDelta = new FuzzySet(2, 4.67, 7.34, 10.01);
    errorpitchDelta_fuzy->addFuzzySet(safeDelta);
    FuzzySet *bigDelta = new FuzzySet(8, 15.35, 22.7, 30.05);
    errorpitchDelta_fuzy->addFuzzySet(bigDelta);
    fuzzypitch->addFuzzyInput(errorpitchDelta_fuzy);

// Instantiating a FuzzyOutput objects
    FuzzyOutput *speed = new FuzzyOutput(1);
    FuzzySet *slow = new FuzzySet(0, 0.5, 0.5, 1);
    speed->addFuzzySet(slow);
    FuzzySet *average = new FuzzySet(0.25, 1, 1, 1.75);
    speed->addFuzzySet(average);
    FuzzySet *fast = new FuzzySet(1, 1.5, 1.5, 2);
    speed->addFuzzySet(fast);
    fuzzypitch->addFuzzyOutput(speed);

  // Building FuzzyRule "IF error = small THEN speed = slow"
  // Instantiating a FuzzyRuleAntecedent objects
  // error small, delta error small
    FuzzyRuleAntecedent *if_small_smallDelta = new FuzzyRuleAntecedent();
    if_small_smallDelta->joinWithAND(small, smallDelta); 
    FuzzyRuleConsequent *thenOP1 = new FuzzyRuleConsequent();
    thenOP1->addOutput(slow);
    FuzzyRule *fuzzyRule01 = new FuzzyRule(1, if_small_smallDelta, thenOP1);
    fuzzypitch->addFuzzyRule(fuzzyRule01);

  // Building FuzzyRule "IF distance = safe THEN speed = average"
  // Instantiating a FuzzyRuleAntecedent objects
  // error avg, delta error small
    FuzzyRuleAntecedent *if_safe_smallDelta = new FuzzyRuleAntecedent();
    if_safe_smallDelta->joinWithAND(safe, safeDelta);
    FuzzyRuleConsequent *thenOP2 = new FuzzyRuleConsequent();
    thenOP2->addOutput(average);
    FuzzyRule *fuzzyRule02 = new FuzzyRule(2, if_safe_smallDelta, thenOP2);
    fuzzypitch->addFuzzyRule(fuzzyRule02);

  // Building FuzzyRule "IF distance = big THEN speed = high"
  // Instantiating a FuzzyRuleAntecedent objects
  // error big, delta error small
    FuzzyRuleAntecedent *if_big_smallDelta = new FuzzyRuleAntecedent();
    if_big_smallDelta->joinWithAND(big, smallDelta);
    FuzzyRuleConsequent *thenOZ1 = new FuzzyRuleConsequent();
    thenOZ1->addOutput(average);
    FuzzyRule *fuzzyRule03 = new FuzzyRule(3, if_big_smallDelta, thenOZ1);
    fuzzypitch->addFuzzyRule(fuzzyRule03);

    // Building FuzzyRule "IF distance = big THEN speed = high"
  // Instantiating a FuzzyRuleAntecedent objects
    FuzzyRuleAntecedent *if_small_safeDelta = new FuzzyRuleAntecedent();
    if_small_safeDelta->joinWithAND(small, safeDelta);
    FuzzyRuleConsequent *thenOZ2 = new FuzzyRuleConsequent();
    thenOZ2->addOutput(average);
    FuzzyRule *fuzzyRule04 = new FuzzyRule(4, if_small_safeDelta, thenOZ2);
    fuzzypitch->addFuzzyRule(fuzzyRule04);

   // Building FuzzyRule "IF distance = big THEN speed = high"
  // Instantiating a FuzzyRuleAntecedent objects
    FuzzyRuleAntecedent *if_safe_safeDelta = new FuzzyRuleAntecedent();
    if_safe_safeDelta->joinWithAND(safe, safeDelta);
    FuzzyRuleConsequent *thenON1 = new FuzzyRuleConsequent();
    thenON1->addOutput(fast);
    FuzzyRule *fuzzyRule05 = new FuzzyRule(5, if_safe_safeDelta, thenON1);
    fuzzypitch->addFuzzyRule(fuzzyRule05);

    // Building FuzzyRule "IF distance = big THEN speed = high"
  // Instantiating a FuzzyRuleAntecedent objects
    FuzzyRuleAntecedent *if_big_safeDelta = new FuzzyRuleAntecedent();
    if_big_safeDelta->joinWithAND(big, safeDelta);
    FuzzyRuleConsequent *thenON2 = new FuzzyRuleConsequent();
    thenON2->addOutput(fast);
    FuzzyRule *fuzzyRule06 = new FuzzyRule(6, if_big_safeDelta, thenON2);
    fuzzypitch->addFuzzyRule(fuzzyRule06);

    // Building FuzzyRule "IF distance = big THEN speed = high"
  // Instantiating a FuzzyRuleAntecedent objects
    FuzzyRuleAntecedent *if_small_bigDelta = new FuzzyRuleAntecedent();
    if_small_bigDelta->joinWithAND(small, bigDelta);
    FuzzyRuleConsequent *thenOZ3 = new FuzzyRuleConsequent();
    thenOZ3->addOutput(fast);
    FuzzyRule *fuzzyRule07 = new FuzzyRule(7, if_small_bigDelta, thenOZ3);
    fuzzypitch->addFuzzyRule(fuzzyRule07);

    // Building FuzzyRule "IF distance = big THEN speed = high"
  // Instantiating a FuzzyRuleAntecedent objects
    FuzzyRuleAntecedent *if_safe_bigDelta = new FuzzyRuleAntecedent();
    if_safe_bigDelta->joinWithAND(safe, bigDelta);
    FuzzyRuleConsequent *thenON3 = new FuzzyRuleConsequent();
    thenON3->addOutput(fast);
    FuzzyRule *fuzzyRule08 = new FuzzyRule(8, if_safe_bigDelta, thenON3);
    fuzzypitch->addFuzzyRule(fuzzyRule08);

    // Building FuzzyRule "IF distance = big THEN speed = high"
  // Instantiating a FuzzyRuleAntecedent objects
    FuzzyRuleAntecedent *if_big_bigDelta = new FuzzyRuleAntecedent();
    if_big_bigDelta->joinWithAND(big, bigDelta);
    FuzzyRuleConsequent *thenON4 = new FuzzyRuleConsequent();
    thenON4->addOutput(fast);
    FuzzyRule *fuzzyRule09 = new FuzzyRule(9, if_big_bigDelta, thenON4);
    fuzzypitch->addFuzzyRule(fuzzyRule09);
}

// void threadFWPitch() {
//     while (true) {
//         test_fuzzy_pitch();
//         threads.yield();
//     }
// }