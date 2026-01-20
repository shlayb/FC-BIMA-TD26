#pragma once
#include "AHRS.h"
#include "FW_ControlModes.h"
#include "FW_controller.h"
#include "FW_roll_controller.h"
#include "Fuzzy.h"
#include "Radio.h"
#include "Telemetry.h"
// #include <Transition.h>

Fuzzy *fuzzyroll = new Fuzzy();

float deltaroll;
float currentroll = 0.0f;
float prevroll = 0.0f;
float outroll = 0.0f;

void  test_fuzzy_roll() {
    currentroll = abs(nav_roll_deg - imu.roll);
    // set the input
    fuzzyroll->setInput(1, currentroll);
    fuzzyroll->setInput(2, imu.delta_roll);
    // running the fuzzification
    fuzzyroll->fuzzify();
    // running the defuzzification
    outroll = fuzzyroll->defuzzify(1);
    // printing something gain K default = 2.09
    float finalroll = 1.0 + outroll;
    float gyro_roll = 0.0f;  // get_gain_K_gyro();
    set_gain_roll(finalroll, gyro_roll);
    // deltaroll = abs(currentroll - prevroll);
    prevroll = currentroll;
    // Serial.println(roll);
}

void setup_fuzzy_roll() {
    // Instantiating a FuzzyInput object
    FuzzyInput *errorRoll_fuzy = new FuzzyInput(1);
    FuzzySet *small = new FuzzySet(5, 10, 10, 15); //20 25 25 30 - 
    errorRoll_fuzy->addFuzzySet(small);
    FuzzySet *safe = new FuzzySet(10, 20, 20, 30); // new FuzzySet(15, 25, 25, 35) - 15 25 25 35
    errorRoll_fuzy->addFuzzySet(safe);
    FuzzySet *big = new FuzzySet(20, 32.5, 32.5, 45); //new FuzzySet(35, 45, 45, 55) - 20 35 35 50
    errorRoll_fuzy->addFuzzySet(big);
    fuzzyroll->addFuzzyInput(errorRoll_fuzy);

    // Instantiating a FuzzyInput object
    FuzzyInput *errorRollDelta_fuzy = new FuzzyInput(2);
    FuzzySet *smallDelta = new FuzzySet(0.8, 1.5, 2.2, 3.0);
    errorRollDelta_fuzy->addFuzzySet(smallDelta);
    FuzzySet *safeDelta = new FuzzySet(2.0, 3.0, 4.0, 5.0);
    errorRollDelta_fuzy->addFuzzySet(safeDelta);
    FuzzySet *bigDelta = new FuzzySet(3.5, 5.7, 7.8, 10.0);
    errorRollDelta_fuzy->addFuzzySet(bigDelta);
    fuzzyroll->addFuzzyInput(errorRollDelta_fuzy);

    // Instantiating a FuzzyOutput objects
    FuzzyOutput *speed = new FuzzyOutput(1);
    FuzzySet *slow = new FuzzySet(0.0, 0.2, 0.2, 0.4);
    speed->addFuzzySet(slow);
    FuzzySet *average = new FuzzySet(0.4, 0.6, 0.6, 0.8);
    speed->addFuzzySet(average);
    FuzzySet *fast = new FuzzySet(0.6, 0.7, 0.7, 0.9); //new FuzzySet(0.7, 0.9, 0.9, 1.1)
    speed->addFuzzySet(fast);
    fuzzyroll->addFuzzyOutput(speed);

    // Building FuzzyRule "IF error = small THEN speed = slow"
    // Instantiating a FuzzyRuleAntecedent objects
    // error small, delta error small
    FuzzyRuleAntecedent *if_small_smallDelta = new FuzzyRuleAntecedent();
    if_small_smallDelta->joinWithAND(small, smallDelta);
    FuzzyRuleConsequent *thenOP1 = new FuzzyRuleConsequent();
    thenOP1->addOutput(slow);
    FuzzyRule *fuzzyRule01 = new FuzzyRule(1, if_small_smallDelta, thenOP1);
    fuzzyroll->addFuzzyRule(fuzzyRule01);

    // Building FuzzyRule "IF distance = safe THEN speed = average"
    // Instantiating a FuzzyRuleAntecedent objects
    // error avg, delta error small
    FuzzyRuleAntecedent *if_safe_smallDelta = new FuzzyRuleAntecedent();
    if_safe_smallDelta->joinWithAND(safe, safeDelta);
    FuzzyRuleConsequent *thenOP2 = new FuzzyRuleConsequent();
    thenOP2->addOutput(average);
    FuzzyRule *fuzzyRule02 = new FuzzyRule(2, if_safe_smallDelta, thenOP2);
    fuzzyroll->addFuzzyRule(fuzzyRule02);

    // Building FuzzyRule "IF distance = big THEN speed = high"
    // Instantiating a FuzzyRuleAntecedent objects
    // error big, delta error small
    FuzzyRuleAntecedent *if_big_smallDelta = new FuzzyRuleAntecedent();
    if_big_smallDelta->joinWithAND(big, smallDelta);
    FuzzyRuleConsequent *thenOZ1 = new FuzzyRuleConsequent();
    thenOZ1->addOutput(average);
    FuzzyRule *fuzzyRule03 = new FuzzyRule(3, if_big_smallDelta, thenOZ1);
    fuzzyroll->addFuzzyRule(fuzzyRule03);

    // Building FuzzyRule "IF distance = big THEN speed = high"
    // Instantiating a FuzzyRuleAntecedent objects
    FuzzyRuleAntecedent *if_small_safeDelta = new FuzzyRuleAntecedent();
    if_small_safeDelta->joinWithAND(small, safeDelta);
    FuzzyRuleConsequent *thenOZ2 = new FuzzyRuleConsequent();
    thenOZ2->addOutput(average);
    FuzzyRule *fuzzyRule04 = new FuzzyRule(4, if_small_safeDelta, thenOZ2);
    fuzzyroll->addFuzzyRule(fuzzyRule04);

    // Building FuzzyRule "IF distance = big THEN speed = high"
    // Instantiating a FuzzyRuleAntecedent objects
    FuzzyRuleAntecedent *if_safe_safeDelta = new FuzzyRuleAntecedent();
    if_safe_safeDelta->joinWithAND(safe, safeDelta);
    FuzzyRuleConsequent *thenON1 = new FuzzyRuleConsequent();
    thenON1->addOutput(fast);
    FuzzyRule *fuzzyRule05 = new FuzzyRule(5, if_safe_safeDelta, thenON1);
    fuzzyroll->addFuzzyRule(fuzzyRule05);

    // Building FuzzyRule "IF distance = big THEN speed = high"
    // Instantiating a FuzzyRuleAntecedent objects
    FuzzyRuleAntecedent *if_big_safeDelta = new FuzzyRuleAntecedent();
    if_big_safeDelta->joinWithAND(big, safeDelta);
    FuzzyRuleConsequent *thenON2 = new FuzzyRuleConsequent();
    thenON2->addOutput(fast);
    FuzzyRule *fuzzyRule06 = new FuzzyRule(6, if_big_safeDelta, thenON2);
    fuzzyroll->addFuzzyRule(fuzzyRule06);

    // Building FuzzyRule "IF distance = big THEN speed = high"
    // Instantiating a FuzzyRuleAntecedent objects
    FuzzyRuleAntecedent *if_small_bigDelta = new FuzzyRuleAntecedent();
    if_small_bigDelta->joinWithAND(small, bigDelta);
    FuzzyRuleConsequent *thenOZ3 = new FuzzyRuleConsequent();
    thenOZ3->addOutput(fast);
    FuzzyRule *fuzzyRule07 = new FuzzyRule(7, if_small_bigDelta, thenOZ3);
    fuzzyroll->addFuzzyRule(fuzzyRule07);

    // Building FuzzyRule "IF distance = big THEN speed = high"
    // Instantiating a FuzzyRuleAntecedent objects
    FuzzyRuleAntecedent *if_safe_bigDelta = new FuzzyRuleAntecedent();
    if_safe_bigDelta->joinWithAND(safe, bigDelta);
    FuzzyRuleConsequent *thenON3 = new FuzzyRuleConsequent();
    thenON3->addOutput(fast);
    FuzzyRule *fuzzyRule08 = new FuzzyRule(8, if_safe_bigDelta, thenON3);
    fuzzyroll->addFuzzyRule(fuzzyRule08);

    // Building FuzzyRule "IF distance = big THEN speed = high"
    // Instantiating a FuzzyRuleAntecedent objects
    FuzzyRuleAntecedent *if_big_bigDelta = new FuzzyRuleAntecedent();
    if_big_bigDelta->joinWithAND(big, bigDelta);
    FuzzyRuleConsequent *thenON4 = new FuzzyRuleConsequent();
    thenON4->addOutput(fast);
    FuzzyRule *fuzzyRule09 = new FuzzyRule(9, if_big_bigDelta, thenON4);
    fuzzyroll->addFuzzyRule(fuzzyRule09);
}

// void threadFWRoll() {
//     while (true) {
//         test_fuzzy_roll();
//         threads.yield();
//     }
// }