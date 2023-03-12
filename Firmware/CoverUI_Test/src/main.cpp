#include <Arduino.h>
#include <Automaton.h>
#include "shiftregister/ShiftRegister.h"
#include "DummyOut.h"
#include "pinDefinitions.h"
#include "Atm_button_matrix.h"
#include "Atm_virtual_led.hpp"
#include <bitset>

#define BM_OUT_SA1 p15
#define BM_OUT_SA2 p16
#define BM_OUT_SA3 p17
#define BM_OUT_SA4 p18

#define BM_IN_SE1 p19
#define BM_IN_SE2 p20
#define BM_IN_SE3 p21
#define BM_IN_SE4 p22

#define UNUSED_PIN p14

mbed::DigitalOut PIN_RCK = mbed::DigitalOut(p11);
mbed::DigitalOut PIN_SER = mbed::DigitalOut(p12);
mbed::DigitalOut PIN_SRCK = mbed::DigitalOut(p10);
mbed::DigitalOut PIN_G = mbed::DigitalOut(p13);
mbed::DigitalOut no_pin = DummyOut(UNUSED_PIN);

ShiftRegister shiftRegister = ShiftRegister(
        PIN_SRCK,
        PIN_SER,
        PIN_RCK,
        no_pin,
        PIN_G
);
std::bitset<18> LEDS;

Atm_led builtin_led;
Atm_button_matrix SW[14];
Atm_virtual_led leds[18];


void setup() {
    LEDS.reset();
    shiftRegister.write(LEDS.to_ulong(), LEDS.size());
    SW[0].begin(BM_OUT_SA1, BM_IN_SE1);
    SW[1].begin(BM_OUT_SA1, BM_IN_SE2);
    SW[2].begin(BM_OUT_SA1, BM_IN_SE3);
    SW[3].begin(BM_OUT_SA1, BM_IN_SE4);
    SW[4].begin(BM_OUT_SA2, BM_IN_SE1);
    SW[5].begin(BM_OUT_SA2, BM_IN_SE2);
    SW[6].begin(BM_OUT_SA2, BM_IN_SE3);
    SW[7].begin(BM_OUT_SA2, BM_IN_SE4);
    SW[8].begin(BM_OUT_SA3, BM_IN_SE1);
    SW[9].begin(BM_OUT_SA3, BM_IN_SE2);
    SW[10].begin(BM_OUT_SA3, BM_IN_SE3);
    SW[11].begin(BM_OUT_SA3, BM_IN_SE4);
    SW[12].begin(BM_OUT_SA4, BM_IN_SE1);
    SW[13].begin(BM_OUT_SA4, BM_IN_SE2);

    builtin_led.begin(LED_BUILTIN);
    for (int i = 0; i < 18; ++i) {
        leds[i].begin();
        leds[i].onEventOn([](int idx, int v, int up) {
            Serial.println(idx);
            LEDS.set(18 - idx, true);
            shiftRegister.write(LEDS.to_ulong(), LEDS.size());
            Serial.println(LEDS.to_ulong());
        }, i);
        leds[i].onEventOff([](int idx, int v, int up) {
            Serial.println(idx);
            LEDS.set(18 - idx, false);
            shiftRegister.write(LEDS.to_ulong(), LEDS.size());
            Serial.println(LEDS.to_ulong());
        }, i);
    }

    for (int i = 0; i < 14; ++i) {
        SW[i].onPress([](int idx, int v, int up) {
            builtin_led.toggle();
            leds[idx].toggle();
        }, i);
    }
}

void loop() {
    automaton.run();
}
