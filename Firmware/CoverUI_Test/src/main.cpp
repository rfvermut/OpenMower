#include <Arduino.h>
#include <Automaton.h>
#include <PacketSerial.h>
#include <FastCRC.h>
#include "shiftregister/ShiftRegister.h"
#include "DummyOut.h"
#include "pinDefinitions.h"
#include "Atm_button_matrix.h"
#include "Atm_virtual_led.hpp"
#include <bitset>
#include "BttnCtl.h"

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

// Maps led index to its actual position in chain of shift registers. First bit is furthest in chain
// i.e. Led D0102 is second. It's index is 1, counting from 0
// It mapped to 12th in bit chain from the end (counting from 0)
constexpr byte LED_INDEX_TO_BIT_POSITION[] = {1, 12, 11, 10, 0, 13, 14, 5, 4,
                                              3, 2, 15, 16, 17, 6, 7, 8, 9};

#define FIRMWARE_VERSION 200
FastCRC16 CRC16;
PacketSerial_<COBS, 0, 1024> myPacketSerial;

void virtualLedCallback(int idx, int isPressed, int _ignored) {
    LEDS.set(LED_INDEX_TO_BIT_POSITION[idx], isPressed);
    shiftRegister.write(LEDS.to_ulong(), LEDS.size());

#ifdef DEBUG
    Serial.print("LED ");
    Serial.print(idx);
    Serial.print("\t");
    Serial.println(LEDS.to_ulong(), BIN);
#endif
};

template<size_t bitsetsize>
void parseLedsBinaryStatus(std::bitset<bitsetsize> &ledsMessage) {

}


/****************************************************************************************************
 * sendMessage send the command structure via COBS and serial Port
 *****************************************************************************************************/
void sendMessage(uint8_t *message, size_t size) {
    // packages need to be at least 1 byte of type, 1 byte of data and 2 bytes of CRC
    if (size < 4) {
        return;
    }

    // calculate the CRC
    auto *crc_pointer = (uint16_t *) (message + (size - 2));
    *crc_pointer = CRC16.ccitt(message, size - 2);

#ifdef DEBUG
    // structure is filled and CRC calculated, so print out, what should be encoded
    printf("\nprint struct before encoding %d byte : ", (int) size);
    uint8_t *temp = message;
    for (int i = 0; i < size; i++) {
        printf("0x%02x , ", *temp);
        temp++;
    }
#endif

    myPacketSerial.send(message, size);
}

// When an encoded packet is received and decoded, it will be delivered here.
// The `decoded_buffer` is a pointer to the decoded byte array. `data_size` is the number of
// bytes in the `buffer`.
void onPacketReceived(const uint8_t *decoded_buffer, size_t data_size) {
    // calculate the CRC only if we have at least three bytes (two CRC, one data)
    if (data_size < 3)
        return;

    uint16_t calc_crc = CRC16.ccitt(decoded_buffer, data_size - 2);

    if (decoded_buffer[0] == Get_Version && data_size == sizeof(struct msg_get_version)) {
        auto message = (struct msg_get_version *) decoded_buffer;
        if (message->crc == calc_crc) {
            // valid get_version request, send reply
            struct msg_get_version reply{};
            reply.type = Get_Version;
            reply.version = FIRMWARE_VERSION;
            sendMessage((uint8_t *) &reply, sizeof(reply));
        }
    } else if (decoded_buffer[0] == Set_Buzzer && data_size == sizeof(struct msg_set_buzzer)) {
        auto message = (struct msg_set_buzzer *) decoded_buffer;
        if (message->crc == calc_crc) {
            // valid set_buzzer request
//            Buzzer_set(message->repeat, message->on_time, message->off_time);
        }
    } else if (decoded_buffer[0] == Set_LEDs && data_size == sizeof(struct msg_set_leds)) {
        auto message = (struct msg_set_leds *) decoded_buffer;
        if (message->crc == calc_crc) {
            // valid set_leds request
            printf("Got valid setled call\n");
//            LED_activity = message->leds;
//            LEDs_refresh(pio_Block1, sm_LEDmux);
        } else {
            printf("Got setled call with crc error\n");
        }
    } else {
        printf("some invalid packet\n");
    }
}

void setup() {
    LEDS.reset();
    shiftRegister.write(LEDS.to_ulong(), LEDS.size());

    // Setting huge debounce to mitigate potential FIFO spamming with button presses
    SW[0].begin(BM_OUT_SA1, BM_IN_SE1).debounce(500);
    SW[1].begin(BM_OUT_SA1, BM_IN_SE2).debounce(500);
    SW[2].begin(BM_OUT_SA1, BM_IN_SE3).debounce(500);
    SW[3].begin(BM_OUT_SA1, BM_IN_SE4).debounce(500);
    SW[4].begin(BM_OUT_SA2, BM_IN_SE1).debounce(500);
    SW[5].begin(BM_OUT_SA2, BM_IN_SE2).debounce(500);
    SW[6].begin(BM_OUT_SA2, BM_IN_SE3).debounce(500);
    SW[7].begin(BM_OUT_SA2, BM_IN_SE4).debounce(500);
    SW[8].begin(BM_OUT_SA3, BM_IN_SE1).debounce(500);
    SW[9].begin(BM_OUT_SA3, BM_IN_SE2).debounce(500);
    SW[10].begin(BM_OUT_SA3, BM_IN_SE3).debounce(500);
    SW[11].begin(BM_OUT_SA3, BM_IN_SE4).debounce(500);
    SW[12].begin(BM_OUT_SA4, BM_IN_SE1).debounce(500);
    SW[13].begin(BM_OUT_SA4, BM_IN_SE2).debounce(500);

    builtin_led.begin(LED_BUILTIN);
    for (int i = 0; i < 18; ++i) {
        leds[i].begin();

        // Virtual led on/off codes does nothing, instead we update shift register bytes in callback
        leds[i].onEventOn(virtualLedCallback, i);
        leds[i].onEventOff(virtualLedCallback, i);
    }

    myPacketSerial.begin(115200);
    myPacketSerial.setPacketHandler(&onPacketReceived);

#ifdef TEST_MODE
    for (int i = 0; i < 18; ++i) {
        leds[i].begin().blink(50 * (i + 1), 50 * (i + 1)).start();
    };
    for (int i = 0; i < 13; ++i) {
        // Switches SW1-SW13 toggle corresponding leds
        SW[i].onPress([](int idx, int v, int up) {
            Serial.print("BTN ");
            Serial.println(idx);
            builtin_led.toggle();
            leds[idx].toggle();
        }, i).trace(Serial);
        // Switch SW14 operates the rest of leds
        SW[13].onPress([](int idx, int v, int up) {
            Serial.print("BTN ");
            Serial.println(idx);
            builtin_led.toggle();
            leds[13].toggle();
            leds[14].toggle();
            leds[15].toggle();
            leds[16].toggle();
            leds[17].toggle();
        }, i).trace(Serial);
    }
#endif

}

void loop() {
    automaton.run();
    myPacketSerial.update();

    // Check for a receive buffer overflow (optional).
    if (myPacketSerial.overflow()) {
        // Send an alert via a pin (e.g. make an overflow LED) or return a
        // user-defined packet to the sender.
    }
}