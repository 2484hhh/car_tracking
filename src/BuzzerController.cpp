#include "BuzzerController.h"
#include <Arduino.h>

BuzzerController::BuzzerController(uint8_t pin) 
    : pin(pin) {}

void BuzzerController::begin() {
    pinMode(pin, OUTPUT);
    stopBeep();
}

void BuzzerController::beep(uint16_t duration) {
    startBeep();
    delay(duration);
    stopBeep();
}

void BuzzerController::startBeep() {
    digitalWrite(pin, HIGH);
}

void BuzzerController::stopBeep() {
    digitalWrite(pin, LOW);
}
