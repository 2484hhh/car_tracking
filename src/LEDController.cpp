#include "LEDController.h"
#include <Arduino.h>

LEDController::LEDController(uint8_t rgbPin, uint8_t numLeds) 
    : strip(numLeds, rgbPin, NEO_GRB + NEO_KHZ800) {}

void LEDController::begin() {
    strip.begin();
    strip.show(); // Initialize all pixels to off
}

uint32_t LEDController::getColorValue(Color color) {
    switch(color) {
        case RED: return strip.Color(255, 0, 0);
        case GREEN: return strip.Color(0, 255, 0);
        case YELLOW: return strip.Color(255, 255, 0);
        default: return strip.Color(0, 0, 0); // OFF
    }
}

void LEDController::set(Color color) {
    strip.setPixelColor(0, getColorValue(color));
    strip.show();
}

void LEDController::blink(Color color, uint16_t interval, uint8_t times) {
    for(uint8_t i = 0; i < times; i++) {
        set(color);
        delay(interval);
        set(OFF);
        if(i < times - 1) {
            delay(interval);
        }
    }
}
