#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <stdint.h>
#include <Adafruit_NeoPixel.h>

class LEDController {
public:
    enum Color {
        OFF,
        RED,
        GREEN,
        YELLOW
    };

    LEDController(uint8_t rgbPin = 6, uint8_t numLeds = 1);
    void begin();
    void set(Color color);
    void blink(Color color, uint16_t interval = 500, uint8_t times = 1);

private:
    Adafruit_NeoPixel strip;
    uint32_t getColorValue(Color color);
};

#endif
