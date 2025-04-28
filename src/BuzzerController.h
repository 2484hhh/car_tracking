#ifndef BUZZER_CONTROLLER_H
#define BUZZER_CONTROLLER_H

#include <stdint.h>

class BuzzerController {
public:
    BuzzerController(uint8_t pin = 10);  // 修改默认引脚为10
    void begin();
    void beep(uint16_t duration = 200);
    void startBeep();
    void stopBeep();

private:
    uint8_t pin;
};

#endif
