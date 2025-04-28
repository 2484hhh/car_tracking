#ifndef CAR_COMMANDER_H
#define CAR_COMMANDER_H

#include <ArduinoJson.h>
#include "MotorController.h"
#include "LEDController.h"
#include "BuzzerController.h"

class CarCommander {
public:
    CarCommander(MotorController& motor, 
                LEDController& led,
                BuzzerController& buzzer,
                uint32_t baudrate = 115200);
    
    void begin();
    bool processCommand(const String& input);

private:
    void processLineCommand(JsonObject& data);
    void processDetectCommand(JsonObject& data);

    MotorController& motor;
    LEDController& led;
    BuzzerController& buzzer;
    uint32_t baudrate;
};

#endif
