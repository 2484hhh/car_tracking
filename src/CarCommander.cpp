#include "CarCommander.h"
#include <Arduino.h>

CarCommander::CarCommander(MotorController& motor, 
                         LEDController& led,
                         BuzzerController& buzzer,
                         uint32_t baudrate)
    : motor(motor), led(led), buzzer(buzzer), baudrate(baudrate) {}

void CarCommander::begin() {
    Serial.begin(baudrate);
}

bool CarCommander::processCommand(const String& input) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, input);

    if (error) {
        Serial.println("JSON解析错误");
        return false;
    }

    const char* cmd = doc["cmd"];
    JsonObject data = doc["data"];

    if (strcmp(cmd, "LINE") == 0) {
        processLineCommand(data);
    } 
    else if (strcmp(cmd, "DETECT") == 0) {
        processDetectCommand(data);
    }
    else {
        Serial.println("未知指令");
        return false;
    }

    Serial.println("ACK");
    return true;
}

void CarCommander::processLineCommand(JsonObject& data) {
    const char* dir = data["dir"];
    int speed = 35; // 默认速度

    if (strcmp(dir, "F") == 0) {
        motor.move(MotorController::FORWARD, speed, 0);
        led.set(LEDController::GREEN);
    } 
    else if (strcmp(dir, "L") == 0) {
        motor.move(MotorController::LEFT_ROTATE, speed);
        led.set(LEDController::YELLOW);
    }
    else if (strcmp(dir, "R") == 0) {
        motor.move(MotorController::RIGHT_ROTATE, speed);
        led.set(LEDController::YELLOW);
    }
    else if (strcmp(dir, "S") == 0) {
        motor.stop();
        led.set(LEDController::OFF);  // 停止时关闭LED
        Serial.println("Motor stopped, LED off");
    }
    else if (strstr(dir, "T") != nullptr) {
        int angle = atoi(dir+1); // 提取角度值
        Serial.print("Turning angle: ");
        Serial.println(angle);
        motor.turn(angle, speed);
    }
}

void CarCommander::processDetectCommand(JsonObject& data) {
    const char* action = data["action"];

    if (strcmp(action, "STOP") == 0) {
        motor.stop();
        led.set(LEDController::RED);
        buzzer.beep(500);
    }
    else if (strcmp(action, "HORN") == 0) {
        for (int i = 0; i < 3; i++) {
            led.set(LEDController::GREEN);
            buzzer.beep(100);
            led.set(LEDController::RED);
            delay(100);
        }
    }
}
