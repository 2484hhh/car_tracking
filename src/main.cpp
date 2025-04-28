#include <Arduino.h>
#include "MotorController.h"
#include "LEDController.h"
#include "BuzzerController.h"
#include "CarCommander.h"

// 实例化控制器
MotorController motorController;
LEDController ledController;
BuzzerController buzzerController;

// 创建指令处理器(波特率115200)
CarCommander carCommander(motorController, ledController, buzzerController, 115200);

void setup() {
    // 初始化各控制器
    motorController.begin();
    ledController.begin();
    buzzerController.begin();
    carCommander.begin();
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        carCommander.processCommand(input);
    }
}
