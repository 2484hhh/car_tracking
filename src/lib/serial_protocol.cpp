#include "serial_protocol.h"
#include <ArduinoJson.h>
#include "car_control.h"

// 引用car_control.cpp中定义的全局变量
extern int currentSpeed;
extern int currentDirection;

void SerialProtocol::begin() {
    Serial.begin(115200);
}

void SerialProtocol::processJsonData() {
    if (Serial.available()) {
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, Serial);
        
        if (!error) {
            // 处理命令类型消息
            const char* cmd = doc["command"];
            if (cmd) {
                if (strcmp(cmd, "emergency_stop") == 0) {
                    handleEmergencyStop(doc);
                } else if (strcmp(cmd, "horn") == 0) {
                    handleHorn(doc);
                } else if (strcmp(cmd, "steer") == 0) {
                    handleSteer(doc);
                } else if (strcmp(cmd, "speed") == 0) {
                    handleSpeed(doc);
                }
                return;
            }
            
            // 处理心跳包
            const char* type = doc["type"];
            if (type && strcmp(type, "heartbeat") == 0) {
                // 收到树莓派的心跳包，可以更新连接状态
                // 这里可以添加额外的处理逻辑，如更新最后一次通信时间
                lastHeartbeat = millis(); // 更新最后通信时间
            }
        }
    }
}

void SerialProtocol::sendHeartbeat() {
    unsigned long now = millis();
    if (now - lastHeartbeat >= 5000) {
        StaticJsonDocument<100> doc;
        doc["type"] = "heartbeat";
        doc["time"] = now;
        serializeJson(doc, Serial);
        Serial.println(); // 添加换行符以便树莓派正确解析
        lastHeartbeat = now;
    }
}

void SerialProtocol::handleEmergencyStop(JsonDocument& cmd) {
    // 急停处理逻辑
    bool enable = cmd["enable"];
    emergencyStop(enable);
    
    // 发送响应
    StaticJsonDocument<100> response;
    response["type"] = "response";
    response["command"] = "emergency_stop";
    response["status"] = "ok";
    serializeJson(response, Serial);
    Serial.println();
}

void SerialProtocol::handleHorn(JsonDocument& cmd) {
    // 鸣笛处理逻辑
    int duration = cmd["duration"] | 500; // 默认500ms
    honkHorn(duration);
    
    // 发送响应
    StaticJsonDocument<100> response;
    response["type"] = "response";
    response["command"] = "horn";
    response["status"] = "ok";
    serializeJson(response, Serial);
    Serial.println();
}

void SerialProtocol::handleSteer(JsonDocument& cmd) {
    // 转向处理逻辑
    int direction = cmd["direction"] | 0; // -100到100，负值左转，正值右转
    
    // 保持当前速度，改变方向
    driveMotors(currentSpeed, direction);
    
    // 发送响应
    StaticJsonDocument<100> response;
    response["type"] = "response";
    response["command"] = "steer";
    response["status"] = "ok";
    response["direction"] = direction;
    serializeJson(response, Serial);
    Serial.println();
}

void SerialProtocol::handleSpeed(JsonDocument& cmd) {
    // 速度控制处理逻辑
    int speed = cmd["speed"] | 0; // -255到255
    
    // 保持当前方向，改变速度
    driveMotors(speed, currentDirection);
    
    // 发送响应
    StaticJsonDocument<100> response;
    response["type"] = "response";
    response["command"] = "speed";
    response["status"] = "ok";
    response["speed"] = speed;
    serializeJson(response, Serial);
    Serial.println();
}
