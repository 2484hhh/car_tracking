#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <ArduinoJson.h>

class SerialProtocol {
public:
    void processJsonData();
    void sendHeartbeat();
    
    // 添加缺失的函数声明
    void handleEmergencyStop(JsonDocument& cmd);
    void handleHorn(JsonDocument& cmd); 
    void handleSteer(JsonDocument& cmd);
    void handleSpeed(JsonDocument& cmd);
    
private:
    unsigned long lastHeartbeat = 0; // 添加缺失的变量
};

