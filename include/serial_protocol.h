/**
 * @file serial_protocol.h
 * @brief 串口通信协议处理类
 * 
 * 负责处理与树莓派的串口通信协议，包括命令解析和心跳发送
 */

#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <ArduinoJson.h>

class SerialProtocol {
public:
    /**
     * @brief 初始化串口通信
     */
    void begin();
    
    /**
     * @brief 处理接收到的JSON数据
     */
    void processJsonData();
    
    /**
     * @brief 发送心跳包 
     */
    void sendHeartbeat();

private:
    // 具体命令处理函数
    void handleEmergencyStop(JsonDocument& cmd);
    void handleHorn(JsonDocument& cmd); 
    void handleSteer(JsonDocument& cmd);
    void handleSpeed(JsonDocument& cmd);
    
    unsigned long lastHeartbeat = 0; ///< 上次发送心跳的时间戳
};
#endif
