#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <ArduinoJson.h>

class SerialProtocol {
public:
  SerialProtocol();
  void begin();
  void processJsonData();
  void sendHeartbeat();
  void sendSensorData(float distance, int leftTrack, int middleTrack, int rightTrack);
  void sendStatusFeedback(const char* cmd, const char* info);
  void handleLine(const JsonDocument& doc);
  void handleDetect(const JsonDocument& doc);
  void handleMove(const JsonDocument& doc);
  void handleStop(const JsonDocument& doc);
  void handleResume(const JsonDocument& doc);
  void handleHonk(const JsonDocument& doc);
private:
  unsigned long lastHeartbeatTime;
  const unsigned long heartbeatInterval = 1000; // 1秒发送一次心跳
  
  void parseCommand(const JsonDocument& doc);
};

#endif // SERIAL_PROTOCOL_H