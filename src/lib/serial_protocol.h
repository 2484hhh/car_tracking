#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <ArduinoJson.h>

class SerialProtocol {
public:
    void begin();
    void processJsonData();
    void sendHeartbeat();
    void handleEmergencyStop(JsonDocument& cmd);
    void handleHorn(JsonDocument& cmd);
    void handleSteer(JsonDocument& cmd);
    void handleSpeed(JsonDocument& cmd);
    void sendSensorData(float distance, int leftTrack, int midTrack, int rightTrack);
    void sendStatusFeedback(const char* status, const char* message);
    
private:
    unsigned long lastHeartbeat = 0;
};

#endif
