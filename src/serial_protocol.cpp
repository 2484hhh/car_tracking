#include "serial_protocol.h"
#include "car_control.h"

SerialProtocol::SerialProtocol() {
  lastHeartbeatTime = 0;
}

void SerialProtocol::begin() {
  Serial.println("Serial protocol initialized");
}

void SerialProtocol::processJsonData() {
  if (Serial.available()) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, Serial);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }
    const char* cmd = doc["command"];
    if (!cmd) {
      Serial.println("No command field in JSON");
      return;
    }
    if (strcmp(cmd, "LINE") == 0) {
      handleLine(doc);
    } else if (strcmp(cmd, "DETECT") == 0) {
      handleDetect(doc);
    } else if (strcmp(cmd, "move") == 0) {
      handleMove(doc);
    } else if (strcmp(cmd, "stop") == 0) {
      handleStop(doc);
    } else if (strcmp(cmd, "resume") == 0) {
      handleResume(doc);
    } else if (strcmp(cmd, "honk") == 0) {
      handleHonk(doc);
    }
  }
}

void SerialProtocol::sendHeartbeat() {
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeatTime >= heartbeatInterval) {
    lastHeartbeatTime = currentTime;
    StaticJsonDocument<128> doc;
    doc["type"] = "heartbeat";
    doc["time"] = currentTime;
    serializeJson(doc, Serial);
    Serial.println();
  }
}

void SerialProtocol::sendSensorData(float distance, int leftTrack, int middleTrack, int rightTrack) {
  StaticJsonDocument<256> doc;
  doc["type"] = "sensor_data";
  doc["distance"] = distance;
  doc["track"]["left"] = leftTrack;
  doc["track"]["middle"] = middleTrack;
  doc["track"]["right"] = rightTrack;
  serializeJson(doc, Serial);
  Serial.println();
}

void SerialProtocol::sendStatusFeedback(const char* cmd, const char* info) {
  StaticJsonDocument<128> doc;
  doc["type"] = "feedback";
  doc["command"] = cmd;
  doc["info"] = info;
  serializeJson(doc, Serial);
  Serial.println();
}

void SerialProtocol::handleLine(const JsonDocument& doc) {
  const char* dir = doc["dir"];
  if (dir) {
    if (strcmp(dir, "L") == 0) {
      setCarMove(LEFT, 35);
    } else if (strcmp(dir, "R") == 0) {
      setCarMove(RIGHT, 35);
    } else if (strcmp(dir, "F") == 0) {
      setCarMove(FORWARD, 35);
    } else if (strcmp(dir, "S") == 0) {
      setCarMove(STOP, 0);
    }
    sendStatusFeedback("LINE", dir);
  }
}

void SerialProtocol::handleDetect(const JsonDocument& doc) {
  const char* action = doc["action"];
  if (action) {
    if (strcmp(action, "STOP") == 0) {
      emergencyStop(true);
      sendStatusFeedback("DETECT", "STOP");
    } else if (strcmp(action, "HORN") == 0) {
      honkHorn(500);
      sendStatusFeedback("DETECT", "HORN");
    }
  }
}

void SerialProtocol::handleMove(const JsonDocument& doc) {
  int speed = doc["speed"];
  int direction = doc["direction"];
  driveMotors(speed, direction);
  sendStatusFeedback("move", "");
}

void SerialProtocol::handleStop(const JsonDocument& doc) {
  emergencyStop(true);
  sendStatusFeedback("stop", "");
}

void SerialProtocol::handleResume(const JsonDocument& doc) {
  emergencyStop(false);
  sendStatusFeedback("resume", "");
}

void SerialProtocol::handleHonk(const JsonDocument& doc) {
  int duration = doc["duration"];
  honkHorn(duration);
  sendStatusFeedback("honk", "");
}