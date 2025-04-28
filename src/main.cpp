#include <Arduino.h>
#include <ArduinoJson.h>
#include "serial_protocol.h"
#include "car_control.h"

SerialProtocol serialProtocol;

void setup() {
  Serial.begin(115200);
  serialProtocol.begin();
  
  // 初始化车辆控制系统
  initCarControl();
  
  Serial.println("System initialized, waiting for commands...");
}

void updateCarStatus() {
  // 处理红外遥控器输入
  processIRRemote();
  
  // 更新状态指示灯
  updateStatusLights();
  
  // 更新状态显示屏
  updateStatusDisplay();
}

void loop() {
  // 处理接收到的JSON命令
  serialProtocol.processJsonData();
  
  // 更新车辆状态（处理传感器和红外遥控器输入）
  updateCarStatus();
  
  // 发送心跳包
  serialProtocol.sendHeartbeat();
  
  delay(10);
}
