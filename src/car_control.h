#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <Arduino.h>

// 添加前向声明
void setCarMove(uint8_t Movement, int Speed);
void updateStatusLights();
void updateStatusDisplay();

// 枚举全向小车的常见运动方式 Enumerate the common movement modes of omnidirectional cars
enum OmniDirectionalCar {
  STOP,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  LEFT_ROTATE,
  RIGHT_ROTATE,
  LEFT_FORWARD,
  RIGHT_BACKWARD,
  RIGHT_FORWARD,
  LEFT_BACKWARD,
};

// 函数声明
void initCarControl();
void setMotorSpeed(uint16_t motor_forward_pin, uint16_t motor_backward_pin, int motor_speed);
void setCarMove(uint8_t Movement, int Speed);
int getKeyState(uint8_t pin);
void PatrolCar(int iLeftPin, int iMidPin, int iRightPin);
bool setCarSwitch();
void emergencyStop(bool enable);
float measureDistance();
void updateStatusLights();
void updateStatusDisplay();
void driveMotors(int speed, int direction);
void processIRRemote();
void honkHorn(int duration);
void updateCarStatus();

#endif // CAR_CONTROL_H
