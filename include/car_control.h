/**
 * @file car_control.h
 * @brief 小车控制功能模块
 * 
 * 包含电机控制、传感器处理、LED显示和红外遥控等功能
 */

#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <Arduino.h>

// 定义三路循迹模块引脚 Define the three-way line patrol module pins
#define L_TRACK_PIN A2
#define M_TRACK_PIN A1
#define R_TRACK_PIN A0

// 定义按键状态 Define key(button) states
#define Press_KEY 0
#define Release_KEY 1

/**
 * @brief 初始化车辆控制系统
 */
void initCarControl();

// 枚举全向小车的常见运动方式
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

/**
 * @brief 设置单个电机速度 Setting the Motor Speed
 * @param motor_forward_pin: 控制电机前进引脚 Control the motor forward pin
 * @param motor_backward_pin: 控制电机后退引脚 Control the motor backward pin
 * @param motor_speed: 设置电机速度 Setting the Motor Speed
 * @retval 无 None
 */
void setMotorSpeed(uint16_t motor_forward_pin, uint16_t motor_backward_pin, int motor_speed);

/**
 * @brief 设置小车运动方式和速度 Set the car movement mode and speed
 * @param Movement: 小车运动方式 Car movement
 * @param Speed: 小车运动速度 Car speed
 * @retval 无 None
 */
void setCarMove(uint8_t Movement, int Speed);

/**
 * @brief 获取按键状态 Get key(button) status
 * @param pin: 按键控制引脚 Control key(button) pins
 * @retval 按键状态 Key(button) Status
 */
int getKeyState(uint8_t pin);

/**
 * @brief 小车巡线功能 Car line patrol function
 * @param iLeftPin: 左边循迹传感器 Left tracking sensor
 * @param iMidPin: 中间循迹传感器 Middle tracking sensor
 * @param iRightPin: 右边循迹传感器 Right tracking sensor
 * @retval 无 None
 */
void PatrolCar(int iLeftPin, int iMidPin, int iRightPin);

/**
 * @brief 设置小车功能开关 Set the car function switch
 * @param 无 None
 * @retval 开启/关闭 true/false
 */
bool setCarSwitch();

/**
 * @brief 驱动电机函数，将速度和方向转换为小车运动
 * @param speed 速度值 (-255到255)
 * @param direction 方向值 (-100到100，负值左转，正值右转)
 */
void driveMotors(int speed, int direction);

/**
 * @brief 急停功能
 * @param enable 是否启用急停
 */
void emergencyStop(bool enable);

/**
 * @brief 测量超声波距离
 * @return 距离，单位厘米
 */
float measureDistance();

/**
 * @brief 更新状态指示灯
 */
void updateStatusLights();

/**
 * @brief 更新状态显示屏
 */
void updateStatusDisplay();

/**
 * @brief 处理红外遥控器输入
 */
void processIRRemote();

/**
 * @brief 鸣笛功能
 * @param duration 鸣笛持续时间(毫秒)
 */
void honkHorn(int duration);

/**
 * @brief 更新车辆状态
 * 应在主循环中定期调用
 */
void updateCarStatus();

#endif // CAR_CONTROL_H