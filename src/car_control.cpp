#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <IRremote.h>
#include <TM1650.h>
#include <Adafruit_PWMServoDriver.h>
#include "car_control.h"
#include "serial_protocol.h"

// 定义电机控制引脚 Define motor control pins
#define Motor_L1_F_PIN 11  // 控制小车左前方电机前进 Control the motor on the left front of the car
#define Motor_L1_B_PIN 10  // 控制小车左前方电机后退 Control the motor back on the left front of the car
#define Motor_L2_F_PIN 8   // 控制小车左后方电机前进 Control car left rear motor forward
#define Motor_L2_B_PIN 9   // 控制小车左后方电机后退 Control the car left rear motor back
#define Motor_R1_F_PIN 13  // 控制小车右前方电机前进 Control the right front motor of the car to move forward
#define Motor_R1_B_PIN 12  // 控制小车右前方电机后退 Control the motor back on the right front of the car
#define Motor_R2_F_PIN 14  // 控制小车右后方电机前进 Control car right rear motor forward
#define Motor_R2_B_PIN 15  // 控制小车右后方电机后退 Control car right rear motor back

// 定义底层驱动芯片参数 Bottom-layer driver chip related parameters
#define Bottom_Layer_Driver_ADDR 0x40

// 定义PWM频率 Define PWM frequency
#define PWM_FREQUENCY 50

// 定义按键引脚和控制状态 Define pin and key(button) states
#define KEY_PIN 7
#define Press_KEY 0
#define Release_KEY 1

// 定义三路循迹模块引脚 Define the three-way line patrol module pins
#define L_TRACK_PIN A2
#define M_TRACK_PIN A1
#define R_TRACK_PIN A0

// 定义巡线阈值 Define the patrol threshold
const int Threshold = 500;

// 传感器引脚定义
#define ULTRASONIC_TRIG 3 // 超声波传感器触发引脚
#define ULTRASONIC_ECHO 4 // 超声波传感器回声引脚
#define IR_RECEIVER_PIN 2  // 红外接收器引脚
#define NEOPIXEL_PIN 5     // NeoPixel LED灯带引脚
#define NEOPIXEL_COUNT 8   // NeoPixel LED数量

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

// 全局对象
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
TM1650 display(4); // 4位数码管显示
IRrecv irReceiver(IR_RECEIVER_PIN);
decode_results irResults;

// 创建Adafruit_PWMServoDriver类的实例 Create an instance of the Adafruit_PWMServoDriver class
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(Bottom_Layer_Driver_ADDR);

// 全局变量
int currentSpeed = 0;      // 当前速度值 (-255到255)
int currentDirection = 0;  // 当前方向值 (-100到100，负值左转，正值右转)
bool isEmergencyStopped = false; // 急停状态
bool bCar_Switch = false;  // 小车功能开关
int iCarSpeed = 35;        // 小车巡线速度
unsigned int uTimeOut = 0; // 超时计数

/**
 * @brief 初始化车辆控制系统
 */
void initCarControl() {
  // 初始化I2C通讯
  Wire.begin();
  delay(1000);  // 如果小车功能异常，可以增加这个延时
  
  // 初始化PWM驱动
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQUENCY);
  
  // 初始化按键引脚
  pinMode(KEY_PIN, INPUT_PULLUP);
  
  // 初始化传感器引脚
  pinMode(L_TRACK_PIN, INPUT);
  pinMode(M_TRACK_PIN, INPUT);
  pinMode(R_TRACK_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  
  // 初始化NeoPixel
  pixels.begin();
  pixels.clear();
  pixels.show();
  
  // 初始化TM1650显示屏
  display.init();
  display.displayString("REDY");
  
  // 初始化红外接收器
  irReceiver.enableIRIn();
  
  // 初始状态：停止
  setCarMove(STOP, 0);
}

/**
 * @brief 设置单个电机速度 Setting the Motor Speed
 * @param motor_forward_pin: 控制电机前进引脚 Control the motor forward pin
 * @param motor_backward_pin: 控制电机后退引脚 Control the motor backward pin
 * @param motor_speed: 设置电机速度 Setting the Motor Speed
 * @retval 无 None
 */
void setMotorSpeed(uint16_t motor_forward_pin, uint16_t motor_backward_pin, int motor_speed) {
  motor_speed = map(motor_speed, -255, 255, -4095, 4095);
  if (motor_speed >= 0) {
    pwm.setPWM(motor_forward_pin, 0, motor_speed);
    pwm.setPWM(motor_backward_pin, 0, 0);
  } else if (motor_speed < 0) {
    pwm.setPWM(motor_forward_pin, 0, 0);
    pwm.setPWM(motor_backward_pin, 0, -(motor_speed));
  }
}

/**
 * @brief 设置小车运动方式和速度 Set the car movement mode and speed
 * @param Movement: 小车运动方式 Car movement
 * @param Speed: 小车运动速度 Car speed
 * @retval 无 None
 */
void setCarMove(uint8_t Movement, int Speed) {
  switch (Movement) {
    case STOP:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, 0);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, 0);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, 0);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, 0);
      break;
    case FORWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, Speed);
      break;
    case BACKWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -Speed);
      break;
    case LEFT:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -Speed);
      break;
    case RIGHT:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, Speed);
      break;
    case LEFT_ROTATE:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, Speed);
      break;
    case RIGHT_ROTATE:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -Speed);
      break;
    case LEFT_FORWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, 0);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, 0);
      break;
    case RIGHT_BACKWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, 0);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, 0);
      break;
    case RIGHT_FORWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, 0);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, 0);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, Speed);
      break;
    case LEFT_BACKWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, 0);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, 0);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -Speed);
      break;
    default:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, 0);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, 0);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, 0);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, 0);
      break;
  }
}

/**
 * @brief 获取按键状态 Get key(button) status
 * @param pin: 按键控制引脚 Control key(button) pins
 * @retval 按键状态 Key(button) Status
 */
int getKeyState(uint8_t pin) {
  if (digitalRead(pin) == LOW) {
    delay(20);
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW)
        ;
      return Press_KEY;
    }
    return Release_KEY;
  } else {
    return Release_KEY;
  }
}

/**
 * @brief 小车巡线功能 Car line patrol function
 * @param iLeftPin: 左边循迹传感器 Left tracking sensor
 * @param iMidPin: 中间循迹传感器 Middle tracking sensor
 * @param iRightPin: 右边循迹传感器 Right tracking sensor
 * @retval 无 None
 */
void PatrolCar(int iLeftPin, int iMidPin, int iRightPin) {
  int leftValue = analogRead(iLeftPin);
  int middleValue = analogRead(iMidPin);
  int rightValue = analogRead(iRightPin);

  // 进行简单的方向判断打印 The following is a simple judgment
  if ((leftValue < Threshold && middleValue > Threshold && rightValue < Threshold) | (leftValue > Threshold && middleValue > Threshold && rightValue > Threshold)) {
    setCarMove(FORWARD, iCarSpeed);
  } else if (leftValue > Threshold && middleValue < Threshold && rightValue < Threshold) {
    setCarMove(LEFT_ROTATE, iCarSpeed);
  } else if (leftValue < Threshold && middleValue < Threshold && rightValue > Threshold) {
    setCarMove(RIGHT_ROTATE, iCarSpeed);
  } else if (leftValue < Threshold && middleValue < Threshold && rightValue < Threshold) {
    uTimeOut++;
    delay(20);
    if (uTimeOut > 100) {
      uTimeOut = 0;
      setCarMove(STOP, 0);
    }
  }
}

/**
 * @brief 设置小车功能开关 Set the car function switch
 * @param 无 None
 * @retval 开启/关闭 true/false
 */
bool setCarSwitch() {
  if (getKeyState(KEY_PIN) == Press_KEY) {
    bCar_Switch = !bCar_Switch;
  }
  return bCar_Switch;
}

/**
 * @brief 急停功能
 * @param enable 是否启用急停
 */
void emergencyStop(bool enable) {
  isEmergencyStopped = enable;
  
  if (enable) {
    // 立即停止电机
    setCarMove(STOP, 0);
    
    // 设置红色警示灯
    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    }
    pixels.show();
    
    // 显示STOP
    display.displayString("STOP");
  } else {
    // 恢复正常状态指示灯
    updateStatusLights();
    updateStatusDisplay();
  }
}

/**
 * @brief 测量超声波距离
 * @return 距离，单位厘米
 */
float measureDistance() {
  // 发送10us的高电平脉冲触发测量
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // 测量回波时间
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH);
  
  // 计算距离 (声速340m/s，来回路程，所以除以2)
  float distance = duration * 0.034 / 2;
  
  return distance;
}

/**
 * @brief 更新状态指示灯
 */
void updateStatusLights() {
  // 根据当前速度设置颜色
  uint32_t color;
  
  if (isEmergencyStopped) {
    color = pixels.Color(255, 0, 0); // 红色表示急停
  } else if (currentSpeed == 0) {
    color = pixels.Color(0, 0, 255); // 蓝色表示待机
  } else if (currentSpeed > 0) {
    color = pixels.Color(0, 255, 0); // 绿色表示前进
  } else {
    color = pixels.Color(255, 165, 0); // 橙色表示后退
  }
  
  // 设置所有像素为相同颜色
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i, color);
  }
  
  // 如果转向，则点亮对应方向的LED
  if (currentDirection > 20) { // 右转
    for (int i = 0; i < NEOPIXEL_COUNT/2; i++) {
      pixels.setPixelColor(i + NEOPIXEL_COUNT/2, pixels.Color(255, 255, 0)); // 黄色
    }
  } else if (currentDirection < -20) { // 左转
    for (int i = 0; i < NEOPIXEL_COUNT/2; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 255, 0)); // 黄色
    }
  }
  
  pixels.show();
}

/**
 * @brief 更新状态显示屏
 */
void updateStatusDisplay() {
  char displayStr[5];
  
  if (isEmergencyStopped) {
    strcpy(displayStr, "STOP");
  } else {
    // 显示速度值
    sprintf(displayStr, "%4d", abs(currentSpeed));
  }
  
  display.displayString(displayStr);
}

/**
 * @brief 处理红外遥控器输入
 */
/**
 * @brief 驱动电机函数，将速度和方向转换为小车运动
 * @param speed 速度值 (-255到255)
 * @param direction 方向值 (-100到100，负值左转，正值右转)
 */
void driveMotors(int speed, int direction) {
  currentSpeed = speed;
  currentDirection = direction;
  
  if (speed == 0) {
    setCarMove(STOP, 0);
    return;
  }
  
  if (direction > 30) {
    // 右转
    if (speed > 0) {
      setCarMove(RIGHT, abs(speed));
    } else {
      setCarMove(RIGHT_BACKWARD, abs(speed));
    }
  } else if (direction < -30) {
    // 左转
    if (speed > 0) {
      setCarMove(LEFT, abs(speed));
    } else {
      setCarMove(LEFT_BACKWARD, abs(speed));
    }
  } else {
    // 直行
    if (speed > 0) {
      setCarMove(FORWARD, speed);
    } else {
      setCarMove(BACKWARD, abs(speed));
    }
  }
}

void processIRRemote() {
  if (irReceiver.decode()) {
    // 使用新版本API获取解码结果  
    uint32_t irCode = irReceiver.decodedIRData.decodedRawData;
    
    // 根据接收到的IR代码执行相应操作
    switch (irCode) {
      case 0xFF629D: // 前进 (上键)
        driveMotors(150, 0);
        break;
      case 0xFF22DD: // 左转 (左键)
        driveMotors(currentSpeed, -50);
        break;
      case 0xFF02FD: // 停止 (OK键)
        driveMotors(0, 0);
        break;
      case 0xFFC23D: // 右转 (右键)
        driveMotors(currentSpeed, 50);
        break;
      case 0xFFA857: // 后退 (下键)
        driveMotors(-150, 0);
        break;
      case 0xFF6897: // 急停 (1键)
        emergencyStop(true);
        break;
      case 0xFF9867: // 解除急停 (2键)
        emergencyStop(false);
        break;
    }
    
    irReceiver.resume(); // 准备接收下一个值
  }
}

/**
 * @brief 鸣笛功能
 * @param duration 鸣笛持续时间(毫秒)
 */
void honkHorn(int duration) {
  // 这里可以添加蜂鸣器控制代码
  // 如果没有蜂鸣器，可以用LED闪烁代替
  
  // 闪烁LED作为鸣笛指示
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i, pixels.Color(255, 255, 255)); // 白色
  }
  pixels.show();
  
  delay(duration);
  
  // 恢复之前的状态
  updateStatusLights();
}

/**
 * @brief 更新车辆状态
 * 应在主循环中定期调用
 */
void updateCarStatus() {
  // 处理红外遥控器输入
  processIRRemote();
  
  // 如果处于急停状态，不执行其他操作
  if (isEmergencyStopped) {
    return;
  }
  
  // 按键控制小车循迹功能启停
  if (setCarSwitch()) {
    PatrolCar(L_TRACK_PIN, M_TRACK_PIN, R_TRACK_PIN);
  } else {
    // 如果巡线功能未启用，则检测障碍物并自动减速或停止
    float distance = measureDistance();
    if (distance < 20 && currentSpeed > 0) {
      if (distance < 10) {
        // 距离小于10cm，停止
        setCarMove(STOP, 0);
      } else {
        // 距离在10-20cm之间，减速
        int reducedSpeed = currentSpeed / 2;
        // 保持当前方向，减小速度
        if (currentDirection > 20) {
          setCarMove(RIGHT, reducedSpeed);
        } else if (currentDirection < -20) {
          setCarMove(LEFT, reducedSpeed);
        } else {
          setCarMove(FORWARD, reducedSpeed);
        }
      }
    }
  }
  
  // 更新状态指示灯
  updateStatusLights();
  
  // 更新状态显示屏
  updateStatusDisplay();
}
