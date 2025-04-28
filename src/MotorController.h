#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Adafruit_PWMServoDriver.h>

class MotorController {
public:
    enum Movement {
        STOP,
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        LEFT_ROTATE,
        RIGHT_ROTATE,
        TURN_ANGLE
    };

    MotorController(uint8_t addr = 0x40, uint16_t freq = 500); // 降低PWM频率
    void begin();
    void move(Movement dir, int speed, int bias = 0);
    void turn(int angle, int speed = 35); // 新增角度控制
    void stop();

private:
    void setMotorSpeed(uint16_t forwardPin, uint16_t backwardPin, int speed);

    Adafruit_PWMServoDriver pwm;
    uint16_t frequency;
    const uint8_t motorPins[4][2] = {
        {11, 10}, // L1
        {8, 9},   // L2
        {13, 12}, // R1
        {14, 15}  // R2
    };
};

#endif
