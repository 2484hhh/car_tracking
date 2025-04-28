#include "MotorController.h"

MotorController::MotorController(uint8_t addr, uint16_t freq) 
    : pwm(addr), frequency(freq) {}

void MotorController::begin() {
    pwm.begin();
    pwm.setPWMFreq(frequency);
    stop();
}

void MotorController::move(Movement dir, int speed, int bias) {
    // bias: 路线偏差值(-100~100)
    switch(dir) {
        case FORWARD:
            for(int i = 0; i < 4; i++) {
                setMotorSpeed(motorPins[i][0], motorPins[i][1], speed);
            }
            break;
        case BACKWARD:
            for(int i = 0; i < 4; i++) {
                setMotorSpeed(motorPins[i][0], motorPins[i][1], -speed);
            }
            break;
        case LEFT:
            setMotorSpeed(motorPins[0][0], motorPins[0][1], -speed);
            setMotorSpeed(motorPins[1][0], motorPins[1][1], speed);
            setMotorSpeed(motorPins[2][0], motorPins[2][1], speed);
            setMotorSpeed(motorPins[3][0], motorPins[3][1], -speed);
            break;
        case RIGHT:
            setMotorSpeed(motorPins[0][0], motorPins[0][1], speed);
            setMotorSpeed(motorPins[1][0], motorPins[1][1], -speed);
            setMotorSpeed(motorPins[2][0], motorPins[2][1], -speed);
            setMotorSpeed(motorPins[3][0], motorPins[3][1], speed);
            break;
        case LEFT_ROTATE:
            setMotorSpeed(motorPins[0][0], motorPins[0][1], -speed);
            setMotorSpeed(motorPins[1][0], motorPins[1][1], -speed);
            setMotorSpeed(motorPins[2][0], motorPins[2][1], speed);
            setMotorSpeed(motorPins[3][0], motorPins[3][1], speed);
            break;
        case RIGHT_ROTATE:
            setMotorSpeed(motorPins[0][0], motorPins[0][1], speed);
            setMotorSpeed(motorPins[1][0], motorPins[1][1], speed);
            setMotorSpeed(motorPins[2][0], motorPins[2][1], -speed);
            setMotorSpeed(motorPins[3][0], motorPins[3][1], -speed);
            break;
        default:
            stop();
            break;
    }
}

void MotorController::turn(int angle, int speed) {
    // 角度转持续时间(ms)
    int duration = map(abs(angle), 0, 360, 0, 6000); // 调整为6秒/360度(16.6ms/度)
    
    if(angle > 0) {
        // 右转
        move(RIGHT_ROTATE, speed);
    } else {
        // 左转 
        move(LEFT_ROTATE, speed);
    }
    
    delay(duration);
    stop();
}

void MotorController::stop() {
    for(int i = 0; i < 4; i++) {
        setMotorSpeed(motorPins[i][0], motorPins[i][1], 0);
    }
}

void MotorController::setMotorSpeed(uint16_t forwardPin, uint16_t backwardPin, int speed) {
    // 限制速度范围
    speed = constrain(speed, -255, 255);
    
    // 输出调试信息
    Serial.print("Setting motor pins ");
    Serial.print(forwardPin);
    Serial.print(",");
    Serial.print(backwardPin);
    Serial.print(" speed: ");
    Serial.println(speed);

    if(speed >= 0) {
        pwm.setPWM(forwardPin, 0, speed * 16); // 0-255映射到0-4095
        pwm.setPWM(backwardPin, 0, 0);
    } else {
        pwm.setPWM(forwardPin, 0, 0);
        pwm.setPWM(backwardPin, 0, -speed * 16); // 0-255映射到0-4095
    }
}
