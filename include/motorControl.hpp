#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

class MotorControl {
  public:
    MotorControl(uint8_t inPin1, uint8_t inPin2, uint8_t pwmPin);
    void setPwm(int pwmVal);
    void setSpeed(int speed);
    void rotateCC();
    void rotateC();
    void stop();

  private:
    uint8_t _inPin1;
    uint8_t _inPin2;
    uint8_t _pwmPin;
    void _initPins();
    void _initPWM();
};

#endif

