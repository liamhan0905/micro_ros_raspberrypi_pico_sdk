// motorDriver.h
#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <stdint.h>

class motorDriver {
  private:
    uint8_t _inPin1;
    uint8_t _inPin2;
    uint8_t _pwmPin;
    void _initPWM(uint8_t motorPin);

  public:
    motorDriver(uint8_t inPin1, uint8_t inPin2, uint8_t pwmPin);
    void initPins();
    void setPwm(int pwmVal);
    void setSpeed(int speed);
    void rotateCC();
    void rotateC();

};

#endif

