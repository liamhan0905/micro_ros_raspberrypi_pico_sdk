// motorDriver.h
#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <stdint.h>

class motorDriver {
  public:
    motorDriver(uint8_t inPin1, uint8_t inPin2, uint8_t pwmPin);
    void setSpeed(int speed);

  private:
    uint8_t _inPin1;
    uint8_t _inPin2;
    uint8_t _pwmPin;
};

#endif

