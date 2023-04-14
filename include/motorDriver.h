// motorDriver.h
#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <stdint.h>

struct motorDriver {
    motorDriver(uint8_t inPin1, uint8_t inPin2, uint8_t pwmPin);
    void setSpeed(int speed);

    uint8_t m_inPin1;
    uint8_t m_inPin2;
    uint8_t m_pwmPin;
};

#endif

