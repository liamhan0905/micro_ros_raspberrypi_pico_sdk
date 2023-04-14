// motorDriver.cpp
#include "motorDriver.h"

motorDriver::motorDriver(uint8_t inPin1, uint8_t inPin2, uint8_t pwmPin) {
  m_inPin1 = inPin1;
  m_inPin2 = inPin2;
  m_pwmPin = pwmPin;

//  pinMode(m_inPin1, OUTPUT);
//  pinMode(m_inPin2, OUTPUT);
//  pinMode(m_pwmPin, OUTPUT);
}

void motorDriver::setSpeed(int speed) {
  if (speed < -255) {
    speed = -255;
  } else if (speed > 255) {
    speed = 255;
  }

//  if (speed > 0) {
//    digitalWrite(m_inPin1, HIGH);
//    digitalWrite(m_inPin2, LOW);
//  } else if (speed < 0) {
//    digitalWrite(m_inPin1, LOW);
//    digitalWrite(m_inPin2, HIGH);
//    speed = -speed;
//  } else {
//    digitalWrite(m_inPin1, LOW);
//    digitalWrite(m_inPin2, LOW);
//  }
//
//  analogWrite(m_pwmPin, speed);
}

