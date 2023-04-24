#include "robotControl.hpp"
#include "motorControl.hpp"
#include <cmath>

// Constructors
//RobotControl(std::unique_ptr<motorControl> motor1, std::unique_ptr<motorControl> motor2, std::unique_ptr<motorControl> motor3, std::unique_ptr<motorControl> motor4)
//	: _motor1(std::move(motor1)), _motor2(std::move(motor2)), _motor3(std::move(motor3)), _motor4(std::move(motor4))
//{
//}

RobotControl::RobotControl(MotorControl& motor1, MotorControl& motor2, MotorControl& motor3, MotorControl& motor4)
	: _motor1(motor1), _motor2(motor2), _motor3(motor3), _motor4(motor4)
{
}

// Destructor
RobotControl::~RobotControl() {}

// val should be between range [0,1]
void RobotControl::setSpeed(float val)
{
   int targetPwm = _mapFloatToPWM(val);
  _motor1.setPwm(targetPwm);
  _motor2.setPwm(targetPwm);
  _motor3.setPwm(targetPwm);
  _motor4.setPwm(targetPwm);
}

// Movement methods
void RobotControl::moveForward()
{
  _motor1.rotateCC();
  _motor2.rotateCC();
  _motor3.rotateCC();
  _motor4.rotateCC();
}

void RobotControl::moveBackward()
{
  _motor1.rotateC();
  _motor2.rotateC();
  _motor3.rotateC();
  _motor4.rotateC();
}

void RobotControl::turnLeft()
{
  _motor1.rotateCC();
  _motor2.rotateCC();
  _motor3.rotateC();
  _motor4.rotateC();
}

void RobotControl::turnRight()
{
  _motor1.rotateC();
  _motor2.rotateC();
  _motor3.rotateCC();
  _motor4.rotateCC();
}

void RobotControl::stop()
{
  _motor1.stop();
  _motor2.stop();
  _motor3.stop();
  _motor4.stop();
}

int RobotControl::_mapFloatToPWM(float val) {
  // Check if the input value is within the valid range
  if (val < 0.0f) {
    val = 0.0f;
  } else if (val > 1.0f) {
    val = 1.0f;
  }

  // Map the input value to the range [0, 65535]
  uint16_t result = static_cast<uint16_t>(val * 65535.0f);

  return result;
}


