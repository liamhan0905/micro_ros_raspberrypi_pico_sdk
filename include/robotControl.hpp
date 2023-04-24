#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <string>
#include "motorControl.hpp"
#include <memory>

class RobotControl {
public:
    // Constructors
    RobotControl(MotorControl& motor1, MotorControl& motor2, MotorControl& motor3, MotorControl& motor4);
    // Destructor
    ~RobotControl();

    // Methods
    void setName(std::string name);

    void moveForward();
    void moveBackward();
    void turnLeft();
    void turnRight();
    void setSpeed(float val);
    void stop();

private:
    MotorControl& _motor1; // front right
    MotorControl& _motor2; // rear right
    MotorControl& _motor3; // rear left
    MotorControl& _motor4; // front left
    static int _mapFloatToPWM(float val);
};

#endif // ROBOT_CONTROL_H

