#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <string>
#include "motorControl.hpp"
#include <memory>

class RobotControl {
public:
    // Constructors
//    RobotControl(std::unique_ptr<MotorControl> motor1, std::unique_ptr<MotorControl> motor2, std::unique_ptr<MotorControl> motor3, std::unique_ptr<MotorControl> motor4);

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
    MotorControl _motor1; // front right
    MotorControl _motor2; // rear right
    MotorControl _motor3; // rear left
    MotorControl _motor4; // front left
    static int _mapFloatToPWM(float val);

    //std::unique_ptr<MotorControl> _motor1; // front right
    //std::unique_ptr<MotorControl> _motor2; // rear right
    //std::unique_ptr<MotorControl> _motor3; // rear left
    //std::unique_ptr<MotorControl> _motor4; // front left
};

#endif // ROBOT_CONTROL_H

