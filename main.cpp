#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

// pwm control
#include "hardware/pwm.h"
#include "hardware/gpio.h"

#ifdef __cplusplus
}
#endif
// include custom files
//#include "robotControl.hpp"
#include "motorControl.hpp"

#include <vector>
#include <memory>

const uint LED_PIN = 25;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
//std_msgs__msg__Int32 send_msg;
std_msgs__msg__Float32 send_msg;
std_msgs__msg__Int32 recv_msg;
geometry_msgs__msg__Twist twist_msg;

MotorControl motor1 {0,1,2};
MotorControl motor2 {3,4,5};
MotorControl motor3 {6,7,8};
MotorControl motor4 {9,10,11};
//TODO: figure out why below doesn't work. using the robot object works fine in the main loop. But it only rotates motor 3 and 4 when using with the subscriber. upon checking, robot.motor1 address and motor1 address are different. Same with motor2. However, motor 3 and 4 are consistent. I've reviewed my code but still have no idea what the issue is. After 3 days of banging my head against the wall, I decided to move on for now.
//RobotControl robot {motor1, motor2, motor3, motor4};

namespace RobotControl
{
  int _mapFloatToPWM(float val) {
    // Check if the input value is within the valid range
    if (val < 0.0f) {
       val = 0.0f;
     } else if (val > 1.0f) {
       val = 1.0f;
     }

    // Map the input value to the range [0, 65535]
    return static_cast<uint16_t>(val * 65535.0f);
  }

  // val should be between range [0,1]
  void setSpeed(float val)
  {
     int targetPwm = _mapFloatToPWM(val);
    motor1.setPwm(targetPwm);
    motor2.setPwm(targetPwm);
    motor3.setPwm(targetPwm);
    motor4.setPwm(targetPwm);
  }

  void moveForward()
  {
    motor1.rotateCC();
    motor2.rotateCC();
    motor3.rotateCC();
    motor4.rotateCC();
  }

  void moveBackward()
  {
    motor1.rotateC();
    motor2.rotateC();
    motor3.rotateC();
    motor4.rotateC();
  }

  void turnLeft()
  {
    motor1.rotateCC();
    motor2.rotateCC();
    motor3.rotateC();
    motor4.rotateC();
  }

  void turnRight()
  {
    motor1.rotateC();
    motor2.rotateC();
    motor3.rotateCC();
    motor4.rotateCC();
  }

  void stop()
  {
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
  }
}

void led_toggle(int time) {
      gpio_put(LED_PIN, 0);
      sleep_ms(time);
      gpio_put(LED_PIN, 1);
}


enum class RobotState {
    STOP,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    TURN_RIGHT,
    TURN_LEFT
};

RobotState robotState = RobotState::STOP;
RobotState currentState{robotState};
float previousSpeed {};
float currentSpeed {};

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  send_msg.data = currentSpeed;
  rcl_ret_t ret = rcl_publish(&publisher, &send_msg, NULL);

  // check if the speed changed
  if (previousSpeed != currentSpeed)
  {
    previousSpeed = currentSpeed;
    RobotControl::setSpeed(abs(currentSpeed));
  }
  if (currentState != robotState)
  {
    currentState = robotState;
    if (robotState == RobotState::MOVE_FORWARD)
    {
      RobotControl::moveForward();
    }
    else if (robotState == RobotState::MOVE_BACKWARD)
    {
      RobotControl::moveBackward();
    }
    else if (robotState == RobotState::TURN_LEFT)
    {
      RobotControl::turnLeft();
    }
    else if (robotState == RobotState::TURN_RIGHT)
    {
      RobotControl::turnRight();
    }
    else if (robotState == RobotState::STOP)
    {
      RobotControl::stop();
    }
  }
}

// responsible for just setting the state
void subscription_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  auto linear_x = msg->linear.x;
  auto angular_z = msg->angular.z;
  //TODO: for now just setting manual speed for testing. will assign the linear_x speed later
  if (linear_x != 0)
  {
    //if (currentSpeed != linear_x)
    //{
      gpio_put(LED_PIN, 1);
      currentSpeed = linear_x;
      // set speed
      //RobotControl::setSpeed(abs(linear_x));
      // set state
      robotState = (linear_x > 0) ? RobotState::MOVE_FORWARD : RobotState::MOVE_BACKWARD;
    //}
  }
  else if (angular_z != 0)
  {
    //if (currentAngularSpeed != angular_z)
    //{
      gpio_put(LED_PIN, 0);
      //currentAngularSpeed = angular_z;
      currentSpeed = angular_z;
      // set speed
      //RobotControl::setSpeed(abs(angular_z));
      // set state
      robotState = (angular_z > 0) ? RobotState::TURN_LEFT : RobotState::TURN_RIGHT;
    //}
  }
  else
  {
    currentSpeed = 0;
    robotState = RobotState::STOP;
  }
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    // node init
    rclc_node_init_default(&node, "pico_node", "", &support);

    // create publisher
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pub");

    // create subscriber
    rclc_subscription_init_default(
        &subscriber,
	&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "robot/cmd_vel");

    // create timer
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback);

    send_msg.data = 0;
    recv_msg.data = 0;

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA);

//    gpio_put(LED_PIN, 1);
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
