#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
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
#include "robotControl.hpp"

#include <vector>
#include <memory>

const uint LED_PIN = 25;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 send_msg;
std_msgs__msg__Int32 recv_msg;
geometry_msgs__msg__Twist twist_msg;

MotorControl motor1 = MotorControl(0,1,2);
MotorControl motor2 = MotorControl(3,4,5);
MotorControl motor3 = MotorControl(6,7,8);
MotorControl motor4 = MotorControl(9,10,11);
RobotControl robot {motor1, motor2, motor3, motor4};

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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  //TODO: for now just setting manual speed for testing. will assign the linear_x speed later
  if (robotState == RobotState::MOVE_FORWARD)
  {
	//only spins motor 3 and 4
    robot.setSpeed(0.4);
    robot.moveForward();
  }
  else if (robotState == RobotState::MOVE_BACKWARD)
  {
    //spins all motors
    motor1.setPwm(20000);
    motor2.setPwm(20000);
    motor3.setPwm(20000);
    motor4.setPwm(20000);
    motor1.rotateCC();
    motor2.rotateCC();
    motor3.rotateCC();
    motor4.rotateCC();
  }
}

void subscription_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  auto linear_x = msg->linear.x;
  auto angular_z = msg->angular.z;
  //TODO: for now just setting manual speed for testing. will assign the linear_x speed later
  if (linear_x > 0)
  {
    robotState = RobotState::MOVE_FORWARD;
  } 
  else
  {
    robotState = RobotState::MOVE_BACKWARD;
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
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
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

    gpio_put(LED_PIN, 1);
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
