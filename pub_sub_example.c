#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

// pwm control
#include "hardware/pwm.h"

const uint LED_PIN = 25;
const uint MOTOR_1_IN1_PIN = 0;
const uint MOTOR_1_IN2_PIN = 1;
const uint MOTOR_2_IN1_PIN = 2;
const uint MOTOR_2_IN2_PIN = 3;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 send_msg;
std_msgs__msg__Int32 recv_msg;

void led_toggle() {
      gpio_put(LED_PIN, 0);
      sleep_ms(100);
      gpio_put(LED_PIN, 1);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t ret = rcl_publish(&publisher, &send_msg, NULL);
  printf("Sent: %d\n",  (int)  send_msg.data);
  send_msg.data = 100;
}

void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  printf("Received: %d\n",  (int)  msg->data);
  led_toggle();
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

//////
    gpio_init(MOTOR_1_IN1_PIN);
    gpio_init(MOTOR_1_IN2_PIN);
    gpio_init(MOTOR_2_IN1_PIN);
    gpio_init(MOTOR_2_IN2_PIN);

    gpio_set_dir(MOTOR_1_IN1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_1_IN2_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_2_IN1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_2_IN2_PIN, GPIO_OUT);
//////




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
        "TESTpub");

    // create subscriber
    rclc_subscription_init_default(
        &subscriber,
	&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "sub");

    // create timer
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
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
	sleep_ms(100);

        gpio_put(MOTOR_1_IN1_PIN, 1);
        gpio_put(MOTOR_1_IN2_PIN, 0);
        gpio_put(MOTOR_2_IN1_PIN, 1);
        gpio_put(MOTOR_2_IN2_PIN, 0);
        sleep_ms(1000);  // Run for 1 second

        // Set the motor to turn counterclockwise at 50% speed
        gpio_put(MOTOR_1_IN1_PIN, 0);
	gpio_put(MOTOR_1_IN2_PIN, 1);
	gpio_put(MOTOR_2_IN1_PIN, 0);
        gpio_put(MOTOR_2_IN2_PIN, 1);
        sleep_ms(1000);  // Run for 1 second

        // Stop the motor
        gpio_put(MOTOR_1_IN1_PIN, 0);
        gpio_put(MOTOR_1_IN2_PIN, 0);
        gpio_put(MOTOR_2_IN1_PIN, 0);
        gpio_put(MOTOR_2_IN2_PIN, 0);
        sleep_ms(1000);  // Run for 1 second
    }
    return 0;
}
