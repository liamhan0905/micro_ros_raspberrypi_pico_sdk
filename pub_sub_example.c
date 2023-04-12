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
#include "hardware/gpio.h"

const uint LED_PIN = 25;
const uint MOTOR_1_IN1_PIN = 0;
const uint MOTOR_1_IN2_PIN = 1;
const uint MOTOR_1_PWM = 2;
//const uint MOTOR_2_IN1_PIN = 3;
//const uint MOTOR_2_IN2_PIN = 4;
//const uint MOTOR_2_PWM = 5;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 send_msg;
std_msgs__msg__Int32 recv_msg;

#define PWM_FREQUENCY 10000
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

void rotate_motor(int pin1, int pin2, float duty_cycle_percent) {
    float on_time = 2000.0 * duty_cycle_percent / 100.0; // 2000 is the period of PWM signal in microseconds
    float off_time = 2000.0 - on_time;
    while(1) {
        gpio_put(pin1, 1);
        sleep_us(on_time); // adjust the length of time IN1 is on
        gpio_put(pin1, 0);
        sleep_us(off_time); // adjust the length of time IN1 is off
        gpio_put(pin2, 1);
        sleep_us(on_time); // adjust the length of time IN2 is on
        gpio_put(pin2, 0);
        sleep_us(off_time); // adjust the length of time IN2 is off
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

//////


    gpio_init(MOTOR_1_IN1_PIN);
    gpio_init(MOTOR_1_IN2_PIN);
    gpio_init(MOTOR_1_PWM);

    gpio_set_dir(MOTOR_1_IN1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_1_IN2_PIN, GPIO_OUT);
    gpio_set_function(MOTOR_1_PWM, GPIO_FUNC_PWM);
    // Set up PWM

    //gpio_set_dir(MOTOR_1_PWM, GPIO_OUT);
//////

    // Define the PWM frequency
    #define PWM_FREQUENCY 10000
    
    // Define the duty cycle range
    #define PWM_RANGE 255
    
    // Define the motor speed
    #define MOTOR_SPEED 100

    // Initialize the PWM hardware
  //  pwm_config pwm_config_a = pwm_get_default_config();
  //  pwm_config_set_wrap(&pwm_config_a, PWM_RANGE);
  //  pwm_config_set_clkdiv(&pwm_config_a, 4.0f);
  //  pwm_init(pwm_gpio_too_slice_num(MOTOR_1_PWM), &pwm_config_a, true);
 
 //   pwm_config pwm_config_b = pwm_get_default_config();
 //   pwm_config_set_wrap(&pwm_config_b, PWM_RANGE);
 //   pwm_config_set_clkdiv(&pwm_config_b, 4.0f);
 //   pwm_init(pwm_gpio_to_slice_num(MOTOR_2_PWM), &pwm_config_b, true);

    uint slice_num = pwm_gpio_to_slice_num(MOTOR_1_PWM);
    pwm_config config = pwm_get_default_config();
    pwm_init(slice_num, &config, true);
    uint8_t duty_cycle = 0;
    pwm_set_wrap(slice_num, 65535);
    pwm_set_clkdiv(slice_num, 2.0f);



    // Set the PWM frequency
  //  pwm_set_wrap(pwm_gpio_to_slice_num(MOTOR_1_PWM), PWM_RANGE);
  //  pwm_set_clkdiv(pwm_gpio_to_slice_num(MOTOR_1_PWM), 4.0f);
 //   pwm_set_wrap(pwm_gpio_to_slice_num(MOTOR_2_PWM), PWM_RANGE);
 //   pwm_set_clkdiv(pwm_gpio_to_slice_num(MOTOR_2_PWM), 4.0f);



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

        // Set the PWM duty cycle to 50%
        pwm_set_gpio_level(MOTOR_1_PWM, PWM_RANGE/2);
//        pwm_set_gpio_level(MOTOR_2_PWM, PWM_RANGE/2);

        // Rotate the motor in the current direction
        gpio_put(MOTOR_1_IN1_PIN, 1);
        gpio_put(MOTOR_1_IN2_PIN, 0);
        pwm_set_gpio_level(MOTOR_1_PWM, 30000);
	
       // gpio_put(MOTOR_2_IN1_PIN, 1);
       // gpio_put(MOTOR_2_IN2_PIN, 0);

        sleep_ms(1000);  // Rotate for 1 second

        // Stop the motor
        gpio_put(MOTOR_1_IN1_PIN, 0);
        gpio_put(MOTOR_1_IN2_PIN, 1);
        pwm_set_gpio_level(MOTOR_1_PWM, 30000);
       // gpio_put(MOTOR_2_IN1_PIN, 0);
       // gpio_put(MOTOR_2_IN2_PIN, 0);

        sleep_ms(1000);  // Wait for 1 second

        gpio_put(MOTOR_1_IN1_PIN, 1);
        gpio_put(MOTOR_1_IN2_PIN, 0);
        pwm_set_gpio_level(MOTOR_1_PWM, 10000);

        sleep_ms(1000);  // Wait for 1 second
        gpio_put(MOTOR_1_IN1_PIN, 0);
        gpio_put(MOTOR_1_IN2_PIN, 1);
        pwm_set_gpio_level(MOTOR_1_PWM, 10000);
       // gpio_put(MOTOR_1_IN1_PIN, 1);
       // sleep_us(750);
       // gpio_put(MOTOR_1_IN2_PIN, 0);
       // sleep_us(250);
       // gpio_put(MOTOR_2_IN1_PIN, 1);
       // sleep_us(750);
       // gpio_put(MOTOR_2_IN2_PIN, 0);
       // sleep_us(250);

       // // Set the motor to turn counterclockwise at 50% speed
       // gpio_put(MOTOR_1_IN1_PIN, 0);
       // sleep_us(250);
       // gpio_put(MOTOR_1_IN2_PIN, 1);
       // sleep_us(750);
       // gpio_put(MOTOR_2_IN1_PIN, 0);
       // sleep_us(250);
       // gpio_put(MOTOR_2_IN2_PIN, 1);
       // sleep_us(750);

       // // Stop the motor
       // gpio_put(MOTOR_1_IN1_PIN, 0);
       // gpio_put(MOTOR_1_IN2_PIN, 0);
       // gpio_put(MOTOR_2_IN1_PIN, 0);
       // gpio_put(MOTOR_2_IN2_PIN, 0);
       // sleep_ms(1000);  // Run for 1 second
    }
    return 0;
}
