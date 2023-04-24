#ifdef __cplusplus
extern "C" {
#endif
#include "pico/stdlib.h"
#ifdef __cplusplus
}
#endif

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "motorControl.hpp"

MotorControl::MotorControl(uint8_t inPin1, uint8_t inPin2, uint8_t pwmPin)
	:_inPin1(inPin1), _inPin2(inPin2), _pwmPin(pwmPin)
{
  _initPins();
  _initPWM();
}

void MotorControl::_initPins()
{
  gpio_init(_inPin1);
  gpio_init(_inPin2);
  gpio_init(_pwmPin);
  gpio_set_dir(_inPin1, GPIO_OUT);
  gpio_set_dir(_inPin2, GPIO_OUT);
  gpio_set_function(_pwmPin, GPIO_FUNC_PWM);
}

void MotorControl::_initPWM()
{
  uint slice_num = pwm_gpio_to_slice_num(_pwmPin);
  pwm_config config = pwm_get_default_config();
  pwm_init(slice_num, &config, true);
  uint8_t duty_cycle = 0;
  pwm_set_wrap(slice_num, 65535);
  pwm_set_clkdiv(slice_num, 2.0f);
}

void MotorControl::rotateCC()
{
  gpio_put(_inPin1, 1);
  gpio_put(_inPin2, 0);
}

void MotorControl::setPwm(int pwmVal)
{
  pwm_set_gpio_level(_pwmPin, pwmVal);
}

void MotorControl::rotateC()
{
  gpio_put(_inPin1, 0);
  gpio_put(_inPin2, 1);
}

void MotorControl::stop()
{
  gpio_put(_inPin1, 0);
  gpio_put(_inPin2, 0);
}
