#include "wheel.h"

void set_velocity(float left, float right)
{
  int leftReverse, rightReverse;
  
  if (left < 0) {
    leftReverse = 0;
    left *= -1;
  }
  else
    leftReverse = 1;
  if (right < 0) {
    rightReverse = 1;
    right *= -1;
  }
  else
    rightReverse = 0;
  gpio_set_level(RODA1_REVERSE, leftReverse);
  gpio_set_level(RODA2_REVERSE, rightReverse);
  
  mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, left);
  mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, right);
}

void wheelMotor_initialize()
{
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = RODA1_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);

  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = RODA2_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);

  // Enable signals
  gpio_set_level(RODA1_ENABLE, RODA_ENABLE);
  gpio_set_level(RODA2_ENABLE, RODA_ENABLE);
  
  // PWM signals
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, RODA1_PWM);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, RODA2_PWM);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 2000;
  pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;    //duty cycle of PWMxA = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);  
}

void disable_wheels()
{
  gpio_set_level(RODA1_ENABLE, RODA_DISABLE);
  gpio_set_level(RODA2_ENABLE, RODA_DISABLE);
  set_velocity(0.0, 0.0);
}

void enable_wheels()
{
  gpio_set_level(RODA1_ENABLE, RODA_ENABLE);
  gpio_set_level(RODA2_ENABLE, RODA_ENABLE);
}
