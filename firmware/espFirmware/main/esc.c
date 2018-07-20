#include "esc.h"

float cur_duty_motor1 = 0;
float cur_duty_motor2 = 0;

void pwm_gpio_initialize()
{
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR2_GPIO);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR1_GPIO);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = PWM_FREQUENCY;
  pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void motor_calibration()
{
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 51.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 51.0);
  vTaskDelay(pdMS_TO_TICKS(1000));

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 50.0);
  vTaskDelay(pdMS_TO_TICKS(1000));  
}

void set_duty_esc(float duty_motor1, float duty_motor2)
{
  cur_duty_motor1 = duty_motor1;
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_motor1);
  cur_duty_motor2 = duty_motor2;          
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_motor2);
}
