#ifndef ESC_H
#define ESC_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/gpio.h"
#include "esp_attr.h"

#define MOTOR1_GPIO    22 // Polia inferior - OPRB
#define MOTOR2_GPIO    23 // Polia superior - OPRA
#define PWM_FREQUENCY 490

extern float cur_duty_motor1;
extern float cur_duty_motor2;

void motor_calibration();
void pwm_gpio_initialize();
void set_duty_esc(float duty_motor1, float duty_motor2);

#endif
