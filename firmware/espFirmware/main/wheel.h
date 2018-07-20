#ifndef WHEEL_H
#define WHEEL_H

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/gpio.h"
#include "esp_attr.h"

#define RODA_ENABLE  0
#define RODA_DISABLE 1

// Left wheel
#define RODA1_PWM     13
#define RODA1_REVERSE 12
#define RODA1_ENABLE  27

// Right wheel
#define RODA2_PWM     26
#define RODA2_REVERSE 25
#define RODA2_ENABLE  32

#define RODA1_OUTPUT_PIN_SEL (1<<RODA1_REVERSE) | (1<<RODA1_ENABLE)
#define RODA2_OUTPUT_PIN_SEL (1ULL<<RODA2_REVERSE) | (1ULL<<RODA2_ENABLE)

void set_velocity(float left, float right);
void wheelMotor_initialize();
void disable_wheels();
void enable_wheels();
#endif
