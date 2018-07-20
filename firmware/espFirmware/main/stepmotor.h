#ifndef STEPMOTOR_H
#define STEPMOTOR_H

#include <math.h>

#include "driver/i2c.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"

#define NBIPS 4
#define DELAY_BIPS 500

#define EPSILON 0.6f
#define STEPS_TO_TAKE 5

#define N 5 // Quantity of measurements from the accelerometer

#define ACC_DELAY 50 // ms

#define PWM_STEPMOTOR 19
#define PWM_STEPMOTOR_FREQUENCY 1000

#define BOBINA_A 4
#define BOBINA_B 16
#define BOBINA_C 17
#define BOBINA_D 5
#define GPIO_OUTPUT_PIN_SEL  ((1<<BOBINA_A) | (1<<BOBINA_B) | (1<<BOBINA_C) | (1<<BOBINA_D))

#define STEPMOTOR_DELAY_MS 30

#define I2C_EXAMPLE_MASTER_SCL_IO          15               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          2               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define MMA8451_SENSOR_ADDR                0x1C             /*!< slave address for MMA8451 sensor */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */
#define MMA8451_REG_OUT_X_MSB     0x01
#define MMA8451_REG_WHOAMI        0x0D
#define MMA8451_REG_XYZ_DATA_CFG  0x0E
#define MMA8451_REG_PL_CFG        0x11
#define MMA8451_REG_CTRL_REG1     0x2A
#define MMA8451_REG_CTRL_REG2     0x2B
#define MMA8451_REG_CTRL_REG4     0x2D
#define MMA8451_REG_CTRL_REG5     0x2E
typedef enum
{
  MMA8451_RANGE_8_G           = 0b10,   // +/- 8g
  MMA8451_RANGE_4_G           = 0b01,   // +/- 4g
  MMA8451_RANGE_2_G           = 0b00    // +/- 2g (default value)
} mma8451_range_t;

extern float last_angle_acc;
extern int thisStep;

void writeRegister8(i2c_port_t i2c_num, uint8_t reg, uint8_t value);
uint8_t readRegister8(i2c_port_t i2c_num, uint8_t reg);
mma8451_range_t getRange(void);
void accelerometer_initialize();
void stepMotor(int step);
void rotate_clockwise(int steps);
void rotate_counterclockwise(int steps);
void set_angle(int angle);
float read_angle();
void stepMotor_initialize();
void assert_running();
#endif
