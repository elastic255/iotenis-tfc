#include "stepmotor.h"

float last_angle_acc;
int thisStep = 0;

/**
 * @brief test code to write esp-i2c-slave
 *
 * 1. set mode
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static esp_err_t i2c_example_master_sensor_test(i2c_port_t i2c_num, uint8_t* data_h, uint8_t* data_l)
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, MMA8451_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, MMA8451_REG_OUT_X_MSB, ACK_CHECK_EN);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    return ret;
  }
  vTaskDelay(30 / portTICK_RATE_MS);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, MMA8451_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data_h[0], ACK_VAL);
  i2c_master_read_byte(cmd, &data_l[0], ACK_VAL);
  i2c_master_read_byte(cmd, &data_h[1], ACK_VAL);
  i2c_master_read_byte(cmd, &data_l[1], ACK_VAL);
  i2c_master_read_byte(cmd, &data_h[2], ACK_VAL);
  i2c_master_read_byte(cmd, &data_l[2], NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

void writeRegister8(i2c_port_t i2c_num, uint8_t reg, uint8_t value)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, MMA8451_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
}

uint8_t readRegister8(i2c_port_t i2c_num, uint8_t reg)
{
  uint8_t data = 0;
  printf("reg: %x\n", reg);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, MMA8451_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(30 / portTICK_RATE_MS);

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, MMA8451_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data, NACK_VAL);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return data;
}

mma8451_range_t getRange(void)
{
  /* Read the data format register to preserve bits */
  return (mma8451_range_t) (readRegister8(I2C_EXAMPLE_MASTER_NUM, MMA8451_REG_XYZ_DATA_CFG) & 0x03);
}

void accelerometer_initialize()
{
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);

  /* Check connection */
  uint8_t deviceid = readRegister8(I2C_EXAMPLE_MASTER_NUM, MMA8451_REG_WHOAMI);
  if (deviceid != 0x2A)
  {
    printf("deu ruim\n");
  }

  writeRegister8(I2C_EXAMPLE_MASTER_NUM, MMA8451_REG_CTRL_REG2, 0x40); // reset

  while (readRegister8(I2C_EXAMPLE_MASTER_NUM, MMA8451_REG_CTRL_REG2) & 0x40);

  // enable 4G range
  writeRegister8(I2C_EXAMPLE_MASTER_NUM, MMA8451_REG_XYZ_DATA_CFG, MMA8451_RANGE_4_G);
  // High res
  writeRegister8(I2C_EXAMPLE_MASTER_NUM, MMA8451_REG_CTRL_REG2, 0x02);
  // DRDY on INT1
  writeRegister8(I2C_EXAMPLE_MASTER_NUM, MMA8451_REG_CTRL_REG4, 0x01);
  writeRegister8(I2C_EXAMPLE_MASTER_NUM, MMA8451_REG_CTRL_REG5, 0x01);

  // Turn on orientation config
  writeRegister8(I2C_EXAMPLE_MASTER_NUM, MMA8451_REG_PL_CFG, 0x40);

  // Activate at max rate, low noise mode
  writeRegister8(I2C_EXAMPLE_MASTER_NUM, MMA8451_REG_CTRL_REG1, 0x01 | 0x04);

  last_angle_acc = read_angle();
}

float read_angle()
{
  uint8_t sensor_data_h[3], sensor_data_l[3];
  int16_t x, y, z;
  float x_g, y_g, z_g;
  float acc_angle = 0.0;
  int ret;

  for (int i = 0; i < N; ++i) {
    do {
      ret = i2c_example_master_sensor_test(I2C_EXAMPLE_MASTER_NUM, sensor_data_h, sensor_data_l);
    } while (ret != ESP_OK);
    uint8_t range = getRange();
    uint16_t divider = 1;
    if (range == MMA8451_RANGE_8_G) divider = 1024;
    if (range == MMA8451_RANGE_4_G) divider = 2048;
    if (range == MMA8451_RANGE_2_G) divider = 4096;
    x = sensor_data_h[0] << 8 | sensor_data_l[0];
    y = sensor_data_h[1] << 8 | sensor_data_l[1];
    z = sensor_data_h[2] << 8 | sensor_data_l[2];
    x_g = (float)x / divider;
    y_g = (float)y / divider;
    z_g = (float)z / divider;
    float total = sqrt(x_g*x_g+y_g*y_g+z_g*z_g);
    acc_angle += acos(x_g/total)*180/M_PI-90;
  }
  return acc_angle / N;
}

void stepMotor(int step)
{
  switch (thisStep) {
  case 0:
    gpio_set_level(BOBINA_A, 1);
    gpio_set_level(BOBINA_B, 0);
    gpio_set_level(BOBINA_C, 0);
    gpio_set_level(BOBINA_D, 1);
    break;
  case 1:
    gpio_set_level(BOBINA_A, 0);
    gpio_set_level(BOBINA_B, 1);
    gpio_set_level(BOBINA_C, 0);
    gpio_set_level(BOBINA_D, 1);
    break;
  case 2:
    gpio_set_level(BOBINA_A, 0);
    gpio_set_level(BOBINA_B, 1);
    gpio_set_level(BOBINA_C, 1);
    gpio_set_level(BOBINA_D, 0);
    break;
  case 3:
    gpio_set_level(BOBINA_A, 1);
    gpio_set_level(BOBINA_B, 0);
    gpio_set_level(BOBINA_C, 1);
    gpio_set_level(BOBINA_D, 0);
    break;
  }
}

void rotate_clockwise(int steps)
{
  int64_t last = esp_timer_get_time();
  
  while (steps) {
    stepMotor(thisStep);
    int64_t cur = esp_timer_get_time();
    if (cur - last >= STEPMOTOR_DELAY_MS*1000) {
      thisStep = thisStep + 1;
      steps--;
      last = cur;
      if (thisStep >= 4)
        thisStep = 0;
    }
  }
}

void rotate_counterclockwise(int steps)
{
  int64_t last = esp_timer_get_time();
  
  while (steps) {
    stepMotor(thisStep);
    int64_t cur = esp_timer_get_time();
    if (cur - last >= STEPMOTOR_DELAY_MS*1000) {
      thisStep = thisStep - 1;
      steps--;
      last = cur;
      if (thisStep < 0)
        thisStep = 4;
    }
  }
}

void set_angle(int angle)
{
  int counter = 0;
  float acc_angle = last_angle_acc, prev_angle;
  do {
    //vTaskDelay(pdMS_TO_TICKS(ACC_DELAY));
    prev_angle = acc_angle;
    acc_angle = read_angle();
    ESP_LOGI("Teste", "angle (degree): %f", acc_angle);
    if (acc_angle < angle-EPSILON) {
      rotate_counterclockwise(STEPS_TO_TAKE);
    }
    else if (acc_angle > angle+EPSILON) {
      rotate_clockwise(STEPS_TO_TAKE);
    }
    if (acc_angle >= prev_angle-EPSILON && acc_angle <= prev_angle+EPSILON)
      counter++;
    else
      counter = 0;

    //if (counter == 10) break;
  } while (acc_angle < angle-EPSILON || acc_angle > angle+EPSILON);
  last_angle_acc = acc_angle;
  return;
}

void stepMotor_initialize()
{
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);

  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, PWM_STEPMOTOR);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = PWM_STEPMOTOR_FREQUENCY;
  pwm_config.cmpr_a = 50;    //duty cycle of PWMxA = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

  stepMotor(thisStep);
}

void assert_running()
{
  // NBIPS bips to assert that everything is fine (DELAY_BIPS ms between bips)
  for (int i = 0; i < NBIPS; ++i) {
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0);
    vTaskDelay(pdMS_TO_TICKS(DELAY_BIPS));
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 50.0);
    vTaskDelay(pdMS_TO_TICKS(DELAY_BIPS));
    stepMotor(thisStep); // this may not be necessary
  }
  set_angle(0);
}
