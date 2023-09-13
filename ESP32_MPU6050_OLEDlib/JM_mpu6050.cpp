#include "JM_mpu6050.h"

JM_mpu6050::JM_mpu6050(uint8_t _mpu6050_slave_addr){
    
    mpu6050_slave_addr = _mpu6050_slave_addr;

}

/**
 * @brief Read a byte to a MPU6050 sensor register
 */

esp_err_t JM_mpu6050:: mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, mpu6050_slave_addr, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU6050 sensor register
 */
esp_err_t JM_mpu6050::mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, mpu6050_slave_addr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ret;
}

void JM_mpu6050:: init_mpu6050(void){

  uint8_t data[2];
  mpu6050_register_read(MPU6050_WHO_AM_I, data, 1);
  Serial.printf("MPU6050_WHO_AM_I_ADDR = 0x%02x \r\n", data[0]);

  mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 0x80);                                 /*                 對電源管理寄存器寫入0x80進行復位                    */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 0x01);                                 /*                 對電源管理寄存器寫入0x01啟動電源                    */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  mpu6050_register_write_byte(MPU6050_GYRO_CONFIG_ADDR, 0x00 | (GYRO_SCALE_RANGE_2000 << 3));     /*    對陀螺儀配置寄存器設置為(0001 1000)設置陀螺儀敏感度(+-2000/s)     */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  mpu6050_register_write_byte(MPU6050_ACCEL_CONFIG_ADDR, 0x00 | (ACCEL_SCALE_RANGE_2G << 3));     /* 對加速度感測器配置寄存器設置為(0000 0001)設置加速度感測器敏感度(+-2g) */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  mpu6050_register_write_byte(MPU6050_ENABLE_FIFO_ADDR, 0x00);                                    /*          對FIFO配置寄存器設置為(0000 0000)，關閉FIFO               */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  mpu6050_register_write_byte(MPU6050_SMPLRT_DIV_ADDR, 0x13);                                     /*     對SMPLRT_DIV寄存器設置為(0001 0011)，設置感測器採樣頻率50Hz     */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  mpu6050_register_write_byte(MPU6050_CONFIG , (0x06 | ACCEL_BAND_5HZ_GYRO_BAND_5HZ));            /*開啟低通濾波，並設置為 ACCEL_BAND_5HZ_GYRO_BAND_5HZ(110)，頻率為1kHz*/
  vTaskDelay(10 / portTICK_PERIOD_MS);

}

float JM_mpu6050::get_mpu6050_temp(){

  uint8_t data[2];
  int16_t temp = 0;
  float ret = 0;

  mpu6050_register_read(MPU6050_TEMP_ADDR , data, 1);                       /*從0x41寄存器讀取溫度數據*/
  temp = ((int16_t)data[0]<<8) | data[1];                                            
  ret =  (float(temp)/340) + 36.53;

  return ret;

}

float JM_mpu6050::get_mpu6050_accel_x(){

  uint8_t data[2];
  int16_t accel_data = 0;
  float ret = 0;

  mpu6050_register_read(MPU6050_ACCEL_X_ADDR  , data, 1);                   /*從0x3B寄存器讀取x軸加速度數據*/
  accel_data = (int16_t)(data[0] << 8) | data[1];
  ret = float(accel_data) / 16384;
  ret *= SENSORS_GRAVITY_STANDARD;
  return ret;

}

float JM_mpu6050::get_mpu6050_accel_y(){
  
  uint8_t data[2] = {0, 0};
  int16_t accel_data = 0;
  float ret = 0;

  mpu6050_register_read(MPU6050_ACCEL_Y_ADDR  , data, 1);                       
  accel_data = (int16_t)(data[0] << 8) | data[1];
  ret = float(accel_data) / 16384;
  ret *= SENSORS_GRAVITY_STANDARD;       
  return ret;

}

float JM_mpu6050::get_mpu6050_accel_z(){

  uint8_t data[2];
  int16_t accel_data = 0;
  float ret = 0;

  mpu6050_register_read(MPU6050_ACCEL_Z_ADDR  , data, 1);                       
  
  accel_data = (int16_t)(data[0] << 8) | data[1];
  ret = float(accel_data) / 16384;
  ret *= SENSORS_GRAVITY_STANDARD;
  
  return ret;

}

float JM_mpu6050::get_mpu6050_gyro_x(){

  uint8_t data[2];
  float ret = 0;
  mpu6050_register_read(MPU6050_GYRO_X_ADDR  , data, 1);                       
  
  ret = (int16_t)(data[0] << 8) | data[1];
  ret = (float)ret / 16.4;
  ret *= SENSORS_DPS_TO_RADS;
  return ret;
}

float JM_mpu6050::get_mpu6050_gyro_y(){
  uint8_t data[2];
  float ret = 0;
  mpu6050_register_read(MPU6050_GYRO_Y_ADDR  , data, 1);                       /*對溫度感測器寄存器取值*/
  
  ret = (int16_t)(data[0] << 8) | data[1];
  ret = (float)ret / 16.4;
  ret *= SENSORS_DPS_TO_RADS;
  return ret;

}

float JM_mpu6050::get_mpu6050_gyro_z(){
  uint8_t data[2];
  float ret = 0;

  mpu6050_register_read(MPU6050_GYRO_Z_ADDR  , data, 1);                       /*對溫度感測器寄存器取值*/

  ret = (int16_t)(data[0] << 8) | data[1];
  ret = (float)ret / 16.4;
  ret *= SENSORS_DPS_TO_RADS;
  return ret;

}