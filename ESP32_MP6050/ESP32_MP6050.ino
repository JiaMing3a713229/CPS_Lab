#include <Arduino.h>
#include "driver/i2c.h"

#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 21                            /*!< GPIO 21 用於I2C SDA使用，提供I2C數據傳輸 */
#define I2C_MASTER_SCL_IO 22                            /*!< GPIO 22 用於I2C SCL使用，提供I2C時脈傳輸 */
#define I2C_MASTER_FREQ_HZ          400000              /*!< I2C master clock frequency，400KHz */
#define I2C_MASTER_TX_BUF_DISABLE   0                   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU6050 sensor */
#define MPU6050_SLAVE_ADDR                  (MPU6050_SENSOR_ADDR)
#define MPU6050_WHO_AM_I                    0x75        

#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register，default 0x00 */ 
#define MPU6050_GYRO_CONFIG_ADDR            0x1B        /*!< Register addresses of the gyroscope register，defult        0x18 */
#define MPU6050_ACCEL_CONFIG_ADDR           0x1C        /*!< Register addresses of the accelerometer register，default   0x01 */
#define MPU6050_ENABLE_FIFO_ADDR            0x23        /*!< Register addresses of the FIFO register，disable            0x00 */
#define MPU6050_SMPLRT_DIV_ADDR             0x19        /*!< Register addresses of the sample rate register，default     0x06 */
#define MPU6050_CONFIG                      0x1A        /*!< Register addresses of the Digital Low Pass Filter(DLPF) register，default is 0x06 */


/*Reading data from MPU6050 */
#define MPU6050_ACCEL_X_ADDR                0x3B        /*!< Register addresses of the "加速度值 X方向*/
#define MPU6050_ACCEL_Y_ADDR                0x3D        /*!< Register addresses of the "加速度值 Y方向*/
#define MPU6050_ACCEL_Z_ADDR                0x3F        /*!< Register addresses of the "加速度值 Z方向*/

#define MPU6050_GYRO_X_ADDR                 0x43        /*!< Register addresses of the "陀螺儀數值 X方向*/
#define MPU6050_GYRO_Y_ADDR                 0x45        /*!< Register addresses of the "陀螺儀數值 Y方向*/
#define MPU6050_GYRO_Z_ADDR                 0x47        /*!< Register addresses of the "陀螺儀數值 Z方向*/

#define MPU6050_TEMP_ADDR                   0x41        /*!< Register addresses of the "溫度值"*/

/*     MPU6050 數據單位換算     */
#define SENSORS_GRAVITY_EARTH (9.80665F)                                /**< Earth's gravity in m/s^2               */
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)                /**從mpu6050所得數據單位為G，1G = 9.8F m/s^2  */
#define SENSORS_DPS_TO_RADS      (0.017453293F)                         /**<Degrees/s to rad/s multiplier           */                                        
#define SENSORS_RADS_TO_DPS      (57.29577793F)                         /**< Rad/s to degrees/s  multiplier         */

                                                        /*Accelerometer          Gyroscope                    */
typedef enum {                                          /*Bandwidth    Delay     Bandwidth    Delay           */
  ACCEL_BAND_260HZ_GYRO_BAND_256HZ = 0,                 /*260HZ        0   ms     256HZ        0.95ms         */
  ACCEL_BAND_184HZ_GYRO_BAND_188HZ,                     /*184HZ        2.0 ms     188HZ        1.9 ms         */
  ACCEL_BAND_94HZ_GYRO_BAND_98HZ,                       /*94 HZ        3.0 ms      98HZ        2.8 ms         */
  ACCEL_BAND_44HZ_GYRO_BAND_42HZ,                       /*44 HZ        4.9 ms      42HZ        4.8 ms         */
  ACCEL_BAND_21HZ_GYRO_BAND_20HZ,                       /*21 HZ        8.5 ms      20HZ        8.3 ms         */
  ACCEL_BAND_10HZ_GYRO_BAND_10HZ,                       /*10 HZ        13.8ms      10HZ        13.4ms         */
  ACCEL_BAND_5HZ_GYRO_BAND_5HZ,                         /*5  HZ        19.0ms       5HZ        18.6ms  Default*/
}DLPF_CFG_mode;                                         /*              配置於0x1A寄存器                       */                      

typedef enum{                                           /*FS_SEL                  Full Scale Range            */
  GYRO_SCALE_RANGE_250 = 0,                             /*  0                     +-250。/s                   */
  GYRO_SCALE_RANGE_500,                                 /*  1                     +-500。/s                   */         
  GYRO_SCALE_RANGE_1000,                                /*  2                     +-1000。/s                  */
  GYRO_SCALE_RANGE_2000                                 /*  3                     +-2000。/s           Default*/

}Gyro_scale_conf;

typedef enum{                                           /*AFS_SEL                  Full Scale Range           */                   

  ACCEL_SCALE_RANGE_2G = 0,                             /*  0                     +-2g                 Default*/
  ACCEL_SCALE_RANGE_4G,                                 /*  1                     +-4g                        */
  ACCEL_SCALE_RANGE_8G,                                 /*  2                     +-8g                        */
  ACCEL_SCALE_RANGE_16G                                 /*  3                     +-16g                       */

}Accel_scale_conf;

/**
 * @brief Read a byte to a MPU6050 sensor register
 */

static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SLAVE_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU6050 sensor register
 */
static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SLAVE_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
          .clk_speed = I2C_MASTER_FREQ_HZ,
        }
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief i2c master initialization
 */

static void init_mpu6050(void){

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
/**
 * @brief Reading temperature data from MPU6050
 */
float get_mpu6050_temp(){

  uint8_t data[2];
  int16_t temp = 0;
  float ret = 0;

  mpu6050_register_read(MPU6050_TEMP_ADDR , data, 1);                       /*從0x41寄存器讀取溫度數據*/
  temp = ((int16_t)data[0]<<8) | data[1];                                            
  ret =  (float(temp)/340) + 36.53;

  return ret;

}

float get_mpu6050_accel_x(){

  uint8_t data[2];
  int16_t accel_data = 0;
  float ret = 0;

  mpu6050_register_read(MPU6050_ACCEL_X_ADDR  , data, 1);                   /*從0x3B寄存器讀取x軸加速度數據*/
  accel_data = (int16_t)(data[0] << 8) | data[1];
  ret = float(accel_data) / 16384;
  ret *= SENSORS_GRAVITY_STANDARD;
  return ret;

}

float get_mpu6050_accel_y(){
  
  uint8_t data[2] = {0, 0};
  int16_t accel_data = 0;
  float ret = 0;

  mpu6050_register_read(MPU6050_ACCEL_Y_ADDR  , data, 1);                       
  accel_data = (int16_t)(data[0] << 8) | data[1];
  ret = float(accel_data) / 16384;
  ret *= SENSORS_GRAVITY_STANDARD;       
  return ret;

}

float get_mpu6050_accel_z(){

  uint8_t data[2];
  int16_t accel_data = 0;
  float ret = 0;

  mpu6050_register_read(MPU6050_ACCEL_Z_ADDR  , data, 1);                       
  
  accel_data = (int16_t)(data[0] << 8) | data[1];
  ret = float(accel_data) / 16384;
  ret *= SENSORS_GRAVITY_STANDARD;
  
  return ret;

}

float get_mpu6050_gyro_x(){

  uint8_t data[2];
  float ret = 0;
  mpu6050_register_read(MPU6050_GYRO_X_ADDR  , data, 1);                       
  
  ret = (int16_t)(data[0] << 8) | data[1];
  ret = (float)ret / 16.4;
  ret *= SENSORS_DPS_TO_RADS;
  return ret;
}

float get_mpu6050_gyro_y(){
  uint8_t data[2];
  float ret = 0;
  mpu6050_register_read(MPU6050_GYRO_Y_ADDR  , data, 1);                       /*對溫度感測器寄存器取值*/
  
  ret = (int16_t)(data[0] << 8) | data[1];
  ret = (float)ret / 16.4;
  ret *= SENSORS_DPS_TO_RADS;
  return ret;

}

float get_mpu6050_gyro_z(){
  uint8_t data[2];
  float ret = 0;

  mpu6050_register_read(MPU6050_GYRO_Z_ADDR  , data, 1);                       /*對溫度感測器寄存器取值*/

  ret = (int16_t)(data[0] << 8) | data[1];
  ret = (float)ret / 16.4;
  ret *= SENSORS_DPS_TO_RADS;
  return ret;

}

void setup() {

  uint8_t data[2];

  Serial.begin(115200);
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  if(i2c_master_init() == ESP_OK){
    Serial.println("I2C initialized successfully !! \r\n");
  }

  init_mpu6050();

  

  

  
}

void loop() {

  float temp = 0;
  float gyro_x, gyro_y, gyro_z;
  float accel_x, accel_y, accel_z;

  temp = get_mpu6050_temp();
  gyro_x = get_mpu6050_gyro_x();
  gyro_y = get_mpu6050_gyro_y();
  gyro_z = get_mpu6050_gyro_z();

  accel_x = get_mpu6050_accel_x();
  accel_y = get_mpu6050_accel_y();
  accel_z = get_mpu6050_accel_z();

  Serial.printf("Temperature = %.2f\t", temp);
  Serial.printf("gyro ");
  Serial.printf("x: %3.2f , ", gyro_x);
  Serial.printf("y: %3.2f , ", gyro_y);
  Serial.printf("z: %3.2f  \t",gyro_z);
  Serial.printf("acceleration ");
  Serial.printf("x: %3.2f , ", accel_x);
  Serial.printf("y: %3.2f , ", accel_y);
  Serial.printf("z: %3.2f , ", accel_z);
  Serial.println("");
  vTaskDelay(200 / portTICK_PERIOD_MS);

}




