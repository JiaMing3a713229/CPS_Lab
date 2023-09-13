#ifndef _JM_mpu6050_H_
#define _JM_mpu6050_H_

#include "Arduino.h"
#include "driver/i2c.h"
// I2C 匯流排參數
#define I2C_MASTER_NUM 0                                /*!< esp32內部有2個I2C控制器，我們使用第1個(0)控制器*/
#define I2C_MASTER_SDA_IO 21                            /*!< GPIO 21 用於I2C SDA使用，提供I2C數據傳輸 */
#define I2C_MASTER_SCL_IO 22                            /*!< GPIO 22 用於I2C SCL使用，提供I2C時脈訊號來源 */
#define I2C_MASTER_FREQ_HZ          400000              /*!< I2C master clock frequency，400KHz */
#define I2C_MASTER_TX_BUF_DISABLE   0                   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

//#define MPU6050_SLAVE_ADDR                  0x68        /*!< Slave address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I                    0x75        

#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register，default 0x01 */ 
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

class JM_mpu6050{

    public:
        JM_mpu6050(uint8_t device_addr);
        esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
        esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data);
        void init_mpu6050(void);
        float get_mpu6050_temp(void);
        float get_mpu6050_accel_x(void);
        float get_mpu6050_accel_y(void);
        float get_mpu6050_accel_z(void);

        float get_mpu6050_gyro_x(void);
        float get_mpu6050_gyro_y(void);
        float get_mpu6050_gyro_z(void);

        uint8_t mpu6050_slave_addr;


};




#endif  //_JM_mpu6050_H_
