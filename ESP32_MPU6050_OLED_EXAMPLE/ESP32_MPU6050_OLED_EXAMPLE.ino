#include <Arduino.h>
#include "JM_mpu6050.h"
#include "JM_ssd1306.h"

JM_mpu6050 mpu6050(0x68);
JM_ssd1306 oled(0x3c);

#define DEBUG_MODE 1

#define I2C_MASTER_NUM 0                                /*!< esp32內部有2個I2C控制器，我們使用第1個(0)控制器*/
#define I2C_MASTER_SDA_IO 21                            /*!< GPIO 21 用於I2C SDA使用，提供I2C數據傳輸 */
#define I2C_MASTER_SCL_IO 22                            /*!< GPIO 22 用於I2C SCL使用，提供I2C時脈訊號來源 */
#define I2C_MASTER_FREQ_HZ          400000              /*!< I2C master clock frequency，400KHz */
#define I2C_MASTER_TX_BUF_DISABLE   0                   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

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

// 全域變數

float temp, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z = 0.0f;

void MpuSampling_task(void *pvParma){

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  mpu6050.init_mpu6050();

  while(1){
      
      temp = mpu6050.get_mpu6050_temp();
      gyro_x = mpu6050.get_mpu6050_gyro_x();
      gyro_y = mpu6050.get_mpu6050_gyro_y();
      gyro_z = mpu6050.get_mpu6050_gyro_z();

      accel_x = mpu6050.get_mpu6050_accel_x();
      accel_y = mpu6050.get_mpu6050_accel_y();
      accel_z = mpu6050.get_mpu6050_accel_z();

      #ifdef DEBUG_MODE

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

      #endif

      vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}


void xDisplay_task(void *pvParmater){

  vTaskDelay(1500 / portTICK_PERIOD_MS);

  oled.OLED_Init();


  char *ptr_temp;
  char *ptr_gyro_x;
  char *ptr_gyro_y;
  char *ptr_gyro_z;

  char *ptr_accel_x;
  char *ptr_accel_y;
  char *ptr_accel_z;

  
  ptr_temp = (char*)malloc(sizeof(char) * 10); 
  ptr_gyro_x = (char*)malloc(sizeof(char) * 10); 
  ptr_gyro_y = (char*)malloc(sizeof(char) * 10); 
  ptr_gyro_z = (char*)malloc(sizeof(char) * 10); 

  ptr_accel_x = (char*)malloc(sizeof(char) * 10); 
  ptr_accel_y = (char*)malloc(sizeof(char) * 10); 
  ptr_accel_z = (char*)malloc(sizeof(char) * 10); 

  oled.oled_draw_logo(0,0);
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  oled.OLED_Clear();

  oled.oled_print(7, 0, "Gyro", FONT_SIZE_F8X16);
  oled.oled_print(70, 0, "ACCEL", FONT_SIZE_F8X16);
  oled.oled_print(1, 2, "x", FONT_SIZE_F6x8);
  oled.oled_print(1, 3, "y", FONT_SIZE_F6x8);
  oled.oled_print(1, 4, "z", FONT_SIZE_F6x8);
  oled.oled_draw_yaxis(65, 0);
  

    

  for(;;){

    vTaskDelay(10 / portTICK_PERIOD_MS);
    snprintf(ptr_temp, 10,  "%.2lf", temp); 
    snprintf(ptr_gyro_x,10,  "%.2lf", gyro_x); 
    snprintf(ptr_gyro_y, 10, "%.2lf", gyro_y); 
    snprintf(ptr_gyro_z, 10, "%.2lf", gyro_z); 
    snprintf(ptr_accel_x, 10, "%.2lf", accel_x); 
    snprintf(ptr_accel_y, 10, "%.2lf", accel_y); 
    //sprintf(ptr_accel_z, "%.2f", accel_z); 
    snprintf(ptr_accel_z, 10, "%3.2lf", accel_z);
    
    oled.oled_print(16, 2, ptr_gyro_x, FONT_SIZE_F6x8);
    oled.oled_print(73, 2, ptr_accel_x, FONT_SIZE_F6x8);
    oled.oled_print(16, 3, ptr_gyro_y, FONT_SIZE_F6x8);
    oled.oled_print(73, 3, ptr_accel_y, FONT_SIZE_F6x8);
    oled.oled_print(16, 4, ptr_gyro_z, FONT_SIZE_F6x8);
    oled.oled_print(73, 4, ptr_accel_z, FONT_SIZE_F6x8);

    oled.oled_print(40, 7, ptr_temp, FONT_SIZE_F6x8);
    oled.oled_draw_icon(80, 7);


  }

}



void setup() {

  Serial.begin(115200);

  while(i2c_master_init() != ESP_OK){
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }

  xTaskCreate(xDisplay_task, "xDisplay_task", 1024 * 12, NULL, (configMAX_PRIORITIES - 3), NULL);
  xTaskCreate(MpuSampling_task, "MpuSampling_task", 1024 * 12, NULL, (configMAX_PRIORITIES - 3), NULL);
  

}

void loop() {
  
}

