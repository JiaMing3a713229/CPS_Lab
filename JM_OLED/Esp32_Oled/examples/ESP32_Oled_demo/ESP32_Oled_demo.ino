#include <Arduino.h>
#include "JM_ssd1306.h"
#include <driver/i2c.h>

JM_ssd1306 Oled(0x3c);

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

void oled_display_task(void *pvParameters){

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Oled.OLED_Init();
  Oled.oled_draw_logo(0,0);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  Oled.Clear();

  for(;;){

    Oled.print(16, 2, "Hello World", FONT_SIZE_F6x8);
    /* Paramater 1 : x座標位置 (0 - 128)
    *  Paramater 2 : y座標位置 (0-7)
    *  Paramater 3 : "顯示字串資料"
    *  Paramater 4 : 字體大小
    */

  }
}

void setup() {

  Serial.begin(115200);

  while(i2c_master_init() != ESP_OK){
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }

  //Create a threads to handle oled task 
  xTaskCreate(oled_display_task, "oled_display_task", 1024 * 5, NULL, (configMAX_PRIORITIES - 3), NULL);
  
}

void loop() {
  
}

