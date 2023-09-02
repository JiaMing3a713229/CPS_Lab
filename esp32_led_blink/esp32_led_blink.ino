#include <Arduino.h>
#include "driver/gpio.h"


#define LED1_IO 2
#define LED1_IO_PIN (1ULL<<2)

uint8_t led_mutex  = 0;
uint8_t led_status = 0;
void setup() {

  Serial.begin(115200);
  printf("\n-----init gpio_driver------\n");

  gpio_config_t led_conf;                          
  led_conf.pin_bit_mask = LED1_IO_PIN;             /* GPIO設置腳位      */
  led_conf.mode         = GPIO_MODE_OUTPUT;        /* GPIO為輸出模式    */
  led_conf.intr_type    = GPIO_INTR_DISABLE;       /* GPIO中斷模式 關閉 */
  led_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;   /* GPIO下拉電阻 關閉 */
  led_conf.pull_up_en   = GPIO_PULLUP_DISABLE;     /* GPIO上拉電阻 關閉 */
  
  if(gpio_config(&led_conf) == ESP_OK){           /*  gpio_config(const gpio_config_t *pGPIOConfig) */
    printf("ESP32_GPIO_INIT_OK\n");
    led_mutex = 0x01;
  }
  else{
    printf("ESP32_GPIO_INIT_FAIL\n");
  }

  
}

void loop() {

  if(led_mutex){
    gpio_set_level((gpio_num_t)LED1_IO, (led_status ^= 0x01));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  

}
