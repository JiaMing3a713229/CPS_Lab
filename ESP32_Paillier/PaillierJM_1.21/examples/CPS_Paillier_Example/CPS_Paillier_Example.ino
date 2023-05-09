#include "PaillierJM.h"


uint64_t p = 29;
uint64_t q = 31;

Paillier paillier(29,31);



void setup() {

  Serial.begin(115200);

  uint16_t m = 0;
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  
  while(m<256){
    Serial.println("-----------------------------------------------------");
    Serial.printf("原始資料m:%d,", m);
    uint64_t c = paillier.EncryptionPaillier(m);
    Serial.printf("密文c1:%d,", c);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uint64_t De_m  = paillier.DecryptionPaillier(c,p,q);
    Serial.print("解密後明文:");
    Serial.print(De_m);
    Serial.printf(" ,err: %d \r\n", (De_m - m));
    m++;
    vTaskDelay(1 / portTICK_PERIOD_MS);
    Serial.println("-----------------------------------------------------");
  }

}
void loop() {
}
