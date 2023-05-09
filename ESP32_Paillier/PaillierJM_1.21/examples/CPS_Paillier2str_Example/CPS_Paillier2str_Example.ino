#include "PaillierJM.h"

uint64_t p = 29;
uint64_t q = 31;

Paillier paillier(p, q);


void setup() {

  Serial.begin(115200);
  
  uint64_t n = p * q;
  uint64_t g = 5652;
  uint16_t m = 0;

 

  vTaskDelay(3000 / portTICK_PERIOD_MS);
 
  
  while(m<256){
    Serial.println("-----------------------------------------------------");
    Serial.printf("原始資料m:%d,", m);
    // uint64_t c = paillier.EncryptionPaillier(m);
    String c = paillier.EncryptionPaillier2Str(m);
    Serial.printf("密文c1:%s,", c.c_str());
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // uint64_t De_m  = paillier.DecryptionPaillier(c,p,q);
    String De_m = paillier.DecryptionPaillier2str(c , p , q);
    Serial.print("解密後明文:");
    Serial.println(De_m);
    // Serial.printf(" ,err: %d \r\n", (De_m - m));
    m++;
    vTaskDelay(1 / portTICK_PERIOD_MS);
    Serial.println("-----------------------------------------------------");
  }

}
void loop() {
}
