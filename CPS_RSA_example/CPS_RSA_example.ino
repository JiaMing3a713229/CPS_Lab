#include <Arduino.h>

uint64_t ModularEXP(uint64_t a, uint64_t n, uint64_t m){  //return a^n (mod m)
  a = a % m;  
  uint64_t r = 1;
  while(n!=0){
    if((n & 1) == 1){
      r = (a % m)*(r % m) % m;
    }
    a = (a % m)*(a % m) % m ;
    n = n >> 1;
  }
  return r;
}
//擴展歐幾里得求模反元素的演算法  
uint64_t exgcd(uint64_t a, uint64_t b, uint64_t a1, uint64_t t1, uint64_t t2){
  if(b==0){
    return (t1 + a1) % a1;
  }
  uint64_t Q = int(a / b);
  uint64_t R = a % b;
  uint64_t t = t1 - (t2 * Q);
  t1 = t2;
  t2 = t;
  a = b;
  b = R;
  return exgcd(a, b, a1, t1, t2);
}
//求模反元素  
uint64_t InverseModular(uint64_t d,uint64_t lambda_n){
  uint64_t r = exgcd(lambda_n, d, lambda_n, 0, 1);
  return r;
}

// RSA Encryption:E(m)=m^e mod n,where e is gcd(e,lambda_n)=1,uint32_t m is message.
uint64_t Encrypt_RSA(uint32_t e, uint32_t n, uint32_t message){
  uint64_t cipher = ModularEXP(message, e, n);                              // m^e (mod n)
  return cipher;
}

// RSA Decryption:D(c)=m = c^d mod n
uint64_t Decrypt_RSA(uint64_t d, uint64_t cipher, uint64_t n){
  uint64_t plaintext = ModularEXP(cipher, d, n);
  return plaintext;
}

const uint64_t key_p = 29;
const uint64_t key_q = 31;
const uint64_t n = key_p * key_q;
const uint64_t e = 113;
const uint64_t lambda_n = (key_p-1) * (key_q-1);
const uint64_t d = InverseModular(e, lambda_n);
int message = 25;

void setup() {
  Serial.begin(115200);
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  
  
  uint64_t cipher = Encrypt_RSA(e, n, message);
  uint64_t plaintext = Decrypt_RSA(d, cipher, n);

  Serial.printf("message:%d", message);
  Serial.printf(" ,cipher:%d", cipher);
  Serial.printf(" ,decrypted message:%d \r\n", plaintext);
  

}

void loop() {
  
}