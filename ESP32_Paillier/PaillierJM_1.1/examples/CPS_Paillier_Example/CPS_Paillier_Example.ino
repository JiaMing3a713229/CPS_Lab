#include "PaillierJM.h"
Paillier paillier(29,31,5652);
uint32_t gcd(uint32_t a, uint32_t b){   //gcd(a,b)=> gcd(b,a mod b)
  if(b == 0){
    return a;
  }
  uint32_t c = a % b;
  return gcd(b, c);
}
// L_function = L(x) = (x-1)/ n
uint64_t L_fuction(uint64_t u, uint64_t n){
  return ((u-1) / n);
}
uint64_t MODEXP(uint64_t a, uint64_t n, uint64_t m){
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

uint64_t InverseModular(uint64_t k,uint64_t n){
  uint64_t r = exgcd(n, k, n, 0, 1);
  return r;
}

uint64_t CRT(uint64_t m1, uint64_t m2, uint64_t p, uint64_t q){
  uint64_t M = p * q;
  uint64_t M2 = p;
  uint64_t M2_Inverse = InverseModular(M2, q);
  uint64_t crt_r_1 = (((((((m2 -m1) + M) % M) * M2_Inverse) * M2) % M) + m1) % M;
  return crt_r_1;
}

/*
API  EncryptionPaillier(uint64_t n, uint64_t g, uint64_t m)
parameter:  public key(n,g), m is message
return encrypted numeric
*/
uint64_t EncryptionPaillier(uint64_t n, uint64_t g, uint64_t m){
  uint64_t r = 23;
  uint64_t n2 = (n*n);
  return ((1 + n*m) * MODEXP(r,n,n2)) % (n2);
}
/*
API  DecryptionPaillier(uint64_t hp, uint64_t hq, uint64_t c, uint64_t p, uint64_t q)
hp = kp = L_fuction(MODEXP(g, (p-1), p*p), p), hq = kq = L_fuction(MODEXP(g, (q-1), q*q), q);
parameter:  private key(p, q), c is encrypted numeric
return original numeric
*/
uint64_t DecryptionPaillier(uint64_t hp, uint64_t hq, uint64_t c, uint64_t p, uint64_t q){

  uint64_t mp = (L_fuction(MODEXP(c, (p-1), (p*p)), p)* hp) % p;
  uint64_t mq = (L_fuction(MODEXP(c, (q-1), (q*q)), q)* hq) % q;
  // Serial.printf("m1:%d,", mp);
  // Serial.printf("m2:%d, \r\n", mq);
  uint64_t m = CRT(mp, mq, p, q);
  return m;
}



void setup() {

  Serial.begin(115200);
  uint64_t p = 29;
  uint64_t q = 31;
  uint64_t n = p * q;
  uint64_t g = 5652;
  uint16_t m = 0;

  // uint64_t kp = L_fuction(MODEXP(g, (p-1), p*p), p);
  // uint64_t hp = InverseModular(kp, p);
  // uint64_t kq = L_fuction(MODEXP(g, (q-1), q*q), q);
  // uint64_t hq = InverseModular(kq, q);

  vTaskDelay(3000 / portTICK_PERIOD_MS);
  // Serial.printf("hp:%d", hp);
  // Serial.printf("hq:%d \r\n", hq);

  
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
