#include "Arduino.h"
#include "PaillierJM.h"

Paillier::Paillier(uint32_t p, uint32_t q){

  p = p;
  q = q;
  n = p * q;
  g = (n+1);
  kp = L_fuction(MODEXP(g, (p-1), p*p), p);
  hp = InverseModular(kp, p);
  kq = L_fuction(MODEXP(g, (q-1), q*q), q);
  hq = InverseModular(kq, q);
}

uint64_t Paillier::MODEXP(uint64_t a, uint64_t n, uint64_t m){
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

uint32_t Paillier::gcd(uint32_t a, uint32_t b){   //gcd(a,b)=> gcd(b,a mod b)
  if(b == 0){
    return a;
  }
  uint32_t c = a % b;
  return gcd(b, c);
}

uint64_t Paillier::L_fuction(uint64_t u, uint64_t n){
  return ((u-1) / n);
}



uint64_t Paillier::exgcd(uint64_t a, uint64_t b, uint64_t a1, uint64_t t1, uint64_t t2){
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

uint64_t Paillier::InverseModular(uint64_t k,uint64_t n){
  uint64_t r = exgcd(n, k, n, 0, 1);
  return r;
}

uint64_t Paillier::CRT(uint64_t m1, uint64_t m2, uint64_t p, uint64_t q){
  uint64_t M = p * q;
  uint64_t M2 = p;
  uint64_t M2_Inverse = InverseModular(M2, q);
  uint64_t crt_r_1 = (((((((m2 -m1) + M) % M) * M2_Inverse) * M2) % M) + m1) % M;
  return crt_r_1;
}


// uint64_t Paillier::EncryptionPaillier(uint64_t n, uint64_t g, uint64_t m){
//   uint64_t r = 23;
//   uint64_t n2 = (n*n);
//   return ((1 + n*m) * MODEXP(r,n,n2)) % (n2);
// }


uint64_t Paillier::EncryptionPaillier(uint64_t m){
  uint64_t r = 23;
  uint64_t n2 = (n*n);
  return ((1 + n*m) * MODEXP(r,n,n2)) % (n2);
}
uint64_t Paillier::DecryptionPaillier(uint64_t c, uint64_t p, uint64_t q){

  uint64_t mp = (L_fuction(MODEXP(c, (p-1), (p*p)), p)* hp) % p;
  uint64_t mq = (L_fuction(MODEXP(c, (q-1), (q*q)), q)* hq) % q;
  // Serial.printf("m1:%d,", mp);
  // Serial.printf("m2:%d, \r\n", mq);
  uint64_t m = CRT(mp, mq, p, q);
  return m;
}

String Paillier::EncryptionPaillier2Str(uint64_t m){
  uint64_t t = (millis() / 100) % (p);
  uint64_t n2 = (n*n);
  // uint64_t r = (1 + n*t) % (n);
  return String(((1 + n*m) * MODEXP(t,n,n2)) % (n2));
  // return String(r);
}

String Paillier::DecryptionPaillier2str(String c, uint64_t p, uint64_t q){
  uint64_t cipher = uint64_t(atoi(c.c_str()));
  uint64_t mp = (L_fuction(MODEXP(cipher, (p-1), (p*p)), p)* hp) % p;
  uint64_t mq = (L_fuction(MODEXP(cipher, (q-1), (q*q)), q)* hq) % q;
  // Serial.printf("m1:%d,", mp);
  // Serial.printf("m2:%d, \r\n", mq);
  uint64_t m = CRT(mp, mq, p, q);
  return String(m);
  // return String(cipher);
}