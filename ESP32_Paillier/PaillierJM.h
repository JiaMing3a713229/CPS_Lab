#ifndef PAILLIERJM_H
#define PAILLIERJM_H

#include "Arduino.h"

class Paillier{
  
  public:
    Paillier(uint32_t p, uint32_t q, uint32_t g);
    uint32_t gcd(uint32_t, uint32_t);
    uint64_t L_fuction(uint64_t , uint64_t );
    uint64_t MODEXP(uint64_t , uint64_t , uint64_t );
    uint64_t exgcd(uint64_t a, uint64_t b, uint64_t a1, uint64_t t1, uint64_t t2);
    uint64_t InverseModular(uint64_t k,uint64_t n);
    uint64_t CRT(uint64_t m1, uint64_t m2, uint64_t p, uint64_t q);
    // uint64_t EncryptionPaillier(uint64_t n, uint64_t g, uint64_t m);
    uint64_t EncryptionPaillier(uint64_t m);
    uint64_t DecryptionPaillier(uint64_t , uint64_t , uint64_t );
    // uint64_t DecryptionPaillier(uint64_t c, uint64_t p, uint64_t q);
    String EncryptionPaillier2Str(uint64_t m);
    String DecryptionPaillier2str(String , uint64_t , uint64_t );

  private:
    uint64_t p;
    uint64_t q;
    uint64_t n;
    uint64_t g;
    uint64_t kp; 
    uint64_t hp;
    uint64_t kq;
    uint64_t hq;

    
};

  

#endif