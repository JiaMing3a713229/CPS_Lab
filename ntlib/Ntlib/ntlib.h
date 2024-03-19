#ifndef _NTLIB_H_
#define _NTLIB_H_
#include<stdint.h>

struct RSA{
    struct Params{
        uint64_t p, q, n, e, d, lambda_n;
    }params;
    uint64_t (*Encrypt)(struct RSA *rsa, uint32_t message);
    uint64_t (*Decrypt)(struct RSA *rsa, uint64_t cipher);
};

int rsa_init(struct RSA *rsa, uint64_t p, uint64_t q, uint64_t e);

#endif