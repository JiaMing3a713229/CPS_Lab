#include "ntlib.h"
#include <stdio.h>


static uint64_t gcdof(uint64_t a, uint64_t b)
{
    if(a > b){
        a ^= b;
        b ^= a;
        a ^= b;
    }
    return (a == 0)? b : gcdof(b % a, a);
}

static uint64_t powof(uint64_t b, uint64_t pow, uint64_t mod_size)
{
    int ret = 1;
    while(pow != 0){
        if((pow & 0x01) == 1){
            ret *= b;
        }
        pow >>= 1;
        b = (b * b) % mod_size;
    }
    return ret % mod_size;
}

static inline uint64_t lcmof(uint64_t a, uint64_t b)
{
    return (a * b) / gcdof(a, b);
}

static uint64_t exgcd(uint32_t mod_size, uint32_t r, uint32_t d1, uint32_t d2)
{
    int q = mod_size / r;
    return (r == 1)? d2 : exgcd(r, (mod_size % r), d2, (d1 - q * d2));
}

static inline uint64_t exgcdof(uint32_t num, uint32_t mod_size)
{   
    int ret = 0;
    if(gcdof(num, mod_size) == 1){
        ret = (gcdof(num, mod_size) == 1) ? exgcd(mod_size, num, 0, 1) : 0;
    }
    return (ret < 0)? (ret+ mod_size): ret;
    
}

void test(void)
{
    printf("%llu \r\n", gcdof(65254, 2104));
    printf("pow: %llu \r\n", powof(7, 50, 71));
    printf("exgcd of %d \r\n", exgcdof(65, 71));
    
}