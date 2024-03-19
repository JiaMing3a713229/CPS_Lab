#ifndef _NTLIB_H_
#define _NTLIB_H_
#include<stdint.h>

struct NT{
    uint64_t (*powof)(uint64_t b, uint64_t pow, uint64_t mod_size);
    uint64_t (*gcdof)(uint64_t a, uint64_t b, uint64_t mod_size);
    uint64_t (*exgcdof)(uint32_t num, uint32_t mod_size);
};

int nt_init(struct NT *self);
void test(void);


#endif