#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include <string.h>

unsigned a[128];
unsigned b=1;
#ifdef __cplusplus
extern "C" {
#endif
unsigned sim_memset_complex(unsigned char v,unsigned * pointer) __attribute__ ((noinline));
unsigned sim_memset_complex(unsigned char v,unsigned * pointer) {
  memset(a, v, sizeof(a));
  unsigned c = *pointer;
  return c;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  long i;
  unsigned * pointer =&b;
  for(i = 0; i < 128; ++i){
    unsigned m = sim_memset_complex(i, pointer);
    printf("a[%d]:%d\n", i, a[i]);
  }

  return 0;
}
