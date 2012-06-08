#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include <string.h>

unsigned a[128];
unsigned b[128];
unsigned c[128];
unsigned d = 1;
#ifdef __cplusplus
extern "C" {
#endif
unsigned sim_memcpy_complex(unsigned char v, unsigned * pointer) __attribute__ ((noinline));
unsigned sim_memcpy_complex(unsigned char v, unsigned * pointer) {
  memcpy(a, b, sizeof(a));
  memcpy(c, b, sizeof(c));
  unsigned d = *pointer;
  return d;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  long i;
  for(i = 0; i < 128; ++i)
    b[i] = i;    
  unsigned *pointer =&d;
  //sim_memcpy(128);

  for(i = 0; i < 128; ++i){
    unsigned m = sim_memcpy_complex(i, pointer);
    printf("a[%d]:%d\n", i, a[i]);
    printf("b[%d]:%d\n", i, b[i]);
  }
  return 0;
}
