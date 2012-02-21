#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include <string.h>

unsigned a[128];

#ifdef __cplusplus
extern "C" {
#endif
void sim_memset(unsigned char v) __attribute__ ((noinline));
void sim_memset(unsigned char v) {
  memset(a, v, sizeof(a));
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  long i;
  sim_memset(128);

  for(i = 0; i < 128; ++i)
    printf("a[%d]:%d\n", i, a[i]);

  return 0;
}
