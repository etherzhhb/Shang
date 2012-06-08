#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include <string.h>

unsigned a[128];
unsigned b[128];

#ifdef __cplusplus
extern "C" {
#endif
void sim_memcpy(unsigned char v) __attribute__ ((noinline));
void sim_memcpy(unsigned char v) {
  memcpy(a, b, sizeof(a));
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  long i;
  for(i = 0; i < 128; ++i)
    a[i] = i;

  sim_memcpy(128);

  for(i = 0; i < 128; ++i)
    printf("a[%d]:%d\n", i, b[i]);

  return 0;
}
