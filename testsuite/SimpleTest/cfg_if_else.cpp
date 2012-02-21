#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned cfg_if_else(unsigned a, unsigned b) __attribute__ ((noinline));
unsigned cfg_if_else(unsigned a, unsigned b) {
  if (a > 0xf)
    a += b + 1;
  else
    b -= 1;

  return a * 2 - b;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  long i;
  for(i = 0; i < 16; ++i) {
    unsigned a = (unsigned ) rand();
    unsigned b = (unsigned ) rand();
    unsigned res = cfg_if_else(a, b);
    printf("result:%d\n", res);
  }

  return 0;
}
