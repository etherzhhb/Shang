#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned implicit_global_code_motion_delay_insertion_0(unsigned *p)
  __attribute__ ((noinline));
unsigned implicit_global_code_motion_delay_insertion_0(unsigned *p) {
  unsigned a = 4, b = 8;

  b = p[a];

  if (b > 0xf) p[a] = b;

  return p[b];
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);
  unsigned A[16];

  long i;

  for (i = 0; i < 16; ++i)
    A[i] = i;

  for(i = 0; i < 16; ++i) {
    unsigned res = implicit_global_code_motion_delay_insertion_0(A);
    printf("result:%d\n", res);
  }

  return 0;
}
