#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
void cfg_loop_pipe(unsigned a[], unsigned b) __attribute__ ((noinline));
void cfg_loop_pipe(unsigned a[], unsigned b) {
  long i;
  for (i = 0; i < 16; ++i)
   a[i] = (a[i + 2] << (b & 0x4)) + b;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  unsigned a[18];

  long i;
  for(i = 0; i < 18; ++i)
    a[i] = (unsigned) rand();

  cfg_loop_pipe(a, 2);
  
  for(i = 0; i < 18; ++i)
    printf("a[%d]:%d\n", i, a[i]);

  return 0;
}
