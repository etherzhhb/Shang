#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
void cfg_loop_vb(unsigned a[], unsigned n) __attribute__ ((noinline));
void cfg_loop_vb(unsigned a[], unsigned n) {
  long i;
  for (i = 0; i < n; ++i)
   a[i] = a[i + 2] + 3;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  unsigned a[16];

  long i;
  for(i = 0; i < 16; ++i)
    a[i] = (unsigned) i;

  cfg_loop_vb(a, 8);
  
  for(i = 0; i < 16; ++i)
    printf("a[%d]:%d\n", i, a[i]);

  return 0;
}
