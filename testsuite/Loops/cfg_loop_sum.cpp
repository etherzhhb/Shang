#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned cfg_loop_sum(unsigned a[], unsigned n) __attribute__ ((noinline));
unsigned cfg_loop_sum(unsigned a[], unsigned n) {
  long i;
  unsigned r = 0;
  for (i = 0; i < n; ++i)
   r += a[i];
   
  return r;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  unsigned N = 5;
  unsigned a[N];

  long i;
  for(i = 0; i < N; ++i)
    a[i] = (unsigned) rand();

  unsigned r = cfg_loop_sum(a, N);
  
  printf("result:%d\n", r);

  return 0;
}
