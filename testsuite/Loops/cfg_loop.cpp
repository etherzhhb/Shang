#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#define N 30

#ifdef __cplusplus
extern "C" {
#endif
void cfg_loop(unsigned a[]) __attribute__ ((noinline));
void cfg_loop(unsigned a[]) {
  long i;
  for (i = 0; i < N; ++i)
   a[i] = a[i + 2] + 3;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  unsigned a[N + 2];

  long i;
  for(i = 0; i < N + 2; ++i)
    a[i] = (unsigned) rand();

  cfg_loop(a);
  
  for(i = 0; i < N + 2; ++i)
    printf("a[%d]:%d\n", i, a[i]);

  return 0;
}
