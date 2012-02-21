#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#define N 32

#ifdef __cplusplus
extern "C" {
#endif
void cfg_loop_two_array(unsigned a[], unsigned b[], long n) __attribute__ ((noinline));
void cfg_loop_two_array(unsigned a[], unsigned b[], long n) {
  long i;
  for(i=2; i < n; ++i){
    a[i] =b[i-2] + 1;
    b[i] =a[i-2] + 2;
  }
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  unsigned a[N];
  unsigned b[N];

  long i;
  for(i = 0; i < N; ++i) {
    a[i] = (unsigned) rand();
    b[i] = (unsigned) rand();
  }

  cfg_loop_two_array(a, b, N);

  for(i = 0; i < N; ++i)
    printf("a[%d]:%d,b[%d]:%d\n", i, a[i], i, b[i]);

  return 0;
}
