#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned memdep_noalias(unsigned a[], long offset) __attribute__ ((noinline));
unsigned memdep_noalias(unsigned a[], long offset) {
  a[0] = (offset >> 3) + 4;

  unsigned res = a[2] + 2;
  
  return res;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  unsigned a[3];
  a[0] = rand();
  a[1] = rand();
  a[2] = rand();

  unsigned res = memdep_noalias(a, 0);
  printf("result:%d\n", res);

  return 0;
}
