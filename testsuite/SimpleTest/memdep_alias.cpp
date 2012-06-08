#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned memdep_alias(unsigned a[], long offset) __attribute__ ((noinline));
unsigned memdep_alias(unsigned a[], long offset) {
  a[0] = (offset >> 3) + 4;

  unsigned res = a[offset] + 2;
  
  return res;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  unsigned a[1];
  a[0] = rand();

  unsigned res = memdep_alias(a, 0);
  printf("result:%d\n", res);

  return 0;
}
