#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned int op_lduint32(unsigned int a[], long offest) __attribute__ ((noinline));
unsigned int op_lduint32(unsigned int a[], long offest) { return a[offest]; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  unsigned int a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    a[i] = (unsigned int) rand();
    unsigned int res = op_lduint32(a, i);
    printf("result:%d\n", res);
  }

  return 0;
}
