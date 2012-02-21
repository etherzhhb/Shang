#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned long op_lduint64(unsigned long a[], long offest) __attribute__ ((noinline));
unsigned long op_lduint64(unsigned long a[], long offest) { return a[offest]; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  unsigned long a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    a[i] = (unsigned long) rand();
    unsigned long res = op_lduint64(a, i);
    printf("result:%d\n", res);
  }

  return 0;
}
