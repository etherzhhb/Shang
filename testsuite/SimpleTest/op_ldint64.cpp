#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
signed long op_ldint64(signed long a[], long offest) __attribute__ ((noinline));
signed long op_ldint64(signed long a[], long offest) { return a[offest]; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  signed long a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    a[i] = (signed long) rand();
    signed long res = op_ldint64(a, i);
    printf("result:%d\n", res);
  }

  return 0;
}
