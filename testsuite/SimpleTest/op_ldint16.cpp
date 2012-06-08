#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
signed short op_ldint16(signed short a[], long offest) __attribute__ ((noinline));
signed short op_ldint16(signed short a[], long offest) { return a[offest]; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  signed short a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    a[i] = (signed short) rand();
    signed short res = op_ldint16(a, i);
    printf("result:%d\n", res);
  }

  return 0;
}
