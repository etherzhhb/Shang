#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
void op_stint16(signed short a[], long offest, signed short v) __attribute__ ((noinline));
void op_stint16(signed short a[], long offest, signed short v) {
  a[offest] = v;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  signed short a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    signed short v = (signed short) rand();
    op_stint16(a, i, v);
    printf("result:%d\n", a[i]);
  }

  return 0;
}
