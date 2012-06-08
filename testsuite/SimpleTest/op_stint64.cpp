#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
void op_stint64(signed long a[], long offest, signed long v) __attribute__ ((noinline));
void op_stint64(signed long a[], long offest, signed long v) {
  a[offest] = v;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  signed long a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    signed long v = (signed long) rand();
    op_stint64(a, i, v);
    printf("result:%d\n", a[i]);
  }

  return 0;
}
