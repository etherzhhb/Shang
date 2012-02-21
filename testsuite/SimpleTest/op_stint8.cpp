#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
void op_stint8(signed char a[], long offest, signed char v) __attribute__ ((noinline));
void op_stint8(signed char a[], long offest, signed char v) {
  a[offest] = v;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  signed char a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    signed char v = (signed char) rand();
    op_stint8(a, i, v);
    printf("result:%d\n", a[i]);
  }

  return 0;
}
