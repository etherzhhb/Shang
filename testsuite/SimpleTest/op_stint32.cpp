#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
void op_stint32(signed int a[], long offest, signed int v) __attribute__ ((noinline));
void op_stint32(signed int a[], long offest, signed int v) {
  a[offest] = v;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  signed int a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    signed int v = (signed int) rand();
    op_stint32(a, i, v);
    printf("result:%d\n", a[i]);
  }

  return 0;
}
