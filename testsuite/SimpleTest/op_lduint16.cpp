#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned short op_lduint16(unsigned short a[], long offest) __attribute__ ((noinline));
unsigned short op_lduint16(unsigned short a[], long offest) { return a[offest]; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  unsigned short a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    a[i] = (unsigned short) rand();
    unsigned short res = op_lduint16(a, i);
    printf("result:%d\n", res);
  }

  return 0;
}
