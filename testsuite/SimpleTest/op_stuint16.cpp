#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
void op_stuint16(unsigned short a[], long offest, unsigned short v) __attribute__ ((noinline));
void op_stuint16(unsigned short a[], long offest, unsigned short v) {
  a[offest] = v;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  unsigned short a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    unsigned short v = (unsigned short) rand();
    op_stuint16(a, i, v);
    printf("result:%d\n", a[i]);
  }

  return 0;
}
