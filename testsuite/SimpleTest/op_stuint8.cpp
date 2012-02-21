#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
void op_stuint8(unsigned char a[], long offest, unsigned char v) __attribute__ ((noinline));
void op_stuint8(unsigned char a[], long offest, unsigned char v) {
  a[offest] = v;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  unsigned char a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    unsigned char v = (unsigned char) rand();
    op_stuint8(a, i, v);
    printf("result:%d\n", a[i]);
  }

  return 0;
}
