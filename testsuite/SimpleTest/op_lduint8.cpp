#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned char op_lduint8(unsigned char a[], long offest) __attribute__ ((noinline));
unsigned char op_lduint8(unsigned char a[], long offest) { return a[offest]; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  unsigned char a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    a[i] = (unsigned char) rand();
    unsigned char res = op_lduint8(a, i);
    printf("result:%d\n", res);
  }

  return 0;
}
