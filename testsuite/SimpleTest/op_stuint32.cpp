#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
void op_stuint32(unsigned int a[], long offest, unsigned int v) __attribute__ ((noinline));
void op_stuint32(unsigned int a[], long offest, unsigned int v) {
  a[offest] = v;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  unsigned int a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    unsigned int v = (unsigned int) rand();
    op_stuint32(a, i, v);
    printf("result:%d\n", a[i]);
  }

  return 0;
}
