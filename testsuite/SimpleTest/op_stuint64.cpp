#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
void op_stuint64(unsigned long a[], long offest, unsigned long v) __attribute__ ((noinline));
void op_stuint64(unsigned long a[], long offest, unsigned long v) {
  a[offest] = v;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  unsigned long a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    unsigned long v = (unsigned long) rand();
    op_stuint64(a, i, v);
    printf("result:%d\n", a[i]);
  }

  return 0;
}
