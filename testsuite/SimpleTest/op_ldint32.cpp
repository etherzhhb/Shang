#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
signed int op_ldint32(signed int a[], long offest) __attribute__ ((noinline));
signed int op_ldint32(signed int a[], long offest) { return a[offest]; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  signed int a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    a[i] = (signed int) rand();
    signed int res = op_ldint32(a, i);
    printf("result:%d\n", res);
  }

  return 0;
}
