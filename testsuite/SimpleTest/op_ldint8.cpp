#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
signed char op_ldint8(signed char a[], long offest) __attribute__ ((noinline));
signed char op_ldint8(signed char a[], long offest) { return a[offest]; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  signed char a[16];

  long i;
  for(i = 0; i < 16; ++i) {
    a[i] = (signed char) rand();
    signed char res = op_ldint8(a, i);
    printf("result:%x\n", res);
  }

  return 0;
}
