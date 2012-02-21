#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
signed char op_continuous_ldint8(signed char a[]) __attribute__ ((noinline));
signed char op_continuous_ldint8(signed char a[]) {
  return a[0] + a[1] + a[2] + a[3] + a[4] + a[5] + a[6] + a[1] + a[8] + a[9] + a[10];
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  signed char a[32];

  long i;
  for(i = 0; i < 32; ++i) {
    a[i] = (signed char) rand();
  }

  signed char res = op_continuous_ldint8(a);
  printf("result:%d\n", res);
  return 0;
}
