#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
int op_shift(int a) __attribute__ ((noinline));
int op_shift(int a) { return a >> 4; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);
  
  int i;
  for(i = 0; i < 16; ++i) {
    int a = rand();
    printf("%x  -> result:%d\n", a, op_shift(a));
  }
  return 0;
}
