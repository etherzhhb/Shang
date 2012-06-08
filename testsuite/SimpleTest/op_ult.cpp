#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
int op_ult(unsigned a, unsigned b) __attribute__ ((noinline));
int op_ult(unsigned a, unsigned b) { return a < b; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);
  
  int i;
  for(i = 0; i < 16; ++i) {
    unsigned a = rand();
    unsigned b = rand();
    printf("%x > %x -> result:%d\n", a, b, op_ult(a, b));
  }
  return 0;
}
