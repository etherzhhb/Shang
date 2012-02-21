#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned op_sel(unsigned a, unsigned  b, unsigned c) __attribute__ ((noinline));
unsigned op_sel(unsigned a, unsigned  b, unsigned c) {
  return a ? b : c;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  long i;
  for(i = 0; i < 16; ++i)
    printf("result:%d\n", op_sel(rand(), 0, 0xffff));

  return 0;
}
