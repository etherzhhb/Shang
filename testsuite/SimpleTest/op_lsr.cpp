#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned op_lsr(unsigned a, unsigned b) __attribute__ ((noinline));
unsigned op_lsr(unsigned a, unsigned b) { return a >> b; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);
  
  int i;
  for(i = 0; i < 16; ++i)
    printf("result:%d\n", op_lsr(rand(), rand()));

  return 0;
}
