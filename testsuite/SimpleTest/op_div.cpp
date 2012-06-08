#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned op_div(unsigned a, unsigned b) __attribute__ ((noinline));
unsigned op_div(unsigned a, unsigned b) { return a / b; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  int i;
  for(i = 0; i < 16; ++i) {
    unsigned a = rand();
    unsigned b = rand();
	if (b == 0) b = 1;
    printf("result:%d\n", op_div(a, b));
  }

  return 0;
}
