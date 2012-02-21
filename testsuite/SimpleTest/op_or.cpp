#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
int op_or(int a, int b) __attribute__ ((noinline));
int op_or(int a, int b) { return a | b; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);
  
  int i;
  for(i = 0; i < 16; ++i)
    printf("result:%d\n", op_or(rand(), rand()));

  return 0;
}
