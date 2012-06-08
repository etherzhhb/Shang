#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
int op_shl(long long int a, long long int b) __attribute__ ((noinline));
int op_shl(long long int a, long long int b) { return a << b; }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);
  
  int i;
  for(i = 0; i < 16; ++i) {
    long long int a = rand();
    long long int b = rand();
    printf("%x >>> %x -> result:%d\n", a, b, op_shl(a, b));
  }
  return 0;
}
