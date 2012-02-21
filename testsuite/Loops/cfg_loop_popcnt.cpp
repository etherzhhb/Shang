#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned cfg_loop_popcnt(unsigned n) __attribute__ ((noinline));
unsigned cfg_loop_popcnt(unsigned n) {
 unsigned int sum = 0, i; 
  for (i = 0; i < 32; i++) {
    sum += (n) & 1; 
    n /= 2; 
  } 
  return sum; 
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  unsigned i;
  for(i = 0; i < 16; ++i) {
    unsigned n = (unsigned) rand();
    unsigned res = cfg_loop_popcnt(n);
    printf("popcount(%x):%d\n", n, res);
  }

  return 0;
}
