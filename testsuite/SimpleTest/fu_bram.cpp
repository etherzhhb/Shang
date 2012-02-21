#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
unsigned fu_bram(long offest) __attribute__ ((noinline));
unsigned fu_bram(long offest) {
  unsigned a[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  return a[offest & 0x7];
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  long i;
  long index;
  int x;
  for(i = 0; i < 16; ++i) {
	index = rand();
	x = i & 0x7;
    printf("%d, %d, result:%d\n", i, x, fu_bram(i));
  }

  return 0;
}
