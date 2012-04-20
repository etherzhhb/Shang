#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
void fu_bram_continuous_load(unsigned p[]) __attribute__ ((noinline));
void fu_bram_continuous_load(unsigned p[]) {
  static unsigned a[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  unsigned x0 = a[p[0]] + 2 * a[p[1]] + 4 * a[p[2]] + 8 * a[p[3]] + 16 * a[4] + 32 * a[5] + 64 * a[6] + 64 * a[7];
  p[0] = x0;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);
  unsigned a[32];
  long i;
  long index;
  int x;
  for(i = 0; i < 32; ++i) {
	  a[i] = i;
  }

  fu_bram_continuous_load(a);

  for(i = 0; i < 32; ++i) {
	  printf("%d, result:%d\n", i, a[i]);
  }

  return 0;
}
