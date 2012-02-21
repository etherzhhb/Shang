#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

unsigned GV1 = 11;
#ifdef __cplusplus
extern "C" {
#endif
void GV_promotion(unsigned a[]) __attribute__ ((noinline));
void GV_promotion(unsigned a[]) {
  a[2] = 0;
  a[1] = 1;
  a[0] = GV1;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  unsigned a[3];
  a[0] = rand();
  a[1] = rand();
  a[2] = rand();
  
  GV_promotion(a);
  
  printf("result:%d,%d,%d\n", a[0], a[1], a[2]);

  return 0;
}
