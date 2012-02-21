#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
void schedule_fu_conflict(unsigned a[]) __attribute__ ((noinline));
void schedule_fu_conflict(unsigned a[]) {
  a[2] = 0;
  a[1] = 1;
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
  
  schedule_fu_conflict(a);
  
  printf("result:%d,%d\n", a[1], a[2]);

  return 0;
}
