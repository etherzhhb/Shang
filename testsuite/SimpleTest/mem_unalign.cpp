#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
int mem_unalign(char *a) __attribute__ ((noinline));
int mem_unalign(char *a) { return *(int *)(a + 1); }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);
  
  int i;
  for(i = 0; i < 16; ++i) {
    long long a = rand();
    printf("result:%d\n", mem_unalign((char*)&a));
  }

  return 0;
}
