#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
int sub_module(int a, int b) __attribute__ ((noinline));
int sub_module(int a, int b) { return a - b; }
int sub_module_non_memory_access(int a, int b) __attribute__ ((noinline));
int sub_module_non_memory_access(int a, int b) { return (a + b) * sub_module(a, b); }
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  int i;
  for(i = 0; i < 16; ++i) {
    int a = rand();
    int b = rand();
    printf("result:%d\n", sub_module_non_memory_access(a, b));
  }

  return 0;
}
