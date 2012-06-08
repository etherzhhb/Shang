#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include <string.h>

unsigned a[128];
unsigned b[128];

#ifdef __cplusplus
extern "C" {
#endif
void sim_memcpy_libcall_opt(unsigned char v) __attribute__ ((noinline));
void sim_memcpy_libcall_opt(unsigned char v) {
  int i;
  for (i = 0; i < sizeof(a); ++i)
    b[i] = a[i];
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  long i;
  for(i = 0; i < 128; ++i)
    a[i] = i;

  sim_memcpy_libcall_opt(128);

  for(i = 0; i < 128; ++i)
    printf("b[%d]:%d\n", i, b[i]);

  return 0;
}
