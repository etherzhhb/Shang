#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
int cfg_nested_if_else(int zSign, int float_rounding_mode) __attribute__ ((noinline));
int cfg_nested_if_else(int zSign, int float_rounding_mode) {
  int roundingMode;
  int roundNearestEven;
  int roundIncrement;

  roundingMode = float_rounding_mode;
  roundNearestEven = (roundingMode == 0);
  roundIncrement = 0x200;
  if (!roundNearestEven)
    {
      if (roundingMode == 1)
	{
	  roundIncrement = 0;
	}
      else
	{
	  roundIncrement = 0x3FF;
	  if (zSign)
	    {
	      if (roundingMode == 2)
		roundIncrement = 0;
	    }
	  else
	    {
	      if (roundingMode == 3)
		roundIncrement = 0;
	    }
	}
    }

 return roundIncrement;
}
#ifdef __cplusplus
}
#endif

int main(int argc, char **argv) {
  srand (16);

  long i;
  for(i = 0; i < 16; ++i) {
	int x = rand();
	int y = rand();
    printf("%d, %d, result:%d\n", x, y, cfg_nested_if_else(x, y));
  }

  return 0;
}
