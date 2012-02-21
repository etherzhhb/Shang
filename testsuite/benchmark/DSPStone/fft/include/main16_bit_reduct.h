
//#ifdef __PROF56__
//#include <stdio.h>
//#endif

#define STORAGE_CLASS register
#define TYPE int

#define N_FFT 16
#define NUMBER_OF_BITS 13    /* fract format 1.NUMBER_OF_BITS = 1.13 */

#define BITS_PER_TWID 13     /* bits per twiddle coefficient */
#define SHIFT BITS_PER_TWID  /* fractional shift after each multiplication */

#ifdef __PROF56__
  volatile extern __time;
  volatile int t;
#include "debug.c"
#endif


