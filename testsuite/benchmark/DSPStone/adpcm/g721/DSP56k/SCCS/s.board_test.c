h52492
s 00030/00000/00000
d D 1.1 94/04/20 14:52:08 dsp 1 0
c date and time created 94/04/20 14:52:08 by dsp
e
u
U
f e 0
t
T
I 1
#include <stdio.h>
#include "g721.h"

U16BIT Input[] = 
{
  0xff, 0xa7, 0x19, 0x92, 0x0f, 0x91, 0x18, 0xa3,
  0x4d, 0x2b, 0x9a, 0x13, 0x8f, 0x11, 0x96, 0x20,
  0xbe, 0xaf, 0x1c, 0x94, 0x10, 0x90, 0x15, 0x9e,
  0x36, 0x36, 0x9e, 0x15, 0x90, 0x10, 0x94, 0x1c
} ;

extern int __time;

int main(void)
{
  int i, sic, eic;
  U16BIT I;

  reset_encoder();
  reset_decoder();

  sic = __time / 2;
  for (i = 0; i < sizeof(Input) / sizeof(U16BIT); i++)
    decoder(encoder(Input[i]));
  eic = __time / 2;
  printf("\nInstruction cycles for transcoding a sample: %d\n",
	 (eic - sic) / (sizeof(Input) / sizeof(U16BIT)));

  return (0);
}
E 1
