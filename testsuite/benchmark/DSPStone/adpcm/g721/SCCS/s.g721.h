h24893
s 00017/00000/00000
d D 1.1 94/03/24 11:30:37 dsp 1 0
c date and time created 94/03/24 11:30:37 by dsp
e
u
U
f e 0
t
T
I 1
#ifndef __G721_H__
#define __G721_H__

#include "portab.h"

#define u_LAW 0
#define A_LAW 1

extern int LAW;

void reset_encoder(void);
U16BIT encoder(U16BIT);

void reset_decoder(void);
U16BIT decoder(U16BIT);

#endif
E 1
