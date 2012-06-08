h26491
s 00002/00001/00036
d D 1.3 94/05/25 13:57:03 schlaege 3 2
c uPD77016 added
e
s 00005/00007/00032
d D 1.2 94/05/05 11:39:25 schlaege 2 1
c sa version
e
s 00039/00000/00000
d D 1.1 94/03/24 11:30:37 dsp 1 0
c date and time created 94/03/24 11:30:37 by dsp
e
u
U
f e 0
t
T
I 1
#ifndef __PORTAB_H__
#define __PORTAB_H__

#define IF_ELSE(a,b,c) ((a) ? (b) : (c))

D 2
#if defined(__G56K__)
E 2
I 2
#if defined(__DSP5600x__)
E 2

#define S16BIT int
#define U16BIT unsigned int
#define S24BIT int
#define U24BIT unsigned int
#define S32BIT long
#define U32BIT unsigned long

D 2
#elif defined(__G21__) || defined(_TMS320C50)
E 2
I 2
#elif defined(__ADSP2101__) || defined(__TMS320C50__) || \
D 3
      defined(__DSP16xx__)  || defined(__DSP56156__)
E 3
I 3
      defined(__DSP16xx__)  || defined(__DSP56156__) || \
      defined(__uPD77016__)
E 3
E 2

#define S16BIT int
#define U16BIT unsigned int
#define S24BIT long
#define U24BIT unsigned long
#define S32BIT long
#define U32BIT unsigned long

#else

D 2
/*
E 2
#define S16BIT short
#define U16BIT unsigned short
#define S24BIT int
#define U24BIT unsigned int
#define S32BIT int
#define U32BIT unsigned int
I 2
/*
#include "mocad.H"  // use this with ObjectCenter only!
E 2
*/
D 2
#include "mocad.H"

E 2
#endif
#endif
D 2


E 2
E 1
