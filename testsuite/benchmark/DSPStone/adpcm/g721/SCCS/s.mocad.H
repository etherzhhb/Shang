h43035
s 00031/00002/00056
d D 1.2 94/05/05 11:39:11 schlaege 2 1
c sa version
e
s 00058/00000/00000
d D 1.1 94/03/24 11:30:42 dsp 1 0
c date and time created 94/03/24 11:30:42 by dsp
e
u
U
f e 0
t
T
I 1
/*
 * Module for Overflow and CArry Detection
 * Use this module to simulate 16 or 24 Bit processors on a Workstation.
 *
 * Last modification: 24-Mar-94 Chris Schlaeger
 *
D 2
 * Beware: The ?: operator cannot be used in the usual way! (See portab.h)
E 2
I 2
 * Beware: The ?: operator cannot always be used in the usual way!
 * (See portab.h (IF_ELSE))
E 2
 */

#ifndef __MOCAD_H__
#define __MOCAD_H__

#include <stream.h>

#undef IF_ELSE
#define IF_ELSE(a,b,c) ((a) ? (unsigned long) (b) : (unsigned long) (c))

class S16BIT
{
private:
  long value;
  long check(long);
public:
  S16BIT(long v = 0) : value(check(v)) {}
  operator long&() { return value; }
I 2

  unsigned long operator<<(long v)
  {
    if (v < 0 || v > 16)
      cerr << "\n S16BIT bad shift width";
    if (value & (0xFFFFL << (16 - v)))
      cerr << "\n S16BIT data lost during shift!";
    return value << v;
  }
  unsigned long operator<<(int v) { return *this << (long) v; }
E 2
} ;

class U16BIT
{
private:
  unsigned long value;
  unsigned long check(unsigned long);
public:
D 2
  U16BIT(long v = 0) : value(check(v)) {}
E 2
I 2
  U16BIT(unsigned long v = 0) : value(check(v)) {}
E 2
  operator unsigned long&() { return value; }
I 2

  unsigned long operator<<(long v)
  {
    if (v < 0 || v > 16)
      cerr << "\n U16BIT bad shift width";
    if (value & (0xFFFFL << (16 - v)))
      cerr << "\n U16BIT data lost during shift!";
    return value << v;
  }
  unsigned long operator<<(int v) { return *this << (long) v; }
E 2
} ;

class S24BIT
{
private:
  long value;
  long check(long);
public:
  S24BIT(long v = 0) : value(check(v)) {}
  operator long&() { return value; }
} ;

class U24BIT
{
private:
  unsigned long value;
  unsigned long check(unsigned long);
public:
  U24BIT(unsigned long v = 0) : value(check(v)) {}
  operator unsigned long&() { return value; }
I 2

  unsigned long operator<<(long v)
  {
    if (value & (0xFFFFFFL << (24 - v)))
      cerr << "\n U24BIT data lost during shift!";
    return value << v;
  }
  unsigned long operator<<(int v) { return *this << (long) v; }
E 2
} ;

#endif
E 1
