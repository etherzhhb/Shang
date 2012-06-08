/*
 * Module for Overflow and CArry Detection
 * Use this module to simulate 16 or 24 Bit processors on a Workstation.
 *
 * Last modification: 24-Mar-94 Chris Schlaeger
 *
 * Beware: The ?: operator cannot be used in the usual way! (See portab.h)
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
} ;

class U16BIT
{
private:
  unsigned long value;
  unsigned long check(unsigned long);
public:
  U16BIT(long v = 0) : value(check(v)) {}
  operator unsigned long&() { return value; }
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
} ;

#endif
