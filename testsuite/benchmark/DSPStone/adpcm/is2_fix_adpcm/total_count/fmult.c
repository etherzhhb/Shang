
#define LSHIFT(a, b) ((b) < 0 ? (a) << -(b) : (a) >> (b))
#define SIGNBIT(a, b) ((a) & (1 << (b)) ? 1 : 0)
 
int f_mult(int An, int SRn)
{
  register int  EXP;
  register int  WAnMANT;
  int           AnS, MAG, AnMANT;

  AnS = SIGNBIT(An, 15);
  MAG = AnS? (16384 - (An >> 2)) & 8191 : An >> 2;
/*  {
    register int mag = MAG << 1;
 
    for (EXP = 0; mag >>= 1; EXP++)
        ;
  } */

  AnMANT = MAG ? LSHIFT(MAG, EXP - 6) : 1 << 5;
  EXP += ((SRn >> 6) & 15);
  WAnMANT = (((SRn & 63) * AnMANT) + 48) >> 4;
  MAG = LSHIFT(WAnMANT, 19 - EXP) & 32767;
  return ((SRn >> 10) ^ AnS ? (65536 - MAG) & 65535 : MAG);
}
