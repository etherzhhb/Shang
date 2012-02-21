h23000
s 00001/00001/00114
d D 1.2 94/05/05 11:40:13 schlaege 2 1
c sa version
e
s 00115/00000/00000
d D 1.1 94/03/24 11:29:48 dsp 1 0
c date and time created 94/03/24 11:29:48 by dsp
e
u
U
f e 0
t
T
I 1
/*
 * itu_decoder.c
 * Test evironment for g721.c decoder with ITU test sequences
 *
 * 16-Mar-94 Chris Schlaeger
 */

#include <stdio.h>
#include <stdlib.h>
#include "portab.h"

#include "g721.h"

#define FALSE 0
#define TRUE (!FALSE)

int ExitFlag = 0;
char* I_file;
char* O_file;

U16BIT I, O;

void get_sample()
{
  static FILE* istrm = NULL;
  static int   csum = 0, next;
  int          i;

  if (istrm == NULL)
  {
    if ((istrm = fopen(I_file, "rb")) == NULL)
    {
      fprintf(stderr, "\nERROR: Cannot open input file %s!", I_file);
      ExitFlag = 1;
    }
    fscanf(istrm, "%2x", &next);
  }

  if (fscanf(istrm, "%2x\r\n", &i) != 1)
  {
    ExitFlag = 1;
    fclose(istrm);
    if ((csum % 255) != next)
      fprintf(stderr, "Checksum error in input file");
    return;
  }
  I = (U16BIT) next;
  csum += next;
  next = i;
}

void put_sample(int state)
{
  static FILE* ostrm = NULL;
  static int column_count = 0, csum = 0;
  
  switch (state)
  {
  case 1:
    if (ostrm == NULL)
    {
      if ((ostrm = fopen(O_file, "wb")) == NULL)
	fprintf(stderr, "\nERROR: Cannot open output file %s!", O_file);
    }
    break;
  case 2:
    if (ostrm != NULL)
    {
      csum += (unsigned int) O;
      fprintf(ostrm, "%02x", O);
      if (column_count++ >= 31)
      {
	column_count = 0;
	fprintf(ostrm, "\r\n");
      }
    }
    break;
  case 3:
    if (ostrm != NULL)
    {
      fprintf(ostrm, "%02x\r\n\032", csum % 255);
      fclose(ostrm);
      break;
    }
  }
}

D 2
main(int argc, char* argv[])
E 2
I 2
int main(int argc, char* argv[])
E 2
{
  if (argc != 4)
  {
    fprintf(stderr, "\nUsage: itu_decoder -[a|u] <ADPCM-File> <PCM-File>");
    exit(-1);
  }
  I_file = argv[2];
  O_file = argv[3];

  put_sample(1);

  reset_decoder();
  LAW = argv[1][1] == 'u' ? u_LAW : A_LAW;

  ExitFlag = FALSE;
  while (!ExitFlag)
  {
    get_sample();
    if (ExitFlag)
      break;
    O = decoder(I);
    put_sample(2);
  }

  put_sample(3);
  return (0);
}
E 1
