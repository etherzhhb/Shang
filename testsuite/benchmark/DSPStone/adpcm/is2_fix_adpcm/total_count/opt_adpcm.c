/*
 * opt_adpcm.c
 * Handwritten version of an optimized ADPCM transcoder applying the
 * CCITT recommendation G.721
 *
 * For our purposes the algorithem has been retimed (a few delays have been
 * moved around). But apart from that the algorithem matches the CCITT's.
 * So for further documentation of the source refer to the recommendation.
 *
 * Compile flags:
 * __SIMULATION__ : use to get a file for run56sim
 * __DOWNLOAD__:    use to get a file for the S56X board
 * __LOOPCOUNT__:   use to count loop cycles
 * __MNECOUNT__:    use to count mnemonics
 *
 * 3-NOV-93 Juan Martinez and Chris Schlaeger
 */

#include <stdio.h>

#define FALSE 0
#define TRUE (!FALSE)

FILE* istrm;
FILE* ostrm;
int   ExitFlag;

#ifdef __LOOPCOUNT__
int Loop1 = 0;
int Loop2 = 0;
int Loop3 = 0;
int Loop4 = 0;
int Loop5 = 0;
int Loop6 = 0;
int Loop7 = 0;
#endif

/* Global signals */
int A_2, AL, D, I, SD, S_E, SL, SP, TDP;

typedef struct
{
  int DQ, T, Y, YL, DQ2, DQ3, DQ4, DQ5, DQ6, DQ7, PK1, PK2, SR2,
      A1, A2, B[6], SE, SEZ, t_dr, LAST_TR, DMS, DML, AP, Y_L, LAST_Y;
} STATES;

/* ENCODER states */
STATES E_STATES = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

/* DECODER states */
STATES D_STATES = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

#define LSHIFT(a, b) ((b) < 0 ? (a) << -(b) : (a) >> (b))
#define SIGNBIT(a, b) ((a) & (1 << (b)) ? 1 : 0)
 
int f_mult(int An, int SRn)
{
  register int  EXP;
  register int  WAnMANT;
  int           AnS, MAG, AnMANT;

  AnS = SIGNBIT(An, 15);
  MAG = AnS? (16384 - (An >> 2)) & 8191 : An >> 2;
  {
    register int mag = MAG << 1;

    for (EXP = 0; mag >>= 1; EXP++)
#ifdef __LOOPCOUNT__
      Loop1++;
#else
        ;
#endif
  } 

  AnMANT = MAG ? LSHIFT(MAG, EXP - 6) : 1 << 5;
  EXP += ((SRn >> 6) & 15);
  WAnMANT = (((SRn & 63) * AnMANT) + 48) >> 4;
  MAG = LSHIFT(WAnMANT, 19 - EXP) & 32767;
  return ((SRn >> 10) ^ AnS ? (65536 - MAG) & 65535 : MAG);
}

void adpt_predict(STATES *S)
{
  /*
   * adaptive predictor
   *
   * Input signals:  DQ, T
   * Output signals: S_E, A_2
   */

  register int DQS, DQI, DQSEZ, PK0; 
  register int SR1;
  register int A1S, AP;
  register int A2R;
  register int WA1, WA2;
  register int MAG, EXP, MANT; 
  int BP[6];

  /* ADDC */
  DQS = S->DQ >> 14;
  DQI = DQS ? 65536 - (S->DQ & 16383) : S->DQ;
  MAG = S->SEZ & (1 << 14) ? (1 << 15) + S->SEZ : S->SEZ;
  DQSEZ = (DQI + MAG) & 65535;
  PK0 = DQSEZ >> 15;
  WA2 = DQSEZ ? 0 : 1;
  
  /* ADDB */
  DQI = DQS ? 65536 - (S->DQ & 16383) : S->DQ;
  MAG = S->SE & (1 << 14) ? (1 << 15) + S->SE : S->SE;
  DQSEZ = (DQI + MAG) & 65535;
  
  /* FLOATB */
  MAG = DQSEZ >> 15 ? (65536 - DQSEZ) & 32767 : DQSEZ;
  {
    register int mag = MAG << 1;

    for (EXP = 0; mag >>= 1; EXP++)
#ifdef __LOOPCOUNT__ 
      Loop2++;
#else
      ;
#endif
  }
  MANT = MAG ? (MAG << 6) >> EXP : 1 << 5;
  SR1 = ((DQSEZ >> 15) << 10) + (EXP << 6) + MANT;
  
  /* UPA2 */
  WA1 = PK0 ^ S->PK1;                     /* WA1 = PKS1 */
  MAG = PK0 ^ S->PK2 ? 114688 : 16384;    /* MAG = UGA2A */
  A1S = S->A1 & (1 << 15) ? 1 : 0;
  EXP = A1S ? (S->A1 >= 57345 ? (S->A1 << 2) & 131071 : 24577 << 2) :
              (S->A1 <= 8191 ? S->A1 << 2 : 8191 << 2); 
  EXP = WA1 == 1 ? EXP : (131072 - EXP) & 131071;
  MAG = ((MAG + EXP) & 131071) >> 7;
  EXP = WA2 == 1 ? 0 : (MAG & (1 << 9) ? MAG + 64512 : MAG);
  MANT = S->A2 & (1 << 15) ? 65536 - ((S->A2 >> 7) + 65024) :
                             65536 - (S->A2 >> 7);
  EXP = (S->A2 + EXP + MANT) & 65535;

  /* LIMC */
  AP = 32768 <= EXP && EXP <= 53248 ? 53248 :
       12288 <= EXP && EXP <= 32767 ? 12288 : EXP;
  
  /* TRIGB */
  A2R = S->T ? 0 : AP;
  
  /* UPA1 */
  MANT = A1S ? (65536 - ((S->A1 >> 8) + 65280)):
               (65536 - (S->A1 >> 8));
  EXP = (S->A1 + ((WA2 == 1 ? 0 : (WA1 ? 65344 : 192)) + MANT)) & 65535;
  
  /* FMULT */
  WA2 = f_mult(A2R, S->SR2); 
 
  /* LIMD */
  MAG = (15360 + 65536 - AP) & 65535;
  S->A1 = (AP + 65536 - 15360) & 65535;
  AP = 32768 <= EXP && EXP <=  S->A1 ? S->A1 :
       MAG <= EXP && EXP <= 32767 ? MAG : EXP;
  
  /* TRIGB */
  S->A1 = S->T ? 0 : AP;
  
  /* FMULT */
  WA1 = f_mult(S->A1, SR1);
  
  /* FLOATA */
  MAG = S->DQ & 16383;
  {
    register int mag = MAG << 1;

    for (EXP = 0; mag >>= 1; EXP++)
#ifdef __LOOPCOUNT__ 
      Loop3++;
#else
      ;
#endif
  }
  MANT = MAG ? (MAG << 6) >> EXP : 1 << 5;
  DQSEZ = (DQS << 10) + (EXP << 6) + MANT;
  
  /* XOR */
  BP[0] = DQS ^ (S->DQ2 >> 10);
  BP[1] = DQS ^ (S->DQ3 >> 10);
  BP[2] = DQS ^ (S->DQ4 >> 10);
  BP[3] = DQS ^ (S->DQ5 >> 10);
  BP[4] = DQS ^ (S->DQ6 >> 10);
  BP[5] = DQS ^ (S->DQ7 >> 10);
  
  /* UPB */
  MANT = S->DQ & 16383;
  for (EXP = 0; EXP < 6; EXP++)
  {
#ifdef __LOOPCOUNT__ 
    Loop4++;
#endif
    DQS = MANT ? (BP[EXP] ? 65408 : 128) : 0;
    DQS += S->B[EXP] & (1 << 15) ? (65536 - ((S->B[EXP] >> 8) + 65280)) :
                                   (65536 - (S->B[EXP] >> 8));
    BP[EXP] = (S->B[EXP] + DQS) & 65535;
  }
  
  /* TRIGB */
  if (S->T)
    for (EXP = 0; EXP < 6; EXP++)
#ifdef __LOOPCOUNT__ 
    {
      Loop5++;
      BP[EXP] = 0 ;
    }
#else
      BP[EXP] = 0 ;
#endif
  
  /* FMULT */
  DQI  = f_mult(BP[0], DQSEZ);
  MAG  = f_mult(BP[1], S->DQ2);
  MANT = f_mult(BP[2], S->DQ3);
  A1S  = f_mult(BP[3], S->DQ4);
  AP   = f_mult(BP[4], S->DQ5);
  EXP  = f_mult(BP[5], S->DQ6);
  
  /* ACCUM */
  DQS = (DQI + MAG + MANT + A1S + AP + EXP) & 65535;
  S->SEZ = DQS >> 1;
  S_E = S->SE = ((DQS + WA2 + WA1) & 65535) >> 1;
  
  /* STATE update */
  S->PK2 = S->PK1;
  S->PK1 = PK0;
  S->SR2 = SR1;
  A_2 = S->A2 = A2R;

  for (EXP = 0; EXP < 6; EXP++)
#ifdef __LOOPCOUNT__
  {
    Loop6++;
    S->B[EXP] = BP[EXP];
  }
#else
    S->B[EXP] = BP[EXP];
#endif

  S->DQ7 = S->DQ6;
  S->DQ6 = S->DQ5;
  S->DQ5 = S->DQ4;
  S->DQ4 = S->DQ3;
  S->DQ3 = S->DQ2;
  S->DQ2 = DQSEZ;
}

void iadpt_quant(STATES* S)
{
  /*
   * inverse adaptive quantizer
   * 
   * Input signals:  I, Y
   * Output signals: DQ
   */
 
  static int qtab[] =
  {
    2048, 4, 135, 213, 273, 323, 373, 425,
    425, 373, 323, 273, 213, 135, 4, 2048,
  } ;
  register int DQL;

  /* RECONST and ADDA */
  DQL = (qtab[I] + (S->Y >> 2)) & 4095;

  /* ANTILOG */
  S->DQ = ((I & (1 << 3)) * (1 << 11)) +
          (DQL & (1 << 11) ? 0 :
          (((1 << 7) + (DQL & 127)) << 7) >>
          (14 - ((DQL >> 7) & 15)));
}

void tone_detector(STATES* S)
{
  /*
   * tone detector
   *
   * Input signals:  DQ, A_2, YL
   * Output signals: T, TDP;
   */

  register int tmp; 
  register int t_d; 

  /* partial band signal detection */
  TDP = ((A_2 > 32769) && (A_2 < 53761)) ? 1 : 0 ; 
  
  /* take last (t_dr) */
  t_d = (S->t_dr);
  
  /* calcutale new  (S->t_dr)  */
  /* TRIGB Function */
  S->t_dr = S->LAST_TR ? 0 : TDP; 
  
  S->LAST_TR = S->YL >> 15; /* (*LAST_TR)  is used here as a temporary variable */
  
  tmp = ((S->LAST_TR) > 8) ? 31 << 9 :
        (32 + ((S->LAST_TR << 5) & 31)) << S->LAST_TR; 
  
  S->LAST_TR = S->T = (((S->DQ & 16383) > (tmp + (tmp >> 1)) >> 1) 
	              && (t_d == 1)) ? 1 : 0;
}

void speed_control(STATES* S)
{
  /*
   * speed control
   *
   * Input signals:  T, TDP, I, Y
   * Output signals: AL
   */

  static int F[] = { 0, 0, 0, 1, 1, 1, 3, 7 };
  register int FI, tmp ; 

  /* FUNTCF */
  FI = F[(I >> 3 ? (15 - I) : I) & 7] ; 

  /* FILTA */
  tmp = ((FI << 9) + 8192 - S->DMS) & 8191;              /* tmp => DIF */
  tmp >>= 5;
  if (tmp & (1 << 7))
    tmp = tmp + 3840;                                    /* tmp => DIFSX */
  S->DMS = (tmp + S->DMS) & 4095;

  /* FILTB */
  tmp = ((FI << 11) + 32768 - S->DML) & 32767;           /* tmp => DIF */
  tmp >>= 7;
  if (tmp & (1 << 7))
    tmp = tmp + 16128;                                   /* tmp => DIFSX */
  S->DML = (tmp + S->DML) & 16383;

  /* SUBTC */
  tmp = ((S->DMS << 2) + 32768 - S->DML) & 32767;        /* tmp => DIF */
  if (tmp & (1 << 14))
    tmp = (32768 - tmp) & 16383;                         /* tmp => DIFM */
  FI = ((S->Y > 1535) && (tmp < (S->DML >> 3)) && (TDP == 0)) ? 0 : 1;

  /* FILTC */
  tmp = ((FI << 9) + 2048 - S->AP) & 2047;               /* tmp => DIF */ 
  tmp = (tmp >> 4) + (tmp >> 10 ? 896 : 0);              /* tmp => DIFSX */
  tmp = (tmp + S->AP) & 1023;                            /* tmp => APP */

  /* TRIGA */
  S->AP = S->T ? 256 : tmp; 

  /* LIMA */
  AL = (S->AP > 255) ? 64 : S->AP >> 2; 
}

void scale_factor(STATES* S)
{
  /*
   * scale factor adaptation
   *
   * Input signals:  AL, I
   * Output signals: YL, Y
   */

  static int W[] = 
  {
    4084, 18, 41, 64, 112, 198, 255, 1122
  } ; 
  register int TMP, YUP, YLP;
 
  /* FUNCTW */
  TMP = W[(I >> 3) ? (15 - I) & 7 : I & 7];
  
  /* FILTD */
  TMP = ((TMP << 5) + 131072 - S->LAST_Y) & 131071;
  TMP >>= 5;
  YUP = (TMP >> 11) ? TMP + 4096 : TMP;
  YUP = (YUP + S->LAST_Y) & 8191;
  
  /* LIMB */
  if ((((YUP + 11264) & 16383) >> 13) == 0)
    YUP = 5120;
  if ((((YUP + 15840) & 16383) >> 13))
    YUP = 544;
  
  /* FILTE */
  TMP = (YUP + ((1048576 - S->Y_L) >> 6)) & 16383;
  if (TMP & (1 << 13))
    TMP = TMP + 507904;
  YLP = (TMP + S->Y_L) & 524287;
  
  /* MIX */
  TMP = (YUP + 16384 - (S->Y_L >> 6)) & 16383;
  S->LAST_Y = TMP & (1 << 13) ? 1 : 0;
  if (S->LAST_Y)
    TMP = (16384 - TMP) & 8191; 
  TMP = (TMP * AL) >> 6;
  if (S->LAST_Y)
    TMP = (16384 - TMP) & 16383;
  
  S->LAST_Y = S->Y = ((S->Y_L >> 6) + TMP) & 8191; 
  S->YL = S->Y_L; 
  S->Y_L = YLP;
}

#ifndef __MNECOUNT__

#ifdef __DSP56K__

void init_disk_play()
{
}

#ifdef __SIMULATION__
#include "data_in.c"
#endif

void disk_play()
{
#ifdef __SIMULATION__

  static int i = 0;

  if (i >= sizeof(Data_in) / sizeof(int))
    ExitFlag = TRUE;

  SL = Data_in[i++] & 0x3FFF;

#else

  ExitFlag = TRUE;
  SL = 0;

#endif
}

void post_disk_play()
{
}

void init_disk_record()
{
}

void disk_record()
{
}

void post_disk_record()
{
}

#else

void init_disk_play()
{
  istrm = fopen("data.in", "rb");
}

void disk_play()
{
  int d;

  if (istrm != NULL && !feof(istrm))
  {
    fread((char*) &d, sizeof(int), 1, istrm);
    SL = (d & 0x3FFF0000) >> 16;
  }
  if (feof(istrm))
    ExitFlag = TRUE;
}

void post_disk_play()
{
  if (istrm != NULL)
    fclose(istrm);
}

void init_disk_record()
{
  ostrm = fopen("data.out", "wb");
}

void disk_record()
{
  int d;

  if (ostrm != NULL)
  {
    d = (SD & 0x0000FFFF) << 16;
    fwrite((char*) &d, 4, 1, ostrm);
  }
}

void post_disk_record()
{
  if (ostrm != NULL)
    fclose(ostrm);
}

#endif
#endif

main()
{
#ifndef __MNECOUNT__
#ifdef __DOWNLOAD__
  run56k_start();
#endif

  init_disk_play();
  init_disk_record();

  ExitFlag = FALSE;
  while (!ExitFlag)
  {
#endif
    disk_play();
    if (ExitFlag)
      break;

    /*
     * ENCODER
     */
    adpt_predict(&E_STATES);
    {
      /* SUBTA (inlined) */
      D = ((SL & (1 << 13) ? 49152 | SL : SL) + 65536 -
          (S_E & (1 << 14) ? 32768 | S_E : S_E)) & 65535;
    }
    {
      /* adaptive quantiziser (inlined) */
      register int DS, DQM, DL, DLN, EXP;

      /* LOG */
      DS = D & (1 << 15);
      DQM = DS ? (65536 - D) & 32767 : D;
      {
	register int dqm = DQM;

	for (EXP = 1; dqm >>= 1; EXP++)
#ifdef __LOOPCOUNT__
	  Loop7++;
#else
	  ;
#endif
      } 
      EXP--;
      DL = (EXP * (1 << 7)) + (((DQM * (1 << 7)) >> EXP) & 127);

      /* SUBTB */
      DLN = (DL + 4096 - (E_STATES.Y >> 2)) & 4095;

      /* QUAN */
      if (DLN > 3971)
	I = DS ? 0xE : 0x1;
      else if (DLN > 2047)
	I = 0xF;
      else if (DLN > 399)
	I = DS ? 0x8 : 0x7;
      else if (DLN > 348)
	I = DS ? 0x9 : 0x6;
      else if (DLN > 299)
	I = DS ? 0xA : 0x5;	
      else if (DLN > 245)
	I = DS ? 0xB : 0x4;
      else if (DLN > 177)
	I = DS ? 0xC : 0x3;
      else if (DLN > 79)
	I = DS ? 0xD : 0x2;
      else
	I = DS ? 0xE : 0x1;
    }
    iadpt_quant(&E_STATES);
    tone_detector(&E_STATES);
    speed_control(&E_STATES);
    scale_factor(&E_STATES);

    /*
     * DECODER
     */
    adpt_predict(&D_STATES);
    iadpt_quant(&D_STATES);
    {
      /* ADD (inlined) */
      register int dq, se;

      /* 15 SM auf 16 TC */
      dq = (D_STATES.DQ & (1 << 14)) ? D_STATES.DQ == (1 << 14) ? 0: 
           (((D_STATES.DQ ^ 0x3FFF) + 0x1) & 0x3FFF) + 0xC000 : D_STATES.DQ ; 

      /* 15 TC auf 16 TC */
      se = S_E & (1 << 14) ? 0x8000 | S_E : S_E ; 
  
      /* perform add operation at 16 TC */
      SP  = (dq + se) & 0xFFFF;
    }
    {
      /* coding_adjector (inlined) */
      /* Just passing a SP (16TC) through */
      /* rest of the signals is ignored */
      SD = SP;
    }
    tone_detector(&D_STATES);
    speed_control(&D_STATES);
    scale_factor(&D_STATES);
#ifndef __MNECOUNT__
    disk_record();
  }

  post_disk_play();
  post_disk_record();

#ifdef __LOOPCOUNT__
  printf("\nLoop1: %d", Loop1);
  printf("\nLoop2: %d", Loop2);
  printf("\nLoop3: %d", Loop3);
  printf("\nLoop4: %d", Loop4);
  printf("\nLoop5: %d", Loop5);
  printf("\nLoop6: %d", Loop6);
  printf("\nLoop7: %d", Loop7);
#endif

#ifdef __DOWNLOAD__
  run56k_end();
#endif
  return (0);
#endif
} 










