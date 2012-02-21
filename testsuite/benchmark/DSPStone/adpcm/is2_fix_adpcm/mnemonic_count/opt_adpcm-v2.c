/*
 * opt_adpcm.c
 * Handwritten version of an optimized ADPCM transcoder applying the
 * CCITT recommendation G.721
 * 26-Oct-93 Chris Schlaeger
 * VERSION 1.2
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

/* ENCODER states */
int E_DQ = 0;
int E_T = 0;
int E_Y = 0;
int E_YL = 0;
int E_DQ2 = 0;
int E_DQ3 = 0;
int E_DQ4 = 0;
int E_DQ5 = 0;
int E_DQ6 = 0;
int E_DQ7 = 0;
int E_PK1 = 0;
int E_PK2 = 0;
int E_SR2 = 0;
int E_A1 = 0;
int E_A2 = 0;
int E_B[6] = { 0, 0, 0, 0, 0, 0 };
int E_SE = 0;
int E_SEZ = 0;
int E_t_dr = 0;
int E_LAST_TR = 0;
int E_DMS = 0;
int E_DML = 0;
int E_AP = 0;
int E_Y_L = 0;
int E_LAST_Y = 0;

/* DECODER states */
int D_DQ = 0;
int D_T = 0;
int D_Y = 0;
int D_YL = 0;
int D_DQ2 = 0;
int D_DQ3 = 0;
int D_DQ4 = 0;
int D_DQ5 = 0;
int D_DQ6 = 0;
int D_DQ7 = 0;
int D_PK1 = 0;
int D_PK2 = 0;
int D_SR2 = 0;
int D_A1 = 0;
int D_A2 = 0;
int D_B[6] = { 0, 0, 0, 0, 0, 0 };
int D_SE = 0;
int D_SEZ = 0;
int D_t_dr = 0;
int D_LAST_TR = 0;
int D_DMS = 0;
int D_DML = 0;
int D_AP = 0;
int D_Y_L = 0;
int D_LAST_Y = 0;

int f_mult(int An, int SRn)
{
  int           AnS, MAG, AnMANT;
  register int  EXP;
  register long WAnMANT;

  AnS = An >> 15;
  MAG = AnS ? (16384 - (An >> 2)) & 8191 : An >> 2;
    for (EXP = 0; (MAG >> EXP) != 0; EXP++)
#ifdef __LOOPCOUNT__
	Loop1++;
#else
	;
#endif

  AnMANT = MAG ? (MAG << 6) >> EXP : 1 << 5;

  EXP += ((SRn >> 6) & 15);
  WAnMANT = ((((long) (SRn & 63) * AnMANT) + 48) >> 4) << 7;
  MAG = (int) (EXP <= 26 ?
               WAnMANT  >> (26 - EXP) :
	       (WAnMANT << (EXP - 26)) & 32767);
  return ((SRn >> 10) ^ AnS ? (65536 - MAG) & 65535 : MAG);
}

void diff_signal(void)
{
  /*
   * diff_signal
   *
   * Input signals:  SL, S_E
   * Output signals: D
   */

  /* SUBTA */
  D = ((SL & (1 << 13) ? 49152 | SL : SL) + 65536 -
       (S_E & (1 << 14) ? 32768 | S_E : S_E)) & 65535;
}

void adpt_predict(int* DQ, int* T,
                  int* DQ2, int* DQ3, int* DQ4, int* DQ5, int* DQ6, int* DQ7,
                  int* PK1, int* PK2, int* SR2, int* A1, int* A2, int B[],
                  int* SE, int* SEZ)
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
  DQS = *DQ >> 14;
  DQI = DQS == 0 ? *DQ : (65536 - (*DQ & 16383)) & 65535;
  MAG = (( (*SEZ)  >> 14) == 0) ?  (*SEZ)  : (1 << 15) +  (*SEZ);
  DQSEZ = (DQI + MAG) & 65535;
  PK0 = DQSEZ >> 15;
  WA2 = DQSEZ == 0 ? 1 : 0;
  
  /* ADDB */
  DQI = DQS == 0 ? *DQ : (65536 - (*DQ & 16383)) & 65535;
  MAG = (( (*SE)  >> 14) == 0) ?  (*SE)  : (1 << 15) +  (*SE) ;
  DQSEZ = (DQI + MAG) & 65535;
  
  /* FLOATB */
  MAG = ((DQSEZ >> 15) == 0) ? DQSEZ : (65536 - DQSEZ) & 32767;
  for (EXP = 0; (MAG >> EXP) != 0; EXP++)
#ifdef __LOOPCOUNT__ 
    Loop2++;
#else
    ;
#endif
  MANT = MAG == 0 ? 1 << 5 : (MAG << 6) >> EXP;
  SR1 = ( (DQSEZ >> 15) << 10)  + (EXP << 6) + MANT;
  
  /* UPA2 */
  WA1 = PK0 ^  (*PK1) ;
  MAG = (PK0 ^  (*PK2) ) == 0 ? 16384 : 114688;
  A1S =  (*A1)  >> 15;
  if (A1S == 0)
    EXP =  (*A1)  <= 8191 ?  (*A1)  << 2 : 8191 << 2;
  else
    EXP =  (*A1)  >= 57345 ? ( (*A1)  << 2) & 131071 : 24577 << 2;
  EXP = WA1 == 1 ? EXP : (131072 - EXP) & 131071;
  MAG = (MAG + EXP) & 131071;
  EXP = WA2 == 1 ? 0 :
    ( (MAG >> 16) == 0)  ? MAG >> 7 : (MAG >> 7) + 64512;
  MANT = ( (*A2)  >> 15) == 0 ? (65536 - ( (*A2)  >> 7)) & 65535 :
    (65536 - (( (*A2)  >> 7) + 65024)) & 65535;
  EXP = (EXP + MANT) & 65535;
  EXP = ( (*A2)  + EXP) & 65535;
  
  /* LIMC */
  AP = 32768 <= EXP && EXP <= 53248 ? 53248 :
    12288 <= EXP && EXP <= 32767 ? 12288 : EXP;
  
  /* TRIGB */
  A2R = *T == 0 ? AP : 0;
  
  /* UPA1 */
  EXP = WA2 == 1 ? 0 : (WA1 == 0 ? 192 : 65344);
  MANT = A1S == 0 ? (65536 - ( (*A1)  >> 8)) & 65535 :
    (65536 - (( (*A1)  >> 8) + 65280)) & 65535;
  EXP = (EXP + MANT) & 65535;
  EXP = ( (*A1)  + EXP) & 65535;
  
  /* FMULT */
  WA2 = f_mult(A2R,  (*SR2) ) ; 
 
  /* LIMD */
  MAG = (15360 + 65536 - AP) & 65535;
   (*A1)  = (AP + 65536 - 15360) & 65535;
  AP = 32768 <= EXP && EXP <=  (*A1)  ?  (*A1)  :
    MAG <= EXP && EXP <= 32767 ? MAG : EXP;
  
  /* TRIGB */
   (*A1)  = *T == 0 ? AP : 0;
  
  /* FMULT */
  WA1 = f_mult( (*A1) , SR1);
  
  /* FLOATA */
  MAG = *DQ & 16383;
  for (EXP = 0; (MAG >> EXP) != 0; EXP++)
#ifdef __LOOPCOUNT__ 
    Loop3++;
#else
    ;
#endif
  MANT = MAG == 0 ? 1 << 5 : (MAG << 6) >> EXP;
  DQSEZ = (DQS << 10) + (EXP << 6) + MANT;
  
  /* XOR */
  BP[0] = DQS ^ ( (*DQ2)  >> 10);
  BP[1] = DQS ^ ( (*DQ3)  >> 10);
  BP[2] = DQS ^ ( (*DQ4)  >> 10);
  BP[3] = DQS ^ ( (*DQ5)  >> 10);
  BP[4] = DQS ^ ( (*DQ6)  >> 10);
  BP[5] = DQS ^ ( (*DQ7)  >> 10);
  
  /* UPB */
  MANT = *DQ & 16383;
  for (EXP = 0; EXP < 6; EXP++)
    {
#ifdef __LOOPCOUNT__ 
      Loop4++;
#endif
      DQS = MANT == 0 ? 0 : (BP[EXP] == 0 ? 128 : 65408);
      MAG = B[EXP] >> 15;
      DQS += MAG == 0 ? (65536 - (B[EXP] >> 8)) & 65535 :
	(65536 - ((B[EXP] >> 8) + 65280)) & 65535;
      DQS &= 65535;
      BP[EXP] = (B[EXP] + DQS) & 65535;
    }
  
  /* TRIGB */
  if (*T != 0)
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
  MAG  = f_mult(BP[1], (*DQ2));
  MANT = f_mult(BP[2], (*DQ3));
  A1S  = f_mult(BP[3], (*DQ4));
  AP   = f_mult(BP[4], (*DQ5));
  EXP  = f_mult(BP[5], (*DQ6));
  
  /* ACCUM */
  DQS = (DQI + MAG + MANT + A1S + AP + EXP) & 65535;
  (*SEZ)  = DQS >> 1;
  S_E =  (*SE)  = ((DQS + WA2 + WA1) & 65535 ) >> 1;
  
  /* STATE update */
  (*PK2) = (*PK1);
  (*PK1) = PK0;
  (*SR2) = SR1;
  A_2 = (*A2) = A2R;

  for (EXP = 0; EXP < 6; EXP++)
#ifdef __LOOPCOUNT__
  {
    Loop6++;
    B[EXP] = BP[EXP];
  }
#else
    B[EXP] = BP[EXP];
#endif

  (*DQ7) = (*DQ6);
  (*DQ6) = (*DQ5);
  (*DQ5) = (*DQ4);
  (*DQ4) = (*DQ3);
  (*DQ3) = (*DQ2);
  (*DQ2) = DQSEZ;
}

void coding_adj(void)
{
  /*
   * coding_adjustment
   *
   * Input signals:  I, SP, S_E, Y
   * Output signals: SD
   */

  /* Just passing a SP (16TC) through */
  /* rest of the signals is ignored */
  SD = SP;
}

void iadpt_quant(int* DQ, int* Y)
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
  DQL = (qtab[I] + (*Y >> 2)) & 4095;

  /* ANTILOG */
  *DQ = ((I & (1 << 3)) * (1 << 11)) +
        (DQL & (1 << 11) ? 0 :
        (((1 << 7) + (DQL & 127)) << 7) >>
        (14 - ((DQL >> 7) & 15)));
}

void adpt_quant(int* Y)
{
  /*
   * adaptive quantizer
   *
   * Input signals:  D, Y
   * Output signals: I
   */

  register int DS, DQM, DL, DLN, EXP;

  /* LOG */
  DS = D & (1 << 15);
  DQM = DS ? (65536 - D) & 32767 : D;
  for (EXP = 1; DQM >> EXP; EXP++)
#ifdef __LOOPCOUNT__
    Loop7++;
#else
    ;
#endif
  EXP--;
  DL = (EXP * (1 << 7)) + (((DQM * (1 << 7)) >> EXP) & 127);

  /* SUBTB */
  DLN = (DL + 4096 - (*Y >> 2)) & 4095;

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

void tone_detector(int* DQ, int* T, int* YL, int* t_dr, int* LAST_TR)
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
  t_d = (*t_dr);
  
  /* calcutale new  (*t_dr)  */
  /* TRIGB Function */
  *t_dr = (*LAST_TR == 0) ? TDP : 0; 
  
  *LAST_TR = *YL >> 15; /* (*LAST_TR)  is used here as a temporary variable */
  
  tmp = ((*LAST_TR) > 8) ? 31 << 9 :
        (32 + ((*LAST_TR << 5) & 31)) << *LAST_TR; 
  
  *LAST_TR = *T = (((*DQ & 16383) > (tmp + (tmp >> 1)) >> 1) 
		 && (t_d == 1)) ? 1 : 0;
}

void speed_control(int* T, int* Y, int* DMS, int* DML, int* AP)
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
  tmp = ((FI << 9) + 8192 - *DMS) & 8191;              /* tmp => DIF */
  tmp = (tmp >> 12) ? (tmp >> 5) + 3840 : (tmp >> 5);  /* tmp => DIFSX */
  *DMS = (tmp + *DMS) & 4095;

  /* FILTB */
  tmp = ((FI << 11) + 32768 - *DML) & 32767;           /* tmp => DIF */
  tmp = (tmp >> 14) ? (tmp >> 7) + 16128 : (tmp >> 7); /* tmp => DIFSX */
  *DML = (tmp + *DML) & 16383;

  /* SUBTC */
  tmp = ((*DMS << 2) + 32768 - *DML) & 32767;          /* tmp => DIF */
  tmp = (tmp >> 14) ? (32768 - tmp) & 16383 : tmp;     /* tmp => DIFM */
  FI = ((*Y > 1535) && (tmp < (*DML >> 3)) && (TDP == 0)) ? 0 : 1;

  /* FILTC */
  tmp = ((FI << 9) + 2048 - *AP) & 2047;               /* tmp => DIF */ 
  tmp = (tmp >> 4) + (tmp >> 10 ? 896 : 0);            /* tmp => DIFSX */
  tmp = (tmp + *AP) & 1023;                            /* tmp => APP */

  /* TRIGA */
  *AP = *T ? 256 : tmp; 

  /* LIMA */
  AL = (*AP > 255) ? 64 : *AP >> 2; 
}

void scale_factor(int* Y, int* YL, int* Y_L, int* LAST_Y)
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
  TMP = ((TMP << 5) + 131072 - *LAST_Y) & 131071;
  YUP = (TMP >> 16) ? (TMP >> 5) + 4096 : (TMP >> 5);
  YUP = (YUP + *LAST_Y) & 8191;
  
  /* LIMB */
  if ((((YUP + 11264) & 16383) >> 13) == 0)
    YUP = 5120;
  if ((((YUP + 15840) & 16383) >> 13))
    YUP = 544;
  
  /* FILTE */
  TMP = (YUP + ((1048576 - *Y_L) >> 6)) & 16383;
  TMP = (TMP >> 13) ? TMP + 507904 : TMP;
  YLP = (TMP + *Y_L) & 524287;
  
  /* MIX */
  TMP = (YUP + 16384 - (*Y_L >> 6)) & 16383;
  *LAST_Y = TMP >> 13;
  TMP = *LAST_Y ? (16384 - TMP) & 8191 : TMP; 
  TMP = (TMP * AL) >> 6;
  TMP = *LAST_Y ? (16384 - TMP) & 16383 : TMP;
  
  *LAST_Y = *Y = ((*Y_L >> 6) + TMP) & 8191; 
  *YL = *Y_L; 
  *Y_L = YLP;
}

void add(int* DQ)
{
  /*
   * add
   *
   * Input signals:  DQ, SE
   * Output signals: SP
   */

  int dq, se;

  /* 15 SM auf 16 TC */
  dq = (*DQ & (1 << 14)) ? *DQ == (1 << 14) ? 0: 
       (((*DQ ^ 0x3FFF) + 0x1) & 0x3FFF) + 0xC000 : *DQ ; 

  /* 15 TC auf 16 TC */
  se = S_E & (1 << 14) ? 0x8000 | S_E : S_E ; 
  
  /* perform add operation at 16 TC */
  SP  = (dq + se) & 0xFFFF;
}

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

main()
{
#ifdef __DOWNLOAD__
  run56k_start();
#endif

  init_disk_play();
  init_disk_record();

  ExitFlag = FALSE;
  while (!ExitFlag)
  {
    disk_play();
    if (ExitFlag)
      break;

    /*
     * ENCODER
     */
    adpt_predict(&E_DQ, &E_T, &E_DQ2, &E_DQ3, &E_DQ4, &E_DQ5, &E_DQ6, &E_DQ7,
		 &E_PK1, &E_PK2, &E_SR2, &E_A1, &E_A2, E_B,
		 &E_SE, &E_SEZ);
    diff_signal();
    adpt_quant(&E_Y);
    iadpt_quant(&E_DQ, &E_Y);
    tone_detector(&E_DQ, &E_T, &E_YL, &E_t_dr, &E_LAST_TR);
    speed_control(&E_T, &E_Y, &E_DMS, &E_DML, &E_AP);
    scale_factor(&E_Y, &E_YL, &E_Y_L, &E_LAST_Y);

    /*
     * DECODER
     */
    adpt_predict(&D_DQ, &D_T, &D_DQ2, &D_DQ3, &D_DQ4, &D_DQ5, &D_DQ6, &D_DQ7,
		 &D_PK1, &D_PK2, &D_SR2, &D_A1, &D_A2, D_B,
		 &D_SE, &D_SEZ);
    iadpt_quant(&D_DQ, &D_Y);
    add(&D_DQ);
    coding_adj();
    tone_detector(&D_DQ, &D_T, &D_YL, &D_t_dr, &D_LAST_TR);
    speed_control(&D_T, &D_Y, &D_DMS, &D_DML, &D_AP);
    scale_factor(&D_Y, &D_YL, &D_Y_L, &D_LAST_Y);

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
} 










