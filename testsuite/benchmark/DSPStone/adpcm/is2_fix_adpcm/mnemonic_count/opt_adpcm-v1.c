/*
 * opt_adpcm.c
 * Handwritten version of an optimized ADPCM transcoder applying the
 * CCITT recommendation G.721
 * 6-10-93 Chris Schlaeger
 * VERSION 1.0
 */

#include <stdio.h>

#define FALSE 0
#define TRUE (!FALSE)

FILE* istrm;
FILE* ostrm;
int   ExitFlag;

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

int fmult(int An, int SRn)
{
  int  AnS, AnMAG, AnEXP, AnMANT;
  int  SRnS, SRnEXP, SRnMANT;
  int  WAnS, WAnEXP, WAnMAG;
  long WAnMANT;

  AnS = An >> 15;
  AnMAG = AnS == 0 ? An >> 2 : (16384 - (An >> 2)) & 8191;
  for (AnEXP = 0; (AnMAG >> AnEXP) != 0; AnEXP++)
    ;
  AnMANT = AnMAG == 0 ? 1 << 5 : (AnMAG << 6) >> AnEXP;

  SRnS = SRn >> 10;
  SRnEXP = (SRn >> 6) & 15;
  SRnMANT = SRn & 63;

  WAnS = SRnS ^ AnS;
  WAnEXP = SRnEXP + AnEXP;
  WAnMANT = (((long) SRnMANT * AnMANT) + 48) >> 4;
  WAnMAG = (int) (WAnEXP <= 26 ?
                  (WAnMANT << 7) >> (26 - WAnEXP) :
	          ((WAnMANT << 7) << (WAnEXP - 26)) & 32767);
  return (WAnS == 0 ? WAnMAG : (65536 - WAnMAG) & 65535);
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
  {
    int SLS, SES, SLI, SEI;
    
    
    /* subta */
    SLS = SL >> 13;
    SLI = SLS == 0 ? SL : 49152 + SL;
    SES = S_E >> 14;
    SEI = SES == 0 ? S_E : 32768 + S_E;
    D = (SLI + 65536 - SEI) & 65535;
  }
  
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
  
  
  int n;
  int DQS, DQI, SEZS, SEZI, DQSEZ, PK0, SIGPK; /* for ADDC */
  int SES, SEI, SR;	/* for ADDB */
  int SRS, SR1;	/* for FLOATB */
  int PKS1, PKS2, UGA2A, A1S, FA1, FA, UGA2B, UGA2S,
  UGA2, A2S, ULA2, UA2, A2T;	/* for UPA2 */
  int A2P;	/* for LIMC */
  int A1R, A2R, BR[6];	/* for TRIGB */
  int WA1, WA2;
  int PKS, UGA1, ULA1, UA1, A1T;	/* for UA1 */
  int A1UL, A1LL, A1P;	/* for LIMD */
  int MAG, EXP, MANT, DQ1; /* for FLOATA */
  int U[6];	/* for XOR */
  int DQMAG, UGB, BS, ULB, UB, BP[6]; /* for UPB */
  int WB1, WB2, WB3, WB4, WB5, WB6;
  
  /* ADDC */
  DQS = *DQ >> 14;
  DQI = DQS == 0 ? *DQ : (65536 - (*DQ & 16383)) & 65535;
  SEZS = *SEZ >> 14;
  SEZI = SEZS == 0 ? *SEZ : (1 << 15) + *SEZ;
  DQSEZ = (DQI + SEZI) & 65535;
  PK0 = DQSEZ >> 15;
  SIGPK = DQSEZ == 0 ? 1 : 0;
  
  /* ADDB */
  DQS = *DQ >> 14;
  DQI = DQS == 0 ? *DQ : (65536 - (*DQ & 16383)) & 65535;
  SES = *SE >> 14;
  SEI = SES == 0 ? *SE : (1 << 15) + *SE;
  SR = (DQI + SEI) & 65535;
  
  /* FLOATB */
  SRS = SR >> 15;
  MAG = SRS == 0 ? SR : (65536 - SR) & 32767;
  for (EXP = 0; (MAG >> EXP) != 0; EXP++)
    ;
  MANT = MAG == 0 ? 1 << 5 : (MAG << 6) >> EXP;
  SR1 = (SRS << 10) + (EXP << 6) + MANT;
  
  /* UPA2 */
  PKS1 = PK0 ^ *PK1;
  PKS2 = PK0 ^ *PK2;
  UGA2A = PKS2 == 0 ? 16384 : 114688;
  A1S = *A1 >> 15;
  if (A1S == 0)
    FA1 = *A1 <= 8191 ? *A1 << 2 : 8191 << 2;
  else
    FA1 = *A1 >= 57345 ? (*A1 << 2) & 131071 : 24577 << 2;
  FA = PKS1 == 1 ? FA1 : (131072 - FA1) & 131071;
  UGA2B = (UGA2A + FA) & 131071;
  UGA2S = UGA2B >> 16;
  UGA2 = SIGPK == 1 ? 0 :
    UGA2S == 0 ? UGA2B >> 7 : (UGA2B >> 7) + 64512;
  A2S = *A2 >> 15;
  ULA2 = A2S == 0 ? (65536 - (*A2 >> 7)) & 65535 :
    (65536 - ((*A2 >> 7) + 65024)) & 65535;
  UA2 = (UGA2 + ULA2) & 65535;
  A2T = (*A2 + UA2) & 65535;
  
  /* LIMC */
  A2P = 32768 <= A2T && A2T <= 53248 ? 53248 :
    12288 <= A2T && A2T <= 32767 ? 12288 : A2T;
  
  /* TRIGB */
  A2R = *T == 0 ? A2P : 0;
  
  /* FMULT */
  WA2 = fmult(A2R, *SR2);
  
  /* UPA1 */
  PKS = PK0 ^ *PK1;
  UGA1 = SIGPK == 1 ? 0 : (PKS == 0 ? 192 : 65344);
  A1S = *A1 >> 15;
  ULA1 = A1S == 0 ? (65536 - (*A1 >> 8)) & 65535 :
    (65536 - ((*A1 >> 8) + 65280)) & 65535;
  UA1 = (UGA1 + ULA1) & 65535;
  A1T = (*A1 + UA1) & 65535;
  
  /* LIMD */
  A1UL = (15360 + 65536 - A2P) & 65535;
  A1LL = (A2P + 65536 - 15360) & 65535;
  A1P = 32768 <= A1T && A1T <= A1LL ? A1LL :
    A1UL <= A1T && A1T <= 32767 ? A1UL : A1T;
  
  /* TRIGB */
  A1R = *T == 0 ? A1P : 0;
  
  /* FMULT */
  WA1 = fmult(A1R, SR1);
  
  /* FLOATA */
  DQS = *DQ >> 14;
  MAG = *DQ & 16383;
  for (EXP = 0; (MAG >> EXP) != 0; EXP++)
    ;
  MANT = MAG == 0 ? 1 << 5 : (MAG << 6) >> EXP;
  DQ1 = (DQS << 10) + (EXP << 6) + MANT;
  
  /* XOR */
  DQS = *DQ >> 14;
  U[0] = DQS ^ (*DQ2 >> 10);
  U[1] = DQS ^ (*DQ3 >> 10);
  U[2] = DQS ^ (*DQ4 >> 10);
  U[3] = DQS ^ (*DQ5 >> 10);
  U[4] = DQS ^ (*DQ6 >> 10);
  U[5] = DQS ^ (*DQ7 >> 10);
  
  /* UPB */
  DQMAG = *DQ & 16383;
  for (n = 0; n < 6; n++)
    {
      UGB = DQMAG == 0 ? 0 : (U[n] == 0 ? 128 : 65408);
      BS = B[n] >> 15;
      ULB = BS == 0 ? (65536 - (B[n] >> 8)) & 65535 :
	(65536 - ((B[n] >> 8) + 65280)) & 65535;
      UB = (UGB + ULB) & 65535;
      BP[n] = (B[n] + UB) & 65535;
    }
  
  /* TRIGB */
  for (n = 0; n < 6; n++)
    BR[n] = *T == 0 ? BP[n] : 0;
  
  /* FMULT */
  WB1 = fmult(BR[0], DQ1);
  WB2 = fmult(BR[1], *DQ2);
  WB3 = fmult(BR[2], *DQ3);
  WB4 = fmult(BR[3], *DQ4);
  WB5 = fmult(BR[4], *DQ5);
  WB6 = fmult(BR[5], *DQ6);
  
  /* ACCUM */
  SEZI = (((((((((WB1 + WB2) & 65535) + WB3) & 65535)
	      + WB4) & 65535) + WB5) & 65535) + WB6) & 65535;
  SEI = (((SEZI + WA2) & 65535) + WA1) & 65535;
  *SEZ = SEZI >> 1;
  *SE = SEI >> 1;
  
  /* STATE update */
  *PK2 = *PK1;
  *PK1 = PK0;
  *SR2 = SR1;
  *A1 = A1R;
  *A2 = A2R;
  for (n = 0; n < 6; n++)
    B[n] = BR[n];
  *DQ7 = *DQ6;
  *DQ6 = *DQ5;
  *DQ5 = *DQ4;
  *DQ4 = *DQ3;
  *DQ3 = *DQ2;
  *DQ2 = DQ1;
  
  S_E = *SE;
  A_2 = *A2;
  

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
  int DQS, DQLN, DQL, DS, DEX, DMN, DQT, DQMAG;
  
  /* RECONST */
  DQS = I >> 3;
  DQLN = qtab[I];
  
  /* ADDA */
  DQL = (DQLN + (*Y >> 2)) & 4095;
  
  /* ANTILOG */
  DS = DQL >> 11;
  DEX = (DQL >> 7) & 15;
  DMN = DQL & 127;
  DQT = (1  << 7) + DMN;
  DQMAG = DS == 0 ? (DQT << 7) >> (14 - DEX) : 0;
  *DQ = (DQS << 14) + DQMAG;
}

void adpt_quant(int* Y)
{
  /*
   * adaptive quantizer
   *
   * Input signals:  D, Y
   * Output signals: I
   */

  int DS, DQM, DL, DLN, EXP, MANT;
  
  /* LOG */
  DS = D >> 15;
  DQM = DS == 0 ? D : (65536 - D) & 32767;
  for (EXP = 1; (DQM >> EXP) != 0; EXP++)
    ;
  EXP--;
  MANT = ((DQM << 7) >> EXP) & 127;
  DL = (EXP << 7) + MANT;
  
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

void speed_control(int* TR, int* Y, int* DMS, int* DML, int* AP)
{
  /*
   * speed control
   *
   * Input signals:  T, TDP, I, Y
   * Output signals: AL
   */

  static int F[] = { 0, 0, 0, 1, 1, 1, 3, 7 };
  int FI, DIF, DIFS, DIFSX, DIFM, DTHR, AX, APP;
  
  /* FUNCTF */ 
  FI = F[(I >> 3 ? (15 - I) : I) & 7] ; 
    
  /* FILTA */
  DIF = ((FI << 9) + 8192 - *DMS) & 8191;
  DIFS = DIF >> 12;
  DIFSX = DIFS == 1 ? (DIF >> 5) + 3840 : DIF >> 5;
  *DMS = (DIFSX + *DMS) & 4095;
  
  /* FILTB */
  DIF = ((FI << 11) + 32768 - *DML) & 32767;
  DIFS = DIF >> 14;
  DIFSX = DIFS == 1 ? (DIF >> 7) + 16128 : DIF >> 7;
  *DML = (DIFSX + *DML) & 16383;
  
  /* SUBTC */
  DIF = ((*DMS << 2) + 32768 - *DML) & 32767;
  DIFS = DIF >> 14;
  DIFM = DIFS == 1 ? (32768 - DIF) & 16383 : DIF;
  DTHR = *DML >> 3;
  AX = (*Y >= 1536 && DIFM < DTHR && TDP == 0) ? 0 : 1;
  
  /* FILTC */
  DIF = ((AX << 9) + 2048 - *AP) & 2047;
  DIFS = DIF >> 10;
  DIFSX = DIFS == 1 ? (DIF >> 4) + 896 : DIF >> 4;
  APP = (DIFSX + *AP) & 1023;
  
  /* TRIGA */
  *AP = *TR++ == 1 ? 256 : APP;
  
  /* LIMA */
  AL = *AP >= 256 ? 64 : *AP >> 2;
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

#ifdef __DOWNLOAD__
  run56k_end();
#endif

  return (0);
} 
