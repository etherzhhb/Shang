
/* Global signals */
int A_2, AL, D, I, SD, S_E, SL, SP, TDP;

typedef struct
{
  int DQ, T, Y, YL, DQ2, DQ3, DQ4, DQ5, DQ6, DQ7, PK1, PK2, SR2,
      A1, A2, B[6], SE, SEZ, t_dr, LAST_TR, DMS, DML, AP, Y_L, LAST_Y;
} STATES;


int fmult()
{
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
/*  {
    register int mag = MAG << 1;

    for (EXP = 0; mag >>= 1; EXP++)
#ifdef __LOOPCOUNT__ 
      Loop2++;
#else
      ;
#endif
  } */
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
/*  {
    register int mag = MAG << 1;

    for (EXP = 0; mag >>= 1; EXP++)
#ifdef __LOOPCOUNT__ 
      Loop3++;
#else
      ;
#endif
  } */
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
 /* for (EXP = 0; EXP < 6; EXP++)
  {
#ifdef __LOOPCOUNT__ 
    Loop4++;
#endif
    DQS = MANT ? (BP[EXP] ? 65408 : 128) : 0;
    DQS += S->B[EXP] & (1 << 15) ? (65536 - ((S->B[EXP] >> 8) + 65280)) :
                                   (65536 - (S->B[EXP] >> 8));
    BP[EXP] = (S->B[EXP] + DQS) & 65535;
  }
  */
  /* TRIGB */
 /* if (S->T)
    for (EXP = 0; EXP < 6; EXP++)
#ifdef __LOOPCOUNT__ 
    {
      Loop5++;
      BP[EXP] = 0 ;
    }
#else
      BP[EXP] = 0 ;
#endif
  */
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
/*
  for (EXP = 0; EXP < 6; EXP++)
#ifdef __LOOPCOUNT__
  {
    Loop6++;
    S->B[EXP] = BP[EXP];
  }
#else
    S->B[EXP] = BP[EXP];
#endif
*/
  S->DQ7 = S->DQ6;
  S->DQ6 = S->DQ5;
  S->DQ5 = S->DQ4;
  S->DQ4 = S->DQ3;
  S->DQ3 = S->DQ2;
  S->DQ2 = DQSEZ;
}
