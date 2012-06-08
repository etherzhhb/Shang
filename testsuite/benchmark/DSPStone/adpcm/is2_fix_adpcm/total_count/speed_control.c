/* Global signals */
int A_2, AL, D, I, SD, S_E, SL, SP, TDP;


typedef struct
{
  int DQ, T, Y, YL, DQ2, DQ3, DQ4, DQ5, DQ6, DQ7, PK1, PK2, SR2,
      A1, A2, B[6], SE, SEZ, t_dr, LAST_TR, DMS, DML, AP, Y_L, LAST_Y;
} STATES;


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
