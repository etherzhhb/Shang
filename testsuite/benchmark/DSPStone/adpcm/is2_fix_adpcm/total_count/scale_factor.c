/* Global signals */
int A_2, AL, D, I, SD, S_E, SL, SP, TDP;


typedef struct
{
  int DQ, T, Y, YL, DQ2, DQ3, DQ4, DQ5, DQ6, DQ7, PK1, PK2, SR2,
      A1, A2, B[6], SE, SEZ, t_dr, LAST_TR, DMS, DML, AP, Y_L, LAST_Y;
} STATES;


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