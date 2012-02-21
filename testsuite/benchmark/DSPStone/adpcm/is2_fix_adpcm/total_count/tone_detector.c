/* Global signals */
int A_2, AL, D, I, SD, S_E, SL, SP, TDP;


typedef struct
{
  int DQ, T, Y, YL, DQ2, DQ3, DQ4, DQ5, DQ6, DQ7, PK1, PK2, SR2,
      A1, A2, B[6], SE, SEZ, t_dr, LAST_TR, DMS, DML, AP, Y_L, LAST_Y;
} STATES;

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
