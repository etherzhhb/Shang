/* Global signals */
int A_2, AL, D, I, SD, S_E, SL, SP, TDP;


typedef struct
{
  int DQ, T, Y, YL, DQ2, DQ3, DQ4, DQ5, DQ6, DQ7, PK1, PK2, SR2,
      A1, A2, B[6], SE, SEZ, t_dr, LAST_TR, DMS, DML, AP, Y_L, LAST_Y;
} STATES;


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
