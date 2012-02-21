/*
 * flt_adpcm.c
 * Handwritten version of an optimized ADPCM transcoder applying the
 * CCITT recommendation G.721
 * floating point version
 * 6-10-1993 Chris Schlaeger
 * 12-10-1993 Juan Martinez Velarde
 * 21-10-1993 Juan Martinez Velarde, last changes
 */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define sgn1(x) ((x) < 0.0 ? -1.0 : 1 )  
#define sgn0(x) ((x) > 0.0 ?  1.0 : ((x) < 0.0 ? -1.0 : 0 )) 

#define FALSE   0
#define TRUE    !FALSE

#ifdef __DSP56K__ /* code defined for the g56k compiler */

#ifdef __SIMULATION__
#include "data_in.c"
#else 
#define N_OF_DATA 1
#endif

#else   /* code not defined for the dsp56k */
FILE* istrm ;
FILE* ostrm ;
FILE* t1strm;
FILE* t2strm ;
FILE* t3strm ;

#endif

int ExitFlag ; 

/* Global signals */
float D = 0; 
int SL = 0  ;
float SP = 0 ; 
int SD = 0 ;
int I = 0;
float A_2 = 0 ; 
float s_e = 0;
float AL = 0;
int TDP = 0 ; 
int TR = 0 ; 
float DQ = 0 ; 
float Y1 = 0 ; 
float YL1 = 0 ; 

/* encoder adpt predictor ports, 2 delayed signals*/
float E_DQ1 = 0 ; 
int E_TR1 = 0 ; 

/* decoder adpt predictor ports, 2 delayed signals*/
float D_DQ1 = 0 ; 
int D_TR1 = 0 ; 

/* encoder scale factor ports, 2 delayed signals */
float E_Y = 0 ; 
float E_YL = 0 ; 

/* decoder scale factor ports, 2 delayed signals */
float D_Y = 0 ; 
float D_YL = 0 ; 

/* ENCODER states */
/* encoder ADPT_PREDICTOR states */
float E_A1 = 0; 
float E_A2 = 0;
float E_P1 = 0;
float E_P2 = 0;
float E_P3 = 0 ; 
float E_SR2 = 0;
float E_B[6] = { 0, 0, 0, 0, 0, 0 };
float E_D_Q[6] = {0, 0, 0, 0, 0, 0} ; 
float E_SE = 0;
float E_SEZ = 0;

/* encoder speed control states */
float E_DMS = 0;
float E_DML = 0;
float E_AP = 0;

/* encoder scale factor states */
float E_Y_L = 0 ; 
float E_YU = 0 ; 

/* DECODER states */
/* decoder ADPT_PREDICTOR states */
float D_A1 = 0;
float D_A2 = 0;
float D_P1 = 0;
float D_P2 = 0;
float D_P3 = 0 ; 
float D_SR2 = 0;
float D_B[6] = { 0, 0, 0, 0, 0, 0 };
float D_D_Q[6] = {0, 0, 0, 0, 0, 0} ; 
float D_SE = 0;
float D_SEZ = 0;

/* decoder speed control states */
float D_DMS = 0;
float D_DML = 0;
float D_AP = 0;

/* decoder scale factor states */
float D_Y_L = 0 ;
float D_YU  = 0 ; 

void adpt_predict( D_Q_1, TR,
		   A1,  A2, 
		   P1,  P2,  P3, 
		   S_R2, 
		   B, D_Q, 
		   S_E,  S_EZ)
float *D_Q_1 ; 
int *TR ; 
float *A1 ; 
float *A2 ; 
float *P1 ; 
float *P2 ; 
float *P3 ; 
float *S_R2 ; 
float *B ; 
float *D_Q ; 
float *S_E ; 
float *S_EZ ; 
{
  register float f_a_0;
  register int  i ; 
  register float abs_a1 ; 

  *P3 = *P2;
  *P2 = *P1;
  *P1 = *D_Q_1 + *S_EZ;
  
  if (*TR == 1)
    {
     *A1 = *A2 = 0.0;
     for (i = 0; i < 6; i++)
       B[i] = 0.0;
   }
  else
    {
      i = (int) sgn0(*P1) ; 
      abs_a1 = fabs(*A1) ; 
      f_a_0 = (abs_a1 > 0.5 ? 2.0 * sgn1(*A1) : 4.0 * *A1);
      *A2 = 0.9921875 * *A2 + 0.0078125 * 
       (i * sgn1(*P3) - f_a_0 * i * sgn1(*P2) );      
      if (fabs(*A2) > 0.75)
	*A2 = (*A2 < 0.0) ? -0.75 : 0.75;
      
      *A1 = 0.99609375 * *A1 + 0.01171875 * i * sgn1(*P2);
      if (abs_a1 > 0.9375 - *A2)
       *A1 = (0.9375 - *A2) * (*A1 < 0.0 ? -1.0 : 1.0);	
      
      for (i = 0; i < 6; i++)
	B[i] = 0.99609375 * B[i] + 
	 0.0078125 *  sgn0(*D_Q_1) * sgn1(D_Q[i]);
    }
  
  for (i = 0 ; i > -5 ; i--)
    D_Q[i + 5] = D_Q[i + 4] ; 
  
  D_Q[0] = *D_Q_1;
  
  *S_EZ = 0.0 ; 
  for (i = 0; i < 5; i++)	
    *S_EZ += B[i] * D_Q[i];
  
  s_e = (*S_E + *D_Q_1) * *A1 + *S_R2 * *A2 + *S_EZ;
  *S_R2 = (*S_E + *D_Q_1) ;
  *S_E  = s_e;
  
  A_2 = *A2;

}

void iadpt_quant(Y)
float *Y ; 
{
  
  static float qtab[8] =
    {
      -1E38, 0.031, 1.05, 1.66, 2.13, 2.52, 2.91, 3.32
    };
 
  register int          d_qs;
  
  d_qs = 1 ; 
  if (I < 0) 
    d_qs = -1 ;
  DQ = d_qs * pow(2.0, qtab[d_qs * I] + *Y) ; 
  
}

void tone_detector(YL)
float *YL ; 
{

  TDP = 0 ; 
  TR = 0 ; 
  
  if (A_2 < -0.71875)
    TDP = 1 ;
  
  if (TDP == 1 && fabs(DQ) > 24.0 * pow(2.0, *YL)) 
    TR = 1 ; 
}

void speed_control(Y, D_MS, D_ML, A_P)
float *Y ; 
float *D_MS ; 
float *D_ML ; 
float *A_P ; 
{
  static   int F[] = { 0, 0, 0, 1, 1, 1, 3, 7 };
  register int f;
  
  f = F[abs(I)];
  
  AL = *A_P > 1.0 ? 1.0 : *A_P;
  
  *D_MS = 0.96875   * *D_MS + 0.03125   * f ;
  *D_ML = 0.9921875 * *D_ML + 0.0078125 * f ;
  
  if (((fabs(*D_MS - *D_ML) >= 0.125 * *D_ML) ||
       (*Y < 3.0)) && (TDP == 0))
    *A_P = 0.9375 * *A_P + 0.125 ;
  else 
    *A_P = (TR == 1) ? 1.0 : 0.9375 * *A_P ; 
  
}

void scale_factor(Y_U, Y_L)
float *Y_U ; 
float *Y_L ; 
{

  static float W[] =
    {
      -0.75, 1.13, 2.56, 4.00, 7.00, 12.38, 22.19, 70.13
    };
  
  Y1 = AL * *Y_U + (1.0 - AL) * *Y_L;
  
  *Y_U = 0.96875 * Y1 + 0.03125 * W[abs(I)];
  
  if (*Y_U < 1.06) *Y_U = 1.06   ; 
  if (*Y_U > 10.0) *Y_U = 10.0   ; 
  
  *Y_L *= 0.984375 ; 
  
  YL1 = *Y_L  = *Y_L + 0.015625 * *Y_U;
    
}


#ifdef __DSP56K__ /* code for the g56k compiler */

void disk_play() 
{
#ifdef __SIMULATION__
  static int i = 0 ; 
  if(i >= sizeof(Data_in)/sizeof(int))
    ExitFlag = TRUE ; 
  SL = Data_in[i++] & 0xFFFF ; 

#else
  SL = 0x7a & 0xFFFF;               /* constant value */
#endif
}

#else /* MACRO __DSP56K__ not defined, code is also for the SUN */

void init_disk_play()
{
  istrm = fopen("data.in.2freq", "rb") ; 
}

void disk_play()
{
  int d ;
  if (istrm != NULL && !feof(istrm))
    {
      fread((char*) &d, sizeof(int), 1, istrm) ; 
      SL = ((d & 0xFFFF0000) >> 16) |
	((d & 0x80000000) ? 0xFFFF0000 : 0) ; 
    }
  if (feof(istrm)) 
    ExitFlag = TRUE ; 
  
}

void post_disk_play()
{
  if (istrm != NULL)
    fclose(istrm) ; 
}

void init_disk_record()
{
  ostrm = fopen("data.out", "wb") ; 
  t1strm = fopen("datain.tmp","wb") ; 
  t2strm = fopen("dataout.tmp","wb") ; 
  t3strm = fopen("error.tmp","wb") ; 
  fprintf(t1strm,"TitleText: Input Data\n") ;
  fprintf(t2strm,"TitleText: Output Data\n") ; 
  fprintf(t3strm,"TitleText: Error\n") ; 
}

void disk_record()
{
  int d ; 
  if (ostrm != NULL)
    {
      d = (SD & 0x0000FFFF) << 16 ; 
      fwrite((char*) &d, 4, 1,ostrm) ; 
    }

}

void post_disk_record()
{
  if (ostrm != NULL)
    fclose(ostrm) ; 
  if (t1strm != NULL)
    fclose(t1strm) ; 
  if (t2strm != NULL)
    fclose(t2strm) ; 
  if (t3strm != NULL)
    fclose(t3strm) ; 
}
#endif

main(void)
{
  int i = 0 ; 
 
#ifndef __DSP56K__
  init_disk_play() ; 
  init_disk_record() ; 
#endif

  /* disk play */
  ExitFlag = FALSE ; 
  while (!ExitFlag)
    {
      disk_play() ; 

#ifdef __DSP56K__
#ifndef __SIMULATION__ 
      if (i == N_OF_DATA)           
	ExitFlag = TRUE ;      /* N_OF_DATA constant values simulation */
      i++ ; 
#endif
#else
      i++ ; 
      if (i < 1000) fprintf(t1strm,"%i\t%i\n",i,SL) ; 
#endif

      if(ExitFlag)
	break ; 

    /*
     * ENCODER
     */
      
      adpt_predict(&E_DQ1,&E_TR1,
		   &E_A1, &E_A2,
		   &E_P1, &E_P2, &E_P3, &E_SR2, 
		   E_B, E_D_Q,
		   &E_SE, &E_SEZ);
      
      /* DIFF SIGNAL is used inlined */
      /* SUBTA */
      
      D = ((float) SL)  - s_e ; 
      
      /* ADPT_QUANT is used inlined */
        
      {
 
	static float table[7] = { -0.98, 0.62, 1.38, 1.91, 2.34, 2.72, 3.12} ; 
	register float* ptable = table ;  

	register float d_ln;
	register int f ;

	d_ln = (D < 0) ? (-1) * D : D ;
	d_ln = log10(d_ln) * 3.321928095  - E_Y ; 
	
	for ( f = 0 ; f < 7 ; f++)
	  if (d_ln < *ptable++)
	  break ; 
		 
	I = (D < 0) ? (-1) * f : f ;
	 
      }
      
      iadpt_quant(&E_Y);
      tone_detector(&E_YL) ;
      speed_control(&E_Y, &E_DMS, &E_DML, &E_AP);
      scale_factor(&E_YU, &E_Y_L);

      /*
       * DELAYS in ENCODER
       */
      
      /* adpt predictor delays */

      E_DQ1 = DQ ;  
      E_TR1 = TR ;

      /* scale factor delays */
      E_YL = YL1 ; 
      E_Y  = Y1 ; 

      /*
       * DECODER
       */
      
      adpt_predict(&D_DQ1, &D_TR1,
		   &D_A1, &D_A2,
		   &D_P1, &D_P2, &D_P3, &D_SR2, 
		   D_B, D_D_Q,
		   &D_SE, &D_SEZ) ; 
      
      iadpt_quant(&D_Y);
      
      /* ADD is used inlined */

      SP = DQ + s_e ; 
      
      /* coding adj is used inlined */

      SD = (int) SP ;
      
      tone_detector(&D_YL);
      speed_control(&D_Y, &D_DMS, &D_DML, &D_AP);
      scale_factor(&D_YU, &D_Y_L);
      
      /*
       * DELAYS in DECODER
       */
      
      /* adpt_predictor delays */

      D_DQ1 = DQ ; 
      D_TR1 = TR ; 
 
      /* scale factor delays */

      D_YL = YL1 ; 
      D_Y  = Y1 ; 
       
#ifndef __DSP56K__
      /* disk record */
      disk_record() ;  

      if (i < 1000) fprintf(t2strm,"%i\t%i\n",i,SD) ; 
      if (i < 1000) fprintf(t3strm,"%i\t%i\n",i,SL-SD) ;     
#endif    
}

#ifndef __DSP56K__  
  post_disk_play() ; 
  post_disk_record() ; 
#endif

  return(0) ; 

  
} 
