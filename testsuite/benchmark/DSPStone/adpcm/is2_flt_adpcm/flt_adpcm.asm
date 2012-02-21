;
;	annotate version 1.03
;
; **************************************************
; **************************************************
	section	flt_adpcm_c
	opt so,nomd
	page 132,66,3,3
;*** DSP56000/1 Motorola 1.03 GNU 1.37.1
	org	p:
	global	FData_in
	org	x:
FData_in
	dc 122
	global	FD
FD
	dc	$000000
	dc	$000000
	global	FSL
FSL
	dc 0
	global	FSP
FSP
	dc	$000000
	dc	$000000
	global	FSD
FSD
	dc 0
	global	FI
FI
	dc 0
	global	FA_2
FA_2
	dc	$000000
	dc	$000000
	global	Fs_e
Fs_e
	dc	$000000
	dc	$000000
	global	FAL
FAL
	dc	$000000
	dc	$000000
	global	FTDP
FTDP
	dc 0
	global	FTR
FTR
	dc 0
	global	FDQ
FDQ
	dc	$000000
	dc	$000000
	global	FY1
FY1
	dc	$000000
	dc	$000000
	global	FYL1
FYL1
	dc	$000000
	dc	$000000
	global	FE_DQ1
FE_DQ1
	dc	$000000
	dc	$000000
	global	FE_TR1
FE_TR1
	dc 0
	global	FD_DQ1
FD_DQ1
	dc	$000000
	dc	$000000
	global	FD_TR1
FD_TR1
	dc 0
	global	FE_Y
FE_Y
	dc	$000000
	dc	$000000
	global	FE_YL
FE_YL
	dc	$000000
	dc	$000000
	global	FD_Y
FD_Y
	dc	$000000
	dc	$000000
	global	FD_YL
FD_YL
	dc	$000000
	dc	$000000
	global	FE_A1
FE_A1
	dc	$000000
	dc	$000000
	global	FE_A2
FE_A2
	dc	$000000
	dc	$000000
	global	FE_P1
FE_P1
	dc	$000000
	dc	$000000
	global	FE_P2
FE_P2
	dc	$000000
	dc	$000000
	global	FE_P3
FE_P3
	dc	$000000
	dc	$000000
	global	FE_SR2
FE_SR2
	dc	$000000
	dc	$000000
	org	p:
	global	FE_B
	org	x:
FE_B
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	org	p:
	global	FE_D_Q
	org	x:
FE_D_Q
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	global	FE_SE
FE_SE
	dc	$000000
	dc	$000000
	global	FE_SEZ
FE_SEZ
	dc	$000000
	dc	$000000
	global	FE_DMS
FE_DMS
	dc	$000000
	dc	$000000
	global	FE_DML
FE_DML
	dc	$000000
	dc	$000000
	global	FE_AP
FE_AP
	dc	$000000
	dc	$000000
	global	FE_Y_L
FE_Y_L
	dc	$000000
	dc	$000000
	global	FE_YU
FE_YU
	dc	$000000
	dc	$000000
	global	FD_A1
FD_A1
	dc	$000000
	dc	$000000
	global	FD_A2
FD_A2
	dc	$000000
	dc	$000000
	global	FD_P1
FD_P1
	dc	$000000
	dc	$000000
	global	FD_P2
FD_P2
	dc	$000000
	dc	$000000
	global	FD_P3
FD_P3
	dc	$000000
	dc	$000000
	global	FD_SR2
FD_SR2
	dc	$000000
	dc	$000000
	org	p:
	global	FD_B
	org	x:
FD_B
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	org	p:
	global	FD_D_Q
	org	x:
FD_D_Q
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	dc	$000000
	global	FD_SE
FD_SE
	dc	$000000
	dc	$000000
	global	FD_SEZ
FD_SEZ
	dc	$000000
	dc	$000000
	global	FD_DMS
FD_DMS
	dc	$000000
	dc	$000000
	global	FD_DML
FD_DML
	dc	$000000
	dc	$000000
	global	FD_AP
FD_AP
	dc	$000000
	dc	$000000
	global	FD_Y_L
FD_Y_L
	dc	$000000
	dc	$000000
	global	FD_YU
FD_YU
	dc	$000000
	dc	$000000
	org	p:
	global	Fadpt_predict
Fadpt_predict
; **************************************************
; /*
;  * flt_adpcm.c
;  * Handwritten version of an optimized ADPCM transcoder applying the
;  * CCITT recommendation G.721
;  * floating point version
;  * 6-10-1993 Chris Schlaeger
;  * 12-10-1993 Juan Martinez Velarde
;  * 21-10-1993 Juan Martinez Velarde, last changes
;  */
; 
; #include <math.h>
; #include <stdlib.h>
; #include <stdio.h>
; 
; #define sgn1(x) ((x) < 0.0 ? -1.0 : 1 )  
; #define sgn0(x) ((x) > 0.0 ?  1.0 : ((x) < 0.0 ? -1.0 : 0 )) 
; 
; #define FALSE   0
; #define TRUE    !FALSE
; 
; #ifdef __DSP56K__ /* code defined for the g56k compiler */
; 
; #ifdef __SIMULATION__
; #include "data_in.c"
; #else 
; #define N_OF_DATA 1
; #endif
; 
; #else   /* code not defined for the dsp56k */
; FILE* istrm ;
; FILE* ostrm ;
; FILE* t1strm;
; FILE* t2strm ;
; FILE* t3strm ;
; 
; #endif
; 
; int ExitFlag ; 
; 
; /* Global signals */
; float D = 0; 
; int SL = 0  ;
; float SP = 0 ; 
; int SD = 0 ;
; int I = 0;
; float A_2 = 0 ; 
; float s_e = 0;
; float AL = 0;
; int TDP = 0 ; 
; int TR = 0 ; 
; float DQ = 0 ; 
; float Y1 = 0 ; 
; float YL1 = 0 ; 
; 
; /* encoder adpt predictor ports, 2 delayed signals*/
; float E_DQ1 = 0 ; 
; int E_TR1 = 0 ; 
; 
; /* decoder adpt predictor ports, 2 delayed signals*/
; float D_DQ1 = 0 ; 
; int D_TR1 = 0 ; 
; 
; /* encoder scale factor ports, 2 delayed signals */
; float E_Y = 0 ; 
; float E_YL = 0 ; 
; 
; /* decoder scale factor ports, 2 delayed signals */
; float D_Y = 0 ; 
; float D_YL = 0 ; 
; 
; /* ENCODER states */
; /* encoder ADPT_PREDICTOR states */
; float E_A1 = 0; 
; float E_A2 = 0;
; float E_P1 = 0;
; float E_P2 = 0;
; float E_P3 = 0 ; 
; float E_SR2 = 0;
; float E_B[6] = { 0, 0, 0, 0, 0, 0 };
; float E_D_Q[6] = {0, 0, 0, 0, 0, 0} ; 
; float E_SE = 0;
; float E_SEZ = 0;
; 
; /* encoder speed control states */
; float E_DMS = 0;
; float E_DML = 0;
; float E_AP = 0;
; 
; /* encoder scale factor states */
; float E_Y_L = 0 ; 
; float E_YU = 0 ; 
; 
; /* DECODER states */
; /* decoder ADPT_PREDICTOR states */
; float D_A1 = 0;
; float D_A2 = 0;
; float D_P1 = 0;
; float D_P2 = 0;
; float D_P3 = 0 ; 
; float D_SR2 = 0;
; float D_B[6] = { 0, 0, 0, 0, 0, 0 };
; float D_D_Q[6] = {0, 0, 0, 0, 0, 0} ; 
; float D_SE = 0;
; float D_SEZ = 0;
; 
; /* decoder speed control states */
; float D_DMS = 0;
; float D_DML = 0;
; float D_AP = 0;
; 
; /* decoder scale factor states */
; float D_Y_L = 0 ;
; float D_YU  = 0 ; 
; 
; void adpt_predict( D_Q_1, TR,
; 		   A1,  A2, 
; 		   P1,  P2,  P3, 
; 		   S_R2, 
; 		   B, D_Q, 
; 		   S_E,  S_EZ)
; float *D_Q_1 ; 
; int *TR ; 
; float *A1 ; 
; float *A2 ; 
; float *P1 ; 
; float *P2 ; 
; float *P3 ; 
; float *S_R2 ; 
; float *B ; 
; float *D_Q ; 
; float *S_E ; 
; float *S_EZ ; 
; {
;   register float f_a_0;
;   register int  i ; 
;   register float abs_a1 ; 
; 
;   *P3 = *P2;
; **************************************************
; **************************************************
;   *P2 = *P1;
; **************************************************
; **************************************************
;   *P1 = *D_Q_1 + *S_EZ;
; **************************************************
	move	#65533,n0
	move	#28,n6
	move	r0,x:(r6)+
	move	(r6)+
	move	r6,r0
	move	(r6)+n6
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	x1,x:(r6)+
	move	y0,x:(r6)+
	move	y1,x:(r6)+
	move	r1,x:(r6)+
	move	r2,x:(r6)+
	move	r3,x:(r6)+
	move	r4,x:(r6)+
	move	r5,x:(r6)+
	move	r7,x:(r6)+
	move	x:(r0+n0),r4
	move	#65531,n0
	move	(r4)+
	move	x:(r0+n0),r3
	move	#65530,n0
	move	x:(r0+n0),r5
	move	#65529,n0
	move	x:(r0+n0),r1
	move	#65528,n0
	move	(r1)+
	move	x:(r0+n0),r2
	move	#65527,n0
	move	(r2)+
	move	x:(r0+n0),r7
	move	#65522,n0
	move	x:(r2)-,y1
	move	x:(r2),y0
	move	y0,x:(r7)+
	move	y1,x:(r7)-
	move	x:(r1)-,y1
	move	x:(r1),y0
	move	y0,x:(r2)+
	move	y1,x:(r2)-
	move	x:(r0+n0),r7
	move	x:(r4)-,b
	move	x:(r4),b0
	move	(r7)+
	move	x:(r7)-,a
	move	x:(r7),a0
	jsr	fadd_ab
; **************************************************
;   
;   if (*TR == 1)
; **************************************************
	move	#65532,n0
	move	#>1,x1
	move	b0,x:(r1)+
	move	b1,x:(r1)-
	move	x:(r0+n0),r7
	move	#24,n0
	move	x:(r7),y0
	tfr	y0,b	y0,x:(r0+n0)
	cmp	x1,b
	jne	L36
; **************************************************
;     {
;      *A1 = *A2 = 0.0;
; **************************************************
; **************************************************
;      for (i = 0; i < 6; i++)
; **************************************************
	clr	a	#26,n0
	move	#0,x0
	move	a0,x:(r5)+
	move	a1,x:(r5)-
	move	a0,x:(r3)+
	move	a1,x:(r3)-
	move	(r0)+
	move	(r0)+n0
	move	a1,x:(r0)-
	move	a0,x:(r0)-n0
	do	#6,L93
L40
; **************************************************
;        B[i] = 0.0;
; **************************************************
; **************************************************
; **************************************************
	move	#24,n0
	move	#>2,y0
	mpy	+y0,x0,b
	asr	b
	move	b0,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	#65525,n0
	move	x:(r0+n0),b
	add	y0,b	#26,n0
	tfr	x0,b	b1,r1
	add	x1,b	(r0)+
	move	b1,x0
	move	(r0)+n0
	move	x:(r0)-,y1
	move	x:(r0)-n0,y0
	move	y0,x:(r1)+
	move	y1,x:(r1)-
L93
	jmp	L91
L36
; **************************************************
;    }
;   else
;     {
;       i = (int) sgn0(*P1) ; 
; **************************************************
	clr	b	(r1)+
	move	x:(r1)-,a
	move	x:(r1),a0
	jsr	fcmp_ba
	jle	L42
	move	#>8192,y0
	move	#>4194304,y1
	jmp	L94
L42
	clr	b	(r1)+
	move	x:(r1)-,a
	move	x:(r1),a0
	jsr	fcmp_ba
	jge	L44
	move	#>8191,y0
	move	#>8388608,y1
	jmp	L94
L44
	move	#>0,y0
	move	#>0,y1
L94
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fixdfsi_b
; **************************************************
;       abs_a1 = fabs(*A1) ; 
; **************************************************
	move	#22,n0
	move	b1,x0
	move	(r0)+
	move	(r0)+n0
	move	(r3)+
	move	x:(r3)-,y1
	tfr	y1,b	x:(r3),y0
	move	y0,b0
	move	y1,x:(r0)-
	move	#>0,y1
	tfr	y1,a	y0,x:(r0)-n0
	move	#24,n0
	move	#>0,y0
	move	y0,a0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	jsr	fcmp_ab
	jle	L47
	move	#22,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,y1
	move	x:(r0)-n0,y0
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	jmp	L48
L47
	move	#22,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fneg_b
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
L48
; **************************************************
; **************************************************
; **************************************************
;       f_a_0 = (abs_a1 > 0.5 ? 2.0 * sgn1(*A1) : 4.0 * *A1);
; **************************************************
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,y1
	tfr	y1,b	x:(r0)-n0,y0
	move	#2,n0
	move	y0,b0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	#>4194304,y1
	tfr	y1,a	y0,x:(r0)-n0
	move	#24,n0
	move	#>8191,y0
	move	y0,a0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	jsr	fcmp_ab
	jle	L49
	clr	b	(r3)+
	move	x:(r3)-,a
	move	x:(r3),a0
	jsr	fcmp_ba
	jge	L51
	move	#>8191,y0
	move	#>8388608,y1
	jmp	L95
L51
	move	#>8192,y0
	move	#>4194304,y1
L95
	move	#24,n0
	move	#>4194304,a
	move	#>8193,a0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jmp	L96
L49
	move	#>4194304,a
	move	#>8194,a0
	move	(r3)+
	move	x:(r3)-,b
	move	x:(r3),b0
L96
	jsr	fmpy_ab
; **************************************************
;       *A2 = 0.9921875 * *A2 + 0.0078125 * 
;        (i * sgn1(*P3) - f_a_0 * i * sgn1(*P2) );      
; **************************************************
	move	#>8323072,a
	move	#>8191,a0
	move	b0,x:(r0)+
	move	b1,x:(r0)-
	move	(r5)+
	move	x:(r5)-,b
	move	x:(r5),b0
	jsr	fmpy_ab
	move	#4,n0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	tfr	x0,b	b0,x:(r0)-n0
	jsr	floatsidf_b
	move	#6,n0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#65527,n0
	move	x:(r0+n0),r7
	move	#24,n0
	move	(r7)+
	move	x:(r7)-,y1
	tfr	y1,b	x:(r7),y0
	move	y0,b0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	#>0,y1
	tfr	y1,a	y0,x:(r0)-n0
	move	#22,n0
	move	#>0,y0
	move	y0,a0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	jsr	fcmp_ab
	jge	L53
	move	#>8191,y0
	move	#>8388608,y1
	jmp	L97
L53
	move	#>8192,y0
	move	#>4194304,y1
L97
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	#6,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,a
	move	x:(r0)-n0,a0
	jsr	fmpy_ab
	move	#8,n0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	tfr	x0,b	b0,x:(r0)-n0
	jsr	floatsidf_b
	tfr	b,a	(r0)+
	move	x:(r0)-,b
	move	x:(r0),b0
	jsr	fmpy_ab
	move	#10,n0
	move	(r2)+
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#24,n0
	move	x:(r2)-,y1
	tfr	y1,b	x:(r2),y0
	move	y0,b0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	#>0,y1
	tfr	y1,a	y0,x:(r0)-n0
	move	#22,n0
	move	#>0,y0
	move	y0,a0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	jsr	fcmp_ab
	jge	L55
	move	#>8191,y0
	move	#>8388608,y1
	jmp	L98
L55
	move	#>8192,y0
	move	#>4194304,y1
L98
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	#10,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,a
	move	x:(r0)-n0,a0
	jsr	fmpy_ab
	tfr	b,a	#8,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fsub_ab
	move	#>4194304,a
	move	#>8185,a0
	jsr	fmpy_ab
	tfr	b,a	#4,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fadd_ab
; **************************************************
;       if (fabs(*A2) > 0.75)
; **************************************************
	move	#22,n0
	move	b0,y0
	move	b1,y1
	move	y0,x:(r5)+
	move	y1,x:(r5)-
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	#>0,y1
	tfr	y1,a	y0,x:(r0)-n0
	move	#24,n0
	move	#>0,y0
	move	y0,a0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	jsr	fcmp_ab
	jle	L59
	move	#22,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,y1
	move	x:(r0)-n0,y0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	jmp	L60
L59
	move	#22,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fneg_b
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
L60
; **************************************************
; **************************************************
	move	#24,n0
	move	#>8191,y0
	move	#>6291456,y1
	tfr	y1,a	(r0)+
	move	y0,a0
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	#22,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fcmp_ab
	jle	L57
; **************************************************
; 	*A2 = (*A2 < 0.0) ? -0.75 : 0.75;
; **************************************************
	clr	b	(r5)+
	move	x:(r5)-,a
	move	x:(r5),a0
	jsr	fcmp_ba
	jge	L61
	move	#>8191,y0
	move	#>10485760,y1
	jmp	L99
L61
	move	#>8191,y0
	move	#>6291456,y1
L99
	move	#26,n0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,y1
	move	x:(r0)-n0,y0
	move	y0,x:(r5)+
	move	y1,x:(r5)-
L57
; **************************************************
;       
;       *A1 = 0.99609375 * *A1 + 0.01171875 * i * sgn1(*P2);
; **************************************************
	move	#>8355840,a
	move	#>8191,a0
	move	(r3)+
	move	x:(r3)-,b
	move	x:(r3),b0
	jsr	fmpy_ab
	move	#12,n0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	tfr	x0,b	b0,x:(r0)-n0
	jsr	floatsidf_b
	move	#>6291456,a
	move	#>8185,a0
	jsr	fmpy_ab
	move	#14,n0
	move	(r2)+
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#24,n0
	move	x:(r2)-,y1
	tfr	y1,b	x:(r2),y0
	move	y0,b0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	#>0,y1
	tfr	y1,a	y0,x:(r0)-n0
	move	#22,n0
	move	#>0,y0
	move	y0,a0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	jsr	fcmp_ab
	jge	L63
	move	#>8191,y0
	move	#>8388608,y1
	jmp	L100
L63
	move	#>8192,y0
	move	#>4194304,y1
L100
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	#14,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,a
	move	x:(r0)-n0,a0
	jsr	fmpy_ab
	tfr	b,a	#12,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fadd_ab
; **************************************************
;       if (abs_a1 > 0.9375 - *A2)
; **************************************************
	move	#22,n0
	move	b0,y0
	move	b1,y1
	move	(r5)+
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#>7864320,b
	move	#>8191,b0
	move	y0,x:(r3)+
	move	y1,x:(r3)-
	move	x:(r5)-,a
	move	x:(r5),a0
	jsr	fsub_ab
	tfr	b,a	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#2,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fcmp_ab
	jle	L65
; **************************************************
;        *A1 = (0.9375 - *A2) * (*A1 < 0.0 ? -1.0 : 1.0);	
; **************************************************
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,y1
	move	x:(r0)-n0,y0
	move	#16,n0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	#>0,y1
	tfr	y1,a	y0,x:(r0)-n0
	move	#24,n0
	move	#>0,y0
	move	y0,a0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	#22,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fcmp_ab
	jge	L66
	move	#>8191,y0
	move	#>8388608,y1
	jmp	L101
L66
	move	#>8192,y0
	move	#>4194304,y1
L101
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	#16,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,a
	move	x:(r0)-n0,a0
	jsr	fmpy_ab
	move	b0,x:(r3)+
	move	b1,x:(r3)-
L65
; **************************************************
;       
;       for (i = 0; i < 6; i++)
; **************************************************
	move	#20,n0
	move	#>1,x1
	move	#>0,y0
	move	#>0,y1
	move	#0,x0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	do	#6,L91
L77
; **************************************************
; 	B[i] = 0.99609375 * B[i] + 
; 	 0.0078125 *  sgn0(*D_Q_1) * sgn1(D_Q[i]);
; **************************************************
	move	#65525,n0
	move	#>8355840,a
	move	#>8191,a0
	move	#>2,y0
	mpy	+y0,x0,b
	asr	b
	move	b0,b
	move	b1,y0
	move	x:(r0+n0),b
	add	y0,b
	move	b1,r2
	move	(r2)+
	move	x:(r2)-,b
	move	x:(r2),b0
	jsr	fmpy_ab
	move	#18,n0
	move	(r4)+
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#24,n0
	move	x:(r4)-,y1
	tfr	y1,b	x:(r4),y0
	move	y0,b0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	#20,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,a
	move	x:(r0)-n0,a0
	jsr	fcmp_ab
	jle	L71
	move	#>8192,y0
	move	#>4194304,y1
	jmp	L102
L71
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	(r4)+
	move	x:(r4)-,y1
	tfr	y1,b	x:(r4),y0
	move	y0,b0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	#20,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,a
	move	x:(r0)-n0,a0
	jsr	fcmp_ab
	jge	L73
	move	#>8191,y0
	move	#>8388608,y1
	jmp	L103
L73
	move	#>0,y0
	move	#>0,y1
L103
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,y1
	move	x:(r0)-n0,y0
L102
	move	#24,n0
	move	#>4194304,a
	move	#>8185,a0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fmpy_ab
	move	#22,n0
	move	#>2,y0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	mpy	+y0,x0,b	b0,x:(r0)-n0
	asr	b	#65524,n0
	move	b0,b
	move	b1,y0
	move	x:(r0+n0),b
	add	y0,b	#24,n0
	move	b1,r1
	move	(r0)+
	move	(r0)+n0
	move	(r1)+
	move	x:(r1)-,y1
	tfr	y1,b	x:(r1),y0
	move	y0,b0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	#20,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,a
	move	x:(r0)-n0,a0
	jsr	fcmp_ab
	jge	L75
	move	#>8191,y0
	move	#>8388608,y1
	jmp	L104
L75
	move	#>8192,y0
	move	#>4194304,y1
L104
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	#22,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,a
	move	x:(r0)-n0,a0
	jsr	fmpy_ab
	tfr	b,a	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#18,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fadd_ab
; **************************************************
; **************************************************
	move	b0,x:(r2)+
	tfr	x0,b	b1,x:(r2)-
	add	x1,b
	move	b1,x0
L91
; **************************************************
;     }
;   
;   for (i = 0 ; i > -5 ; i--)
; **************************************************
	move	#>16777215,x1
	move	#0,x0
	do	#5,L89
L81
; **************************************************
;     D_Q[i + 5] = D_Q[i + 4] ; 
; **************************************************
; **************************************************
; **************************************************
	move	#8,n1
	move	#65524,n0
	move	#>2,y0
	mpy	+y0,x0,b	x:(r0+n0),y0
	asr	b	#24,n0
	move	b0,b
	add	y0,b	(r0)+
	tfr	x0,b	b1,r1
	add	x1,b	(r0)+n0
	move	b1,x0
	move	(r1)+
	move	(r1)+n1
	move	x:(r1)-,y1
	move	x:(r1)-n1,y0
	move	#10,n1
	move	(r1)+
	move	(r1)+n1
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	y1,x:(r1)-
	move	y0,x:(r1)-n1
L89
; **************************************************
;   
;   D_Q[0] = *D_Q_1;
; **************************************************
; **************************************************
;   
;   *S_EZ = 0.0 ; 
; **************************************************
; **************************************************
;   for (i = 0; i < 5; i++)	
; **************************************************
	move	#65524,n0
	move	#>1,x1
	move	x:(r0+n0),r7
	move	#24,n0
	move	#0,x0
	move	(r0)+
	move	(r0)+n0
	move	(r4)+
	move	x:(r4)-,y1
	move	x:(r4),y0
	move	y0,x:(r7)+
	move	#>0,y0
	move	y1,x:(r7)-
	move	#>0,y1
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	#65522,n0
	move	x:(r0+n0),r7
	move	y0,x:(r7)+
	move	y1,x:(r7)-
	do	#5,L87
L85
; **************************************************
;     *S_EZ += B[i] * D_Q[i];
; **************************************************
	move	#24,n0
	move	#>2,y0
	mpy	+y0,x0,b
	asr	b
	move	b0,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	#65525,n0
	move	x:(r0+n0),b
	add	y0,b	#65524,n0
	move	b1,r1
	move	x:(r0+n0),b
	add	y0,b	(r1)+
	move	b1,r2
	move	x:(r1)-,b
	move	x:(r1),b0
	move	(r2)+
	move	x:(r2)-,a
	move	x:(r2),a0
	jsr	fmpy_ab
	move	#65522,n0
	move	x:(r0+n0),r7
	move	(r7)+
	move	x:(r7)-,a
	move	x:(r7),a0
	jsr	fadd_ab
; **************************************************
; **************************************************
	move	b0,x:(r7)+
	tfr	x0,b	b1,x:(r7)-
	add	x1,b
	move	b1,x0
L87
; **************************************************
;   
;   s_e = (*S_E + *D_Q_1) * *A1 + *S_R2 * *A2 + *S_EZ;
; **************************************************
	move	#65523,n0
	move	(r4)+
	move	x:(r0+n0),r7
	move	x:(r4)-,a
	move	x:(r4),a0
	move	(r7)+
	move	x:(r7)-,b
	move	x:(r7),b0
	jsr	fadd_ab
	move	(r3)+
	move	x:(r3)-,a
	move	x:(r3),a0
	jsr	fmpy_ab
	move	#24,n0
	move	(r5)+
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#65526,n0
	move	x:(r5)-,a
	move	x:(r0+n0),r7
	move	x:(r5),a0
	move	(r7)+
	move	x:(r7)-,b
	move	x:(r7),b0
	jsr	fmpy_ab
	tfr	b,a	#22,n0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#24,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fadd_ab
	move	#65522,n0
	move	x:(r0+n0),r7
	move	(r7)+
	move	x:(r7)-,a
	move	x:(r7),a0
	jsr	fadd_ab
; **************************************************
;   *S_R2 = (*S_E + *D_Q_1) ;
; **************************************************
	move	#65523,n0
	move	b0,x:Fs_e
	move	b1,x:(Fs_e+1)
	move	x:(r0+n0),r7
	move	(r4)+
	move	x:(r4)-,a
	move	x:(r4),a0
	move	(r7)+
	move	x:(r7)-,b
	move	x:(r7),b0
	jsr	fadd_ab
; **************************************************
;   *S_E  = s_e;
; **************************************************
; **************************************************
;   
;   A_2 = *A2;
; **************************************************
; **************************************************
; 
; }
; **************************************************
	move	#65526,n0
	move	(r6)-
	move	x:(r0+n0),r7
	move	#65523,n0
	move	b0,x:(r7)+
	move	b1,x:(r7)-
	move	x:(r0+n0),r7
	move	#24,n0
	move	x:(Fs_e+1),y1
	move	x:Fs_e,y0
	move	y0,x:(r7)+
	move	y1,x:(r7)-
	move	(r0)+
	move	(r0)+n0
	move	(r5)+
	move	x:(r5)-,y1
	move	x:(r5),y0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
	move	y0,x:FA_2
	move	y1,x:(FA_2+1)
	move	x:(r6)-,r7
	move	x:(r6)-,r5
	move	x:(r6)-,r4
	move	x:(r6)-,r3
	move	x:(r6)-,r2
	move	x:(r6)-,r1
	move	x:(r6)-,y1
	move	x:(r6)-,y0
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	(r0)-
	move	(r0)-
	move	r0,r6
	move	x:(r0),r0
	rts

	org	x:
F___qtab0
	dc	$00207e
	dc	$b4c4b3
	dc	$001ffa
	dc	$7ef9db
	dc	$002000
	dc	$433333
	dc	$002000
	dc	$6a3d71
	dc	$002001
	dc	$4428f6
	dc	$002001
	dc	$50a3d7
	dc	$002001
	dc	$5d1eb8
	dc	$002001
	dc	$6a3d71
	org	p:
	global	Fiadpt_quant
Fiadpt_quant
	move	r0,x:(r6)+
	move	ssh,x:(r6)+
; **************************************************
; **************************************************
; **************************************************
; 
; void iadpt_quant(Y)
; float *Y ; 
; {
;   
;   static float qtab[8] =
;     {
;       -1E38, 0.031, 1.05, 1.66, 2.13, 2.52, 2.91, 3.32
;     };
;  
;   register int          d_qs;
;   
;   d_qs = 1 ; 
; **************************************************
; **************************************************
;   if (I < 0) 
; **************************************************
	move	#5,n6
	move	r6,r0
	move	(r6)+n6
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	x1,x:(r6)+
	move	#>1,x1
	move	y0,x:(r6)+
	move	y1,x:(r6)+
	move	r1,x:(r6)+
	move	r2,x:(r6)+
	move	x:FI,a
	tst	a
	jge	L106
; **************************************************
;     d_qs = -1 ;
; **************************************************
	move	#>16777215,x1
L106
; **************************************************
;   DQ = d_qs * pow(2.0, qtab[d_qs * I] + *Y) ; 
; **************************************************
	move	#65533,n0
	move	#F___qtab0,r1
	move	x:(r0+n0),r2
	move	#>2,y0
	move	x:FI,x0
	mpy	+x0,x1,b	(r2)+
	asr	b	x:(r2)-,a
	move	b0,b
	move	b1,x0
	mpy	+y0,x0,b	x:(r2),a0
	asr	b
	move	b0,b
	move	b1,y0
	move	r1,b
	add	y0,b
	move	b1,r1
	move	(r1)+
	move	x:(r1)-,b
	move	x:(r1),b0
	jsr	fadd_ab
	move	#>8193,y0
	move	#>4194304,y1
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	y0,x:(r6)+
	move	y1,x:(r6)+
	jsr	Fpow
	tfr	x1,b	#4,n6
	move	(r6)-n6
	jsr	floatsidf_b
	jsr	fmpy_ab
; **************************************************
;   
; }
; **************************************************
	move	b0,x:FDQ
	move	b1,x:(FDQ+1)
	move	(r0)-
	move	(r6)-
	move	x:(r6)-,r2
	move	x:(r6)-,r1
	move	x:(r6)-,y1
	move	x:(r6)-,y0
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	x:(r0)-,ssh
	move	r0,r6
	move	x:(r0),r0
	rts

	global	Ftone_detector
Ftone_detector
	move	r0,x:(r6)+
	move	ssh,x:(r6)+
; **************************************************
; **************************************************
; **************************************************
; 
; void tone_detector(YL)
; float *YL ; 
; {
; 
;   TDP = 0 ; 
; **************************************************
; **************************************************
;   TR = 0 ; 
; **************************************************
; **************************************************
;   
;   if (A_2 < -0.71875)
; **************************************************
	clr	a	#2,n6
	move	r6,r0
	move	(r6)+n6
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	#>10747904,b
	move	#>8191,b0
	move	x0,x:(r6)+
	move	x1,x:(r6)+
	move	r1,x:(r6)+
	move	a1,x:FTDP
	move	a1,x:FTR
	move	x:(FA_2+1),a
	move	x:FA_2,a0
	jsr	fcmp_ba
	jge	L108
; **************************************************
;     TDP = 1 ;
; **************************************************
	move	#>1,a
	move	a1,x:FTDP
L108
; **************************************************
;   
;   if (TDP == 1 && fabs(DQ) > 24.0 * pow(2.0, *YL)) 
; **************************************************
	move	#>1,b
	move	x:FTDP,a
	cmp	b,a
	jne	L109
	clr	a	x:FDQ,x0
	move	x:(FDQ+1),x1
	move	x0,x:(r0)+
	move	x1,x:(r0)-
	move	(r0)+
	move	x:(r0)-,b
	move	x:(r0),b0
	jsr	fcmp_ab
	jle	L111
	move	(r0)+
	move	x:(r0)-,x1
	move	x:(r0),x0
	move	x0,x:(r0)+
	move	x1,x:(r0)-
	jmp	L112
L111
	move	(r0)+
	move	x:(r0)-,b
	move	x:(r0),b0
	jsr	fneg_b
	move	b0,x:(r0)+
	move	b1,x:(r0)-
L112
; **************************************************
; **************************************************
	move	#65533,n0
	move	x:(r0+n0),r1
	move	(r1)+
	move	x:(r1)-,a
	move	x:(r1),a0
	move	a0,x:(r6)+
	move	a1,x:(r6)+
	move	#>4194304,a
	move	#>8193,a0
	move	a0,x:(r6)+
	move	a1,x:(r6)+
	jsr	Fpow
	move	#4,n6
	move	#>6291456,b
	move	#>8196,b0
	move	(r6)-n6
	jsr	fmpy_ba
	move	(r0)+
	move	x:(r0)-,b
	move	x:(r0),b0
	jsr	fcmp_ab
	jle	L109
; **************************************************
;     TR = 1 ; 
; **************************************************
	move	#>1,a
	move	a1,x:FTR
L109
; **************************************************
; }
; **************************************************
	move	(r0)-
	move	(r6)-
	move	x:(r6)-,r1
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	x:(r0)-,ssh
	move	r0,r6
	move	x:(r0),r0
	rts

	org	x:
F___F1
	dc 0
	dc 0
	dc 0
	dc 1
	dc 1
	dc 1
	dc 3
	dc 7
	org	p:
	global	Fspeed_control
Fspeed_control
; **************************************************
; **************************************************
; **************************************************
; 
; void speed_control(Y, D_MS, D_ML, A_P)
; float *Y ; 
; float *D_MS ; 
; float *D_ML ; 
; float *A_P ; 
; {
;   static   int F[] = { 0, 0, 0, 1, 1, 1, 3, 7 };
;   register int f;
;   
;   f = F[abs(I)];
; **************************************************
; **************************************************
;   
;   AL = *A_P > 1.0 ? 1.0 : *A_P;
; **************************************************
	move	#65532,n0
	move	#8,n6
	move	r0,x:(r6)+
	move	(r6)+
	move	r6,r0
	move	(r6)+n6
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	x1,x:(r6)+
	move	y0,x:(r6)+
	move	y1,x:(r6)+
	move	r1,x:(r6)+
	move	#F___F1,r1
	move	r2,x:(r6)+
	move	r3,x:(r6)+
	move	r4,x:(r6)+
	move	r5,x:(r6)+
	move	x:(r0+n0),r3
	move	#65531,n0
	move	x:FI,x0
	tfr	x0,b	x:(r0+n0),r2
	abs	b	#65530,n0
	move	b1,x0
	move	x:(r0+n0),r4
	move	#6,n0
	move	r1,b
	add	x0,b	(r4)+
	move	b1,r1
	move	x:(r1),x0
	move	x0,x:(r0)
	move	x:(r4)-,x1
	tfr	x1,b	x:(r4),x0
	move	x0,b0
	move	(r0)+
	move	(r0)+n0
	move	x1,x:(r0)-
	move	#>4194304,x1
	tfr	x1,a	x0,x:(r0)-n0
	move	#3,n0
	move	#>8192,x0
	move	(r0)+
	move	(r0)+n0
	move	x1,x:(r0)-
	move	x0,x:(r0)-n0
	move	x0,a0
	jsr	fcmp_ab
	jle	L114
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,y1
	move	x:(r0)-n0,y0
	jmp	L115
L114
	move	(r4)+
	move	x:(r4)-,y1
	move	x:(r4),y0
L115
; **************************************************
;   
;   *D_MS = 0.96875   * *D_MS + 0.03125   * f ;
; **************************************************
	move	#>8126464,a
	move	#>8191,a0
	move	(r3)+
	move	y0,x:FAL
	move	y1,x:(FAL+1)
	move	x:(r3)-,b
	move	x:(r3),b0
	jsr	fmpy_ab
	move	#3,n0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	x:(r0),b
	jsr	floatsidf_b
	move	#6,n0
	move	#>4194304,a
	move	#>8187,a0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	jsr	fmpy_ab
	tfr	b,a	#3,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fadd_ab
; **************************************************
;   *D_ML = 0.9921875 * *D_ML + 0.0078125 * f ;
; **************************************************
	move	#>8323072,a
	move	#>8191,a0
	move	b0,x:(r3)+
	move	b1,x:(r3)-
	move	(r2)+
	move	x:(r2)-,b
	move	x:(r2),b0
	jsr	fmpy_ab
	move	#>4194304,a
	move	#>8185,a0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#6,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fmpy_ab
	tfr	b,a	#3,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fadd_ab
; **************************************************
;   
;   if (((fabs(*D_MS - *D_ML) >= 0.125 * *D_ML) ||
;        (*Y < 3.0)) && (TDP == 0))
; **************************************************
	move	b0,x:(r2)+
	move	b1,x:(r2)-
	move	(r2)+
	move	x:(r2)-,a
	move	x:(r2),a0
	move	(r3)+
	move	x:(r3)-,b
	move	x:(r3),b0
	jsr	fsub_ab
	move	#>0,x0
	move	#>0,x1
	tfr	x1,a	(r0)+
	move	x0,a0
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#6,n0
	move	(r0)+
	move	(r0)+n0
	move	x1,x:(r0)-
	move	x0,x:(r0)-n0
	jsr	fcmp_ab
	jle	L119
	move	#3,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,x1
	move	x:(r0)-n0,x0
	move	(r0)+
	move	(r0)+n0
	move	x1,x:(r0)-
	move	x0,x:(r0)-n0
	jmp	L120
L119
	move	#3,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fneg_b
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
L120
; **************************************************
; **************************************************
	move	#>4194304,a
	move	#>8189,a0
	move	(r2)+
	move	x:(r2)-,b
	move	x:(r2),b0
	jsr	fmpy_ab
	tfr	b,a	#6,n0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#3,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fcmp_ab
	jge	L117
	move	#65533,n0
	move	x:(r0+n0),r5
	move	#6,n0
	move	(r5)+
	move	x:(r5)-,x1
	move	x:(r5),x0
	move	(r0)+
	move	(r0)+n0
	move	x1,x:(r0)-
	move	#>6291456,x1
	tfr	x1,a	x0,x:(r0)-n0
	move	#>8193,x0
	move	x0,a0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	move	#3,n0
	move	(r0)+
	move	(r0)+n0
	move	x1,x:(r0)-
	move	x0,x:(r0)-n0
	jsr	fcmp_ab
	jge	L116
L117
	move	x:FTDP,a
	tst	a
	jne	L116
; **************************************************
;     *A_P = 0.9375 * *A_P + 0.125 ;
; **************************************************
	move	#>7864320,a
	move	#>8191,a0
	move	(r4)+
	move	x:(r4)-,b
	move	x:(r4),b0
	jsr	fmpy_ab
	move	#>4194304,a
	move	#>8189,a0
	jsr	fadd_ab
	move	b0,x:(r4)+
	move	b1,x:(r4)-
	jmp	L121
L116
; **************************************************
;   else 
;     *A_P = (TR == 1) ? 1.0 : 0.9375 * *A_P ; 
; **************************************************
	move	#>1,b
	move	x:FTR,a
	cmp	b,a
	jne	L122
	move	#6,n0
	move	#>8192,x0
	move	#>4194304,x1
	move	(r0)+
	move	(r0)+n0
	move	x1,x:(r0)-
	move	x0,x:(r0)-n0
	jmp	L123
L122
	move	#>7864320,a
	move	#>8191,a0
	move	(r4)+
	move	x:(r4)-,b
	move	x:(r4),b0
	jsr	fmpy_ab
	move	#6,n0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
L123
	move	#6,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,x1
	move	x:(r0)-n0,x0
	move	x0,x:(r4)+
	move	x1,x:(r4)-
L121
; **************************************************
;   
; }
; **************************************************
	move	(r0)-
	move	(r0)-
	move	(r6)-
	move	x:(r6)-,r5
	move	x:(r6)-,r4
	move	x:(r6)-,r3
	move	x:(r6)-,r2
	move	x:(r6)-,r1
	move	x:(r6)-,y1
	move	x:(r6)-,y0
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	r0,r6
	move	x:(r0),r0
	rts

	org	x:
F___W2
	dc	$001fff
	dc	$a00000
	dc	$002000
	dc	$4851ec
	dc	$002001
	dc	$51eb85
	dc	$002002
	dc	$400000
	dc	$002002
	dc	$700000
	dc	$002003
	dc	$630a3d
	dc	$002004
	dc	$58c28f
	dc	$002006
	dc	$462148
	org	p:
	global	Fscale_factor
Fscale_factor
; **************************************************
; **************************************************
; **************************************************
; 
; void scale_factor(Y_U, Y_L)
; float *Y_U ; 
; float *Y_L ; 
; {
; 
;   static float W[] =
;     {
;       -0.75, 1.13, 2.56, 4.00, 7.00, 12.38, 22.19, 70.13
;     };
;   
;   Y1 = AL * *Y_U + (1.0 - AL) * *Y_L;
; **************************************************
	move	#65533,n0
	move	#5,n6
	move	r0,x:(r6)+
	move	(r6)+
	move	r6,r0
	move	(r6)+n6
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	x1,x:(r6)+
	move	y0,x:(r6)+
	move	r1,x:(r6)+
	move	r2,x:(r6)+
	move	r3,x:(r6)+
	move	x:(r0+n0),r2
	move	#65532,n0
	move	x:(FAL+1),b
	move	x:(r0+n0),r3
	move	x:FAL,b0
	move	(r2)+
	move	x:(r2)-,a
	move	x:(r2),a0
	jsr	fmpy_ab
	move	b0,x:(r0)+
	move	b1,x:(r0)-
	move	x:(FAL+1),a
	move	x:FAL,a0
	move	#>4194304,b
	move	#>8192,b0
	jsr	fsub_ab
	move	(r3)+
	move	x:(r3)-,a
	move	x:(r3),a0
	jsr	fmpy_ab
	tfr	b,a	(r0)+
	move	x:(r0)-,b
	move	x:(r0),b0
	jsr	fadd_ab
; **************************************************
;   
;   *Y_U = 0.96875 * Y1 + 0.03125 * W[abs(I)];
; **************************************************
	move	#>8126464,a
	move	#>8191,a0
	move	b0,x:FY1
	move	b1,x:(FY1+1)
	jsr	fmpy_ab
	move	#2,n0
	move	#F___W2,r1
	move	#>4194304,a
	move	#>8187,a0
	move	#>2,y0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	x:FI,x0
	tfr	x0,b
	abs	b
	move	b1,x0
	mpy	+y0,x0,b
	asr	b
	move	b0,b
	move	b1,x0
	move	r1,b
	add	x0,b
	move	b1,r1
	move	(r1)+
	move	x:(r1)-,b
	move	x:(r1),b0
	jsr	fmpy_ab
	tfr	b,a	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fadd_ab
; **************************************************
;   
;   if (*Y_U < 1.06) *Y_U = 1.06   ; 
; **************************************************
	move	b0,x0
	move	b1,x1
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	x0,x:(r2)+
	move	#>8192,x0
	move	x1,x:(r2)-
	move	#>4445962,x1
	tfr	x1,a	x0,x:(r0)+
	move	x0,a0
	move	x1,x:(r0)-
	jsr	fcmp_ab
	jge	L125
	move	(r0)+
	move	x:(r0)-,x1
	move	x:(r0),x0
	move	x0,x:(r2)+
	move	x1,x:(r2)-
L125
; **************************************************
;   if (*Y_U > 10.0) *Y_U = 10.0   ; 
; **************************************************
	move	#2,n0
	move	(r0)+
	move	(r0)+n0
	move	(r2)+
	move	x:(r2)-,x1
	tfr	x1,b	x:(r2),x0
	move	x0,b0
	move	x1,x:(r0)-
	move	#>5242880,x1
	tfr	x1,a	x0,x:(r0)-n0
	move	#>8195,x0
	move	x0,a0
	move	x0,x:(r0)+
	move	x1,x:(r0)-
	jsr	fcmp_ab
	jle	L126
	move	(r0)+
	move	x:(r0)-,x1
	move	x:(r0),x0
	move	x0,x:(r2)+
	move	x1,x:(r2)-
L126
; **************************************************
;   
;   *Y_L *= 0.984375 ; 
; **************************************************
	move	#>8257536,a
	move	#>8191,a0
	move	(r3)+
	move	x:(r3)-,b
	move	x:(r3),b0
	jsr	fmpy_ab
; **************************************************
;   
;   YL1 = *Y_L  = *Y_L + 0.015625 * *Y_U;
; **************************************************
	move	#>4194304,a
	move	#>8186,a0
	move	b0,x:(r3)+
	move	b1,x:(r3)-
	move	(r2)+
	move	x:(r2)-,b
	move	x:(r2),b0
	jsr	fmpy_ab
	move	(r3)+
	move	x:(r3)-,a
	move	x:(r3),a0
	jsr	fadd_ab
; **************************************************
;     
; }
; **************************************************
	move	#2,n0
	move	b0,x0
	move	b1,x1
	move	(r6)-
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	x0,x:(r3)+
	move	x1,x:(r3)-
	move	x0,x:FYL1
	move	x1,x:(FYL1+1)
	move	x:(r6)-,r3
	move	x:(r6)-,r2
	move	x:(r6)-,r1
	move	x:(r6)-,y0
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	(r0)-
	move	(r0)-
	move	r0,r6
	move	x:(r0),r0
	rts

	org	x:
F___i3
	dc 0
	org	p:
	global	Fdisk_play
Fdisk_play
; **************************************************
; **************************************************
; **************************************************
; 
; 
; #ifdef __DSP56K__ /* code for the g56k compiler */
; 
; void disk_play() 
; {
; #ifdef __SIMULATION__
;   static int i = 0 ; 
;   if(i >= sizeof(Data_in)/sizeof(int))
; **************************************************
	move	r0,x:(r6)+
	move	r6,r0
	move	(r6)+
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	x1,x:(r6)+
	move	r1,x:(r6)+
	move	x:F___i3,a
	tst	a
	jle	L128
; **************************************************
;     ExitFlag = TRUE ; 
; **************************************************
	move	#>1,a
	move	a1,x:FExitFlag
L128
; **************************************************
;   SL = Data_in[i++] & 0xFFFF ; 
; **************************************************
; **************************************************
; 
; #else
;   SL = 0x7a & 0xFFFF;               /* constant value */
; #endif
; }
; **************************************************
	move	#FData_in,r1
	move	x:F___i3,a
	tfr	a,b	#>65535,x0
	move	#>1,x1
	add	x1,b	(r0)-
	move	b1,x1
	move	r1,b
	add	a,b	x1,x:F___i3
	move	b1,r1
	move	(r6)-
	move	x:(r1),a
	and	x0,a
	move	a1,a
	move	a1,x:FSL
	move	x:(r6)-,r1
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	r0,r6
	move	x:(r0),r0
	rts

	org	x:
F___table4
	dc	$001fff
	dc	$828f5c
	dc	$001fff
	dc	$4f5c29
	dc	$002000
	dc	$5851ec
	dc	$002000
	dc	$7a3d71
	dc	$002001
	dc	$4ae148
	dc	$002001
	dc	$570a3d
	dc	$002001
	dc	$63d70a
	org	p:
	global	Fmain
Fmain
	move	r0,x:(r6)+
	move	ssh,x:(r6)+
; **************************************************
; **************************************************
; **************************************************
; 
; #else /* MACRO __DSP56K__ not defined, code is also for the SUN */
; 
; void init_disk_play()
; {
;   istrm = fopen("data.in.2freq", "rb") ; 
; }
; 
; void disk_play()
; {
;   int d ;
;   if (istrm != NULL && !feof(istrm))
;     {
;       fread((char*) &d, sizeof(int), 1, istrm) ; 
;       SL = ((d & 0xFFFF0000) >> 16) |
; 	((d & 0x80000000) ? 0xFFFF0000 : 0) ; 
;     }
;   if (feof(istrm)) 
;     ExitFlag = TRUE ; 
;   
; }
; 
; void post_disk_play()
; {
;   if (istrm != NULL)
;     fclose(istrm) ; 
; }
; 
; void init_disk_record()
; {
;   ostrm = fopen("data.out", "wb") ; 
;   t1strm = fopen("datain.tmp","wb") ; 
;   t2strm = fopen("dataout.tmp","wb") ; 
;   t3strm = fopen("error.tmp","wb") ; 
;   fprintf(t1strm,"TitleText: Input Data\n") ;
;   fprintf(t2strm,"TitleText: Output Data\n") ; 
;   fprintf(t3strm,"TitleText: Error\n") ; 
; }
; 
; void disk_record()
; {
;   int d ; 
;   if (ostrm != NULL)
;     {
;       d = (SD & 0x0000FFFF) << 16 ; 
;       fwrite((char*) &d, 4, 1,ostrm) ; 
;     }
; 
; }
; 
; void post_disk_record()
; {
;   if (ostrm != NULL)
;     fclose(ostrm) ; 
;   if (t1strm != NULL)
;     fclose(t1strm) ; 
;   if (t2strm != NULL)
;     fclose(t2strm) ; 
;   if (t3strm != NULL)
;     fclose(t3strm) ; 
; }
; #endif
; 
; main(void)
; {
;   int i = 0 ; 
; **************************************************
; **************************************************
;  
; #ifndef __DSP56K__
;   init_disk_play() ; 
;   init_disk_record() ; 
; #endif
; 
;   /* disk play */
;   ExitFlag = FALSE ; 
; **************************************************
; **************************************************
;   while (!ExitFlag)
; **************************************************
	clr	a	#2,n0
	move	#5,n6
	move	r6,r0
	move	(r6)+n6
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	x1,x:(r6)+
	move	#>1,x1
	move	y0,x:(r6)+
	move	#>0,y0
	move	y1,x:(r6)+
	move	#>0,y1
	move	r1,x:(r6)+
	move	r2,x:(r6)+
	move	#FD_Y,r2
	move	r3,x:(r6)+
	move	#FE_Y,r3
	move	r4,x:(r6)+
	move	#FD_YL,r4
	move	r5,x:(r6)+
	move	#FD_DQ1,r5
	move	r7,x:(r6)+
	move	#FD_TR1,r7
	move	a1,x:FExitFlag
	move	(r0)+
	move	(r0)+n0
	move	y1,x:(r0)-
	move	y0,x:(r0)-n0
L142
; **************************************************
;     {
;       disk_play() ; 
; **************************************************
	jsr	Fdisk_play
; **************************************************
; 
; #ifdef __DSP56K__
; #ifndef __SIMULATION__ 
;       if (i == N_OF_DATA)           
; 	ExitFlag = TRUE ;      /* N_OF_DATA constant values simulation */
;       i++ ; 
; #endif
; #else
;       i++ ; 
;       if (i < 1000) fprintf(t1strm,"%i\t%i\n",i,SL) ; 
; #endif
; 
;       if(ExitFlag)
; **************************************************
	move	x:FExitFlag,a
	tst	a
	jne	L131
; **************************************************
; 	break ; 
; **************************************************
; **************************************************
; 
;     /*
;      * ENCODER
;      */
;       
;       adpt_predict(&E_DQ1,&E_TR1,
; 		   &E_A1, &E_A2,
; 		   &E_P1, &E_P2, &E_P3, &E_SR2, 
; 		   E_B, E_D_Q,
; 		   &E_SE, &E_SEZ);
; **************************************************
	move	#FE_SEZ,r1
	move	r1,x:(r6)+
	move	#FE_SE,r1
	move	r1,x:(r6)+
	move	#FE_D_Q,r1
	move	r1,x:(r6)+
	move	#FE_B,r1
	move	r1,x:(r6)+
	move	#FE_SR2,r1
	move	r1,x:(r6)+
	move	#FE_P3,r1
	move	r1,x:(r6)+
	move	#FE_P2,r1
	move	r1,x:(r6)+
	move	#FE_P1,r1
	move	r1,x:(r6)+
	move	#FE_A2,r1
	move	r1,x:(r6)+
	move	#FE_A1,r1
	move	r1,x:(r6)+
	move	#FE_TR1,r1
	move	r1,x:(r6)+
	move	#FE_DQ1,r1
	move	r1,x:(r6)+
	jsr	Fadpt_predict
; **************************************************
;       
;       /* DIFF SIGNAL is used inlined */
;       /* SUBTA */
;       
;       D = ((float) SL)  - s_e ; 
; **************************************************
	move	#12,n6
	move	x:FSL,a
	move	(r6)-n6
	jsr	floatsidf_a
	move	x:(Fs_e+1),b
	move	x:Fs_e,b0
	jsr	fsub_ba
; **************************************************
;       
;       /* ADPT_QUANT is used inlined */
;         
;       {
;  
; 	static float table[7] = { -0.98, 0.62, 1.38, 1.91, 2.34, 2.72, 3.12} ; 
; 	register float* ptable = table ;  
; **************************************************
; **************************************************
; 
; 	register float d_ln;
; 	register int f ;
; 
; 	d_ln = (D < 0) ? (-1) * D : D ;
; **************************************************
	move	#2,n0
	move	#F___table4,r1
	move	(r0)+
	move	(r0)+n0
	move	a0,x:FD
	move	a1,x:(FD+1)
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fcmp_ba
	jge	L133
	move	#>8388608,b
	move	#>8191,b0
	move	x:(FD+1),a
	move	x:FD,a0
	jsr	fmpy_ba
	jmp	L134
L133
	move	x:(FD+1),a
	move	x:FD,a0
L134
; **************************************************
; 	d_ln = log10(d_ln) * 3.321928095  - E_Y ; 
; **************************************************
	move	a0,x:(r0)+
	move	a1,x:(r0)-
	move	(r0)+
	move	x:(r0)-,y1
	move	x:(r0),y0
	move	y0,x:(r6)+
	move	y1,x:(r6)+
	jsr	Flog10
	move	#>6966588,b
	move	#>8193,b0
	move	(r6)-
	move	(r6)-
	jsr	fmpy_ba
	move	x:(FE_Y+1),b
	move	x:FE_Y,b0
	jsr	fsub_ba
; **************************************************
; 	
; 	for ( f = 0 ; f < 7 ; f++)
; **************************************************
	move	#4,n0
	move	#>6,x0
	move	#0,y0
	move	a0,x:(r0)+
	move	a1,x:(r0)-
	move	y0,x:(r0+n0)
L139
; **************************************************
; 	  if (d_ln < *ptable++)
; **************************************************
	move	#2,n1
	move	(r0)+
	move	x:(r0)-,b
	move	x:(r0),b0
	move	(r1)+
	move	x:(r1)-,a
	move	x:(r1)+n1,a0
	jsr	fcmp_ab
	jlt	L136
; **************************************************
; 	  break ; 
; **************************************************
; **************************************************
; **************************************************
	move	#4,n0
	move	x:(r0+n0),b
	add	x1,b
	cmp	x0,b	b1,x:(r0+n0)
	jle	L139
L136
; **************************************************
; 		 
; 	I = (D < 0) ? (-1) * f : f ;
; **************************************************
	move	#2,n0
	move	x:(FD+1),a
	move	x:FD,a0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jsr	fcmp_ba
	jge	L140
	move	#4,n0
	move	x:(r0+n0),a
	neg	a
	jmp	L141
L140
	move	#4,n0
	move	x:(r0+n0),a
L141
; **************************************************
; 	 
;       }
;       
;       iadpt_quant(&E_Y);
; **************************************************
	move	a1,x:FI
	move	r3,x:(r6)+
	jsr	Fiadpt_quant
; **************************************************
;       tone_detector(&E_YL) ;
; **************************************************
	move	#>FE_YL,y0
	move	(r6)-
	move	y0,x:(r6)+
	jsr	Ftone_detector
; **************************************************
;       speed_control(&E_Y, &E_DMS, &E_DML, &E_AP);
; **************************************************
	move	#FE_AP,r1
	move	(r6)-
	move	r1,x:(r6)+
	move	#FE_DML,r1
	move	r1,x:(r6)+
	move	#FE_DMS,r1
	move	r1,x:(r6)+
	move	r3,x:(r6)+
	jsr	Fspeed_control
; **************************************************
;       scale_factor(&E_YU, &E_Y_L);
; **************************************************
	move	#4,n6
	move	#FE_Y_L,r1
	move	(r6)-n6
	move	r1,x:(r6)+
	move	#FE_YU,r1
	move	r1,x:(r6)+
	jsr	Fscale_factor
; **************************************************
; 
;       /*
;        * DELAYS in ENCODER
;        */
;       
;       /* adpt predictor delays */
; 
;       E_DQ1 = DQ ;  
; **************************************************
; **************************************************
;       E_TR1 = TR ;
; **************************************************
; **************************************************
; 
;       /* scale factor delays */
;       E_YL = YL1 ; 
; **************************************************
; **************************************************
;       E_Y  = Y1 ; 
; **************************************************
; **************************************************
; 
;       /*
;        * DECODER
;        */
;       
;       adpt_predict(&D_DQ1, &D_TR1,
; 		   &D_A1, &D_A2,
; 		   &D_P1, &D_P2, &D_P3, &D_SR2, 
; 		   D_B, D_D_Q,
; 		   &D_SE, &D_SEZ) ; 
; **************************************************
	move	#FD_SEZ,r1
	move	x:(FDQ+1),a
	move	x:FDQ,a0
	move	a0,x:FE_DQ1
	move	a1,x:(FE_DQ1+1)
	move	x:FTR,a
	move	a1,x:FE_TR1
	move	x:(FYL1+1),a
	move	x:FYL1,a0
	move	a0,x:FE_YL
	move	a1,x:(FE_YL+1)
	move	x:(FY1+1),a
	move	x:FY1,a0
	move	a0,x:(r3)+
	move	a1,x:(r3)-
	move	(r6)-
	move	(r6)-
	move	r1,x:(r6)+
	move	#FD_SE,r1
	move	r1,x:(r6)+
	move	#FD_D_Q,r1
	move	r1,x:(r6)+
	move	#FD_B,r1
	move	r1,x:(r6)+
	move	#FD_SR2,r1
	move	r1,x:(r6)+
	move	#FD_P3,r1
	move	r1,x:(r6)+
	move	#FD_P2,r1
	move	r1,x:(r6)+
	move	#FD_P1,r1
	move	r1,x:(r6)+
	move	#FD_A2,r1
	move	r1,x:(r6)+
	move	#FD_A1,r1
	move	r1,x:(r6)+
	move	r7,x:(r6)+
	move	r5,x:(r6)+
	jsr	Fadpt_predict
; **************************************************
;       
;       iadpt_quant(&D_Y);
; **************************************************
	move	#12,n6
	move	(r6)-n6
	move	r2,x:(r6)+
	jsr	Fiadpt_quant
; **************************************************
;       
;       /* ADD is used inlined */
; 
;       SP = DQ + s_e ; 
; **************************************************
	move	x:(Fs_e+1),b
	move	x:Fs_e,b0
	move	x:(FDQ+1),a
	move	x:FDQ,a0
	move	(r6)-
	jsr	fadd_ba
; **************************************************
;       
;       /* coding adj is used inlined */
; 
;       SD = (int) SP ;
; **************************************************
	move	a0,x:FSP
	move	a1,x:(FSP+1)
	jsr	fixdfsi_a
; **************************************************
;       
;       tone_detector(&D_YL);
; **************************************************
	move	a1,x:FSD
	move	r4,x:(r6)+
	jsr	Ftone_detector
; **************************************************
;       speed_control(&D_Y, &D_DMS, &D_DML, &D_AP);
; **************************************************
	move	#FD_AP,r1
	move	(r6)-
	move	r1,x:(r6)+
	move	#FD_DML,r1
	move	r1,x:(r6)+
	move	#FD_DMS,r1
	move	r1,x:(r6)+
	move	r2,x:(r6)+
	jsr	Fspeed_control
; **************************************************
;       scale_factor(&D_YU, &D_Y_L);
; **************************************************
	move	#4,n6
	move	#FD_Y_L,r1
	move	(r6)-n6
	move	r1,x:(r6)+
	move	#FD_YU,r1
	move	r1,x:(r6)+
	jsr	Fscale_factor
; **************************************************
;       
;       /*
;        * DELAYS in DECODER
;        */
;       
;       /* adpt_predictor delays */
; 
;       D_DQ1 = DQ ; 
; **************************************************
; **************************************************
;       D_TR1 = TR ; 
; **************************************************
; **************************************************
;  
;       /* scale factor delays */
; 
;       D_YL = YL1 ; 
; **************************************************
; **************************************************
;       D_Y  = Y1 ; 
; **************************************************
	move	x:(FDQ+1),a
	move	x:FDQ,a0
	move	a0,x:(r5)+
	move	a1,x:(r5)-
	move	x:FTR,a
	move	a1,x:(r7)
	move	x:(FYL1+1),a
	move	x:FYL1,a0
	move	a0,x:(r4)+
	move	a1,x:(r4)-
	move	x:(FY1+1),a
	move	x:FY1,a0
	move	a0,x:(r2)+
	move	a1,x:(r2)-
	move	x:FExitFlag,a
	tst	a	(r6)-
	move	(r6)-
	jeq	L142
L131
; **************************************************
;        
; #ifndef __DSP56K__
;       /* disk record */
;       disk_record() ;  
; 
;       if (i < 1000) fprintf(t2strm,"%i\t%i\n",i,SD) ; 
;       if (i < 1000) fprintf(t3strm,"%i\t%i\n",i,SL-SD) ;     
; #endif    
; }
; 
; #ifndef __DSP56K__  
;   post_disk_play() ; 
;   post_disk_record() ; 
; #endif
; 
;   return(0) ; 
; **************************************************
; **************************************************
; 
;   
; } 
; **************************************************
	clr	a	(r0)-
	move	(r6)-
	move	x:(r6)-,r7
	move	x:(r6)-,r5
	move	x:(r6)-,r4
	move	x:(r6)-,r3
	move	x:(r6)-,r2
	move	x:(r6)-,r1
	move	x:(r6)-,y1
	move	x:(r6)-,y0
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	x:(r0)-,ssh
	tst	a	r0,r6
	move	x:(r0),r0
	rts

	org	x:
	global	FExitFlag
FExitFlag	bsc	1

	endsec

; 

