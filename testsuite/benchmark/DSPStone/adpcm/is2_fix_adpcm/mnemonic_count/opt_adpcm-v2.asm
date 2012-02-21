;
;	annotate version 1.03
;
; **************************************************
; **************************************************
	section	opt_adpcm_v2_c
	opt so,nomd
	page 132,66,3,3
;*** DSP56000/1 Motorola 1.03 GNU 1.37.1
	org	p:
	global	FE_DQ
	org	x:
FE_DQ
	dc 0
	global	FE_T
FE_T
	dc 0
	global	FE_Y
FE_Y
	dc 0
	global	FE_YL
FE_YL
	dc 0
	global	FE_DQ2
FE_DQ2
	dc 0
	global	FE_DQ3
FE_DQ3
	dc 0
	global	FE_DQ4
FE_DQ4
	dc 0
	global	FE_DQ5
FE_DQ5
	dc 0
	global	FE_DQ6
FE_DQ6
	dc 0
	global	FE_DQ7
FE_DQ7
	dc 0
	global	FE_PK1
FE_PK1
	dc 0
	global	FE_PK2
FE_PK2
	dc 0
	global	FE_SR2
FE_SR2
	dc 0
	global	FE_A1
FE_A1
	dc 0
	global	FE_A2
FE_A2
	dc 0
	org	p:
	global	FE_B
	org	x:
FE_B
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	global	FE_SE
FE_SE
	dc 0
	global	FE_SEZ
FE_SEZ
	dc 0
	global	FE_t_dr
FE_t_dr
	dc 0
	global	FE_LAST_TR
FE_LAST_TR
	dc 0
	global	FE_DMS
FE_DMS
	dc 0
	global	FE_DML
FE_DML
	dc 0
	global	FE_AP
FE_AP
	dc 0
	global	FE_Y_L
FE_Y_L
	dc 0
	global	FE_LAST_Y
FE_LAST_Y
	dc 0
	global	FD_DQ
FD_DQ
	dc 0
	global	FD_T
FD_T
	dc 0
	global	FD_Y
FD_Y
	dc 0
	global	FD_YL
FD_YL
	dc 0
	global	FD_DQ2
FD_DQ2
	dc 0
	global	FD_DQ3
FD_DQ3
	dc 0
	global	FD_DQ4
FD_DQ4
	dc 0
	global	FD_DQ5
FD_DQ5
	dc 0
	global	FD_DQ6
FD_DQ6
	dc 0
	global	FD_DQ7
FD_DQ7
	dc 0
	global	FD_PK1
FD_PK1
	dc 0
	global	FD_PK2
FD_PK2
	dc 0
	global	FD_SR2
FD_SR2
	dc 0
	global	FD_A1
FD_A1
	dc 0
	global	FD_A2
FD_A2
	dc 0
	org	p:
	global	FD_B
	org	x:
FD_B
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	global	FD_SE
FD_SE
	dc 0
	global	FD_SEZ
FD_SEZ
	dc 0
	global	FD_t_dr
FD_t_dr
	dc 0
	global	FD_LAST_TR
FD_LAST_TR
	dc 0
	global	FD_DMS
FD_DMS
	dc 0
	global	FD_DML
FD_DML
	dc 0
	global	FD_AP
FD_AP
	dc 0
	global	FD_Y_L
FD_Y_L
	dc 0
	global	FD_LAST_Y
FD_LAST_Y
	dc 0
	org	p:
	global	Ff_mult
Ff_mult
; **************************************************
; /*
;  * opt_adpcm.c
;  * Handwritten version of an optimized ADPCM transcoder applying the
;  * CCITT recommendation G.721
;  * 26-Oct-93 Chris Schlaeger
;  * VERSION 1.2
;  */
; 
; #include <stdio.h>
; 
; #define FALSE 0
; #define TRUE (!FALSE)
; 
; FILE* istrm;
; FILE* ostrm;
; int   ExitFlag;
; 
; #ifdef __LOOPCOUNT__
; int Loop1 = 0;
; int Loop2 = 0;
; int Loop3 = 0;
; int Loop4 = 0;
; int Loop5 = 0;
; int Loop6 = 0;
; int Loop7 = 0;
; #endif
; 
; /* Global signals */
; int A_2, AL, D, I, SD, S_E, SL, SP, TDP;
; 
; /* ENCODER states */
; int E_DQ = 0;
; int E_T = 0;
; int E_Y = 0;
; int E_YL = 0;
; int E_DQ2 = 0;
; int E_DQ3 = 0;
; int E_DQ4 = 0;
; int E_DQ5 = 0;
; int E_DQ6 = 0;
; int E_DQ7 = 0;
; int E_PK1 = 0;
; int E_PK2 = 0;
; int E_SR2 = 0;
; int E_A1 = 0;
; int E_A2 = 0;
; int E_B[6] = { 0, 0, 0, 0, 0, 0 };
; int E_SE = 0;
; int E_SEZ = 0;
; int E_t_dr = 0;
; int E_LAST_TR = 0;
; int E_DMS = 0;
; int E_DML = 0;
; int E_AP = 0;
; int E_Y_L = 0;
; int E_LAST_Y = 0;
; 
; /* DECODER states */
; int D_DQ = 0;
; int D_T = 0;
; int D_Y = 0;
; int D_YL = 0;
; int D_DQ2 = 0;
; int D_DQ3 = 0;
; int D_DQ4 = 0;
; int D_DQ5 = 0;
; int D_DQ6 = 0;
; int D_DQ7 = 0;
; int D_PK1 = 0;
; int D_PK2 = 0;
; int D_SR2 = 0;
; int D_A1 = 0;
; int D_A2 = 0;
; int D_B[6] = { 0, 0, 0, 0, 0, 0 };
; int D_SE = 0;
; int D_SEZ = 0;
; int D_t_dr = 0;
; int D_LAST_TR = 0;
; int D_DMS = 0;
; int D_DML = 0;
; int D_AP = 0;
; int D_Y_L = 0;
; int D_LAST_Y = 0;
; 
; int f_mult(int An, int SRn)
; {
;   int           AnS, MAG, AnMANT;
;   register int  EXP;
;   register long WAnMANT;
; 
;   AnS = An >> 15;
; **************************************************
	move	#65533,n0
	move	#10,n6
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
	move	x:(r0+n0),b
	move	b1,b
	rep	#15
	asr	b
; **************************************************
;   MAG = AnS ? (16384 - (An >> 2)) & 8191 : An >> 2;
; **************************************************
	move	b1,b
	tst	b	b1,x:(r0)
	jeq	L2
	move	x:(r0+n0),b
	move	b1,b
	asr	b
	asr	b
	move	b1,b
	move	b1,x1
	move	#>16384,b
	sub	x1,b	#>8191,x1
	and	x1,b
	move	b1,b
	jmp	L15
L2
	move	#65533,n0
	move	x:(r0+n0),b
	move	b1,b
	asr	b
	asr	b
	move	b1,b
L15
; **************************************************
;     for (EXP = 0; (MAG >> EXP) != 0; EXP++)
; **************************************************
	move	#2,n0
	move	#0,x1
	move	b1,x:(r0+n0)
	move	(r0)+
	move	x1,x:(r0)-
	move	x:(r0+n0),b
	tst	b
	jeq	L14
	move	#>1,y0
L6
	move	#2,n0
	move	(r0)+
	move	x:(r0)-,b
	add	y0,b	(r0)+
	tfr	b,a	b1,x:(r0)-
	tst	a	x:(r0+n0),b
	jeq	L16
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L16
	tst	b
	jne	L6
L14
; **************************************************
; #ifdef __LOOPCOUNT__
; 	Loop1++;
; #else
; 	;
; #endif
; 
;   AnMANT = MAG ? (MAG << 6) >> EXP : 1 << 5;
; **************************************************
	move	#2,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L8
	rep	#6
	asl	b
	move	b1,b
	move	(r0)+
	move	x:(r0)-,a
	tst	a
	jeq	L17
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L17
	move	b1,x:(r0+n0)
	jmp	L9
L8
	move	#2,n0
	move	#>32,x1
	move	x1,x:(r0+n0)
L9
; **************************************************
; 
;   EXP += ((SRn >> 6) & 15);
; **************************************************
	move	#65532,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#6
	asr	b
; **************************************************
;   WAnMANT = ((((long) (SRn & 63) * AnMANT) + 48) >> 4) << 7;
; **************************************************
	move	#>15,x1
	move	b1,b
	and	x1,b	(r0)+
	move	b1,b
	move	b1,x1
	move	x:(r0)-,b
	add	x1,b	#>63,x1
	move	(r0)+
	move	b1,x:(r0)-
	move	x:(r0+n0),b
	and	x1,b	#2,n0
	move	b1,b
	move	b1,x0
	move	b2,x1
	move	x:(r0+n0),b
	move	b1,y0
	tfr	x1,b	b2,y1
	tfr	y1,a	x0,b0
	move	y0,a0
	jsr	lmpy_ab
	move	#8,n0
	move	#>48,x0
	move	#>0,x1
	add	x,b	(r0)+
	move	(r0)+n0
	move	x1,x:(r0)-
	move	x0,x:(r0)-n0
	move	b0,x:(r6)
	move	b1,b
	move	x:(r6),b0
	rep	#4
	asr	b
	rep	#7
	asl	b
; **************************************************
;   MAG = (int) (EXP <= 26 ?
;                WAnMANT  >> (26 - EXP) :
; 	       (WAnMANT << (EXP - 26)) & 32767);
; **************************************************
	move	#3,n0
	move	#>26,x1
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	(r0)+
	move	x:(r0)-,b
	cmp	x1,b
	jgt	L10
	move	#>26,b
	move	(r0)+
	move	x:(r0)-,x1
	sub	x1,b	(r0)+
	tfr	b,a	(r0)+n0
	tst	a	x:(r0)-,b
	move	x:(r0)-n0,b0
	move	b0,x:(r6)
	move	b1,b
	move	x:(r6),b0
	jeq	L18
	rep	a
	asr	b
L18
	move	#8,n0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	x:(r0+n0),x1
	move	#2,n0
	move	x1,x:(r0+n0)
	jmp	L11
L10
	move	#3,n0
	move	#>16777190,x1
	move	(r0)+
	move	x:(r0)-,b
	add	x1,b	(r0)+
	tfr	b,a	(r0)+n0
	tst	a	x:(r0)-,b
	move	x:(r0)-n0,b0
	jeq	L19
	rep	a
	asl	b
L19
	move	#8,n0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	x:(r0+n0),x1
	tfr	x1,b	#2,n0
	move	x1,x:(r0+n0)
	move	#>32767,x1
	and	x1,b
	move	b1,b
	move	b1,x:(r0+n0)
L11
; **************************************************
;   return ((SRn >> 10) ^ AnS ? (65536 - MAG) & 65535 : MAG);
; **************************************************
	move	#65532,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#10
	asr	b
	move	x:(r0),x1
	move	b1,b
	eor	x1,b
	move	b1,b
	tst	b
	jeq	L12
	move	#2,n0
	move	#>65536,b
	move	x:(r0+n0),x1
	sub	x1,b	#8,n0
	move	#>65535,x1
	and	x1,b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L13
L12
	move	#2,n0
	move	x:(r0+n0),x1
	move	#8,n0
	move	x1,x:(r0+n0)
L13
; **************************************************
; }
; **************************************************
	move	#8,n0
	move	(r6)-
	move	x:(r0+n0),a
	tst	a	x:(r6)-,y1
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

	global	Fdiff_signal
Fdiff_signal
; **************************************************
; **************************************************
; **************************************************
; 
; void diff_signal(void)
; {
;   /*
;    * diff_signal
;    *
;    * Input signals:  SL, S_E
;    * Output signals: D
;    */
; 
;   /* SUBTA */
;   D = ((SL & (1 << 13) ? 49152 | SL : SL) + 65536 -
;        (S_E & (1 << 14) ? 32768 | S_E : S_E)) & 65535;
; **************************************************
	move	#2,n6
	move	r0,x:(r6)+
	move	r6,r0
	move	(r6)+n6
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	#>8192,x0
	move	x1,x:(r6)+
	move	x:FSL,x1
	move	(r0)+
	move	x1,x:(r0)-
	move	(r0)+
	move	x:(r0)-,a
	and	x0,a
	move	a1,a
	tst	a
	jeq	L21
	move	#>49152,x0
	move	(r0)+
	move	x:(r0)-,b
	or	x0,b
	move	b1,b
	move	b1,x:(r0)
	jmp	L22
L21
	move	x:FSL,x1
	move	x1,x:(r0)
L22
	move	x:FS_E,x1
	tfr	x1,a	#>16384,x0
	and	x0,a	(r0)+
	move	a1,a
	tst	a	x1,x:(r0)-
	jeq	L23
	move	#>32768,x0
	move	(r0)+
	move	x:(r0)-,b
	or	x0,b	(r0)+
	move	b1,b
	move	b1,x:(r0)-
	jmp	L24
L23
	move	x:FS_E,x1
	move	(r0)+
	move	x1,x:(r0)-
L24
; **************************************************
; }
; **************************************************
	move	#>65535,x0
	move	#>16711680,a
	move	(r6)-
	move	(r0)+
	move	x:(r0)-,b
	add	a,b
	tfr	b,a	x:(r0),b
	sub	a,b	(r0)-
	tfr	b,a
	and	x0,a
	move	a1,a
	move	a1,x:FD
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	r0,r6
	move	x:(r0),r0
	rts

	global	Fadpt_predict
Fadpt_predict
	move	r0,x:(r6)+
	move	ssh,x:(r6)+
; **************************************************
; **************************************************
; **************************************************
; 
; void adpt_predict(int* DQ, int* T,
;                   int* DQ2, int* DQ3, int* DQ4, int* DQ5, int* DQ6, int* DQ7,
;                   int* PK1, int* PK2, int* SR2, int* A1, int* A2, int B[],
;                   int* SE, int* SEZ)
; {
;   /*
;    * adaptive predictor
;    *
;    * Input signals:  DQ, T
;    * Output signals: S_E, A_2
;    */
; 
;   register int DQS, DQI, DQSEZ, PK0; 
;   register int SR1;
;   register int A1S, AP;
;   register int A2R;
;   register int WA1, WA2;
;   register int MAG, EXP, MANT; 
;   int BP[6];
; 
;   /* ADDC */
;   DQS = *DQ >> 14;
; **************************************************
	move	#65533,n0
	move	#21,n6
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
	move	x:(r0+n0),r2
	move	#65529,n0
	move	x:(r2),y0
	tfr	y0,b	x:(r0+n0),r7
	move	#65527,n0
	move	b1,b
	move	x:(r0+n0),r5
	move	#65522,n0
	move	x:(r0+n0),r1
	move	#65520,n0
	move	x:(r0+n0),r3
	move	#19,n0
	move	y0,x:(r0+n0)
	rep	#14
	asr	b
; **************************************************
;   DQI = DQS == 0 ? *DQ : (65536 - (*DQ & 16383)) & 65535;
; **************************************************
	move	#6,n0
	move	b1,b
	tst	b	b1,x:(r0+n0)
	jne	L26
	move	#7,n0
	move	y0,x:(r0+n0)
	jmp	L27
L26
	move	#19,n0
	move	x:(r2),y0
	tfr	y0,b	#>16383,x0
	and	x0,b	#>65535,x0
	move	b1,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	#7,n0
	move	#>65536,b
	sub	y0,b
	and	x0,b	b1,x:(r0+n0)
	move	b1,b
	move	b1,x:(r0+n0)
L27
; **************************************************
;   MAG = (( (*SEZ)  >> 14) == 0) ?  (*SEZ)  : (1 << 15) +  (*SEZ);
; **************************************************
	move	#65518,n0
	move	x:(r0+n0),r4
	move	#18,n0
	move	x:(r4),y0
	tfr	y0,b	y0,x:(r0+n0)
	move	b1,b
	rep	#14
	asr	b
	move	b1,b
	tst	b
	jne	L28
	move	y0,x:(r0+n0)
	jmp	L29
L28
	move	#65518,n0
	move	x:(r0+n0),r4
	move	#19,n0
	move	x:(r4),y0
	tfr	y0,b	y0,x:(r0+n0)
	move	#18,n0
	move	#>32768,y0
	add	y0,b
	move	b1,x:(r0+n0)
L29
; **************************************************
;   DQSEZ = (DQI + MAG) & 65535;
; **************************************************
; **************************************************
;   PK0 = DQSEZ >> 15;
; **************************************************
	move	#7,n0
	move	#>65535,x0
	move	x:(r0+n0),b
	move	#18,n0
	move	x:(r0+n0),y0
	add	y0,b	#8,n0
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	b1,b
	rep	#15
	asr	b
; **************************************************
;   WA2 = DQSEZ == 0 ? 1 : 0;
; **************************************************
	move	#9,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#8,n0
	move	x:(r0+n0),b
	tst	b
	jne	L30
	move	#>1,y0
	jmp	L111
L30
	move	#0,y0
L111
; **************************************************
;   
;   /* ADDB */
;   DQI = DQS == 0 ? *DQ : (65536 - (*DQ & 16383)) & 65535;
; **************************************************
	move	#12,n0
	move	y0,x:(r0+n0)
	move	#6,n0
	move	x:(r0+n0),b
	tst	b
	jne	L32
	move	#7,n0
	move	x:(r2),y0
	move	y0,x:(r0+n0)
	jmp	L33
L32
	move	#19,n0
	move	x:(r2),y0
	tfr	y0,b	#>16383,x0
	and	x0,b	#>65535,x0
	move	b1,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	#7,n0
	move	#>65536,b
	sub	y0,b
	and	x0,b	b1,x:(r0+n0)
	move	b1,b
	move	b1,x:(r0+n0)
L33
; **************************************************
;   MAG = (( (*SE)  >> 14) == 0) ?  (*SE)  : (1 << 15) +  (*SE) ;
; **************************************************
	move	#65519,n0
	move	x:(r0+n0),r4
	move	#18,n0
	move	x:(r4),y0
	tfr	y0,b	y0,x:(r0+n0)
	move	b1,b
	rep	#14
	asr	b
	move	b1,b
	tst	b
	jne	L34
	move	y0,x:(r0+n0)
	jmp	L35
L34
	move	#65519,n0
	move	x:(r0+n0),r4
	move	#19,n0
	move	x:(r4),y0
	tfr	y0,b	y0,x:(r0+n0)
	move	#18,n0
	move	#>32768,y0
	add	y0,b
	move	b1,x:(r0+n0)
L35
; **************************************************
;   DQSEZ = (DQI + MAG) & 65535;
; **************************************************
; **************************************************
;   
;   /* FLOATB */
;   MAG = ((DQSEZ >> 15) == 0) ? DQSEZ : (65536 - DQSEZ) & 32767;
; **************************************************
	move	#7,n0
	move	#>65535,x0
	move	x:(r0+n0),b
	move	#18,n0
	move	x:(r0+n0),y0
	add	y0,b	#8,n0
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	b1,b
	rep	#15
	asr	b
	move	b1,b
	tst	b
	jne	L36
	move	x:(r0+n0),y0
	move	#18,n0
	move	y0,x:(r0+n0)
	jmp	L37
L36
	move	#8,n0
	move	#>32767,x0
	move	x:(r0+n0),y0
	move	#18,n0
	move	#>65536,b
	sub	y0,b
	and	x0,b	b1,x:(r0+n0)
	move	b1,b
	move	b1,x:(r0+n0)
L37
; **************************************************
;   for (EXP = 0; (MAG >> EXP) != 0; EXP++)
; **************************************************
	move	#13,n0
	move	#0,y0
	move	y0,x:(r0+n0)
	move	#18,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L110
	move	#>1,x0
L40
	move	#13,n0
	move	x:(r0+n0),b
	add	x0,b
	tfr	b,a	b1,x:(r0+n0)
	tst	a	#18,n0
	move	x:(r0+n0),b
	jeq	L121
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L121
	tst	b
	jne	L40
L110
; **************************************************
; #ifdef __LOOPCOUNT__ 
;     Loop2++;
; #else
;     ;
; #endif
;   MANT = MAG == 0 ? 1 << 5 : (MAG << 6) >> EXP;
; **************************************************
	move	#18,n0
	move	x:(r0+n0),b
	tst	b
	jne	L42
	move	#14,n0
	move	#>32,y0
	move	y0,x:(r0+n0)
	jmp	L43
L42
	move	#18,n0
	move	x:(r0+n0),b
	rep	#6
	asl	b
	move	#14,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#13,n0
	move	x:(r0+n0),a
	tst	a
	jeq	L122
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L122
	move	#14,n0
	move	b1,x:(r0+n0)
L43
; **************************************************
;   SR1 = ( (DQSEZ >> 15) << 10)  + (EXP << 6) + MANT;
; **************************************************
	move	#8,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#15
	asr	b
	move	b1,b
	rep	#10
	asl	b
	move	#19,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#13,n0
	move	x:(r0+n0),b
	rep	#6
	asl	b
; **************************************************
;   
;   /* UPA2 */
;   WA1 = PK0 ^  (*PK1) ;
; **************************************************
; **************************************************
;   MAG = (PK0 ^  (*PK2) ) == 0 ? 16384 : 114688;
; **************************************************
	move	#18,n0
	move	b1,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	#19,n0
	move	x:(r0+n0),b
	add	y0,b	#14,n0
	move	b1,y1
	move	x:(r0+n0),y0
	add	y0,b	#65525,n0
	move	b1,y1
	move	x:(r0+n0),r4
	move	#9,n0
	move	x:(r4),x0
	move	x:(r0+n0),b
	eor	x0,b	#11,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#65524,n0
	move	x:(r0+n0),r4
	move	#9,n0
	move	x:(r4),x0
	move	x:(r0+n0),b
	eor	x0,b
	move	b1,b
	tst	b
	jne	L44
	move	#>16384,y0
	jmp	L112
L44
	move	#>114688,y0
L112
; **************************************************
;   A1S =  (*A1)  >> 15;
; **************************************************
	move	#18,n0
	move	y0,x:(r0+n0)
	move	#15,n0
	move	x:(r1),y0
	tfr	y0,b	y0,x:(r0+n0)
	move	b1,b
	rep	#15
	asr	b
; **************************************************
;   if (A1S == 0)
; **************************************************
	move	#10,n0
	move	b1,b
	tst	b	b1,x:(r0+n0)
	jne	L46
; **************************************************
;     EXP =  (*A1)  <= 8191 ?  (*A1)  << 2 : 8191 << 2;
; **************************************************
	move	#15,n0
	move	#>8191,y0
	move	x:(r0+n0),b
	cmp	y0,b
	jgt	L47
	asl	b	#13,n0
	asl	b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L49
L47
	move	#>32764,y0
	jmp	L113
L46
; **************************************************
;   else
;     EXP =  (*A1)  >= 57345 ? ( (*A1)  << 2) & 131071 : 24577 << 2;
; **************************************************
	move	x:(r1),y0
	tfr	y0,b	#>57344,y0
	cmp	y0,b
	jle	L50
	asl	b	#13,n0
	asl	b	#>131071,x0
	move	b1,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L49
L50
	move	#>98308,y0
L113
	move	#13,n0
	move	y0,x:(r0+n0)
L49
; **************************************************
;   EXP = WA1 == 1 ? EXP : (131072 - EXP) & 131071;
; **************************************************
	move	#11,n0
	move	#>1,y0
	move	x:(r0+n0),b
	cmp	y0,b
	jeq	L53
	move	#13,n0
	move	#>131071,x0
	move	x:(r0+n0),y0
	move	#>131072,b
	sub	y0,b
	and	x0,b	b1,x:(r0+n0)
	move	b1,b
	move	b1,x:(r0+n0)
L53
; **************************************************
;   MAG = (MAG + EXP) & 131071;
; **************************************************
; **************************************************
;   EXP = WA2 == 1 ? 0 :
;     ( (MAG >> 16) == 0)  ? MAG >> 7 : (MAG >> 7) + 64512;
; **************************************************
	move	#18,n0
	move	#>131071,x0
	move	x:(r0+n0),b
	move	#13,n0
	move	x:(r0+n0),y0
	add	y0,b	#18,n0
	and	x0,b	#>1,y0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#12,n0
	move	x:(r0+n0),b
	cmp	y0,b
	jne	L54
	move	#13,n0
	move	#0,y0
	move	y0,x:(r0+n0)
	jmp	L55
L54
	move	#18,n0
	move	x:(r0+n0),a
	move	a1,a
	rep	#16
	asr	a
	move	a1,a
	tst	a
	jne	L56
	move	x:(r0+n0),b
	move	b1,b
	rep	#7
	asr	b
	move	b1,b
	jmp	L114
L56
	move	#18,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#7
	asr	b
	move	#13,n0
	move	#>64512,y0
	move	b1,b
	add	y0,b	b1,x:(r0+n0)
L114
	move	#13,n0
	move	b1,x:(r0+n0)
L55
; **************************************************
;   MANT = ( (*A2)  >> 15) == 0 ? (65536 - ( (*A2)  >> 7)) & 65535 :
;     (65536 - (( (*A2)  >> 7) + 65024)) & 65535;
; **************************************************
	move	#65521,n0
	move	x:(r0+n0),r4
	move	#18,n0
	move	x:(r4),y0
	tfr	y0,b	y0,x:(r0+n0)
	move	b1,b
	rep	#15
	asr	b
	move	b1,b
	tst	b
	jne	L58
	move	x:(r0+n0),b
	move	b1,b
	rep	#7
	asr	b
	move	#19,n0
	move	b1,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	#>65536,b
	sub	y0,b
	jmp	L115
L58
	move	#65521,n0
	move	x:(r0+n0),r4
	move	x:(r4),y0
	tfr	y0,b
	move	b1,b
	rep	#7
	asr	b
	move	#19,n0
	move	b1,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	#14,n0
	move	#>512,b
	sub	y0,b
	move	b1,x:(r0+n0)
L115
; **************************************************
;   EXP = (EXP + MANT) & 65535;
; **************************************************
; **************************************************
;   EXP = ( (*A2)  + EXP) & 65535;
; **************************************************
; **************************************************
;   
;   /* LIMC */
;   AP = 32768 <= EXP && EXP <= 53248 ? 53248 :
;     12288 <= EXP && EXP <= 32767 ? 12288 : EXP;
; **************************************************
	move	#14,n0
	move	#>65535,x0
	and	x0,b	#>65535,x0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#13,n0
	move	x:(r0+n0),b
	move	#14,n0
	move	x:(r0+n0),y0
	add	y0,b	#65521,n0
	and	x0,b
	move	x:(r0+n0),r4
	move	#19,n0
	move	x:(r4),y0
	move	b1,b
	add	y0,b	y0,x:(r0+n0)
	and	x0,b	#13,n0
	move	b1,b
	move	#>32767,y0
	cmp	y0,b	b1,x:(r0+n0)
	jle	L60
	move	#>53248,x0
	cmp	x0,b
	jgt	L60
	move	x0,x1
	jmp	L61
L60
	move	#13,n0
	move	#>12287,y0
	move	x:(r0+n0),b
	cmp	y0,b
	jle	L62
	move	#>32767,y0
	cmp	y0,b
	jgt	L62
	move	#>12288,x1
	jmp	L61
L62
	move	#13,n0
	move	x:(r0+n0),x1
L61
; **************************************************
;   
;   /* TRIGB */
;   A2R = *T == 0 ? AP : 0;
; **************************************************
	move	#65532,n0
	move	x:(r0+n0),r4
	move	#19,n0
	move	x:(r4),y0
	tfr	y0,b	y0,x:(r0+n0)
	tst	b
	jne	L64
	move	#20,n0
	move	x1,x:(r0+n0)
	jmp	L65
L64
	move	#20,n0
	move	#0,y0
	move	y0,x:(r0+n0)
L65
; **************************************************
;   
;   /* UPA1 */
;   EXP = WA2 == 1 ? 0 : (WA1 == 0 ? 192 : 65344);
; **************************************************
	move	#12,n0
	move	#>1,y0
	move	x:(r0+n0),b
	cmp	y0,b
	jne	L66
	move	#0,y0
	jmp	L116
L66
	move	#11,n0
	move	x:(r0+n0),b
	tst	b
	jne	L68
	move	#>192,y0
	jmp	L116
L68
	move	#>65344,y0
L116
; **************************************************
;   MANT = A1S == 0 ? (65536 - ( (*A1)  >> 8)) & 65535 :
;     (65536 - (( (*A1)  >> 8) + 65280)) & 65535;
; **************************************************
	move	#13,n0
	move	y0,x:(r0+n0)
	move	#10,n0
	move	x:(r0+n0),b
	tst	b
	jne	L70
	move	x:(r1),y0
	tfr	y0,b
	move	b1,b
	rep	#8
	asr	b
	move	#19,n0
	move	b1,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	#>65536,b
	sub	y0,b
	jmp	L117
L70
	move	x:(r1),y0
	tfr	y0,b
	move	b1,b
	rep	#8
	asr	b
	move	#19,n0
	move	b1,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	#14,n0
	move	#>256,b
	sub	y0,b
	move	b1,x:(r0+n0)
L117
; **************************************************
;   EXP = (EXP + MANT) & 65535;
; **************************************************
; **************************************************
;   EXP = ( (*A1)  + EXP) & 65535;
; **************************************************
; **************************************************
;   
;   /* FMULT */
;   WA2 = f_mult(A2R,  (*SR2) ) ; 
; **************************************************
	move	#14,n0
	move	#>65535,x0
	and	x0,b	#>65535,x0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#13,n0
	move	x:(r0+n0),b
	move	#14,n0
	move	x:(r0+n0),y0
	add	y0,b	#13,n0
	and	x0,b	x:(r1),y0
	move	b1,b
	add	y0,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	#65523,n0
	move	x:(r0+n0),r4
	move	#19,n0
	move	x:(r4),y0
	move	y0,x:(r0+n0)
	move	#20,n0
	move	y0,x:(r6)+
	move	x:(r0+n0),y0
	move	y0,x:(r6)+
	jsr	Ff_mult
; **************************************************
;  
;   /* LIMD */
;   MAG = (15360 + 65536 - AP) & 65535;
; **************************************************
; **************************************************
;    (*A1)  = (AP + 65536 - 15360) & 65535;
; **************************************************
; **************************************************
;   AP = 32768 <= EXP && EXP <=  (*A1)  ?  (*A1)  :
;     MAG <= EXP && EXP <= 32767 ? MAG : EXP;
; **************************************************
	move	#12,n0
	move	#>50176,y0
	move	#>80896,b
	sub	x1,b	a1,x:(r0+n0)
	and	x0,b	#18,n0
	move	b1,b
	tfr	x1,b	b1,x:(r0+n0)
	add	y0,b	#17,n0
	and	x0,b	(r6)-
	move	b1,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	#13,n0
	move	y0,x:(r1)
	move	x:(r0+n0),b
	move	#>32767,y0
	cmp	y0,b	(r6)-
	jle	L72
	move	#17,n0
	move	x:(r0+n0),y0
	cmp	y0,b
	jgt	L72
	move	x:(r0+n0),x1
	jmp	L73
L72
	move	#18,n0
	move	x:(r0+n0),b
	move	#13,n0
	move	x:(r0+n0),y0
	cmp	y0,b
	jgt	L74
	move	#>32767,y0
	move	x:(r0+n0),b
	cmp	y0,b
	jgt	L74
	move	#18,n0
	move	x:(r0+n0),x1
	jmp	L73
L74
	move	#13,n0
	move	x:(r0+n0),x1
L73
; **************************************************
;   
;   /* TRIGB */
;    (*A1)  = *T == 0 ? AP : 0;
; **************************************************
	move	#65532,n0
	move	x:(r0+n0),r4
	move	#19,n0
	move	x:(r4),y0
	tfr	y0,b	y0,x:(r0+n0)
	tst	b
	jne	L76
	move	x1,x0
	jmp	L77
L76
	move	#0,x0
L77
; **************************************************
;   
;   /* FMULT */
;   WA1 = f_mult( (*A1) , SR1);
; **************************************************
	move	x0,x:(r1)
	move	y1,x:(r6)+
	move	x0,x:(r6)+
	jsr	Ff_mult
; **************************************************
;   
;   /* FLOATA */
;   MAG = *DQ & 16383;
; **************************************************
; **************************************************
;   for (EXP = 0; (MAG >> EXP) != 0; EXP++)
; **************************************************
	move	#11,n0
	move	#>16383,x0
	move	a1,x:(r0+n0)
	move	#19,n0
	move	x:(r2),y0
	tfr	y0,b	y0,x:(r0+n0)
	and	x0,b	#18,n0
	move	b1,b
	tst	b	#0,y0
	move	b1,x:(r0+n0)
	move	#13,n0
	move	(r6)-
	move	(r6)-
	move	y0,x:(r0+n0)
	jeq	L109
	move	#>1,x0
L80
	move	#13,n0
	move	x:(r0+n0),b
	add	x0,b
	tfr	b,a	b1,x:(r0+n0)
	tst	a	#18,n0
	move	x:(r0+n0),b
	jeq	L123
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L123
	tst	b
	jne	L80
L109
; **************************************************
; #ifdef __LOOPCOUNT__ 
;     Loop3++;
; #else
;     ;
; #endif
;   MANT = MAG == 0 ? 1 << 5 : (MAG << 6) >> EXP;
; **************************************************
	move	#18,n0
	move	x:(r0+n0),b
	tst	b
	jne	L82
	move	#14,n0
	move	#>32,y0
	move	y0,x:(r0+n0)
	jmp	L83
L82
	move	#18,n0
	move	x:(r0+n0),b
	rep	#6
	asl	b
	move	#14,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#13,n0
	move	x:(r0+n0),a
	tst	a
	jeq	L124
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L124
	move	#14,n0
	move	b1,x:(r0+n0)
L83
; **************************************************
;   DQSEZ = (DQS << 10) + (EXP << 6) + MANT;
; **************************************************
	move	#6,n0
	move	x:(r0+n0),b
	rep	#10
	asl	b
	move	#19,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#13,n0
	move	x:(r0+n0),b
	rep	#6
	asl	b
; **************************************************
;   
;   /* XOR */
;   BP[0] = DQS ^ ( (*DQ2)  >> 10);
; **************************************************
	move	#18,n0
	move	b1,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	#19,n0
	move	x:(r0+n0),b
	add	y0,b	#14,n0
	move	x:(r0+n0),y0
	add	y0,b	#8,n0
	move	b1,x:(r0+n0)
	move	#65531,n0
	move	x:(r0+n0),r4
	move	x:(r4),y0
	tfr	y0,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[1] = DQS ^ ( (*DQ3)  >> 10);
; **************************************************
	move	#6,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#65530,n0
	move	b1,b
	move	b1,y0
	move	y0,x:(r0)
	move	x:(r0+n0),r4
	move	x:(r4),y0
	tfr	y0,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[2] = DQS ^ ( (*DQ4)  >> 10);
; **************************************************
	move	#6,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	(r0)+
	move	b1,b
	move	b1,y0
	move	y0,x:(r0)-
	move	x:(r7),y0
	tfr	y0,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[3] = DQS ^ ( (*DQ5)  >> 10);
; **************************************************
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#2,n0
	move	b1,b
	move	b1,y0
	move	y0,x:(r0+n0)
	move	#65528,n0
	move	x:(r0+n0),r4
	move	x:(r4),y0
	tfr	y0,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[4] = DQS ^ ( (*DQ6)  >> 10);
; **************************************************
	move	#6,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#3,n0
	move	b1,b
	move	b1,y0
	move	y0,x:(r0+n0)
	move	x:(r5),y0
	tfr	y0,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[5] = DQS ^ ( (*DQ7)  >> 10);
; **************************************************
	move	#6,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#4,n0
	move	b1,b
	move	b1,y0
	move	y0,x:(r0+n0)
	move	#65526,n0
	move	x:(r0+n0),r4
	move	x:(r4),y0
	tfr	y0,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   
;   /* UPB */
;   MANT = *DQ & 16383;
; **************************************************
; **************************************************
;   for (EXP = 0; EXP < 6; EXP++)
; **************************************************
	move	#6,n0
	move	#>1,x1
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#5,n0
	move	b1,b
	move	b1,y0
	move	#>16383,x0
	move	y0,x:(r0+n0)
	move	#19,n0
	move	x:(r2),y0
	tfr	y0,b	r0,r2
	and	x0,b	#>65535,x0
	move	b1,b
	move	y0,x:(r0+n0)
	move	#14,n0
	move	#0,y0
	move	b1,x:(r0+n0)
	move	#13,n0
	move	y0,x:(r0+n0)
	do	#6,L108
L93
; **************************************************
;     {
; #ifdef __LOOPCOUNT__ 
;       Loop4++;
; #endif
;       DQS = MANT == 0 ? 0 : (BP[EXP] == 0 ? 128 : 65408);
; **************************************************
	move	#14,n0
	move	x:(r0+n0),b
	tst	b
	jne	L87
	move	#0,y0
	jmp	L118
L87
	move	x:(r2),a
	tst	a
	jne	L89
	move	#>128,y0
	jmp	L119
L89
	move	#>65408,y0
L119
	move	#19,n0
	move	y0,x:(r0+n0)
	move	x:(r0+n0),y0
L118
; **************************************************
;       MAG = B[EXP] >> 15;
; **************************************************
	move	#19,n0
	move	r3,b
	move	y0,x:(r0+n0)
	move	x:(r0+n0),y0
	move	#6,n0
	move	y0,x:(r0+n0)
	move	#13,n0
	move	x:(r0+n0),y0
	add	y0,b	#19,n0
	move	b1,r1
	move	x:(r1),y0
	tfr	y0,b	y0,x:(r0+n0)
	move	b1,b
	rep	#15
	asr	b
; **************************************************
;       DQS += MAG == 0 ? (65536 - (B[EXP] >> 8)) & 65535 :
; 	(65536 - ((B[EXP] >> 8) + 65280)) & 65535;
; **************************************************
	move	#18,n0
	move	b1,b
	tst	b	b1,x:(r0+n0)
	jne	L91
	move	#19,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#8
	asr	b
	move	b1,b
	move	b1,y0
	move	#>65536,b
	jmp	L120
L91
	move	#13,n0
	move	r3,b
	move	x:(r0+n0),y0
	add	y0,b
	move	b1,r1
	move	x:(r1),y0
	tfr	y0,b
	move	b1,b
	rep	#8
	asr	b
	move	b1,b
	move	b1,y0
	move	#>256,b
L120
; **************************************************
;       DQS &= 65535;
; **************************************************
; **************************************************
;       BP[EXP] = (B[EXP] + DQS) & 65535;
; **************************************************
; **************************************************
; **************************************************
	sub	y0,b	#19,n0
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	#6,n0
	move	x:(r0+n0),b
	move	#19,n0
	move	x:(r0+n0),y0
	add	y0,b	#6,n0
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	#13,n0
	move	r3,b
	move	x:(r0+n0),y0
	add	y0,b	#6,n0
	move	b1,r1
	move	x:(r0+n0),b
	move	#19,n0
	move	x:(r1),y0
	add	y0,b
	and	x0,b
	move	b1,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	#13,n0
	move	r2,b
	add	x1,b	y0,x:(r2)
	move	b1,r2
	move	x:(r0+n0),b
	add	x1,b
	move	b1,x:(r0+n0)
L108
; **************************************************
;     }
;   
;   /* TRIGB */
;   if (*T != 0)
; **************************************************
	move	#65532,n0
	move	x:(r0+n0),r4
	move	#19,n0
	move	x:(r4),y0
	tfr	y0,b	y0,x:(r0+n0)
	tst	b
	jeq	L94
; **************************************************
;     for (EXP = 0; EXP < 6; EXP++)
; **************************************************
	move	r0,r1
	move	#0,x0
	do	#6,L106
L98
; **************************************************
; #ifdef __LOOPCOUNT__ 
;     {
;       Loop5++;
;       BP[EXP] = 0 ;
;     }
; #else
;       BP[EXP] = 0 ;
; **************************************************
; **************************************************
; **************************************************
	move	x0,x:(r1)+
L106
	nop
L94
; **************************************************
; #endif
;   
;   /* FMULT */
;   DQI  = f_mult(BP[0], DQSEZ);
; **************************************************
	move	#8,n0
	move	#Ff_mult,r1
	move	x:(r0+n0),y0
	move	y0,x:(r6)+
	move	x:(r0),y0
	move	y0,x:(r6)+
	jsr	(r1)
; **************************************************
;   MAG  = f_mult(BP[1], (*DQ2));
; **************************************************
	move	#7,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	x:(r0+n0),b
	move	#65531,n0
	move	(r6)-
	move	x:(r0+n0),r4
	move	(r0)+
	move	x:(r4),y0
	move	y0,x:(r6)+
	move	x:(r0)-,y0
	move	y0,x:(r6)+
	jsr	(r1)
; **************************************************
;   MANT = f_mult(BP[2], (*DQ3));
; **************************************************
	move	#18,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	#65530,n0
	move	(r6)-
	move	x:(r0+n0),r4
	move	#2,n0
	move	x:(r4),y0
	move	y0,x:(r6)+
	move	x:(r0+n0),y0
	move	y0,x:(r6)+
	jsr	(r1)
; **************************************************
;   A1S  = f_mult(BP[3], (*DQ4));
; **************************************************
	move	#14,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	#3,n0
	move	x:(r7),y0
	move	(r6)-
	move	y0,x:(r6)+
	move	x:(r0+n0),y0
	move	y0,x:(r6)+
	jsr	(r1)
; **************************************************
;   AP   = f_mult(BP[4], (*DQ5));
; **************************************************
	move	#10,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	#65528,n0
	move	(r6)-
	move	x:(r0+n0),r4
	move	#4,n0
	move	x:(r4),y0
	move	y0,x:(r6)+
	move	x:(r0+n0),y0
	move	y0,x:(r6)+
	jsr	(r1)
; **************************************************
;   EXP  = f_mult(BP[5], (*DQ6));
; **************************************************
	move	#5,n0
	move	x:(r5),y0
	move	a1,x1
	move	(r6)-
	move	(r6)-
	move	y0,x:(r6)+
	move	x:(r0+n0),y0
	move	y0,x:(r6)+
	jsr	(r1)
; **************************************************
;   
;   /* ACCUM */
;   DQS = (DQI + MAG + MANT + A1S + AP + EXP) & 65535;
; **************************************************
; **************************************************
;   (*SEZ)  = DQS >> 1;
; **************************************************
; **************************************************
;   S_E =  (*SE)  = ((DQS + WA2 + WA1) & 65535 ) >> 1;
; **************************************************
; **************************************************
;   
;   /* STATE update */
;   (*PK2) = (*PK1);
; **************************************************
	move	#13,n0
	move	#>65535,x0
	move	a1,x:(r0+n0)
	move	#18,n0
	move	(r6)-
	move	x:(r0+n0),y0
	add	y0,b	#14,n0
	move	(r6)-
	move	x:(r0+n0),y0
	add	y0,b	#10,n0
	move	x:(r0+n0),y0
	add	y0,b	#13,n0
	add	x1,b
	move	x:(r0+n0),y0
	add	y0,b	#6,n0
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	#65518,n0
	move	b1,b
	asr	b	x:(r0+n0),r4
	move	#6,n0
	move	b1,b
	move	b1,y0
	move	y0,x:(r4)
	move	x:(r0+n0),b
	move	#12,n0
	move	x:(r0+n0),y0
	add	y0,b	#11,n0
	move	x:(r0+n0),y0
	add	y0,b	#65519,n0
	and	x0,b
	move	x:(r0+n0),r4
	move	#65525,n0
	move	b1,b
	move	b1,b
	asr	b
	move	b1,b
	move	b1,y0
	move	y0,x:(r4)
	move	y0,x:FS_E
	move	x:(r0+n0),r4
	move	x:(r4),y0
; **************************************************
;   (*PK1) = PK0;
; **************************************************
; **************************************************
;   (*SR2) = SR1;
; **************************************************
; **************************************************
;   A_2 = (*A2) = A2R;
; **************************************************
; **************************************************
; 
;   for (EXP = 0; EXP < 6; EXP++)
; **************************************************
	move	#19,n0
	move	r0,r1
	move	r3,r2
	move	y0,x:(r0+n0)
	move	#65524,n0
	move	x:(r0+n0),r4
	move	#65525,n0
	move	y0,x:(r4)
	move	x:(r0+n0),r4
	move	#9,n0
	move	x:(r0+n0),y0
	move	#65523,n0
	move	y0,x:(r4)
	move	x:(r0+n0),r4
	move	#65521,n0
	move	y1,x:(r4)
	move	x:(r0+n0),r4
	move	#20,n0
	move	x:(r0+n0),y0
	move	y0,x:(r4)
	move	y0,x:FA_2
	do	#6,L104
L102
; **************************************************
; #ifdef __LOOPCOUNT__
;   {
;     Loop6++;
;     B[EXP] = BP[EXP];
;   }
; #else
;     B[EXP] = BP[EXP];
; **************************************************
; **************************************************
; **************************************************
	move	x:(r1)+,a
	move	a1,x:(r2)+
L104
; **************************************************
; #endif
; 
;   (*DQ7) = (*DQ6);
; **************************************************
; **************************************************
;   (*DQ6) = (*DQ5);
; **************************************************
; **************************************************
;   (*DQ5) = (*DQ4);
; **************************************************
; **************************************************
;   (*DQ4) = (*DQ3);
; **************************************************
; **************************************************
;   (*DQ3) = (*DQ2);
; **************************************************
; **************************************************
;   (*DQ2) = DQSEZ;
; **************************************************
; **************************************************
; }
; **************************************************
	move	#65526,n0
	move	x:(r5),y0
	move	x:(r0+n0),r4
	move	#65528,n0
	move	y0,x:(r4)
	move	x:(r0+n0),r4
	move	#65530,n0
	move	x:(r4),y0
	move	(r6)-
	move	y0,x:(r5)
	move	x:(r7),y0
	move	y0,x:(r4)
	move	x:(r0+n0),r4
	move	#65531,n0
	move	x:(r4),y0
	move	y0,x:(r7)
	move	x:(r0+n0),r4
	move	#19,n0
	move	x:(r4),y0
	move	y0,x:(r0+n0)
	move	#65530,n0
	move	x:(r0+n0),r4
	move	#65531,n0
	move	y0,x:(r4)
	move	x:(r0+n0),r4
	move	#8,n0
	move	x:(r0+n0),y0
	move	y0,x:(r4)
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
	move	x:(r0)-,ssh
	move	r0,r6
	move	x:(r0),r0
	rts

	global	Fcoding_adj
Fcoding_adj
; **************************************************
; **************************************************
; **************************************************
; 
; void coding_adj(void)
; {
;   /*
;    * coding_adjustment
;    *
;    * Input signals:  I, SP, S_E, Y
;    * Output signals: SD
;    */
; 
;   /* Just passing a SP (16TC) through */
;   /* rest of the signals is ignored */
;   SD = SP;
; **************************************************
; **************************************************
; }
; **************************************************
	move	x:FSP,a
	move	a1,x:FSD
	rts

	org	x:
F___qtab0
	dc 2048
	dc 4
	dc 135
	dc 213
	dc 273
	dc 323
	dc 373
	dc 425
	dc 425
	dc 373
	dc 323
	dc 273
	dc 213
	dc 135
	dc 4
	dc 2048
	org	p:
	global	Fiadpt_quant
Fiadpt_quant
; **************************************************
; **************************************************
; **************************************************
; 
; void iadpt_quant(int* DQ, int* Y)
; {
;   /*
;    * inverse adaptive quantizer
;    * 
;    * Input signals:  I, Y
;    * Output signals: DQ
;    */
;  
;   static int qtab[] =
;   {
;     2048, 4, 135, 213, 273, 323, 373, 425,
;     425, 373, 323, 273, 213, 135, 4, 2048,
;   } ;
;   register int DQL;
; 
;   /* RECONST and ADDA */
;   DQL = (qtab[I] + (*Y >> 2)) & 4095;
; **************************************************
; **************************************************
; 
;   /* ANTILOG */
;   *DQ = ((I & (1 << 3)) * (1 << 11)) +
;         (DQL & (1 << 11) ? 0 :
;         (((1 << 7) + (DQL & 127)) << 7) >>
;         (14 - ((DQL >> 7) & 15)));
; **************************************************
	move	#65532,n0
	move	#3,n6
	move	r0,x:(r6)+
	move	(r6)+
	move	r6,r0
	move	(r6)+n6
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	x1,x:(r6)+
	move	r1,x:(r6)+
	move	#F___qtab0,r1
	move	r2,x:(r6)+
	move	x:(r0+n0),r2
	move	r1,b
	move	x:(r2),x1
	move	x:FI,a
	add	a,b
	tfr	x1,b	b1,r1
	move	b1,b
	asr	b	x:(r1),x0
	asr	b
	move	b1,b
	add	x0,b	#>4095,x0
	and	x0,b	#>8,x0
	and	x0,a	b1,b
	move	a1,a
	move	a1,x1
	move	#>2048,x0
	mpy	+x0,x1,b	b1,x:(r0)
	asr	b	(r0)+
	move	b0,b
	move	b1,x:(r0)-
	move	x:(r0),a
	and	x0,a
	move	a1,a
	tst	a
	jne	L127
	move	#>128,x1
	move	x:(r0),a
	move	#>127,x0
	and	x0,a
	move	a1,a
	add	x1,a
	rep	#7
	asl	a
	move	x:(r0),b
	move	b1,b
	move	a1,a
	rep	#7
	asr	b
	move	#>15,x0
	move	b1,b
	and	x0,b
	move	b1,b
	move	b1,x1
	move	#>14,b
	sub	x1,b
	tst	b
	jeq	L128
	move	a1,a
	rep	b
	asr	a
	move	a1,a
L128
	move	(r0)+
	move	x:(r0)-,b
	add	a,b	(r0)+
	move	b1,x:(r0)-
L127
; **************************************************
; }
; **************************************************
	move	#65533,n0
	move	(r6)-
	move	x:(r0+n0),r2
	move	(r0)+
	move	x:(r0)-,x1
	move	x1,x:(r2)
	move	x:(r6)-,r2
	move	x:(r6)-,r1
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	(r0)-
	move	(r0)-
	move	r0,r6
	move	x:(r0),r0
	rts

	global	Fadpt_quant
Fadpt_quant
; **************************************************
; **************************************************
; **************************************************
; 
; void adpt_quant(int* Y)
; {
;   /*
;    * adaptive quantizer
;    *
;    * Input signals:  D, Y
;    * Output signals: I
;    */
; 
;   register int DS, DQM, DL, DLN, EXP;
; 
;   /* LOG */
;   DS = D & (1 << 15);
; **************************************************
; **************************************************
;   DQM = DS ? (65536 - D) & 32767 : D;
; **************************************************
	move	#2,n0
	move	#3,n6
	move	r0,x:(r6)+
	move	(r6)+
	move	r6,r0
	move	(r6)+n6
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	#>32768,x0
	move	x1,x:(r6)+
	move	y0,x:(r6)+
	move	r1,x:(r6)+
	move	x:FD,y0
	tfr	y0,b	y0,x:(r0+n0)
	and	x0,b
	move	b1,b
	tst	b	b1,x:(r0)
	jeq	L130
	move	#>32767,x0
	move	#>65536,a
	sub	y0,a	(r0)+
	tfr	a,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0)-
	jmp	L131
L130
	move	x:FD,y0
	move	(r0)+
	move	y0,x:(r0)-
L131
; **************************************************
;   for (EXP = 1; DQM >> EXP; EXP++)
; **************************************************
	move	#2,n0
	move	#>1,y0
	tfr	y0,b	y0,x:(r0+n0)
	tst	b	(r0)+
	move	x:(r0)-,a
	jeq	L170
	move	a1,a
	rep	b
	asr	a
	move	a1,a
L170
	tst	a
	jeq	L168
	move	x:(r0+n0),x0
L134
	move	#2,n0
	move	x:(r0+n0),b
	add	x0,b
	tst	b	b1,x:(r0+n0)
	move	(r0)+
	move	x:(r0)-,a
	jeq	L171
	move	a1,a
	rep	b
	asr	a
	move	a1,a
L171
	tst	a
	jne	L134
L168
; **************************************************
; #ifdef __LOOPCOUNT__
;     Loop7++;
; #else
;     ;
; #endif
;   EXP--;
; **************************************************
; **************************************************
;   DL = (EXP * (1 << 7)) + (((DQM * (1 << 7)) >> EXP) & 127);
; **************************************************
	move	#2,n0
	move	#>128,x0
	move	x:(r0+n0),b
	move	#>16777215,a
	add	a,b	(r0)+
	move	x:(r0)-,y0
	mpy	+x0,y0,a
	asr	a
	tst	b	a0,a
	jeq	L172
	move	a1,a
	rep	b
	asr	a
	move	a1,a
L172
; **************************************************
; 
;   /* SUBTB */
;   DLN = (DL + 4096 - (*Y >> 2)) & 4095;
; **************************************************
; **************************************************
; 
;   /* QUAN */
;   if (DLN > 3971)
; **************************************************
	move	#65533,n0
	move	b1,y0
	move	x:(r0+n0),r1
	move	#2,n0
	move	#>127,x1
	and	x1,a
	move	a1,a
	move	a,a0
	asl	a
	mac	+y0,x0,a	x:(r1),y0
	asr	a	#>16773120,x0
	tfr	y0,b	a0,a
	move	b1,b
	asr	b
	asr	b
	move	b1,b
	add	x0,b	#>4095,x0
	move	b1,y0
	sub	y0,a
	tfr	a,b	#>3971,a
	and	x0,b
	move	b1,b
	cmp	a,b	b1,x:(r0+n0)
	jle	L136
; **************************************************
;     I = DS ? 0xE : 0x1;
; **************************************************
	move	x:(r0),b
	tst	b
	jne	L169
	jmp	L166
L136
; **************************************************
;   else if (DLN > 2047)
; **************************************************
	move	#2,n0
	move	#>2047,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L140
; **************************************************
;     I = 0xF;
; **************************************************
	move	#>15,a
	jmp	L167
L140
; **************************************************
;   else if (DLN > 399)
; **************************************************
	move	#2,n0
	move	#>399,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L142
; **************************************************
;     I = DS ? 0x8 : 0x7;
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L143
	move	#>8,a
	jmp	L167
L143
	move	#>7,a
	jmp	L167
L142
; **************************************************
;   else if (DLN > 348)
; **************************************************
	move	#2,n0
	move	#>348,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L146
; **************************************************
;     I = DS ? 0x9 : 0x6;
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L147
	move	#>9,a
	jmp	L167
L147
	move	#>6,a
	jmp	L167
L146
; **************************************************
;   else if (DLN > 299)
; **************************************************
	move	#2,n0
	move	#>299,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L150
; **************************************************
;     I = DS ? 0xA : 0x5;	
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L151
	move	#>10,a
	jmp	L167
L151
	move	#>5,a
	jmp	L167
L150
; **************************************************
;   else if (DLN > 245)
; **************************************************
	move	#2,n0
	move	#>245,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L154
; **************************************************
;     I = DS ? 0xB : 0x4;
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L155
	move	#>11,a
	jmp	L167
L155
	move	#>4,a
	jmp	L167
L154
; **************************************************
;   else if (DLN > 177)
; **************************************************
	move	#2,n0
	move	#>177,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L158
; **************************************************
;     I = DS ? 0xC : 0x3;
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L159
	move	#>12,a
	jmp	L167
L159
	move	#>3,a
	jmp	L167
L158
; **************************************************
;   else if (DLN > 79)
; **************************************************
	move	#2,n0
	move	#>79,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L162
; **************************************************
;     I = DS ? 0xD : 0x2;
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L163
	move	#>13,a
	jmp	L167
L163
	move	#>2,a
	jmp	L167
L162
; **************************************************
;   else
;     I = DS ? 0xE : 0x1;
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L166
L169
	move	#>14,a
	jmp	L167
L166
	move	#>1,a
L167
; **************************************************
; }
; **************************************************
	move	(r0)-
	move	(r0)-
	move	(r6)-
	move	a1,x:FI
	move	x:(r6)-,r1
	move	x:(r6)-,y0
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	r0,r6
	move	x:(r0),r0
	rts

	global	Ftone_detector
Ftone_detector
; **************************************************
; **************************************************
; **************************************************
; 
; void tone_detector(int* DQ, int* T, int* YL, int* t_dr, int* LAST_TR)
; {
;   /*
;    * tone detector
;    *
;    * Input signals:  DQ, A_2, YL
;    * Output signals: T, TDP;
;    */
; 
;   register int tmp; 
;   register int t_d; 
; 
;   /* partial band signal detection */
;   TDP = ((A_2 > 32769) && (A_2 < 53761)) ? 1 : 0 ; 
; **************************************************
	move	#65530,n0
	move	#3,n6
	move	#>32769,a
	move	r0,x:(r6)+
	move	(r6)+
	move	r6,r0
	move	(r6)+n6
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	x1,x:(r6)+
	move	r1,x:(r6)+
	move	r2,x:(r6)+
	move	r3,x:(r6)+
	move	x:(r0+n0),r2
	move	#65529,n0
	move	x:FA_2,x1
	tfr	x1,b	x:(r0+n0),r1
	cmp	a,b
	jle	L174
	move	#>53760,a
	cmp	a,b
	jgt	L174
	move	#>1,x0
	jmp	L175
L174
	move	#0,x0
L175
; **************************************************
;   
;   /* take last (t_dr) */
;   t_d = (*t_dr);
; **************************************************
; **************************************************
;   
;   /* calcutale new  (*t_dr)  */
;   /* TRIGB Function */
;   *t_dr = (*LAST_TR == 0) ? TDP : 0; 
; **************************************************
	move	(r0)+
	move	x0,x:FTDP
	move	x:(r2),x1
	move	x1,x:(r0)-
	move	x:(r1),a
	tst	a
	jne	L176
	tfr	x0,a
	jmp	L177
L176
	clr	a
L177
; **************************************************
;   
;   *LAST_TR = *YL >> 15; /* (*LAST_TR)  is used here as a temporary variable */
; **************************************************
	move	#65531,n0
	move	a1,x:(r2)
	move	x:(r0+n0),r3
	move	x:(r3),a
	move	a1,a
	rep	#15
	asr	a
; **************************************************
;   
;   tmp = ((*LAST_TR) > 8) ? 31 << 9 :
;         (32 + ((*LAST_TR << 5) & 31)) << *LAST_TR; 
; **************************************************
	move	#>8,x1
	move	a1,a
	cmp	x1,a	a1,x:(r1)
	jle	L178
	move	#>15872,x1
	move	x1,x:(r0)
	jmp	L179
L178
	move	x:(r1),a
	rep	#5
	asl	a
	move	x:(r1),b
	move	#>32,x1
	move	#>31,x0
	move	a1,a
	and	x0,a
	move	a1,a
	add	x1,a
	tst	b
	jeq	L182
	rep	b
	asl	a
	move	a1,a
L182
	move	a1,x:(r0)
L179
; **************************************************
;   
;   *LAST_TR = *T = (((*DQ & 16383) > (tmp + (tmp >> 1)) >> 1) 
; 		 && (t_d == 1)) ? 1 : 0;
; **************************************************
	move	#65533,n0
	move	x:(r0),b
	move	x:(r0+n0),r3
	move	b1,b
	asr	b	x:(r3),a
	move	b1,b
	move	b1,x1
	move	x:(r0),b
	add	x1,b	#>16383,x0
	and	x0,a	b1,b
	asr	b	a1,a
	move	b1,b
	move	b1,x1
	cmp	x1,a
	jle	L180
	move	#>1,a
	move	(r0)+
	move	x:(r0)-,b
	cmp	a,b
	jeq	L181
L180
	clr	a
L181
; **************************************************
; }
; **************************************************
	move	#65532,n0
	move	(r6)-
	move	x:(r0+n0),r3
	move	(r0)-
	move	a1,x:(r3)
	move	a1,x:(r1)
	move	x:(r6)-,r3
	move	x:(r6)-,r2
	move	x:(r6)-,r1
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	(r0)-
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
; void speed_control(int* T, int* Y, int* DMS, int* DML, int* AP)
; {
;   /*
;    * speed control
;    *
;    * Input signals:  T, TDP, I, Y
;    * Output signals: AL
;    */
; 
;   static int F[] = { 0, 0, 0, 1, 1, 1, 3, 7 };
;   register int FI, tmp ; 
; 
;   /* FUNTCF */
;   FI = F[(I >> 3 ? (15 - I) : I) & 7] ; 
; **************************************************
	move	#65531,n0
	move	#3,n6
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
	move	r4,x:(r6)+
	move	r5,x:(r6)+
	move	x:(r0+n0),r2
	move	#65530,n0
	move	x:FI,y0
	tfr	y0,a	x:(r0+n0),r3
	move	#65529,n0
	move	a1,a
	move	x:(r0+n0),r4
	move	#2,n0
	move	y0,x:(r0+n0)
	rep	#3
	asr	a
	move	a1,a
	tst	a
	jeq	L184
	move	#>15,a
	sub	y0,a
	jmp	L185
L184
	move	x:FI,a
L185
; **************************************************
; 
;   /* FILTA */
;   tmp = ((FI << 9) + 8192 - *DMS) & 8191;              /* tmp => DIF */
; **************************************************
	move	#F___F1,r1
	move	#>7,x0
	and	x0,a	r1,b
	move	a1,a
	add	a,b
	move	b1,r1
	move	x:(r1),y0
	tfr	y0,b	y0,x:(r0)
	rep	#9
	asl	b
; **************************************************
;   tmp = (tmp >> 12) ? (tmp >> 5) + 3840 : (tmp >> 5);  /* tmp => DIFSX */
; **************************************************
	move	x:(r2),a
	move	#>16769024,x0
	add	x0,a	b1,b
	sub	a,b	#>8191,x0
	and	x0,b	(r0)+
	move	b1,b
	tfr	b,a	b1,x:(r0)-
	move	a1,a
	rep	#12
	asr	a
	move	a1,a
	tst	a
	jeq	L186
	move	b1,b
	rep	#5
	asr	b
	move	#>3840,a
	move	b1,b
	add	a,b
	jmp	L199
L186
	move	(r0)+
	move	x:(r0)-,b
	move	b1,b
	rep	#5
	asr	b
	move	b1,b
L199
; **************************************************
;   *DMS = (tmp + *DMS) & 4095;
; **************************************************
; **************************************************
; 
;   /* FILTB */
;   tmp = ((FI << 11) + 32768 - *DML) & 32767;           /* tmp => DIF */
; **************************************************
	move	#>4095,x0
	move	(r0)+
	move	b1,x:(r0)-
	move	x:(r2),a
	move	(r0)+
	move	x:(r0)-,b
	add	a,b
	tfr	b,a
	and	x0,a
	move	a1,a
	move	a1,x:(r2)
	move	x:(r0),b
	rep	#11
	asl	b
; **************************************************
;   tmp = (tmp >> 14) ? (tmp >> 7) + 16128 : (tmp >> 7); /* tmp => DIFSX */
; **************************************************
	move	x:(r3),a
	move	#>16744448,x0
	add	x0,a	b1,b
	sub	a,b	#>32767,x0
	and	x0,b	(r0)+
	move	b1,b
	tfr	b,a	b1,x:(r0)-
	move	a1,a
	rep	#14
	asr	a
	move	a1,a
	tst	a
	jeq	L188
	move	b1,b
	rep	#7
	asr	b
	move	#>16128,a
	move	b1,b
	add	a,b
	jmp	L200
L188
	move	(r0)+
	move	x:(r0)-,b
	move	b1,b
	rep	#7
	asr	b
	move	b1,b
L200
; **************************************************
;   *DML = (tmp + *DML) & 16383;
; **************************************************
; **************************************************
; 
;   /* SUBTC */
;   tmp = ((*DMS << 2) + 32768 - *DML) & 32767;          /* tmp => DIF */
; **************************************************
; **************************************************
;   tmp = (tmp >> 14) ? (32768 - tmp) & 16383 : tmp;     /* tmp => DIFM */
; **************************************************
	move	#2,n0
	move	#>16744448,x0
	move	#>16383,x1
	move	(r0)+
	move	b1,x:(r0)-
	move	x:(r3),a
	move	(r0)+
	move	x:(r0)-,b
	add	a,b
	tfr	b,a
	and	x1,a
	move	a1,a
	add	x0,a	a1,x:(r3)
	move	x:(r2),y0
	tfr	y0,b	#>32767,x0
	asl	b	y0,x:(r0+n0)
	asl	b	(r0)+
	move	b1,b
	sub	a,b
	and	x0,b
	move	b1,b
	tfr	b,a	b1,x:(r0)-
	move	a1,a
	rep	#14
	asr	a
	move	a1,a
	tst	a
	jeq	L191
	move	#>32768,a
	move	(r0)+
	move	x:(r0)-,y0
	sub	y0,a	(r0)+
	tfr	a,b	a1,x:(r0)-
	and	x1,b	(r0)+
	move	b1,b
	move	b1,x:(r0)-
L191
; **************************************************
;   FI = ((*Y > 1535) && (tmp < (*DML >> 3)) && (TDP == 0)) ? 0 : 1;
; **************************************************
	move	#65532,n0
	move	#>1535,y0
	move	x:(r0+n0),r5
	move	x:(r5),a
	cmp	y0,a
	jle	L192
	move	x:(r3),a
	move	a1,a
	rep	#3
	asr	a
	move	a1,a
	move	(r0)+
	move	x:(r0)-,b
	cmp	a,b
	jge	L192
	move	x:FTDP,a
	tst	a
	jne	L192
	move	#0,y0
	jmp	L201
L192
	move	#>1,y0
L201
; **************************************************
; 
;   /* FILTC */
;   tmp = ((FI << 9) + 2048 - *AP) & 2047;               /* tmp => DIF */ 
; **************************************************
	move	y0,x:(r0)
	move	x:(r0),b
	rep	#9
	asl	b
; **************************************************
;   tmp = (tmp >> 4) + (tmp >> 10 ? 896 : 0);            /* tmp => DIFSX */
; **************************************************
	move	x:(r4),a
	move	#>16775168,x0
	add	x0,a	b1,b
	sub	a,b	#>2047,x0
	and	x0,b	(r0)+
	move	b1,b
	tfr	b,a	b1,x:(r0)-
	move	b1,b
	rep	#4
	asr	b
	move	#2,n0
	move	a1,a
	move	b1,b
	move	b1,x:(r0+n0)
	rep	#10
	asr	a
	move	a1,a
	tst	a
	jeq	L194
	move	#>896,a
	add	a,b
	move	b1,x:(r0+n0)
L194
; **************************************************
;   tmp = (tmp + *AP) & 1023;                            /* tmp => APP */
; **************************************************
; **************************************************
; 
;   /* TRIGA */
;   *AP = *T ? 256 : tmp; 
; **************************************************
	move	#2,n0
	move	x:(r4),a
	move	x:(r0+n0),y0
	tfr	y0,b	#65533,n0
	add	a,b	#>1023,x0
	and	x0,b	(r0)+
	move	b1,b
	move	b1,x:(r0)-
	move	x:(r0+n0),r5
	move	x:(r5),a
	tst	a
	jeq	L195
	move	#>256,y0
	jmp	L202
L195
	move	(r0)+
	move	x:(r0)-,y0
L202
; **************************************************
; 
;   /* LIMA */
;   AL = (*AP > 255) ? 64 : *AP >> 2; 
; **************************************************
	move	#2,n0
	move	#>255,a
	move	y0,x:(r0+n0)
	move	x:(r0+n0),y0
	tfr	y0,b	y0,x:(r4)
	cmp	a,b
	jle	L197
	move	#>64,a
	jmp	L198
L197
	move	x:(r4),a
	move	a1,a
	asr	a
	asr	a
	move	a1,a
L198
; **************************************************
; }
; **************************************************
	move	(r0)-
	move	(r0)-
	move	(r6)-
	move	a1,x:FAL
	move	x:(r6)-,r5
	move	x:(r6)-,r4
	move	x:(r6)-,r3
	move	x:(r6)-,r2
	move	x:(r6)-,r1
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
	dc 4084
	dc 18
	dc 41
	dc 64
	dc 112
	dc 198
	dc 255
	dc 1122
	org	p:
	global	Fscale_factor
Fscale_factor
; **************************************************
; **************************************************
; **************************************************
; 
; void scale_factor(int* Y, int* YL, int* Y_L, int* LAST_Y)
; {
;   /*
;    * scale factor adaptation
;    *
;    * Input signals:  AL, I
;    * Output signals: YL, Y
;    */
; 
;   static int W[] = 
;   {
;     4084, 18, 41, 64, 112, 198, 255, 1122
;   } ; 
;   register int TMP, YUP, YLP;
;  
;   /* FUNCTW */
;   TMP = W[(I >> 3) ? (15 - I) & 7 : I & 7];
; **************************************************
	move	#65531,n0
	move	#3,n6
	move	r0,x:(r6)+
	move	(r6)+
	move	r6,r0
	move	(r6)+n6
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	x1,x:(r6)+
	move	r1,x:(r6)+
	move	r2,x:(r6)+
	move	r3,x:(r6)+
	move	r4,x:(r6)+
	move	x:(r0+n0),r3
	move	#65530,n0
	move	x:FI,x1
	tfr	x1,a	x:(r0+n0),r2
	move	#2,n0
	move	a1,a
	move	x1,x:(r0+n0)
	rep	#3
	asr	a
	move	a1,a
	tst	a
	jeq	L204
	move	#>15,a
	sub	x1,a
	jmp	L216
L204
	move	x:FI,a
L216
; **************************************************
;   
;   /* FILTD */
;   TMP = ((TMP << 5) + 131072 - *LAST_Y) & 131071;
; **************************************************
	move	#F___W2,r1
	move	#>7,x0
	and	x0,a	r1,b
	move	a1,a
	add	a,b
	move	b1,r1
	move	x:(r1),x1
	tfr	x1,b
	rep	#5
	asl	b
; **************************************************
;   YUP = (TMP >> 16) ? (TMP >> 5) + 4096 : (TMP >> 5);
; **************************************************
	move	#2,n0
	move	x:(r2),a
	move	#>16646144,x0
	add	x0,a	b1,b
	sub	a,b	#>131071,x0
	and	x0,b
	move	b1,b
	tfr	b,a	b1,x:(r0+n0)
	move	a1,a
	rep	#16
	asr	a
	move	a1,a
	tst	a
	jeq	L206
	move	b1,b
	rep	#5
	asr	b
	move	#>4096,a
	move	b1,b
	add	a,b
	jmp	L217
L206
	move	#2,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#5
	asr	b
	move	b1,b
L217
; **************************************************
;   YUP = (YUP + *LAST_Y) & 8191;
; **************************************************
; **************************************************
;   
;   /* LIMB */
;   if ((((YUP + 11264) & 16383) >> 13) == 0)
; **************************************************
	move	#>8191,x0
	move	b1,x:(r0)
	move	x:(r2),a
	move	x:(r0),b
	add	a,b	#>11264,a
	and	x0,b	#>16383,x0
	move	b1,b
	add	a,b	b1,x:(r0)
	tfr	b,a
	and	x0,a
	move	a1,a
	move	a1,a
	rep	#13
	asr	a
	move	a1,a
	tst	a
	jne	L208
; **************************************************
;     YUP = 5120;
; **************************************************
	move	#>5120,x1
	move	x1,x:(r0)
L208
; **************************************************
;   if ((((YUP + 15840) & 16383) >> 13))
; **************************************************
	move	#>16383,x0
	move	x:(r0),b
	move	#>15840,a
	add	a,b
	tfr	b,a
	and	x0,a
	move	a1,a
	move	a1,a
	rep	#13
	asr	a
	move	a1,a
	tst	a
	jeq	L209
; **************************************************
;     YUP = 544;
; **************************************************
	move	#>544,x1
	move	x1,x:(r0)
L209
; **************************************************
;   
;   /* FILTE */
;   TMP = (YUP + ((1048576 - *Y_L) >> 6)) & 16383;
; **************************************************
	move	x:(r3),x1
	move	#>1048576,a
	sub	x1,a
	move	a1,a
	rep	#6
	asr	a
; **************************************************
;   TMP = (TMP >> 13) ? TMP + 507904 : TMP;
; **************************************************
	move	#2,n0
	move	x:(r0),b
	move	#>16383,x0
	move	a1,a
	add	a,b
	and	x0,b
	move	b1,b
	tfr	b,a	b1,x:(r0+n0)
	move	a1,a
	rep	#13
	asr	a
	move	a1,a
	tst	a
	jeq	L211
	move	#>507904,a
	add	a,b
	move	b1,x:(r0+n0)
L211
; **************************************************
;   YLP = (TMP + *Y_L) & 524287;
; **************************************************
; **************************************************
;   
;   /* MIX */
;   TMP = (YUP + 16384 - (*Y_L >> 6)) & 16383;
; **************************************************
	move	#2,n0
	move	x:(r3),a
	move	x:(r0+n0),b
	add	a,b	#>524287,x0
	and	x0,b	a1,a
	move	b1,b
	move	(r0)+
	move	b1,x:(r0)-
	rep	#6
	asr	a
; **************************************************
;   *LAST_Y = TMP >> 13;
; **************************************************
	move	x:(r0),b
	move	#>16383,x0
	move	#>16760832,x1
	move	a1,a
	add	x1,a
	sub	a,b
	and	x0,b
	move	b1,b
	tfr	b,a	b1,x:(r0+n0)
	move	a1,a
	rep	#13
	asr	a
; **************************************************
;   TMP = *LAST_Y ? (16384 - TMP) & 8191 : TMP; 
; **************************************************
	move	a1,a
	tst	a	a1,x:(r2)
	jeq	L213
	move	x:(r0+n0),x1
	move	#>8191,x0
	move	#>16384,a
	sub	x1,a
	tfr	a,b	a1,x:(r0+n0)
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
L213
; **************************************************
;   TMP = (TMP * AL) >> 6;
; **************************************************
	move	#2,n0
	move	x:FAL,x0
	move	x:(r0+n0),x1
	mpy	+x0,x1,b
	asr	b
	move	b0,b
	move	b1,b
	rep	#6
	asr	b
; **************************************************
;   TMP = *LAST_Y ? (16384 - TMP) & 16383 : TMP;
; **************************************************
	move	b1,b
	move	b1,x:(r0+n0)
	move	x:(r2),a
	tst	a
	jeq	L215
	move	x:(r0+n0),x1
	move	#>16383,x0
	move	#>16384,a
	sub	x1,a
	tfr	a,b	a1,x:(r0+n0)
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
L215
; **************************************************
;   
;   *LAST_Y = *Y = ((*Y_L >> 6) + TMP) & 8191; 
; **************************************************
	move	x:(r3),a
	move	a1,a
	rep	#6
	asr	a
; **************************************************
;   *YL = *Y_L; 
; **************************************************
; **************************************************
;   *Y_L = YLP;
; **************************************************
; **************************************************
; }
; **************************************************
	move	#2,n0
	move	#>8191,x0
	move	x:(r0+n0),x1
	move	#65533,n0
	move	a1,a
	add	x1,a	x:(r0+n0),r4
	and	x0,a	#65532,n0
	move	a1,a
	move	a1,x:(r4)
	move	a1,x:(r2)
	move	x:(r0+n0),r4
	move	x:(r3),a
	move	a1,x:(r4)
	move	(r0)+
	move	x:(r0)-,x1
	move	x1,x:(r3)
	move	(r0)-
	move	(r0)-
	move	(r6)-
	move	x:(r6)-,r4
	move	x:(r6)-,r3
	move	x:(r6)-,r2
	move	x:(r6)-,r1
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	r0,r6
	move	x:(r0),r0
	rts

	global	Fadd
Fadd
; **************************************************
; **************************************************
; **************************************************
; 
; void add(int* DQ)
; {
;   /*
;    * add
;    *
;    * Input signals:  DQ, SE
;    * Output signals: SP
;    */
; 
;   int dq, se;
; 
;   /* 15 SM auf 16 TC */
;   dq = (*DQ & (1 << 14)) ? *DQ == (1 << 14) ? 0: 
;        (((*DQ ^ 0x3FFF) + 0x1) & 0x3FFF) + 0xC000 : *DQ ; 
; **************************************************
	move	#65533,n0
	move	r0,x:(r6)+
	move	(r6)+
	move	r6,r0
	move	(r6)+
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	#>16384,x0
	move	x1,x:(r6)+
	move	r1,x:(r6)+
	move	x:(r0+n0),r1
	move	x:(r1),b
	tfr	b,a
	and	x0,a
	move	a1,a
	tst	a
	jeq	L219
	cmp	x0,b
	jne	L221
	clr	b
	jmp	L220
L221
	move	#>16383,x0
	move	x:(r1),a
	tfr	a,b	#>1,a
	eor	x0,b
	move	b1,b
	add	a,b	#>49152,a
	and	x0,b
	move	b1,b
	add	a,b
	jmp	L220
L219
	move	x:(r1),b
L220
; **************************************************
; 
;   /* 15 TC auf 16 TC */
;   se = S_E & (1 << 14) ? 0x8000 | S_E : S_E ; 
; **************************************************
	move	x:FS_E,x1
	tfr	x1,a	#>16384,x0
	and	x0,a	x1,x:(r0)
	move	a1,a
	tst	a
	jeq	L223
	move	x:(r0),a
	move	#>32768,x0
	or	x0,a
	move	a1,a
	jmp	L224
L223
	move	x:FS_E,a
L224
; **************************************************
;   
;   /* perform add operation at 16 TC */
;   SP  = (dq + se) & 0xFFFF;
; **************************************************
; **************************************************
; }
; **************************************************
	add	a,b	#>65535,x0
	tfr	b,a	(r6)-
	and	x0,a
	move	a1,a
	move	a1,x:FSP
	move	x:(r6)-,r1
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	(r6)-
	move	(r6)-
	move	(r6)-
	move	x:(r6),r0
	rts

	global	Finit_disk_play
Finit_disk_play
; **************************************************
; **************************************************
; **************************************************
; 
; #ifdef __DSP56K__
; 
; void init_disk_play()
; {
; }
; **************************************************
	rts

	global	FData_in
	org	x:
FData_in
	dc 122
	dc 121
	dc 123
	dc 119
	dc 116
	dc 115
	dc 118
	dc 116
	dc 114
	dc 115
F___i3
	dc 0
	org	p:
	global	Fdisk_play
Fdisk_play
; **************************************************
; **************************************************
; **************************************************
; 
; #ifdef __SIMULATION__
; #include "data_in.c"
; #endif
; 
; void disk_play()
; {
; #ifdef __SIMULATION__
; 
;   static int i = 0;
; 
;   if (i >= sizeof(Data_in) / sizeof(int))
; **************************************************
	move	r0,x:(r6)+
	move	r6,r0
	move	(r6)+
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	#>9,b
	move	x0,x:(r6)+
	move	x1,x:(r6)+
	move	r1,x:(r6)+
	move	x:F___i3,a
	cmp	b,a
	jle	L227
; **************************************************
;     ExitFlag = TRUE;
; **************************************************
	move	#>1,a
	move	a1,x:FExitFlag
L227
; **************************************************
; 
;   SL = Data_in[i++] & 0x3FFF;
; **************************************************
; **************************************************
; 
; #else
; 
;   ExitFlag = TRUE;
;   SL = 0;
; 
; #endif
; }
; **************************************************
	move	#FData_in,r1
	move	x:F___i3,a
	tfr	a,b	#>16383,x0
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

	global	Fpost_disk_play
Fpost_disk_play
; **************************************************
; **************************************************
; **************************************************
; 
; void post_disk_play()
; {
; }
; **************************************************
	rts

	global	Finit_disk_record
Finit_disk_record
; **************************************************
; **************************************************
; **************************************************
; 
; void init_disk_record()
; {
; }
; **************************************************
	rts

	global	Fdisk_record
Fdisk_record
; **************************************************
; **************************************************
; **************************************************
; 
; void disk_record()
; {
; }
; **************************************************
	rts

	global	Fpost_disk_record
Fpost_disk_record
; **************************************************
; **************************************************
; **************************************************
; 
; void post_disk_record()
; {
; }
; **************************************************
	rts

	global	Fmain
Fmain
	move	ssh,x:(r6)+
; **************************************************
; **************************************************
; **************************************************
; 
; #else
; 
; void init_disk_play()
; {
;   istrm = fopen("data.in", "rb");
; }
; 
; void disk_play()
; {
;   int d;
; 
;   if (istrm != NULL && !feof(istrm))
;   {
;     fread((char*) &d, sizeof(int), 1, istrm);
;     SL = (d & 0x3FFF0000) >> 16;
;   }
;   if (feof(istrm))
;     ExitFlag = TRUE;
; }
; 
; void post_disk_play()
; {
;   if (istrm != NULL)
;     fclose(istrm);
; }
; 
; void init_disk_record()
; {
;   ostrm = fopen("data.out", "wb");
; }
; 
; void disk_record()
; {
;   int d;
; 
;   if (ostrm != NULL)
;   {
;     d = (SD & 0x0000FFFF) << 16;
;     fwrite((char*) &d, 4, 1, ostrm);
;   }
; }
; 
; void post_disk_record()
; {
;   if (ostrm != NULL)
;     fclose(ostrm);
; }
; 
; #endif
; 
; main()
; {
; #ifdef __DOWNLOAD__
;   run56k_start();
; #endif
; 
;   init_disk_play();
; **************************************************
	move	x0,x:(r6)+
	move	r1,x:(r6)+
	move	r2,x:(r6)+
	move	r3,x:(r6)+
	move	r4,x:(r6)+
	move	r5,x:(r6)+
	move	r7,x:(r6)+
	jsr	Finit_disk_play
; **************************************************
;   init_disk_record();
; **************************************************
	jsr	Finit_disk_record
; **************************************************
; 
;   ExitFlag = FALSE;
; **************************************************
; **************************************************
;   while (!ExitFlag)
; **************************************************
	clr	a	#FD_Y,r4
	move	#FD_DQ,r2
	move	#FD_T,r5
	move	#FE_Y,r3
	move	#FE_DQ,r7
	move	a1,x:FExitFlag
L236
; **************************************************
;   {
;     disk_play();
; **************************************************
	jsr	Fdisk_play
; **************************************************
;     if (ExitFlag)
; **************************************************
	move	x:FExitFlag,a
	tst	a
	jne	L234
; **************************************************
;       break;
; **************************************************
; **************************************************
; 
;     /*
;      * ENCODER
;      */
;     adpt_predict(&E_DQ, &E_T, &E_DQ2, &E_DQ3, &E_DQ4, &E_DQ5, &E_DQ6, &E_DQ7,
; 		 &E_PK1, &E_PK2, &E_SR2, &E_A1, &E_A2, E_B,
; 		 &E_SE, &E_SEZ);
; **************************************************
	move	#FE_SEZ,r1
	move	#>FE_T,x0
	move	r1,x:(r6)+
	move	#FE_SE,r1
	move	r1,x:(r6)+
	move	#FE_B,r1
	move	r1,x:(r6)+
	move	#FE_A2,r1
	move	r1,x:(r6)+
	move	#FE_A1,r1
	move	r1,x:(r6)+
	move	#FE_SR2,r1
	move	r1,x:(r6)+
	move	#FE_PK2,r1
	move	r1,x:(r6)+
	move	#FE_PK1,r1
	move	r1,x:(r6)+
	move	#FE_DQ7,r1
	move	r1,x:(r6)+
	move	#FE_DQ6,r1
	move	r1,x:(r6)+
	move	#FE_DQ5,r1
	move	r1,x:(r6)+
	move	#FE_DQ4,r1
	move	r1,x:(r6)+
	move	#FE_DQ3,r1
	move	r1,x:(r6)+
	move	#FE_DQ2,r1
	move	r1,x:(r6)+
	move	x0,x:(r6)+
	move	r7,x:(r6)+
	jsr	Fadpt_predict
; **************************************************
;     diff_signal();
; **************************************************
	move	#16,n6
	move	(r6)-n6
	jsr	Fdiff_signal
; **************************************************
;     adpt_quant(&E_Y);
; **************************************************
	move	r3,x:(r6)+
	jsr	Fadpt_quant
; **************************************************
;     iadpt_quant(&E_DQ, &E_Y);
; **************************************************
	move	(r6)-
	move	r3,x:(r6)+
	move	r7,x:(r6)+
	jsr	Fiadpt_quant
; **************************************************
;     tone_detector(&E_DQ, &E_T, &E_YL, &E_t_dr, &E_LAST_TR);
; **************************************************
	move	#FE_LAST_TR,r1
	move	#>FE_YL,x0
	move	(r6)-
	move	(r6)-
	move	r1,x:(r6)+
	move	#FE_t_dr,r1
	move	r1,x:(r6)+
	move	x0,x:(r6)+
	move	#>FE_T,x0
	move	x0,x:(r6)+
	move	r7,x:(r6)+
	jsr	Ftone_detector
; **************************************************
;     speed_control(&E_T, &E_Y, &E_DMS, &E_DML, &E_AP);
; **************************************************
	move	#5,n6
	move	#FE_AP,r1
	move	(r6)-n6
	move	r1,x:(r6)+
	move	#FE_DML,r1
	move	r1,x:(r6)+
	move	#FE_DMS,r1
	move	r1,x:(r6)+
	move	r3,x:(r6)+
	move	x0,x:(r6)+
	jsr	Fspeed_control
; **************************************************
;     scale_factor(&E_Y, &E_YL, &E_Y_L, &E_LAST_Y);
; **************************************************
	move	#5,n6
	move	#FE_LAST_Y,r1
	move	#>FE_YL,x0
	move	(r6)-n6
	move	r1,x:(r6)+
	move	#FE_Y_L,r1
	move	r1,x:(r6)+
	move	x0,x:(r6)+
	move	r3,x:(r6)+
	jsr	Fscale_factor
; **************************************************
; 
;     /*
;      * DECODER
;      */
;     adpt_predict(&D_DQ, &D_T, &D_DQ2, &D_DQ3, &D_DQ4, &D_DQ5, &D_DQ6, &D_DQ7,
; 		 &D_PK1, &D_PK2, &D_SR2, &D_A1, &D_A2, D_B,
; 		 &D_SE, &D_SEZ);
; **************************************************
	move	#4,n6
	move	#FD_SEZ,r1
	move	(r6)-n6
	move	r1,x:(r6)+
	move	#FD_SE,r1
	move	r1,x:(r6)+
	move	#FD_B,r1
	move	r1,x:(r6)+
	move	#FD_A2,r1
	move	r1,x:(r6)+
	move	#FD_A1,r1
	move	r1,x:(r6)+
	move	#FD_SR2,r1
	move	r1,x:(r6)+
	move	#FD_PK2,r1
	move	r1,x:(r6)+
	move	#FD_PK1,r1
	move	r1,x:(r6)+
	move	#FD_DQ7,r1
	move	r1,x:(r6)+
	move	#FD_DQ6,r1
	move	r1,x:(r6)+
	move	#FD_DQ5,r1
	move	r1,x:(r6)+
	move	#FD_DQ4,r1
	move	r1,x:(r6)+
	move	#FD_DQ3,r1
	move	r1,x:(r6)+
	move	#FD_DQ2,r1
	move	r1,x:(r6)+
	move	r5,x:(r6)+
	move	r2,x:(r6)+
	jsr	Fadpt_predict
; **************************************************
;     iadpt_quant(&D_DQ, &D_Y);
; **************************************************
	move	#16,n6
	move	(r6)-n6
	move	r4,x:(r6)+
	move	r2,x:(r6)+
	jsr	Fiadpt_quant
; **************************************************
;     add(&D_DQ);
; **************************************************
	move	(r6)-
	move	(r6)-
	move	r2,x:(r6)+
	jsr	Fadd
; **************************************************
;     coding_adj();
; **************************************************
	move	(r6)-
	jsr	Fcoding_adj
; **************************************************
;     tone_detector(&D_DQ, &D_T, &D_YL, &D_t_dr, &D_LAST_TR);
; **************************************************
	move	#FD_LAST_TR,r1
	move	#>FD_YL,x0
	move	r1,x:(r6)+
	move	#FD_t_dr,r1
	move	r1,x:(r6)+
	move	x0,x:(r6)+
	move	r5,x:(r6)+
	move	r2,x:(r6)+
	jsr	Ftone_detector
; **************************************************
;     speed_control(&D_T, &D_Y, &D_DMS, &D_DML, &D_AP);
; **************************************************
	move	#5,n6
	move	#FD_AP,r1
	move	(r6)-n6
	move	r1,x:(r6)+
	move	#FD_DML,r1
	move	r1,x:(r6)+
	move	#FD_DMS,r1
	move	r1,x:(r6)+
	move	r4,x:(r6)+
	move	r5,x:(r6)+
	jsr	Fspeed_control
; **************************************************
;     scale_factor(&D_Y, &D_YL, &D_Y_L, &D_LAST_Y);
; **************************************************
	move	#5,n6
	move	#FD_LAST_Y,r1
	move	(r6)-n6
	move	r1,x:(r6)+
	move	#FD_Y_L,r1
	move	r1,x:(r6)+
	move	x0,x:(r6)+
	move	r4,x:(r6)+
	jsr	Fscale_factor
; **************************************************
; 
;     disk_record();
; **************************************************
	move	#4,n6
	move	(r6)-n6
	jsr	Fdisk_record
	move	x:FExitFlag,a
	tst	a
	jeq	L236
L234
; **************************************************
;   }
; 
;   post_disk_play();
; **************************************************
	jsr	Fpost_disk_play
; **************************************************
;   post_disk_record();
; **************************************************
	jsr	Fpost_disk_record
; **************************************************
; 
; #ifdef __LOOPCOUNT__
;   printf("\nLoop1: %d", Loop1);
;   printf("\nLoop2: %d", Loop2);
;   printf("\nLoop3: %d", Loop3);
;   printf("\nLoop4: %d", Loop4);
;   printf("\nLoop5: %d", Loop5);
;   printf("\nLoop6: %d", Loop6);
;   printf("\nLoop7: %d", Loop7);
; #endif
; 
; #ifdef __DOWNLOAD__
;   run56k_end();
; #endif
;   return (0);
; **************************************************
; **************************************************
; } 
; **************************************************
	clr	a	(r6)-
	tst	a	x:(r6)-,r7
	move	x:(r6)-,r5
	move	x:(r6)-,r4
	move	x:(r6)-,r3
	move	x:(r6)-,r2
	move	x:(r6)-,r1
	move	x:(r6),x0
	move	(r6)-
	move	x:(r6),ssh
	rts

	org	x:
	global	FTDP
FTDP	bsc	1
	global	FSP
FSP	bsc	1
	global	FSL
FSL	bsc	1
	global	FS_E
FS_E	bsc	1
	global	FSD
FSD	bsc	1
	global	FI
FI	bsc	1
	global	FD
FD	bsc	1
	global	FAL
FAL	bsc	1
	global	FA_2
FA_2	bsc	1
	global	FExitFlag
FExitFlag	bsc	1
	global	Fostrm
Fostrm	bsc	1
	global	Fistrm
Fistrm	bsc	1

	endsec

; 
; 
; 
; 
; 
; 
; 
; 
; 
; 
; 

