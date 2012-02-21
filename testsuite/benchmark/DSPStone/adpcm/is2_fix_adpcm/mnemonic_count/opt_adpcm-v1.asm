;
;	annotate version 1.03
;
; **************************************************
; **************************************************
	section	opt_adpcm_v1_c
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
	global	Ffmult
Ffmult
; **************************************************
; /*
;  * opt_adpcm.c
;  * Handwritten version of an optimized ADPCM transcoder applying the
;  * CCITT recommendation G.721
;  * 6-10-93 Chris Schlaeger
;  * VERSION 1.0
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
; int fmult(int An, int SRn)
; {
;   int  AnS, AnMAG, AnEXP, AnMANT;
;   int  SRnS, SRnEXP, SRnMANT;
;   int  WAnS, WAnEXP, WAnMAG;
;   long WAnMANT;
; 
;   AnS = An >> 15;
; **************************************************
	move	#65533,n0
	move	#14,n6
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
;   AnMAG = AnS == 0 ? An >> 2 : (16384 - (An >> 2)) & 8191;
; **************************************************
	move	b1,b
	tst	b	b1,x:(r0)
	jne	L2
	move	x:(r0+n0),b
	move	b1,b
	asr	b
	asr	b
	move	b1,b
	jmp	L15
L2
	move	#65533,n0
	move	x:(r0+n0),b
	move	#10,n0
	move	b1,b
	asr	b
	asr	b
	move	b1,b
	move	b1,x1
	move	b1,x:(r0+n0)
	move	#6,n0
	move	#>16384,b
	sub	x1,b	#>8191,x1
	and	x1,b	b1,x:(r0+n0)
	move	b1,b
L15
; **************************************************
;   for (AnEXP = 0; (AnMAG >> AnEXP) != 0; AnEXP++)
; **************************************************
	move	#6,n0
	move	#0,x1
	move	b1,x:(r0+n0)
	move	(r0)+
	move	x1,x:(r0)-
	move	x:(r0+n0),b
	tst	b
	jeq	L14
	move	#>1,y0
L6
	move	#6,n0
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
;     ;
;   AnMANT = AnMAG == 0 ? 1 << 5 : (AnMAG << 6) >> AnEXP;
; **************************************************
	move	#6,n0
	move	x:(r0+n0),b
	tst	b
	jne	L8
	move	#2,n0
	move	#>32,x1
	move	x1,x:(r0+n0)
	jmp	L9
L8
	move	#6,n0
	move	x:(r0+n0),b
	rep	#6
	asl	b
	move	#2,n0
	move	b1,b
	move	b1,x:(r0+n0)
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
L9
; **************************************************
; 
;   SRnS = SRn >> 10;
; **************************************************
	move	#65532,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   SRnEXP = (SRn >> 6) & 15;
; **************************************************
	move	#6,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#65532,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#6
	asr	b
; **************************************************
;   SRnMANT = SRn & 63;
; **************************************************
; **************************************************
; 
;   WAnS = SRnS ^ AnS;
; **************************************************
; **************************************************
;   WAnEXP = SRnEXP + AnEXP;
; **************************************************
; **************************************************
;   WAnMANT = (((long) SRnMANT * AnMANT) + 48) >> 4;
; **************************************************
	move	#9,n0
	move	#>15,x1
	move	b1,b
	and	x1,b	#>63,x1
	move	b1,b
	move	b1,x:(r0+n0)
	move	#65532,n0
	move	x:(r0+n0),b
	and	x1,b	#3,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#6,n0
	move	x:(r0),x1
	move	x:(r0+n0),b
	eor	x1,b	#4,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#9,n0
	move	x:(r0+n0),b
	move	#5,n0
	move	(r0)+
	move	x:(r0)-,x1
	add	x1,b
	move	b1,x:(r0+n0)
	move	#3,n0
	move	x:(r0+n0),b
	move	#2,n0
	move	b1,x0
	move	b2,x1
	move	x:(r0+n0),b
	move	b1,y0
	tfr	x1,b	b2,y1
	tfr	y1,a	x0,b0
	move	y0,a0
	jsr	lmpy_ab
	move	#10,n0
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
; **************************************************
;   WAnMAG = (int) (WAnEXP <= 26 ?
;                   (WAnMANT << 7) >> (26 - WAnEXP) :
; 	          ((WAnMANT << 7) << (WAnEXP - 26)) & 32767);
; **************************************************
	move	#7,n0
	move	#>26,x1
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#5,n0
	move	x:(r0+n0),b
	cmp	x1,b
	jgt	L10
	move	#7,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	rep	#7
	asl	b
	move	#10,n0
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#5,n0
	move	#>26,b
	move	x:(r0+n0),x1
	sub	x1,b	#7,n0
	tfr	b,a
	tst	a	b1,x:(r0+n0)
	move	#10,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	move	b0,x:(r6)
	move	b1,b
	move	x:(r6),b0
	jeq	L18
	rep	a
	asr	b
L18
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	x:(r0+n0),x1
	move	#6,n0
	move	x1,x:(r0+n0)
	jmp	L11
L10
	move	#7,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	rep	#7
	asl	b
	move	#10,n0
	move	#>16777190,x1
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	#5,n0
	move	x:(r0+n0),b
	add	x1,b	#7,n0
	tfr	b,a
	tst	a	b1,x:(r0+n0)
	move	#10,n0
	move	(r0)+
	move	(r0)+n0
	move	x:(r0)-,b
	move	x:(r0)-n0,b0
	jeq	L19
	rep	a
	asl	b
L19
	move	(r0)+
	move	(r0)+n0
	move	b1,x:(r0)-
	move	b0,x:(r0)-n0
	move	x:(r0+n0),x1
	tfr	x1,b	#6,n0
	move	x1,x:(r0+n0)
	move	#>32767,x1
	and	x1,b
	move	b1,b
	move	b1,x:(r0+n0)
L11
; **************************************************
;   return (WAnS == 0 ? WAnMAG : (65536 - WAnMAG) & 65535);
; **************************************************
	move	#4,n0
	move	x:(r0+n0),b
	tst	b
	jne	L12
	move	#6,n0
	move	x:(r0+n0),x1
	move	#10,n0
	move	x1,x:(r0+n0)
	jmp	L13
L12
	move	#6,n0
	move	#>65536,b
	move	x:(r0+n0),x1
	sub	x1,b	#10,n0
	move	#>65535,x1
	and	x1,b	b1,x:(r0+n0)
	move	b1,b
	move	b1,x:(r0+n0)
L13
; **************************************************
; }
; **************************************************
	move	#10,n0
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
;   {
;     int SLS, SES, SLI, SEI;
;     
;     
;     /* subta */
;     SLS = SL >> 13;
; **************************************************
	move	#2,n6
	move	r0,x:(r6)+
	move	r6,r0
	move	(r6)+n6
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	x1,x:(r6)+
	move	x:FSL,x1
	tfr	x1,a	(r0)+
	move	a1,a
	move	x1,x:(r0)-
	rep	#13
	asr	a
; **************************************************
;     SLI = SLS == 0 ? SL : 49152 + SL;
; **************************************************
	move	a1,a
	tst	a
	jne	L21
	move	x1,x:(r0)
	jmp	L22
L21
	move	x:FSL,a
	move	#>49152,b
	add	b,a
	move	a1,x:(r0)
L22
; **************************************************
;     SES = S_E >> 14;
; **************************************************
	move	x:FS_E,x1
	tfr	x1,a	(r0)+
	move	a1,a
	move	x1,x:(r0)-
	rep	#14
	asr	a
; **************************************************
;     SEI = SES == 0 ? S_E : 32768 + S_E;
; **************************************************
	move	a1,a
	tst	a
	jne	L23
	move	(r0)+
	move	x:(r0)-,x0
	jmp	L24
L23
	move	#>32768,b
	move	x:FS_E,a
	add	b,a
	move	a1,x0
L24
; **************************************************
;     D = (SLI + 65536 - SEI) & 65535;
; **************************************************
; **************************************************
;   }
;   
; }
; **************************************************
	tfr	x0,b	#>16711680,a
	add	a,b	#>65535,x0
	tfr	b,a	(r6)-
	move	x:(r0),b
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
;   
;   int n;
;   int DQS, DQI, SEZS, SEZI, DQSEZ, PK0, SIGPK; /* for ADDC */
;   int SES, SEI, SR;	/* for ADDB */
;   int SRS, SR1;	/* for FLOATB */
;   int PKS1, PKS2, UGA2A, A1S, FA1, FA, UGA2B, UGA2S,
;   UGA2, A2S, ULA2, UA2, A2T;	/* for UPA2 */
;   int A2P;	/* for LIMC */
;   int A1R, A2R, BR[6];	/* for TRIGB */
;   int WA1, WA2;
;   int PKS, UGA1, ULA1, UA1, A1T;	/* for UA1 */
;   int A1UL, A1LL, A1P;	/* for LIMD */
;   int MAG, EXP, MANT, DQ1; /* for FLOATA */
;   int U[6];	/* for XOR */
;   int DQMAG, UGB, BS, ULB, UB, BP[6]; /* for UPB */
;   int WB1, WB2, WB3, WB4, WB5, WB6;
;   
;   /* ADDC */
;   DQS = *DQ >> 14;
; **************************************************
	move	#65533,n0
	move	#41,n6
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
	move	x:(r0+n0),r1
	move	#65522,n0
	move	x:(r1),y1
	tfr	y1,b	x:(r0+n0),r4
	move	#65521,n0
	move	b1,b
	move	x:(r0+n0),r5
	move	#65520,n0
	move	x:(r0+n0),r3
	move	#40,n0
	move	y1,x:(r0+n0)
	rep	#14
	asr	b
; **************************************************
;   DQI = DQS == 0 ? *DQ : (65536 - (*DQ & 16383)) & 65535;
; **************************************************
	move	#18,n0
	move	b1,b
	tst	b	b1,x:(r0+n0)
	jne	L26
	move	#19,n0
	move	y1,x:(r0+n0)
	jmp	L27
L26
	move	#40,n0
	move	x:(r1),y1
	tfr	y1,b	#>16383,x0
	and	x0,b	#>65535,x0
	move	b1,b
	move	b1,y1
	move	b1,x:(r0+n0)
	move	#19,n0
	move	#>65536,b
	sub	y1,b
	and	x0,b	b1,x:(r0+n0)
	move	b1,b
	move	b1,x:(r0+n0)
L27
; **************************************************
;   SEZS = *SEZ >> 14;
; **************************************************
	move	#65518,n0
	move	x:(r0+n0),r7
	move	#38,n0
	move	x:(r7),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	b1,b
	rep	#14
	asr	b
; **************************************************
;   SEZI = SEZS == 0 ? *SEZ : (1 << 15) + *SEZ;
; **************************************************
	move	b1,b
	tst	b
	jne	L28
	move	y1,x:(r0+n0)
	jmp	L29
L28
	move	#65518,n0
	move	x:(r0+n0),r7
	move	#40,n0
	move	x:(r7),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	#38,n0
	move	#>32768,y1
	add	y1,b
	move	b1,x:(r0+n0)
L29
; **************************************************
;   DQSEZ = (DQI + SEZI) & 65535;
; **************************************************
; **************************************************
;   PK0 = DQSEZ >> 15;
; **************************************************
	move	#19,n0
	move	#>65535,x0
	move	x:(r0+n0),b
	move	#38,n0
	move	x:(r0+n0),y1
	add	y1,b	#40,n0
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	b1,b
	rep	#15
	asr	b
; **************************************************
;   SIGPK = DQSEZ == 0 ? 1 : 0;
; **************************************************
	move	#20,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#40,n0
	move	x:(r0+n0),b
	tst	b
	jne	L30
	move	#>1,y1
	jmp	L112
L30
	move	#0,y1
L112
; **************************************************
;   
;   /* ADDB */
;   DQS = *DQ >> 14;
; **************************************************
	move	#21,n0
	move	y1,x:(r0+n0)
	move	#40,n0
	move	x:(r1),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	b1,b
	rep	#14
	asr	b
; **************************************************
;   DQI = DQS == 0 ? *DQ : (65536 - (*DQ & 16383)) & 65535;
; **************************************************
	move	#18,n0
	move	b1,b
	tst	b	b1,x:(r0+n0)
	jne	L32
	move	#19,n0
	move	y1,x:(r0+n0)
	jmp	L33
L32
	move	#40,n0
	move	x:(r1),y1
	tfr	y1,b	#>16383,x0
	and	x0,b	#>65535,x0
	move	b1,b
	move	b1,y1
	move	b1,x:(r0+n0)
	move	#19,n0
	move	#>65536,b
	sub	y1,b
	and	x0,b	b1,x:(r0+n0)
	move	b1,b
	move	b1,x:(r0+n0)
L33
; **************************************************
;   SES = *SE >> 14;
; **************************************************
	move	#65519,n0
	move	x:(r0+n0),r7
	move	#38,n0
	move	x:(r7),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	b1,b
	rep	#14
	asr	b
; **************************************************
;   SEI = SES == 0 ? *SE : (1 << 15) + *SE;
; **************************************************
	move	b1,b
	tst	b
	jne	L34
	move	#22,n0
	move	y1,x:(r0+n0)
	jmp	L35
L34
	move	#65519,n0
	move	x:(r0+n0),r7
	move	#40,n0
	move	x:(r7),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	#22,n0
	move	#>32768,y1
	add	y1,b
	move	b1,x:(r0+n0)
L35
; **************************************************
;   SR = (DQI + SEI) & 65535;
; **************************************************
; **************************************************
;   
;   /* FLOATB */
;   SRS = SR >> 15;
; **************************************************
	move	#19,n0
	move	#>65535,x0
	move	x:(r0+n0),b
	move	#22,n0
	move	x:(r0+n0),y1
	add	y1,b	#38,n0
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	b1,b
	rep	#15
	asr	b
; **************************************************
;   MAG = SRS == 0 ? SR : (65536 - SR) & 32767;
; **************************************************
	move	#23,n0
	move	b1,b
	tst	b	b1,x:(r0+n0)
	jne	L36
	move	#38,n0
	move	x:(r0+n0),y1
	move	#34,n0
	move	y1,x:(r0+n0)
	jmp	L37
L36
	move	#38,n0
	move	#>32767,x0
	move	x:(r0+n0),y1
	move	#34,n0
	move	#>65536,b
	sub	y1,b
	and	x0,b	b1,x:(r0+n0)
	move	b1,b
	move	b1,x:(r0+n0)
L37
; **************************************************
;   for (EXP = 0; (MAG >> EXP) != 0; EXP++)
; **************************************************
	move	#38,n0
	move	#0,y1
	move	y1,x:(r0+n0)
	move	#34,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L111
	move	#>1,x0
L40
	move	#38,n0
	move	x:(r0+n0),b
	add	x0,b
	tfr	b,a	b1,x:(r0+n0)
	tst	a	#34,n0
	move	x:(r0+n0),b
	jeq	L118
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L118
	tst	b
	jne	L40
L111
; **************************************************
;     ;
;   MANT = MAG == 0 ? 1 << 5 : (MAG << 6) >> EXP;
; **************************************************
	move	#34,n0
	move	x:(r0+n0),b
	tst	b
	jne	L42
	move	#35,n0
	move	#>32,y1
	move	y1,x:(r0+n0)
	jmp	L43
L42
	move	#34,n0
	move	x:(r0+n0),b
	rep	#6
	asl	b
	move	#35,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#38,n0
	move	x:(r0+n0),a
	tst	a
	jeq	L119
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L119
	move	#35,n0
	move	b1,x:(r0+n0)
L43
; **************************************************
;   SR1 = (SRS << 10) + (EXP << 6) + MANT;
; **************************************************
	move	#23,n0
	move	x:(r0+n0),b
	rep	#10
	asl	b
	move	#40,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#38,n0
	move	x:(r0+n0),b
	rep	#6
	asl	b
; **************************************************
;   
;   /* UPA2 */
;   PKS1 = PK0 ^ *PK1;
; **************************************************
; **************************************************
;   PKS2 = PK0 ^ *PK2;
; **************************************************
; **************************************************
;   UGA2A = PKS2 == 0 ? 16384 : 114688;
; **************************************************
	move	b1,b
	move	b1,y1
	move	b1,x:(r0+n0)
	move	#40,n0
	move	x:(r0+n0),b
	add	y1,b	#35,n0
	move	x:(r0+n0),y1
	add	y1,b	#24,n0
	move	b1,x:(r0+n0)
	move	#65525,n0
	move	x:(r0+n0),r7
	move	#20,n0
	move	x:(r7),x0
	move	x:(r0+n0),b
	eor	x0,b	#25,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#65524,n0
	move	x:(r0+n0),r7
	move	#20,n0
	move	x:(r7),x0
	move	x:(r0+n0),b
	eor	x0,b
	move	b1,b
	tst	b
	jne	L44
	move	#>16384,x1
	jmp	L45
L44
	move	#>114688,x1
L45
; **************************************************
;   A1S = *A1 >> 15;
; **************************************************
	move	#38,n0
	move	x:(r4),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	b1,b
	rep	#15
	asr	b
; **************************************************
;   if (A1S == 0)
; **************************************************
	move	#40,n0
	move	b1,b
	tst	b	b1,x:(r0+n0)
	jne	L46
; **************************************************
;     FA1 = *A1 <= 8191 ? *A1 << 2 : 8191 << 2;
; **************************************************
	move	#38,n0
	move	#>8191,y1
	move	x:(r0+n0),b
	cmp	y1,b
	jgt	L47
	asl	b
	asl	b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L49
L47
	move	#>32764,y1
	jmp	L113
L46
; **************************************************
;   else
;     FA1 = *A1 >= 57345 ? (*A1 << 2) & 131071 : 24577 << 2;
; **************************************************
	move	#38,n0
	move	x:(r4),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	#>57344,y1
	cmp	y1,b
	jle	L50
	asl	b	#>131071,x0
	asl	b
	move	b1,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L49
L50
	move	#>98308,y1
L113
	move	#38,n0
	move	y1,x:(r0+n0)
L49
; **************************************************
;   FA = PKS1 == 1 ? FA1 : (131072 - FA1) & 131071;
; **************************************************
	move	#25,n0
	move	#>1,y1
	move	x:(r0+n0),b
	cmp	y1,b
	jne	L52
	move	#38,n0
	move	x:(r0+n0),y1
	move	#40,n0
	move	y1,x:(r0+n0)
	jmp	L53
L52
	move	#38,n0
	move	#>131071,x0
	move	x:(r0+n0),y1
	move	#40,n0
	move	#>131072,b
	sub	y1,b
	and	x0,b	b1,x:(r0+n0)
	move	b1,b
	move	b1,x:(r0+n0)
L53
; **************************************************
;   UGA2B = (UGA2A + FA) & 131071;
; **************************************************
; **************************************************
;   UGA2S = UGA2B >> 16;
; **************************************************
	tfr	x1,b	#40,n0
	move	#>131071,x0
	move	x:(r0+n0),y1
	add	y1,b	#38,n0
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	b1,b
	rep	#16
	asr	b
; **************************************************
;   UGA2 = SIGPK == 1 ? 0 :
;     UGA2S == 0 ? UGA2B >> 7 : (UGA2B >> 7) + 64512;
; **************************************************
	move	#26,n0
	move	#>1,y1
	move	b1,b
	move	b1,x:(r0+n0)
	move	#21,n0
	move	x:(r0+n0),b
	cmp	y1,b
	jne	L54
	move	#27,n0
	move	#0,y1
	move	y1,x:(r0+n0)
	jmp	L55
L54
	move	#26,n0
	move	x:(r0+n0),b
	tst	b
	jne	L56
	move	#38,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#7
	asr	b
	move	b1,b
	jmp	L114
L56
	move	#38,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#7
	asr	b
	move	#27,n0
	move	#>64512,y1
	move	b1,b
	add	y1,b	b1,x:(r0+n0)
L114
	move	#27,n0
	move	b1,x:(r0+n0)
L55
; **************************************************
;   A2S = *A2 >> 15;
; **************************************************
	move	#38,n0
	move	x:(r5),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	b1,b
	rep	#15
	asr	b
; **************************************************
;   ULA2 = A2S == 0 ? (65536 - (*A2 >> 7)) & 65535 :
;     (65536 - ((*A2 >> 7) + 65024)) & 65535;
; **************************************************
	move	b1,b
	tst	b
	jne	L58
	move	x:(r0+n0),b
	move	b1,b
	rep	#7
	asr	b
	move	b1,b
	move	b1,y1
	move	#>65536,b
	sub	y1,b
	jmp	L115
L58
	move	x:(r5),y1
	tfr	y1,b
	move	b1,b
	rep	#7
	asr	b
	move	#40,n0
	move	b1,b
	move	b1,y1
	move	#>512,b
	sub	y1,b
	move	b1,x:(r0+n0)
L115
; **************************************************
;   UA2 = (UGA2 + ULA2) & 65535;
; **************************************************
; **************************************************
;   A2T = (*A2 + UA2) & 65535;
; **************************************************
; **************************************************
;   
;   /* LIMC */
;   A2P = 32768 <= A2T && A2T <= 53248 ? 53248 :
;     12288 <= A2T && A2T <= 32767 ? 12288 : A2T;
; **************************************************
	move	#40,n0
	move	#>65535,x0
	and	x0,b	#>65535,x0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#27,n0
	move	x:(r0+n0),b
	move	#40,n0
	move	x:(r0+n0),y1
	add	y1,b	#38,n0
	and	x0,b	x:(r5),y1
	move	b1,b
	add	y1,b	#>32767,y1
	and	x0,b
	move	b1,b
	cmp	y1,b	b1,x:(r0+n0)
	jle	L60
	move	#>53248,x0
	cmp	x0,b
	jgt	L60
	move	x0,y0
	jmp	L61
L60
	move	#38,n0
	move	#>12287,y1
	move	x:(r0+n0),b
	cmp	y1,b
	jle	L62
	move	#>32767,y1
	cmp	y1,b
	jgt	L62
	move	#>12288,y0
	jmp	L61
L62
	move	#38,n0
	move	x:(r0+n0),y0
L61
; **************************************************
;   
;   /* TRIGB */
;   A2R = *T == 0 ? A2P : 0;
; **************************************************
	move	#65532,n0
	move	x:(r0+n0),r7
	move	#40,n0
	move	x:(r7),y1
	tfr	y1,b	y1,x:(r0+n0)
	tst	b
	jne	L64
	move	#29,n0
	move	y0,x:(r0+n0)
	jmp	L65
L64
	move	#29,n0
	move	#0,y1
	move	y1,x:(r0+n0)
L65
; **************************************************
;   
;   /* FMULT */
;   WA2 = fmult(A2R, *SR2);
; **************************************************
	move	#65523,n0
	move	x:(r0+n0),r7
	move	#40,n0
	move	x:(r7),y1
	move	y1,x:(r0+n0)
	move	#29,n0
	move	y1,x:(r6)+
	move	x:(r0+n0),y1
	move	y1,x:(r6)+
	jsr	Ffmult
; **************************************************
;   
;   /* UPA1 */
;   PKS = PK0 ^ *PK1;
; **************************************************
; **************************************************
;   UGA1 = SIGPK == 1 ? 0 : (PKS == 0 ? 192 : 65344);
; **************************************************
	move	#31,n0
	move	#>1,y1
	move	a1,x:(r0+n0)
	move	#65525,n0
	move	(r6)-
	move	x:(r0+n0),r7
	move	#20,n0
	move	x:(r7),x0
	move	x:(r0+n0),b
	eor	x0,b	#38,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#21,n0
	move	(r6)-
	move	x:(r0+n0),b
	cmp	y1,b
	jne	L66
	move	#0,x1
	jmp	L67
L66
	move	#38,n0
	move	x:(r0+n0),b
	tst	b
	jne	L68
	move	#>192,x1
	jmp	L67
L68
	move	#>65344,x1
L67
; **************************************************
;   A1S = *A1 >> 15;
; **************************************************
	move	#38,n0
	move	x:(r4),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	b1,b
	rep	#15
	asr	b
; **************************************************
;   ULA1 = A1S == 0 ? (65536 - (*A1 >> 8)) & 65535 :
;     (65536 - ((*A1 >> 8) + 65280)) & 65535;
; **************************************************
	move	#40,n0
	move	b1,b
	tst	b	b1,x:(r0+n0)
	jne	L70
	move	#38,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#8
	asr	b
	move	b1,b
	move	b1,y1
	move	#>65536,b
	sub	y1,b
	jmp	L116
L70
	move	x:(r4),y1
	tfr	y1,b
	move	b1,b
	rep	#8
	asr	b
	move	#40,n0
	move	b1,b
	move	b1,y1
	move	#>256,b
	sub	y1,b
	move	b1,x:(r0+n0)
L116
; **************************************************
;   UA1 = (UGA1 + ULA1) & 65535;
; **************************************************
; **************************************************
;   A1T = (*A1 + UA1) & 65535;
; **************************************************
; **************************************************
;   
;   /* LIMD */
;   A1UL = (15360 + 65536 - A2P) & 65535;
; **************************************************
; **************************************************
;   A1LL = (A2P + 65536 - 15360) & 65535;
; **************************************************
; **************************************************
;   A1P = 32768 <= A1T && A1T <= A1LL ? A1LL :
;     A1UL <= A1T && A1T <= 32767 ? A1UL : A1T;
; **************************************************
	move	#40,n0
	move	#>65535,x0
	and	x0,b	#>65535,x0
	move	b1,b
	tfr	x1,b	b1,x:(r0+n0)
	move	x:(r0+n0),y1
	add	y1,b	#38,n0
	and	x0,b	x:(r4),y1
	move	b1,b
	add	y1,b	#>50176,y1
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	#32,n0
	move	#>80896,b
	sub	y0,b
	and	x0,b
	move	b1,b
	tfr	y0,b	b1,x:(r0+n0)
	add	y1,b	#33,n0
	and	x0,b	#>32767,y1
	move	b1,b
	move	b1,x:(r0+n0)
	move	#38,n0
	move	x:(r0+n0),b
	cmp	y1,b
	jle	L72
	move	#33,n0
	move	x:(r0+n0),y1
	cmp	y1,b
	jgt	L72
	move	x:(r0+n0),x0
	jmp	L73
L72
	move	#32,n0
	move	x:(r0+n0),b
	move	#38,n0
	move	x:(r0+n0),y1
	cmp	y1,b
	jgt	L74
	move	#>32767,y1
	move	x:(r0+n0),b
	cmp	y1,b
	jgt	L74
	move	#32,n0
	move	x:(r0+n0),x0
	jmp	L73
L74
	move	#38,n0
	move	x:(r0+n0),x0
L73
; **************************************************
;   
;   /* TRIGB */
;   A1R = *T == 0 ? A1P : 0;
; **************************************************
	move	#65532,n0
	move	x:(r0+n0),r7
	move	#40,n0
	move	x:(r7),y1
	tfr	y1,b	y1,x:(r0+n0)
	tst	b
	jne	L76
	move	#28,n0
	move	x0,x:(r0+n0)
	jmp	L77
L76
	move	#28,n0
	move	#0,y1
	move	y1,x:(r0+n0)
L77
; **************************************************
;   
;   /* FMULT */
;   WA1 = fmult(A1R, SR1);
; **************************************************
	move	#24,n0
	move	x:(r0+n0),y1
	move	#28,n0
	move	y1,x:(r6)+
	move	x:(r0+n0),y1
	move	y1,x:(r6)+
	jsr	Ffmult
; **************************************************
;   
;   /* FLOATA */
;   DQS = *DQ >> 14;
; **************************************************
	move	#30,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	#40,n0
	move	x:(r1),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	b1,b
	move	(r6)-
	rep	#14
	asr	b
; **************************************************
;   MAG = *DQ & 16383;
; **************************************************
; **************************************************
;   for (EXP = 0; (MAG >> EXP) != 0; EXP++)
; **************************************************
	move	#18,n0
	move	#>16383,x0
	move	b1,b
	tfr	y1,b	b1,x:(r0+n0)
	and	x0,b	#34,n0
	move	b1,b
	tst	b	#0,y1
	move	b1,x:(r0+n0)
	move	#38,n0
	move	y1,x:(r0+n0)
	jeq	L110
	move	#>1,x0
L80
	move	#38,n0
	move	x:(r0+n0),b
	add	x0,b
	tfr	b,a	b1,x:(r0+n0)
	tst	a	#34,n0
	move	x:(r0+n0),b
	jeq	L120
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L120
	tst	b
	jne	L80
L110
; **************************************************
;     ;
;   MANT = MAG == 0 ? 1 << 5 : (MAG << 6) >> EXP;
; **************************************************
	move	#34,n0
	move	x:(r0+n0),b
	tst	b
	jne	L82
	move	#35,n0
	move	#>32,y1
	move	y1,x:(r0+n0)
	jmp	L83
L82
	move	#34,n0
	move	x:(r0+n0),b
	rep	#6
	asl	b
	move	#35,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#38,n0
	move	x:(r0+n0),a
	tst	a
	jeq	L121
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L121
	move	#35,n0
	move	b1,x:(r0+n0)
L83
; **************************************************
;   DQ1 = (DQS << 10) + (EXP << 6) + MANT;
; **************************************************
	move	#18,n0
	move	x:(r0+n0),b
	rep	#10
	asl	b
	move	#40,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#38,n0
	move	x:(r0+n0),b
	rep	#6
	asl	b
; **************************************************
;   
;   /* XOR */
;   DQS = *DQ >> 14;
; **************************************************
	move	b1,b
	move	b1,y1
	move	b1,x:(r0+n0)
	move	#40,n0
	move	x:(r0+n0),b
	add	y1,b	#35,n0
	move	x:(r0+n0),y1
	add	y1,b	#36,n0
	move	b1,x:(r0+n0)
	move	x:(r1),y1
	tfr	y1,b
	move	b1,b
	rep	#14
	asr	b
; **************************************************
;   U[0] = DQS ^ (*DQ2 >> 10);
; **************************************************
	move	#18,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#65531,n0
	move	x:(r0+n0),r7
	move	x:(r7),y1
	tfr	y1,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   U[1] = DQS ^ (*DQ3 >> 10);
; **************************************************
	move	#18,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#6,n0
	move	b1,b
	move	b1,y1
	move	y1,x:(r0+n0)
	move	#65530,n0
	move	x:(r0+n0),r7
	move	x:(r7),y1
	tfr	y1,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   U[2] = DQS ^ (*DQ4 >> 10);
; **************************************************
	move	#18,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#7,n0
	move	b1,b
	move	b1,y1
	move	y1,x:(r0+n0)
	move	#65529,n0
	move	x:(r0+n0),r7
	move	x:(r7),y1
	tfr	y1,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   U[3] = DQS ^ (*DQ5 >> 10);
; **************************************************
	move	#18,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#8,n0
	move	b1,b
	move	b1,y1
	move	y1,x:(r0+n0)
	move	#65528,n0
	move	x:(r0+n0),r7
	move	x:(r7),y1
	tfr	y1,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   U[4] = DQS ^ (*DQ6 >> 10);
; **************************************************
	move	#18,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#9,n0
	move	b1,b
	move	b1,y1
	move	y1,x:(r0+n0)
	move	#65527,n0
	move	x:(r0+n0),r7
	move	x:(r7),y1
	tfr	y1,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   U[5] = DQS ^ (*DQ7 >> 10);
; **************************************************
	move	#18,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#10,n0
	move	b1,b
	move	b1,y1
	move	y1,x:(r0+n0)
	move	#65526,n0
	move	x:(r0+n0),r7
	move	x:(r7),y1
	tfr	y1,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   
;   /* UPB */
;   DQMAG = *DQ & 16383;
; **************************************************
; **************************************************
;   for (n = 0; n < 6; n++)
; **************************************************
	move	#12,n2
	move	#18,n0
	move	r0,r2
	move	#>65535,y0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#11,n0
	move	b1,b
	move	b1,y1
	move	#>16383,x0
	move	y1,x:(r0+n0)
	move	#40,n0
	move	x:(r1),y1
	tfr	y1,b	y1,x:(r0+n0)
	and	x0,b	#37,n0
	move	b1,b
	move	#0,x0
	move	b1,x:(r0+n0)
	move	(r2)+n2
	do	#6,L109
L93
; **************************************************
;     {
;       UGB = DQMAG == 0 ? 0 : (U[n] == 0 ? 128 : 65408);
; **************************************************
	move	#37,n0
	move	x:(r0+n0),b
	tst	b
	jne	L87
	move	#0,x1
	jmp	L88
L87
	move	#40,n0
	move	#6,n1
	move	r0,b
	add	x0,b
	move	b1,r1
	move	x:(r1+n1),y1
	tfr	y1,b	y1,x:(r0+n0)
	tst	b
	jne	L89
	move	#>128,x1
	jmp	L90
L89
	move	#>65408,x1
L90
L88
; **************************************************
;       BS = B[n] >> 15;
; **************************************************
	move	#38,n0
	move	r3,b
	add	x0,b
	move	b1,r1
	move	x:(r1),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	b1,b
	rep	#15
	asr	b
; **************************************************
;       ULB = BS == 0 ? (65536 - (B[n] >> 8)) & 65535 :
; 	(65536 - ((B[n] >> 8) + 65280)) & 65535;
; **************************************************
	move	b1,b
	tst	b
	jne	L91
	move	x:(r0+n0),b
	move	b1,b
	rep	#8
	asr	b
	move	b1,b
	move	b1,y1
	move	#>65536,b
	jmp	L117
L91
	move	r3,b
	add	x0,b
	move	b1,r1
	move	x:(r1),y1
	tfr	y1,b
	move	b1,b
	rep	#8
	asr	b
	move	b1,b
	move	b1,y1
	move	#>256,b
L117
; **************************************************
;       UB = (UGB + ULB) & 65535;
; **************************************************
; **************************************************
;       BP[n] = (B[n] + UB) & 65535;
; **************************************************
; **************************************************
; **************************************************
	sub	y1,b	#40,n0
	and	y0,b
	move	b1,b
	tfr	x1,b	b1,x:(r0+n0)
	move	x:(r0+n0),y1
	add	y1,b
	and	y0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	#38,n0
	move	r3,b
	add	x0,b
	move	b1,r1
	move	x:(r1),y1
	move	y1,x:(r0+n0)
	move	#40,n0
	move	x:(r0+n0),b
	add	y1,b
	and	y0,b
	move	b1,b
	move	b1,y1
	tfr	x0,b	b1,x:(r0+n0)
	move	y1,x:(r2)
	move	#>1,y1
	add	y1,b	(r2)+
	move	b1,x0
L109
; **************************************************
;     }
;   
;   /* TRIGB */
;   for (n = 0; n < 6; n++)
; **************************************************
	move	#12,n2
	move	r0,r2
	move	#>1,x1
	move	#0,x0
	move	(r2)+n2
	do	#6,L107
L99
; **************************************************
;     BR[n] = *T == 0 ? BP[n] : 0;
; **************************************************
	move	#65532,n0
	move	r0,b
	add	x0,b	x:(r0+n0),r7
	move	#40,n0
	move	b1,r1
	move	x:(r7),y1
	tfr	y1,b	y1,x:(r0+n0)
	tst	b
	jne	L97
	move	x:(r2),y0
	jmp	L98
L97
	move	#0,y0
L98
; **************************************************
; **************************************************
	move	r2,b
	add	x1,b	y0,x:(r1)
	tfr	x0,b	b1,r2
	add	x1,b
	move	b1,x0
L107
; **************************************************
;   
;   /* FMULT */
;   WB1 = fmult(BR[0], DQ1);
; **************************************************
	move	#36,n0
	move	#Ffmult,r1
	move	x:(r0+n0),y1
	move	y1,x:(r6)+
	move	x:(r0),y1
	move	y1,x:(r6)+
	jsr	(r1)
; **************************************************
;   WB2 = fmult(BR[1], *DQ2);
; **************************************************
	move	#38,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	x:(r0+n0),b
	move	#65531,n0
	move	(r6)-
	move	x:(r0+n0),r7
	move	(r0)+
	move	x:(r7),y1
	move	y1,x:(r6)+
	move	x:(r0)-,y1
	move	y1,x:(r6)+
	jsr	(r1)
; **************************************************
;   WB3 = fmult(BR[2], *DQ3);
; **************************************************
	move	#65530,n0
	move	a1,x0
	move	x:(r0+n0),r7
	move	#2,n0
	move	x:(r7),y1
	move	(r6)-
	move	(r6)-
	move	y1,x:(r6)+
	move	x:(r0+n0),y1
	move	y1,x:(r6)+
	jsr	(r1)
; **************************************************
;   WB4 = fmult(BR[3], *DQ4);
; **************************************************
	move	#65529,n0
	move	a1,x1
	move	x:(r0+n0),r7
	move	#3,n0
	move	x:(r7),y1
	move	(r6)-
	move	(r6)-
	move	y1,x:(r6)+
	move	x:(r0+n0),y1
	move	y1,x:(r6)+
	jsr	(r1)
; **************************************************
;   WB5 = fmult(BR[4], *DQ5);
; **************************************************
	move	#65528,n0
	move	a1,y0
	move	x:(r0+n0),r7
	move	#4,n0
	move	x:(r7),y1
	move	(r6)-
	move	(r6)-
	move	y1,x:(r6)+
	move	x:(r0+n0),y1
	move	y1,x:(r6)+
	jsr	(r1)
; **************************************************
;   WB6 = fmult(BR[5], *DQ6);
; **************************************************
	move	#39,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	#65527,n0
	move	(r6)-
	move	x:(r0+n0),r7
	move	#5,n0
	move	x:(r7),y1
	move	y1,x:(r6)+
	move	x:(r0+n0),y1
	move	y1,x:(r6)+
	jsr	(r1)
; **************************************************
;   
;   /* ACCUM */
;   SEZI = (((((((((WB1 + WB2) & 65535) + WB3) & 65535)
; 	      + WB4) & 65535) + WB5) & 65535) + WB6) & 65535;
; **************************************************
; **************************************************
;   SEI = (((SEZI + WA2) & 65535) + WA1) & 65535;
; **************************************************
; **************************************************
;   *SEZ = SEZI >> 1;
; **************************************************
; **************************************************
;   *SE = SEI >> 1;
; **************************************************
	add	x0,b	#40,n0
	move	#>65535,x0
	and	x0,b	a1,x:(r0+n0)
	move	#39,n0
	move	b1,b
	add	x1,b	x:(r0+n0),y1
	and	x0,b	#40,n0
	move	b1,b
	add	y0,b	(r6)-
	and	x0,b	(r6)-
	move	b1,b
	add	y1,b	x:(r0+n0),y1
	and	x0,b	#38,n0
	move	b1,b
	add	y1,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	#31,n0
	move	x:(r0+n0),y1
	add	y1,b	#30,n0
	and	x0,b
	move	x:(r0+n0),y1
	move	#22,n0
	move	b1,b
	add	y1,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	#38,n0
	move	x:(r0+n0),b
	move	#65518,n0
	move	b1,b
	asr	b	x:(r0+n0),r7
	move	#22,n0
	move	b1,b
	move	b1,y1
	move	y1,x:(r7)
	move	x:(r0+n0),b
	move	b1,b
; **************************************************
;   
;   /* STATE update */
;   *PK2 = *PK1;
; **************************************************
; **************************************************
;   *PK1 = PK0;
; **************************************************
; **************************************************
;   *SR2 = SR1;
; **************************************************
; **************************************************
;   *A1 = A1R;
; **************************************************
; **************************************************
;   *A2 = A2R;
; **************************************************
; **************************************************
;   for (n = 0; n < 6; n++)
; **************************************************
	asr	b	#65519,n0
	move	r0,r1
	move	x:(r0+n0),r7
	move	#65525,n0
	move	r3,r2
	move	b1,b
	move	b1,y1
	move	y1,x:(r7)
	move	x:(r0+n0),r7
	move	#40,n0
	move	x:(r7),y1
	move	y1,x:(r0+n0)
	move	#65524,n0
	move	x:(r0+n0),r7
	move	#65525,n0
	move	y1,x:(r7)
	move	x:(r0+n0),r7
	move	#20,n0
	move	x:(r0+n0),y1
	move	#65523,n0
	move	y1,x:(r7)
	move	x:(r0+n0),r7
	move	#24,n0
	move	x:(r0+n0),y1
	move	#28,n0
	move	y1,x:(r7)
	move	x:(r0+n0),y1
	move	#29,n0
	move	y1,x:(r4)
	move	x:(r0+n0),y1
	move	y1,x:(r5)
	do	#6,L105
L103
; **************************************************
;     B[n] = BR[n];
; **************************************************
; **************************************************
; **************************************************
	move	x:(r1)+,a
	move	a1,x:(r2)+
L105
; **************************************************
;   *DQ7 = *DQ6;
; **************************************************
; **************************************************
;   *DQ6 = *DQ5;
; **************************************************
; **************************************************
;   *DQ5 = *DQ4;
; **************************************************
; **************************************************
;   *DQ4 = *DQ3;
; **************************************************
; **************************************************
;   *DQ3 = *DQ2;
; **************************************************
; **************************************************
;   *DQ2 = DQ1;
; **************************************************
; **************************************************
;   
;   S_E = *SE;
; **************************************************
; **************************************************
;   A_2 = *A2;
; **************************************************
; **************************************************
;   
; 
; }
; **************************************************
	move	#65527,n0
	move	(r6)-
	move	x:(r0+n0),r7
	move	#65526,n0
	move	x:(r7),y1
	move	x:(r0+n0),r7
	move	#65528,n0
	move	y1,x:(r7)
	move	x:(r0+n0),r7
	move	#65527,n0
	move	x:(r7),y1
	move	x:(r0+n0),r7
	move	#65529,n0
	move	y1,x:(r7)
	move	x:(r0+n0),r7
	move	#65528,n0
	move	x:(r7),y1
	move	x:(r0+n0),r7
	move	#65530,n0
	move	y1,x:(r7)
	move	x:(r0+n0),r7
	move	#65529,n0
	move	x:(r7),y1
	move	x:(r0+n0),r7
	move	#65531,n0
	move	y1,x:(r7)
	move	x:(r0+n0),r7
	move	#65530,n0
	move	x:(r7),y1
	move	x:(r0+n0),r7
	move	#65531,n0
	move	y1,x:(r7)
	move	x:(r0+n0),r7
	move	#36,n0
	move	x:(r0+n0),y1
	move	#65519,n0
	move	y1,x:(r7)
	move	x:(r0+n0),r7
	move	#40,n0
	move	x:(r7),y1
	move	y1,x:FS_E
	move	x:(r5),y1
	move	y1,x:(r0+n0)
	move	y1,x:FA_2
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
;     {
;       2048, 4, 135, 213, 273, 323, 373, 425,
;       425, 373, 323, 273, 213, 135, 4, 2048,
;     } ;
;   int DQS, DQLN, DQL, DS, DEX, DMN, DQT, DQMAG;
;   
;   /* RECONST */
;   DQS = I >> 3;
; **************************************************
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
	move	x:FI,a
	tfr	a,b
	move	b1,b
	rep	#3
	asr	b
; **************************************************
;   DQLN = qtab[I];
; **************************************************
; **************************************************
;   
;   /* ADDA */
;   DQL = (DQLN + (*Y >> 2)) & 4095;
; **************************************************
; **************************************************
;   
;   /* ANTILOG */
;   DS = DQL >> 11;
; **************************************************
	move	#65532,n0
	move	#F___qtab0,r1
	move	#>4095,x0
	move	b1,b
	move	b1,x:(r0)
	move	x:(r0+n0),r2
	move	r1,b
	add	a,b	x:(r2),x1
	tfr	x1,b	b1,r1
	move	b1,b
	asr	b	x:(r1),a
	asr	b
	move	b1,b
	move	b1,x1
	add	x1,a
	and	x0,a
	move	a1,a
	tfr	a,b
	move	b1,b
	rep	#11
	asr	b
; **************************************************
;   DEX = (DQL >> 7) & 15;
; **************************************************
	move	#2,n0
	move	b1,b
	tfr	a,b	b1,x:(r0+n0)
	move	b1,b
	rep	#7
	asr	b
; **************************************************
;   DMN = DQL & 127;
; **************************************************
; **************************************************
;   DQT = (1  << 7) + DMN;
; **************************************************
; **************************************************
;   DQMAG = DS == 0 ? (DQT << 7) >> (14 - DEX) : 0;
; **************************************************
	move	#>15,x0
	move	b1,b
	and	x0,b	#>127,x0
	and	x0,a	b1,b
	move	a1,a
	move	#>128,x0
	add	x0,a	(r0)+
	move	b1,x:(r0)-
	move	x:(r0+n0),b
	tst	b
	jne	L124
	rep	#7
	asl	a
	move	a1,a
	tfr	a,b	(r0)+
	move	x:(r0)-,x1
	move	#>14,a
	sub	x1,a
	tst	a
	jeq	L126
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L126
	move	b1,x:(r0+n0)
	jmp	L125
L124
	move	#2,n0
	move	#0,x1
	move	x1,x:(r0+n0)
L125
; **************************************************
;   *DQ = (DQS << 14) + DQMAG;
; **************************************************
	move	x:(r0),a
	rep	#14
	asl	a
; **************************************************
; }
; **************************************************
	move	#2,n0
	move	a1,a
	move	x:(r0+n0),x1
	add	x1,a	#65533,n0
	move	(r6)-
	move	x:(r0+n0),r2
	move	(r0)-
	move	a1,x:(r2)
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
;   int DS, DQM, DL, DLN, EXP, MANT;
;   
;   /* LOG */
;   DS = D >> 15;
; **************************************************
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
	move	x:FD,a
	tfr	a,b
	move	b1,b
	rep	#15
	asr	b
; **************************************************
;   DQM = DS == 0 ? D : (65536 - D) & 32767;
; **************************************************
	move	b1,b
	tst	b	b1,x:(r0)
	jne	L128
	move	(r0)+
	move	a1,x:(r0)-
	jmp	L129
L128
	move	#2,n0
	move	x:FD,x1
	move	#>32767,x0
	move	#>65536,a
	sub	x1,a	x1,x:(r0+n0)
	tfr	a,b	(r0)+
	and	x0,b	a1,x:(r0)-
	move	b1,b
	move	(r0)+
	move	b1,x:(r0)-
L129
; **************************************************
;   for (EXP = 1; (DQM >> EXP) != 0; EXP++)
; **************************************************
	move	#2,n0
	move	#>1,x1
	tfr	x1,b	x1,x:(r0+n0)
	tst	b	(r0)+
	move	x:(r0)-,a
	jeq	L168
	move	a1,a
	rep	b
	asr	a
	move	a1,a
L168
	tst	a
	jeq	L166
	move	x:(r0+n0),x0
L132
	move	#2,n0
	move	x:(r0+n0),b
	add	x0,b
	tst	b	b1,x:(r0+n0)
	move	(r0)+
	move	x:(r0)-,a
	jeq	L169
	move	a1,a
	rep	b
	asr	a
	move	a1,a
L169
	tst	a
	jne	L132
L166
; **************************************************
;     ;
;   EXP--;
; **************************************************
; **************************************************
;   MANT = ((DQM << 7) >> EXP) & 127;
; **************************************************
	move	#2,n0
	move	#>16777215,a
	move	x:(r0+n0),b
	add	a,b	(r0)+
	move	x:(r0)-,a
	rep	#7
	asl	a
	tst	b	a1,a
	jeq	L170
	move	a1,a
	rep	b
	asr	a
	move	a1,a
L170
; **************************************************
;   DL = (EXP << 7) + MANT;
; **************************************************
	move	#>127,x0
	and	x0,a
	move	a1,a
	rep	#7
	asl	b
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
	move	#>16773120,x0
	move	x:(r0+n0),r1
	move	#2,n0
	move	b1,b
	add	a,b	x:(r1),a
	move	a1,a
	asr	a
	asr	a
	move	a1,a
	add	x0,a	#>4095,x0
	sub	a,b	#>3971,a
	and	x0,b
	move	b1,b
	cmp	a,b	b1,x:(r0+n0)
	jle	L134
; **************************************************
;     I = DS ? 0xE : 0x1;
; **************************************************
	move	x:(r0),b
	tst	b
	jne	L167
	jmp	L164
L134
; **************************************************
;   else if (DLN > 2047)
; **************************************************
	move	#2,n0
	move	#>2047,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L138
; **************************************************
;     I = 0xF;
; **************************************************
	move	#>15,a
	jmp	L165
L138
; **************************************************
;   else if (DLN > 399)
; **************************************************
	move	#2,n0
	move	#>399,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L140
; **************************************************
;     I = DS ? 0x8 : 0x7;
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L141
	move	#>8,a
	jmp	L165
L141
	move	#>7,a
	jmp	L165
L140
; **************************************************
;   else if (DLN > 348)
; **************************************************
	move	#2,n0
	move	#>348,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L144
; **************************************************
;     I = DS ? 0x9 : 0x6;
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L145
	move	#>9,a
	jmp	L165
L145
	move	#>6,a
	jmp	L165
L144
; **************************************************
;   else if (DLN > 299)
; **************************************************
	move	#2,n0
	move	#>299,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L148
; **************************************************
;     I = DS ? 0xA : 0x5;	
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L149
	move	#>10,a
	jmp	L165
L149
	move	#>5,a
	jmp	L165
L148
; **************************************************
;   else if (DLN > 245)
; **************************************************
	move	#2,n0
	move	#>245,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L152
; **************************************************
;     I = DS ? 0xB : 0x4;
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L153
	move	#>11,a
	jmp	L165
L153
	move	#>4,a
	jmp	L165
L152
; **************************************************
;   else if (DLN > 177)
; **************************************************
	move	#2,n0
	move	#>177,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L156
; **************************************************
;     I = DS ? 0xC : 0x3;
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L157
	move	#>12,a
	jmp	L165
L157
	move	#>3,a
	jmp	L165
L156
; **************************************************
;   else if (DLN > 79)
; **************************************************
	move	#2,n0
	move	#>79,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L160
; **************************************************
;     I = DS ? 0xD : 0x2;
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L161
	move	#>13,a
	jmp	L165
L161
	move	#>2,a
	jmp	L165
L160
; **************************************************
;   else
;     I = DS ? 0xE : 0x1;
; **************************************************
	move	x:(r0),b
	tst	b
	jeq	L164
L167
	move	#>14,a
	jmp	L165
L164
	move	#>1,a
L165
; **************************************************
;   
; }
; **************************************************
	move	(r0)-
	move	(r0)-
	move	(r6)-
	move	a1,x:FI
	move	x:(r6)-,r1
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
	jle	L172
	move	#>53760,a
	cmp	a,b
	jgt	L172
	move	#>1,x0
	jmp	L173
L172
	move	#0,x0
L173
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
	jne	L174
	tfr	x0,a
	jmp	L175
L174
	clr	a
L175
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
	jle	L176
	move	#>15872,x1
	move	x1,x:(r0)
	jmp	L177
L176
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
	jeq	L180
	rep	b
	asl	a
	move	a1,a
L180
	move	a1,x:(r0)
L177
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
	jle	L178
	move	#>1,a
	move	(r0)+
	move	x:(r0)-,b
	cmp	a,b
	jeq	L179
L178
	clr	a
L179
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
; void speed_control(int* TR, int* Y, int* DMS, int* DML, int* AP)
; {
;   /*
;    * speed control
;    *
;    * Input signals:  T, TDP, I, Y
;    * Output signals: AL
;    */
; 
;   static int F[] = { 0, 0, 0, 1, 1, 1, 3, 7 };
;   int FI, DIF, DIFS, DIFSX, DIFM, DTHR, AX, APP;
;   
;   /* FUNCTF */ 
;   FI = F[(I >> 3 ? (15 - I) : I) & 7] ; 
; **************************************************
	move	#65531,n0
	move	#6,n6
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
	move	#5,n0
	move	y0,x:(r0+n0)
	rep	#3
	asr	a
	move	a1,a
	tst	a
	jeq	L182
	move	#>15,a
	sub	y0,a
	jmp	L183
L182
	move	x:FI,a
L183
; **************************************************
;     
;   /* FILTA */
;   DIF = ((FI << 9) + 8192 - *DMS) & 8191;
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
;   DIFS = DIF >> 12;
; **************************************************
	move	#5,n0
	move	x:(r2),a
	move	#>16769024,x0
	add	x0,a	b1,b
	sub	a,b	#>8191,x0
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	b1,b
	rep	#12
	asr	b
; **************************************************
;   DIFSX = DIFS == 1 ? (DIF >> 5) + 3840 : DIF >> 5;
; **************************************************
	move	#>1,a
	move	b1,b
	cmp	a,b
	jne	L184
	move	x:(r0+n0),b
	move	b1,b
	rep	#5
	asr	b
	move	#>3840,a
	move	b1,b
	add	a,b
	jmp	L198
L184
	move	#5,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#5
	asr	b
	move	b1,b
L198
; **************************************************
;   *DMS = (DIFSX + *DMS) & 4095;
; **************************************************
; **************************************************
;   
;   /* FILTB */
;   DIF = ((FI << 11) + 32768 - *DML) & 32767;
; **************************************************
	move	#5,n0
	move	#>4095,x0
	move	b1,x:(r0+n0)
	move	x:(r2),a
	move	x:(r0+n0),b
	add	a,b
	tfr	b,a
	and	x0,a
	move	a1,a
	move	a1,x:(r2)
	move	x:(r0),b
	rep	#11
	asl	b
; **************************************************
;   DIFS = DIF >> 14;
; **************************************************
	move	x:(r3),a
	move	#>16744448,x0
	add	x0,a	b1,b
	sub	a,b	#>32767,x0
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	b1,b
	rep	#14
	asr	b
; **************************************************
;   DIFSX = DIFS == 1 ? (DIF >> 7) + 16128 : DIF >> 7;
; **************************************************
	move	#>1,a
	move	b1,b
	cmp	a,b
	jne	L186
	move	x:(r0+n0),b
	move	b1,b
	rep	#7
	asr	b
	move	#>16128,a
	move	b1,b
	add	a,b
	jmp	L199
L186
	move	#5,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#7
	asr	b
	move	b1,b
L199
; **************************************************
;   *DML = (DIFSX + *DML) & 16383;
; **************************************************
; **************************************************
;   
;   /* SUBTC */
;   DIF = ((*DMS << 2) + 32768 - *DML) & 32767;
; **************************************************
; **************************************************
;   DIFS = DIF >> 14;
; **************************************************
	move	#5,n0
	move	#>16744448,x0
	move	#>16383,x1
	move	b1,x:(r0+n0)
	move	x:(r3),a
	move	x:(r0+n0),b
	add	a,b
	tfr	b,a
	and	x1,a
	move	a1,a
	add	x0,a	a1,x:(r3)
	move	x:(r2),y0
	tfr	y0,b	#>32767,x0
	asl	b
	asl	b
	move	b1,b
	sub	a,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	b1,b
	rep	#14
	asr	b
; **************************************************
;   DIFM = DIFS == 1 ? (32768 - DIF) & 16383 : DIF;
; **************************************************
	move	#>1,a
	move	b1,b
	cmp	a,b
	jne	L188
	move	x:(r0+n0),y0
	move	#2,n0
	move	#>32768,a
	sub	y0,a
	tfr	a,b
	and	x1,b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L189
L188
	move	#5,n0
	move	x:(r0+n0),y0
	move	#2,n0
	move	y0,x:(r0+n0)
L189
; **************************************************
;   DTHR = *DML >> 3;
; **************************************************
	move	x:(r3),a
	move	a1,a
	rep	#3
	asr	a
; **************************************************
;   AX = (*Y >= 1536 && DIFM < DTHR && TDP == 0) ? 0 : 1;
; **************************************************
	move	#3,n0
	move	#>1535,y0
	move	a1,a
	move	a1,x:(r0+n0)
	move	#65532,n0
	move	x:(r0+n0),r5
	move	x:(r5),a
	cmp	y0,a
	jle	L190
	move	#2,n0
	move	x:(r0+n0),b
	move	#3,n0
	move	x:(r0+n0),y0
	cmp	y0,b
	jge	L190
	move	x:FTDP,a
	tst	a
	jne	L190
	clr	a
	jmp	L191
L190
	move	#>1,a
L191
; **************************************************
;   
;   /* FILTC */
;   DIF = ((AX << 9) + 2048 - *AP) & 2047;
; **************************************************
	rep	#9
	asl	a
; **************************************************
;   DIFS = DIF >> 10;
; **************************************************
	move	#5,n0
	move	#>16775168,x0
	move	a1,a
	tfr	a,b	x:(r4),a
	add	x0,a	#>2047,x0
	sub	a,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   DIFSX = DIFS == 1 ? (DIF >> 4) + 896 : DIF >> 4;
; **************************************************
	move	#>1,a
	move	b1,b
	cmp	a,b
	jne	L192
	move	x:(r0+n0),b
	move	b1,b
	rep	#4
	asr	b
	move	#>896,a
	move	b1,b
	add	a,b
	jmp	L200
L192
	move	#5,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#4
	asr	b
	move	b1,b
L200
; **************************************************
;   APP = (DIFSX + *AP) & 1023;
; **************************************************
; **************************************************
;   
;   /* TRIGA */
;   *AP = *TR++ == 1 ? 256 : APP;
; **************************************************
	move	#5,n0
	move	#>1,y0
	move	#>1023,x0
	move	b1,x:(r0+n0)
	move	x:(r0+n0),b
	move	#4,n0
	move	x:(r4),a
	add	a,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	#65533,n0
	move	x:(r0+n0),r5
	move	x:(r5),a
	cmp	y0,a
	jne	L194
	move	#>256,y0
	jmp	L201
L194
	move	#4,n0
	move	x:(r0+n0),y0
L201
; **************************************************
;   
;   /* LIMA */
;   AL = *AP >= 256 ? 64 : *AP >> 2;
; **************************************************
	move	#5,n0
	move	#>255,a
	move	y0,x:(r0+n0)
	move	x:(r0+n0),y0
	tfr	y0,b	y0,x:(r4)
	cmp	a,b
	jle	L196
	move	#>64,a
	jmp	L197
L196
	move	x:(r4),a
	move	a1,a
	asr	a
	asr	a
	move	a1,a
L197
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
	jeq	L203
	move	#>15,a
	sub	x1,a
	jmp	L215
L203
	move	x:FI,a
L215
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
	jeq	L205
	move	b1,b
	rep	#5
	asr	b
	move	#>4096,a
	move	b1,b
	add	a,b
	jmp	L216
L205
	move	#2,n0
	move	x:(r0+n0),b
	move	b1,b
	rep	#5
	asr	b
	move	b1,b
L216
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
	jne	L207
; **************************************************
;     YUP = 5120;
; **************************************************
	move	#>5120,x1
	move	x1,x:(r0)
L207
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
	jeq	L208
; **************************************************
;     YUP = 544;
; **************************************************
	move	#>544,x1
	move	x1,x:(r0)
L208
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
	jeq	L210
	move	#>507904,a
	add	a,b
	move	b1,x:(r0+n0)
L210
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
	jeq	L212
	move	x:(r0+n0),x1
	move	#>8191,x0
	move	#>16384,a
	sub	x1,a
	tfr	a,b	a1,x:(r0+n0)
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
L212
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
	jeq	L214
	move	x:(r0+n0),x1
	move	#>16383,x0
	move	#>16384,a
	sub	x1,a
	tfr	a,b	a1,x:(r0+n0)
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
L214
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
;      
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
	jeq	L218
	cmp	x0,b
	jne	L220
	clr	b
	jmp	L219
L220
	move	#>16383,x0
	move	x:(r1),a
	tfr	a,b	#>1,a
	eor	x0,b
	move	b1,b
	add	a,b	#>49152,a
	and	x0,b
	move	b1,b
	add	a,b
	jmp	L219
L218
	move	x:(r1),b
L219
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
	jeq	L222
	move	x:(r0),a
	move	#>32768,x0
	or	x0,a
	move	a1,a
	jmp	L223
L222
	move	x:FS_E,a
L223
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
	jle	L226
; **************************************************
;     ExitFlag = TRUE;
; **************************************************
	move	#>1,a
	move	a1,x:FExitFlag
L226
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
L235
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
	jne	L233
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
	jeq	L235
L233
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
; #ifdef __DOWNLOAD__
;   run56k_end();
; #endif
; 
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

