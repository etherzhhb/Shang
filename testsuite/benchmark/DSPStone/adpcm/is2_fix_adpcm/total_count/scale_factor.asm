;
;	annotate version 1.03
;
; **************************************************
; **************************************************
	section	scale_factor_c
	opt so,nomd
	page 132,66,3,3
;*** DSP56000/1 Motorola 1.03 GNU 1.37.1
	org	p:
	org	x:
F___W0
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
; /* Global signals */
; int A_2, AL, D, I, SD, S_E, SL, SP, TDP;
; 
; 
; typedef struct
; {
;   int DQ, T, Y, YL, DQ2, DQ3, DQ4, DQ5, DQ6, DQ7, PK1, PK2, SR2,
;       A1, A2, B[6], SE, SEZ, t_dr, LAST_TR, DMS, DML, AP, Y_L, LAST_Y;
; } STATES;
; 
; 
; void scale_factor(STATES* S)
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
	move	#65533,n0
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
	move	x:(r0+n0),r2
	move	#2,n0
	move	x:FI,x1
	tfr	x1,a	x1,x:(r0+n0)
	move	a1,a
	rep	#3
	asr	a
	move	a1,a
	tst	a
	jeq	L2
	move	#>15,a
	sub	x1,a
	jmp	L13
L2
	move	x:FI,a
L13
; **************************************************
;   
;   /* FILTD */
;   TMP = ((TMP << 5) + 131072 - S->LAST_Y) & 131071;
; **************************************************
	move	#F___W0,r1
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
;   TMP >>= 5;
; **************************************************
	move	#29,n2
	move	#>16646144,x0
	move	x:(r2+n2),a
	add	x0,a	b1,b
	sub	a,b	#>131071,x0
	and	x0,b
	move	b1,b
	move	b1,b
	rep	#5
	asr	b
; **************************************************
;   YUP = (TMP >> 11) ? TMP + 4096 : TMP;
; **************************************************
	move	#2,n0
	move	b1,b
	tfr	b,a	b1,x:(r0+n0)
	move	a1,a
	rep	#11
	asr	a
	move	a1,a
	tst	a
	jeq	L4
	move	#>4096,a
	add	a,b
	move	b1,x:(r0)
	jmp	L5
L4
	move	#2,n0
	move	x:(r0+n0),x1
	move	x1,x:(r0)
L5
; **************************************************
;   YUP = (YUP + S->LAST_Y) & 8191;
; **************************************************
; **************************************************
;   
;   /* LIMB */
;   if ((((YUP + 11264) & 16383) >> 13) == 0)
; **************************************************
	move	#29,n2
	move	x:(r0),b
	move	x:(r2+n2),a
	add	a,b	#>8191,x0
	and	x0,b	#>11264,a
	move	b1,b
	move	#>16383,x0
	add	a,b	b1,x:(r0)
	tfr	b,a
	and	x0,a
	move	a1,a
	move	a1,a
	rep	#13
	asr	a
	move	a1,a
	tst	a
	jne	L6
; **************************************************
;     YUP = 5120;
; **************************************************
	move	#>5120,x1
	move	x1,x:(r0)
L6
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
	jeq	L7
; **************************************************
;     YUP = 544;
; **************************************************
	move	#>544,x1
	move	x1,x:(r0)
L7
; **************************************************
;   
;   /* FILTE */
;   TMP = (YUP + ((1048576 - S->Y_L) >> 6)) & 16383;
; **************************************************
	move	#28,n2
	move	#>1048576,a
	move	x:(r2+n2),x1
	sub	x1,a
	move	a1,a
	rep	#6
	asr	a
; **************************************************
;   if (TMP & (1 << 13))
; **************************************************
	move	#2,n0
	move	x:(r0),b
	move	#>16383,x0
	move	a1,a
	add	a,b
	and	x0,b	#>8192,x0
	move	b1,b
	tfr	b,a	b1,x:(r0+n0)
	and	x0,a
	move	a1,a
	tst	a
	jeq	L8
; **************************************************
;     TMP = TMP + 507904;
; **************************************************
	move	#>507904,a
	add	a,b
	move	b1,x:(r0+n0)
L8
; **************************************************
;   YLP = (TMP + S->Y_L) & 524287;
; **************************************************
; **************************************************
;   
;   /* MIX */
;   TMP = (YUP + 16384 - (S->Y_L >> 6)) & 16383;
; **************************************************
	move	#2,n0
	move	#28,n2
	move	x:(r0+n0),b
	move	x:(r2+n2),a
	add	a,b	#>524287,x0
	and	x0,b	a1,a
	move	b1,b
	move	(r0)+
	move	b1,x:(r0)-
	rep	#6
	asr	a
; **************************************************
;   S->LAST_Y = TMP & (1 << 13) ? 1 : 0;
; **************************************************
	move	x:(r0),b
	move	#>16383,x0
	move	#>16760832,x1
	move	a1,a
	add	x1,a
	sub	a,b
	and	x0,b	#>8192,x0
	move	b1,b
	tfr	b,a	b1,x:(r0+n0)
	and	x0,a
	move	a1,a
	tst	a
	jeq	L9
	move	#>1,a
	jmp	L10
L9
	clr	a
L10
; **************************************************
;   if (S->LAST_Y)
; **************************************************
	tst	a	#29,n2
	move	a1,x:(r2+n2)
	jeq	L11
; **************************************************
;     TMP = (16384 - TMP) & 8191; 
; **************************************************
	move	#2,n0
	move	#>8191,x0
	move	x:(r0+n0),x1
	move	#>16384,a
	sub	x1,a
	tfr	a,b	a1,x:(r0+n0)
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
L11
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
;   if (S->LAST_Y)
; **************************************************
	move	#29,n2
	move	b1,b
	move	b1,x:(r0+n0)
	move	x:(r2+n2),a
	tst	a
	jeq	L12
; **************************************************
;     TMP = (16384 - TMP) & 16383;
; **************************************************
	move	x:(r0+n0),x1
	move	#>16383,x0
	move	#>16384,a
	sub	x1,a
	tfr	a,b	a1,x:(r0+n0)
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
L12
; **************************************************
;   
;   S->LAST_Y = S->Y = ((S->Y_L >> 6) + TMP) & 8191; 
; **************************************************
	move	#28,n2
	move	x:(r2+n2),a
	move	a1,a
	rep	#6
	asr	a
; **************************************************
;   S->YL = S->Y_L; 
; **************************************************
; **************************************************
;   S->Y_L = YLP;
; **************************************************
; **************************************************
; }
; **************************************************
	move	#2,n2
	move	#2,n0
	move	#>8191,x0
	move	x:(r0+n0),x1
	move	a1,a
	add	x1,a	(r0)+
	and	x0,a	(r6)-
	move	a1,a
	move	a1,x:(r2+n2)
	move	#29,n2
	move	a1,x:(r2+n2)
	move	#28,n2
	move	x:(r2+n2),a
	move	#3,n2
	move	a1,x:(r2+n2)
	move	#28,n2
	move	x:(r0)-,x1
	move	x1,x:(r2+n2)
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

	endsec


