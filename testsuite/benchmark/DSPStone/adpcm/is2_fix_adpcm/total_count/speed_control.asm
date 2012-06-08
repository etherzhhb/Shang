;
;	annotate version 1.03
;
; **************************************************
; **************************************************
	section	speed_control_c
	opt so,nomd
	page 132,66,3,3
;*** DSP56000/1 Motorola 1.03 GNU 1.37.1
	org	p:
	org	x:
F___F0
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
; void speed_control(STATES* S)
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
	move	y0,x:(r6)+
	move	r1,x:(r6)+
	move	r2,x:(r6)+
	move	x:(r0+n0),r2
	move	#2,n0
	move	x:FI,y0
	tfr	y0,a	y0,x:(r0+n0)
	move	a1,a
	rep	#3
	asr	a
	move	a1,a
	tst	a
	jeq	L2
	move	#>15,a
	sub	y0,a
	jmp	L3
L2
	move	x:FI,a
L3
; **************************************************
; 
;   /* FILTA */
;   tmp = ((FI << 9) + 8192 - S->DMS) & 8191;              /* tmp => DIF */
; **************************************************
	move	#F___F0,r1
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
;   tmp >>= 5;
; **************************************************
	move	#25,n2
	move	#>16769024,x0
	move	x:(r2+n2),a
	add	x0,a	b1,b
	sub	a,b	#>8191,x0
	and	x0,b
	move	b1,b
	move	b1,b
	rep	#5
	asr	b
; **************************************************
;   if (tmp & (1 << 7))
; **************************************************
	move	#>128,x0
	move	b1,b
	tfr	b,a	(r0)+
	and	x0,a	b1,x:(r0)-
	move	a1,a
	tst	a
	jeq	L4
; **************************************************
;     tmp = tmp + 3840;                                    /* tmp => DIFSX */
; **************************************************
	move	#>3840,a
	add	a,b	(r0)+
	move	b1,x:(r0)-
L4
; **************************************************
;   S->DMS = (tmp + S->DMS) & 4095;
; **************************************************
; **************************************************
; 
;   /* FILTB */
;   tmp = ((FI << 11) + 32768 - S->DML) & 32767;           /* tmp => DIF */
; **************************************************
	move	#25,n2
	move	#>4095,x0
	move	x:(r2+n2),a
	move	(r0)+
	move	x:(r0)-,b
	add	a,b
	tfr	b,a
	and	x0,a
	move	a1,a
	move	a1,x:(r2+n2)
	move	x:(r0),b
	rep	#11
	asl	b
; **************************************************
;   tmp >>= 7;
; **************************************************
	move	#26,n2
	move	#>16744448,x0
	move	x:(r2+n2),a
	add	x0,a	b1,b
	sub	a,b	#>32767,x0
	and	x0,b
	move	b1,b
	move	b1,b
	rep	#7
	asr	b
; **************************************************
;   if (tmp & (1 << 7))
; **************************************************
	move	#>128,x0
	move	b1,b
	tfr	b,a	(r0)+
	and	x0,a	b1,x:(r0)-
	move	a1,a
	tst	a
	jeq	L5
; **************************************************
;     tmp = tmp + 16128;                                   /* tmp => DIFSX */
; **************************************************
	move	#>16128,a
	add	a,b	(r0)+
	move	b1,x:(r0)-
L5
; **************************************************
;   S->DML = (tmp + S->DML) & 16383;
; **************************************************
; **************************************************
; 
;   /* SUBTC */
;   tmp = ((S->DMS << 2) + 32768 - S->DML) & 32767;        /* tmp => DIF */
; **************************************************
; **************************************************
;   if (tmp & (1 << 14))
; **************************************************
	move	#2,n0
	move	#26,n2
	move	#>16744448,x0
	move	x:(r2+n2),a
	move	#>16383,x1
	move	(r0)+
	move	x:(r0)-,b
	add	a,b
	tfr	b,a
	and	x1,a
	move	a1,a
	add	x0,a	a1,x:(r2+n2)
	move	#25,n2
	move	#>32767,x0
	move	x:(r2+n2),y0
	tfr	y0,b	y0,x:(r0+n0)
	asl	b	(r0)+
	asl	b
	move	b1,b
	sub	a,b
	and	x0,b	#>16384,x0
	move	b1,b
	tfr	b,a	b1,x:(r0)-
	and	x0,a
	move	a1,a
	tst	a
	jeq	L6
; **************************************************
;     tmp = (32768 - tmp) & 16383;                         /* tmp => DIFM */
; **************************************************
	move	#>32768,a
	move	(r0)+
	move	x:(r0)-,y0
	sub	y0,a	(r0)+
	tfr	a,b	a1,x:(r0)-
	and	x1,b	(r0)+
	move	b1,b
	move	b1,x:(r0)-
L6
; **************************************************
;   FI = ((S->Y > 1535) && (tmp < (S->DML >> 3)) && (TDP == 0)) ? 0 : 1;
; **************************************************
	move	#2,n2
	move	#>1535,b
	move	x:(r2+n2),a
	cmp	b,a
	jle	L7
	move	#26,n2
	move	x:(r2+n2),a
	move	a1,a
	rep	#3
	asr	a
	move	a1,a
	move	(r0)+
	move	x:(r0)-,b
	cmp	a,b
	jge	L7
	move	x:FTDP,a
	tst	a
	jne	L7
	move	#0,y0
	jmp	L14
L7
	move	#>1,y0
L14
; **************************************************
; 
;   /* FILTC */
;   tmp = ((FI << 9) + 2048 - S->AP) & 2047;               /* tmp => DIF */ 
; **************************************************
	move	y0,x:(r0)
	move	x:(r0),b
	rep	#9
	asl	b
; **************************************************
;   tmp = (tmp >> 4) + (tmp >> 10 ? 896 : 0);              /* tmp => DIFSX */
; **************************************************
	move	#27,n2
	move	#>16775168,x0
	move	x:(r2+n2),a
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
	jeq	L9
	move	#>896,a
	add	a,b
	move	b1,x:(r0+n0)
L9
; **************************************************
;   tmp = (tmp + S->AP) & 1023;                            /* tmp => APP */
; **************************************************
; **************************************************
; 
;   /* TRIGA */
;   S->AP = S->T ? 256 : tmp; 
; **************************************************
	move	#27,n2
	move	#2,n0
	move	x:(r2+n2),a
	move	x:(r0+n0),y0
	tfr	y0,b	#>1023,x0
	add	a,b	(r0)+
	and	x0,b	(r2)+
	move	b1,b
	move	b1,x:(r0)-
	move	x:(r2)-,a
	tst	a
	jeq	L10
	move	#>256,y0
	jmp	L15
L10
	move	(r0)+
	move	x:(r0)-,y0
L15
; **************************************************
; 
;   /* LIMA */
;   AL = (S->AP > 255) ? 64 : S->AP >> 2; 
; **************************************************
	move	#27,n2
	move	#2,n0
	move	#>255,a
	move	y0,x:(r0+n0)
	move	x:(r0+n0),y0
	tfr	y0,b	y0,x:(r2+n2)
	cmp	a,b
	jle	L12
	move	#>64,a
	jmp	L13
L12
	move	#27,n2
	move	x:(r2+n2),a
	move	a1,a
	asr	a
	asr	a
	move	a1,a
L13
; **************************************************
; }
; **************************************************
	move	(r0)-
	move	(r0)-
	move	(r6)-
	move	a1,x:FAL
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


