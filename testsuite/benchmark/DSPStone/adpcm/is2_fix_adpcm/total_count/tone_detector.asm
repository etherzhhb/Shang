;
;	annotate version 1.03
;
; **************************************************
; **************************************************
	section	tone_detector_c
	opt so,nomd
	page 132,66,3,3
;*** DSP56000/1 Motorola 1.03 GNU 1.37.1
	org	p:
	global	Ftone_detector
Ftone_detector
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
; void tone_detector(STATES* S)
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
	move	#65533,n0
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
	move	x:(r0+n0),r1
	move	x:FA_2,x1
	tfr	x1,b
	cmp	a,b
	jle	L2
	move	#>53760,a
	cmp	a,b
	jgt	L2
	move	#>1,a
	jmp	L3
L2
	clr	a
L3
; **************************************************
;   
;   /* take last (t_dr) */
;   t_d = (S->t_dr);
; **************************************************
; **************************************************
;   
;   /* calcutale new  (S->t_dr)  */
;   /* TRIGB Function */
;   S->t_dr = S->LAST_TR ? 0 : TDP; 
; **************************************************
	move	#23,n1
	move	a1,x:FTDP
	move	x:(r1+n1),x1
	move	#24,n1
	move	(r0)+
	move	x1,x:(r0)-
	move	x:(r1+n1),a
	tst	a
	jeq	L4
	clr	a
	jmp	L5
L4
	move	x:FTDP,a
L5
; **************************************************
;   
;   S->LAST_TR = S->YL >> 15; /* (*LAST_TR)  is used here as a temporary variable */
; **************************************************
	move	#23,n1
	move	a1,x:(r1+n1)
	move	#3,n1
	move	x:(r1+n1),a
	move	a1,a
	rep	#15
	asr	a
; **************************************************
;   
;   tmp = ((S->LAST_TR) > 8) ? 31 << 9 :
;         (32 + ((S->LAST_TR << 5) & 31)) << S->LAST_TR; 
; **************************************************
	move	#24,n1
	move	#>8,b
	move	a1,a
	cmp	b,a	a1,x:(r1+n1)
	jle	L6
	move	#>15872,x1
	move	x1,x:(r0)
	jmp	L7
L6
	move	#24,n1
	move	x:(r1+n1),a
	rep	#5
	asl	a
	move	x:(r1+n1),b
	move	#>32,x1
	move	#>31,x0
	move	a1,a
	and	x0,a
	move	a1,a
	add	x1,a
	tst	b
	jeq	L10
	rep	b
	asl	a
	move	a1,a
L10
	move	a1,x:(r0)
L7
; **************************************************
;   
;   S->LAST_TR = S->T = (((S->DQ & 16383) > (tmp + (tmp >> 1)) >> 1) 
; 	              && (t_d == 1)) ? 1 : 0;
; **************************************************
	move	x:(r0),b
	move	b1,b
	asr	b	#>16383,x0
	move	b1,b
	move	b1,x1
	move	x:(r0),b
	add	x1,b	x:(r1),a
	and	x0,a	b1,b
	asr	b	a1,a
	move	b1,b
	move	b1,x1
	cmp	x1,a
	jle	L8
	move	#>1,a
	move	(r0)+
	move	x:(r0)-,b
	cmp	a,b
	jeq	L9
L8
	clr	a
L9
; **************************************************
; }
; **************************************************
	move	#24,n1
	move	(r0)-
	move	(r0)-
	move	(r6)-
	move	(r1)+
	move	a1,x:(r1)-
	move	a1,x:(r1+n1)
	move	x:(r6)-,r1
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


