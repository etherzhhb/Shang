;
;	annotate version 1.03
;
; **************************************************
; **************************************************
	section	iadpt_quant_c
	opt so,nomd
	page 132,66,3,3
;*** DSP56000/1 Motorola 1.03 GNU 1.37.1
	org	p:
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
; void iadpt_quant(STATES* S)
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
;   DQL = (qtab[I] + (S->Y >> 2)) & 4095;
; **************************************************
; **************************************************
; 
;   /* ANTILOG */
;   S->DQ = ((I & (1 << 3)) * (1 << 11)) +
;           (DQL & (1 << 11) ? 0 :
;           (((1 << 7) + (DQL & 127)) << 7) >>
;           (14 - ((DQL >> 7) & 15)));
; **************************************************
	move	#2,n2
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
	move	#F___qtab0,r1
	move	r2,x:(r6)+
	move	x:(r0+n0),r2
	move	r1,b
	move	x:(r2+n2),x1
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
	jne	L2
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
	jeq	L3
	move	a1,a
	rep	b
	asr	a
	move	a1,a
L3
	move	(r0)+
	move	x:(r0)-,b
	add	a,b	(r0)+
	move	b1,x:(r0)-
L2
; **************************************************
; }
; **************************************************
	move	(r6)-
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

; 

