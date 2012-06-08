;
;	annotate version 1.03
;
; **************************************************
; **************************************************
	section	adpt_predict_c
	opt so,nomd
	page 132,66,3,3
;*** DSP56000/1 Motorola 1.03 GNU 1.37.1
	org	p:
	global	Ffmult
Ffmult
; **************************************************
; 
; /* Global signals */
; int A_2, AL, D, I, SD, S_E, SL, SP, TDP;
; 
; typedef struct
; {
;   int DQ, T, Y, YL, DQ2, DQ3, DQ4, DQ5, DQ6, DQ7, PK1, PK2, SR2,
;       A1, A2, B[6], SE, SEZ, t_dr, LAST_TR, DMS, DML, AP, Y_L, LAST_Y;
; } STATES;
; 
; 
; int fmult()
; {
; }
; **************************************************
	tst	a
	rts

	global	Fadpt_predict
Fadpt_predict
	move	r0,x:(r6)+
	move	ssh,x:(r6)+
; **************************************************
; **************************************************
; **************************************************
; 
; 
; void adpt_predict(STATES *S)
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
;   DQS = S->DQ >> 14;
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
	move	x:(r0+n0),r2
	move	#19,n0
	move	x:(r2),y0
	tfr	y0,b	y0,x:(r0+n0)
	move	b1,b
	rep	#14
	asr	b
; **************************************************
;   DQI = DQS ? 65536 - (S->DQ & 16383) : S->DQ;
; **************************************************
	move	#6,n0
	move	b1,b
	tst	b	b1,x:(r0+n0)
	jeq	L3
	move	#19,n0
	move	#>65536,b
	move	x:(r0+n0),a
	move	#7,n0
	move	#>16383,x0
	and	x0,a
	move	a1,a
	sub	a,b
	move	b1,x:(r0+n0)
	jmp	L4
L3
	move	#7,n0
	move	x:(r2),y0
	move	y0,x:(r0+n0)
L4
; **************************************************
;   MAG = S->SEZ & (1 << 14) ? (1 << 15) + S->SEZ : S->SEZ;
; **************************************************
	move	#18,n0
	move	#22,n2
	move	#>16384,x0
	move	x:(r2+n2),y0
	tfr	y0,b	y0,x:(r0+n0)
	and	x0,b
	move	b1,b
	tst	b
	jeq	L5
	move	x:(r0+n0),b
	move	#>32768,y0
	add	y0,b
	move	b1,x:(r0+n0)
	jmp	L6
L5
	move	#18,n0
	move	#22,n2
	move	x:(r2+n2),y0
	move	y0,x:(r0+n0)
L6
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
;   WA2 = DQSEZ ? 0 : 1;
; **************************************************
	move	#9,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#8,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L7
	move	#0,y0
	jmp	L54
L7
	move	#>1,y0
L54
; **************************************************
;   
;   /* ADDB */
;   DQI = DQS ? 65536 - (S->DQ & 16383) : S->DQ;
; **************************************************
	move	#12,n0
	move	y0,x:(r0+n0)
	move	#6,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L9
	move	#7,n0
	move	x:(r2),a
	move	#>65536,b
	move	#>16383,x0
	and	x0,a
	move	a1,a
	sub	a,b
	move	b1,x:(r0+n0)
	jmp	L10
L9
	move	#7,n0
	move	x:(r2),y0
	move	y0,x:(r0+n0)
L10
; **************************************************
;   MAG = S->SE & (1 << 14) ? (1 << 15) + S->SE : S->SE;
; **************************************************
	move	#18,n0
	move	#21,n2
	move	#>16384,x0
	move	x:(r2+n2),y0
	tfr	y0,b	y0,x:(r0+n0)
	and	x0,b
	move	b1,b
	tst	b
	jeq	L11
	move	x:(r0+n0),b
	move	#>32768,y0
	add	y0,b
	move	b1,x:(r0+n0)
	jmp	L12
L11
	move	#18,n0
	move	#21,n2
	move	x:(r2+n2),y0
	move	y0,x:(r0+n0)
L12
; **************************************************
;   DQSEZ = (DQI + MAG) & 65535;
; **************************************************
; **************************************************
;   
;   /* FLOATB */
;   MAG = DQSEZ >> 15 ? (65536 - DQSEZ) & 32767 : DQSEZ;
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
	jeq	L13
	move	x:(r0+n0),y0
	move	#18,n0
	move	#>32767,x0
	move	#>65536,b
	sub	y0,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L14
L13
	move	#8,n0
	move	x:(r0+n0),y0
	move	#18,n0
	move	y0,x:(r0+n0)
L14
; **************************************************
; /*  {
;     register int mag = MAG << 1;
; 
;     for (EXP = 0; mag >>= 1; EXP++)
; #ifdef __LOOPCOUNT__ 
;       Loop2++;
; #else
;       ;
; #endif
;   } */
;   MANT = MAG ? (MAG << 6) >> EXP : 1 << 5;
; **************************************************
	move	#18,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L15
	rep	#6
	asl	b
	move	#13,n0
	move	b1,b
	move	x:(r0+n0),a
	tst	a
	jeq	L62
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L62
	move	#14,n0
	move	b1,x:(r0+n0)
	jmp	L16
L15
	move	#14,n0
	move	#>32,y0
	move	y0,x:(r0+n0)
L16
; **************************************************
;   SR1 = ((DQSEZ >> 15) << 10) + (EXP << 6) + MANT;
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
;   WA1 = PK0 ^ S->PK1;                     /* WA1 = PKS1 */
; **************************************************
; **************************************************
;   MAG = PK0 ^ S->PK2 ? 114688 : 16384;    /* MAG = UGA2A */
; **************************************************
	move	#10,n2
	move	#18,n0
	move	b1,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	#19,n0
	move	x:(r2+n2),x0
	move	#11,n2
	move	x:(r0+n0),b
	add	y0,b	#14,n0
	move	b1,y1
	move	x:(r0+n0),y0
	add	y0,b	#9,n0
	move	b1,y1
	move	x:(r0+n0),b
	eor	x0,b	#11,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#9,n0
	move	x:(r2+n2),x0
	move	x:(r0+n0),b
	eor	x0,b
	move	b1,b
	tst	b
	jeq	L17
	move	#>114688,y0
	jmp	L55
L17
	move	#>16384,y0
L55
; **************************************************
;   A1S = S->A1 & (1 << 15) ? 1 : 0;
; **************************************************
	move	#13,n2
	move	#18,n0
	move	#>32768,x0
	move	y0,x:(r0+n0)
	move	x:(r2+n2),a
	and	x0,a
	move	a1,a
	tst	a
	jeq	L19
	move	#>1,y0
	jmp	L56
L19
	move	#0,y0
L56
; **************************************************
;   EXP = A1S ? (S->A1 >= 57345 ? (S->A1 << 2) & 131071 : 24577 << 2) :
;               (S->A1 <= 8191 ? S->A1 << 2 : 8191 << 2); 
; **************************************************
	move	#10,n0
	move	y0,x:(r0+n0)
	move	x:(r0+n0),b
	tst	b
	jeq	L21
	move	#15,n0
	move	#13,n2
	move	x:(r2+n2),y0
	move	y0,x:(r0+n0)
	move	x:(r0+n0),b
	move	#>57344,y0
	cmp	y0,b
	jle	L23
	asl	b	#13,n0
	asl	b	#>131071,x0
	move	b1,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L22
L23
	move	#>98308,y0
	jmp	L57
L21
	move	#13,n2
	move	x:(r2+n2),y0
	tfr	y0,b	#>8191,y0
	cmp	y0,b
	jgt	L25
	asl	b	#13,n0
	asl	b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L22
L25
	move	#>32764,y0
L57
	move	#13,n0
	move	y0,x:(r0+n0)
L22
; **************************************************
;   EXP = WA1 == 1 ? EXP : (131072 - EXP) & 131071;
; **************************************************
	move	#11,n0
	move	#>1,y0
	move	x:(r0+n0),b
	cmp	y0,b
	jeq	L28
	move	#13,n0
	move	#>131071,x0
	move	x:(r0+n0),y0
	move	#>131072,b
	sub	y0,b
	and	x0,b	b1,x:(r0+n0)
	move	b1,b
	move	b1,x:(r0+n0)
L28
; **************************************************
;   MAG = ((MAG + EXP) & 131071) >> 7;
; **************************************************
	move	#18,n0
	move	#>131071,x0
	move	x:(r0+n0),b
	move	#13,n0
	move	x:(r0+n0),y0
	add	y0,b
	and	x0,b
	move	b1,b
	move	b1,b
	rep	#7
	asr	b
; **************************************************
;   EXP = WA2 == 1 ? 0 : (MAG & (1 << 9) ? MAG + 64512 : MAG);
; **************************************************
	move	#18,n0
	move	#>1,y0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#12,n0
	move	x:(r0+n0),b
	cmp	y0,b
	jne	L29
	move	#0,y0
	jmp	L58
L29
	move	#18,n0
	move	#>512,x0
	move	x:(r0+n0),a
	and	x0,a
	move	a1,a
	tst	a
	jeq	L31
	move	x:(r0+n0),b
	move	#13,n0
	move	#>64512,y0
	add	y0,b
	move	b1,x:(r0+n0)
	jmp	L30
L31
	move	#18,n0
	move	x:(r0+n0),y0
L58
	move	#13,n0
	move	y0,x:(r0+n0)
L30
; **************************************************
;   MANT = S->A2 & (1 << 15) ? 65536 - ((S->A2 >> 7) + 65024) :
;                              65536 - (S->A2 >> 7);
; **************************************************
	move	#18,n0
	move	#14,n2
	move	#>32768,x0
	move	x:(r2+n2),y0
	tfr	y0,b	y0,x:(r0+n0)
	and	x0,b
	move	b1,b
	tst	b
	jeq	L33
	move	x:(r0+n0),a
	move	a1,a
	rep	#7
	asr	a
	move	#>512,b
	move	a1,a
	jmp	L59
L33
	move	#14,n2
	move	x:(r2+n2),a
	move	a1,a
	rep	#7
	asr	a
	move	#>65536,b
	move	a1,a
L59
; **************************************************
;   EXP = (S->A2 + EXP + MANT) & 65535;
; **************************************************
; **************************************************
; 
;   /* LIMC */
;   AP = 32768 <= EXP && EXP <= 53248 ? 53248 :
;        12288 <= EXP && EXP <= 32767 ? 12288 : EXP;
; **************************************************
	sub	a,b	#14,n2
	move	#14,n0
	move	#>65535,x0
	move	b1,x:(r0+n0)
	move	#19,n0
	move	x:(r2+n2),y0
	move	y0,x:(r0+n0)
	move	#13,n0
	move	x:(r0+n0),b
	add	y0,b	#14,n0
	move	x:(r0+n0),y0
	add	y0,b	#13,n0
	and	x0,b	#>32767,y0
	move	b1,b
	cmp	y0,b	b1,x:(r0+n0)
	jle	L35
	move	#>53248,x0
	cmp	x0,b
	jgt	L35
	move	x0,x1
	jmp	L36
L35
	move	#13,n0
	move	#>12287,y0
	move	x:(r0+n0),b
	cmp	y0,b
	jle	L37
	move	#>32767,y0
	cmp	y0,b
	jgt	L37
	move	#>12288,x1
	jmp	L36
L37
	move	#13,n0
	move	x:(r0+n0),x1
L36
; **************************************************
;   
;   /* TRIGB */
;   A2R = S->T ? 0 : AP;
; **************************************************
	move	(r2)+
	move	x:(r2)-,a
	tst	a
	jeq	L39
	move	#20,n0
	move	#0,y0
	move	y0,x:(r0+n0)
	jmp	L40
L39
	move	#20,n0
	move	x1,x:(r0+n0)
L40
; **************************************************
;   
;   /* UPA1 */
;   MANT = A1S ? (65536 - ((S->A1 >> 8) + 65280)):
;                (65536 - (S->A1 >> 8));
; **************************************************
	move	#10,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L41
	move	#13,n2
	move	x:(r2+n2),a
	move	a1,a
	rep	#8
	asr	a
	move	#>256,b
	move	a1,a
	jmp	L60
L41
	move	#13,n2
	move	x:(r2+n2),a
	move	a1,a
	rep	#8
	asr	a
	move	#>65536,b
	move	a1,a
L60
; **************************************************
;   EXP = (S->A1 + ((WA2 == 1 ? 0 : (WA1 ? 65344 : 192)) + MANT)) & 65535;
; **************************************************
	sub	a,b	#14,n0
	move	#>1,y0
	move	b1,x:(r0+n0)
	move	x:(r0+n0),x0
	move	#12,n0
	move	x:(r0+n0),b
	cmp	y0,b
	jeq	L43
	move	#11,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L44
	move	#>65344,y0
	jmp	L61
L44
	move	#>192,y0
L61
	tfr	x0,b	#19,n0
	move	y0,x:(r0+n0)
	move	x:(r0+n0),y0
	add	y0,b
	move	b1,x0
L43
; **************************************************
;   
;   /* FMULT */
;   WA2 = f_mult(A2R, S->SR2); 
; **************************************************
	tfr	x0,b	#13,n0
	move	#13,n2
	move	#>65535,x0
	move	x:(r2+n2),y0
	add	y0,b	#12,n2
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	#19,n0
	move	x:(r2+n2),y0
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
;   S->A1 = (AP + 65536 - 15360) & 65535;
; **************************************************
; **************************************************
;   AP = 32768 <= EXP && EXP <=  S->A1 ? S->A1 :
;        MAG <= EXP && EXP <= 32767 ? MAG : EXP;
; **************************************************
	move	#13,n2
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
	move	y0,x:(r2+n2)
	move	x:(r0+n0),b
	move	#>32767,y0
	cmp	y0,b	(r6)-
	jle	L46
	move	#17,n0
	move	x:(r0+n0),y0
	cmp	y0,b
	jgt	L46
	move	x:(r0+n0),x1
	jmp	L47
L46
	move	#18,n0
	move	x:(r0+n0),b
	move	#13,n0
	move	x:(r0+n0),y0
	cmp	y0,b
	jgt	L48
	move	#>32767,y0
	move	x:(r0+n0),b
	cmp	y0,b
	jgt	L48
	move	#18,n0
	move	x:(r0+n0),x1
	jmp	L47
L48
	move	#13,n0
	move	x:(r0+n0),x1
L47
; **************************************************
;   
;   /* TRIGB */
;   S->A1 = S->T ? 0 : AP;
; **************************************************
	move	(r2)+
	move	x:(r2)-,a
	tst	a
	jeq	L50
	move	#0,x0
	jmp	L51
L50
	move	x1,x0
L51
; **************************************************
;   
;   /* FMULT */
;   WA1 = f_mult(S->A1, SR1);
; **************************************************
	move	#13,n2
	move	x0,x:(r2+n2)
	move	y1,x:(r6)+
	move	x0,x:(r6)+
	jsr	Ff_mult
; **************************************************
;   
;   /* FLOATA */
;   MAG = S->DQ & 16383;
; **************************************************
; **************************************************
; /*  {
;     register int mag = MAG << 1;
; 
;     for (EXP = 0; mag >>= 1; EXP++)
; #ifdef __LOOPCOUNT__ 
;       Loop3++;
; #else
;       ;
; #endif
;   } */
;   MANT = MAG ? (MAG << 6) >> EXP : 1 << 5;
; **************************************************
	move	#11,n0
	move	#>16383,x0
	move	a1,x:(r0+n0)
	move	#19,n0
	move	x:(r2),y0
	tfr	y0,b	y0,x:(r0+n0)
	and	x0,b	#18,n0
	move	b1,b
	tst	b	b1,x:(r0+n0)
	move	(r6)-
	move	(r6)-
	jeq	L52
	rep	#6
	asl	b
	move	#13,n0
	move	b1,b
	move	x:(r0+n0),a
	tst	a
	jeq	L63
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L63
	move	#14,n0
	move	b1,x:(r0+n0)
	jmp	L53
L52
	move	#14,n0
	move	#>32,y0
	move	y0,x:(r0+n0)
L53
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
;   BP[0] = DQS ^ (S->DQ2 >> 10);
; **************************************************
	move	#4,n2
	move	#19,n0
	move	b1,b
	move	b1,y0
	move	x:(r0+n0),b
	add	y0,b	#14,n0
	move	x:(r0+n0),y0
	add	y0,b	#8,n0
	move	b1,x:(r0+n0)
	move	x:(r2+n2),y0
	tfr	y0,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[1] = DQS ^ (S->DQ3 >> 10);
; **************************************************
	move	#5,n2
	move	#6,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#19,n0
	move	b1,b
	move	b1,y0
	move	b1,x:(r0+n0)
	move	y0,x:(r0)
	move	x:(r2+n2),y0
	tfr	y0,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[2] = DQS ^ (S->DQ4 >> 10);
; **************************************************
	move	#6,n2
	move	#6,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	(r0)+
	move	b1,b
	move	b1,y0
	move	y0,x:(r0)-
	move	x:(r2+n2),y0
	tfr	y0,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[3] = DQS ^ (S->DQ5 >> 10);
; **************************************************
	move	#7,n2
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#2,n0
	move	b1,b
	move	b1,y0
	move	y0,x:(r0+n0)
	move	x:(r2+n2),y0
	tfr	y0,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[4] = DQS ^ (S->DQ6 >> 10);
; **************************************************
	move	#8,n2
	move	#6,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#3,n0
	move	b1,b
	move	b1,y0
	move	y0,x:(r0+n0)
	move	x:(r2+n2),y0
	tfr	y0,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[5] = DQS ^ (S->DQ7 >> 10);
; **************************************************
	move	#9,n2
	move	#6,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#4,n0
	move	b1,b
	move	b1,y0
	move	y0,x:(r0+n0)
	move	x:(r2+n2),y0
	tfr	y0,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   
;   /* UPB */
;   MANT = S->DQ & 16383;
; **************************************************
; **************************************************
;  /* for (EXP = 0; EXP < 6; EXP++)
;   {
; #ifdef __LOOPCOUNT__ 
;     Loop4++;
; #endif
;     DQS = MANT ? (BP[EXP] ? 65408 : 128) : 0;
;     DQS += S->B[EXP] & (1 << 15) ? (65536 - ((S->B[EXP] >> 8) + 65280)) :
;                                    (65536 - (S->B[EXP] >> 8));
;     BP[EXP] = (S->B[EXP] + DQS) & 65535;
;   }
;   */
;   /* TRIGB */
;  /* if (S->T)
;     for (EXP = 0; EXP < 6; EXP++)
; #ifdef __LOOPCOUNT__ 
;     {
;       Loop5++;
;       BP[EXP] = 0 ;
;     }
; #else
;       BP[EXP] = 0 ;
; #endif
;   */
;   /* FMULT */
;   DQI  = f_mult(BP[0], DQSEZ);
; **************************************************
	move	#6,n0
	move	#Ff_mult,r1
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#5,n0
	move	b1,b
	move	b1,y0
	move	y0,x:(r0+n0)
	move	#8,n0
	move	x:(r0+n0),y0
	move	#19,n0
	move	y0,x:(r6)+
	move	x:(r0+n0),y0
	move	y0,x:(r6)+
	jsr	(r1)
; **************************************************
;   MAG  = f_mult(BP[1], S->DQ2);
; **************************************************
	move	#4,n2
	move	#7,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	x:(r0+n0),b
	move	x:(r2+n2),y0
	move	(r0)+
	move	(r6)-
	move	y0,x:(r6)+
	move	x:(r0)-,y0
	move	y0,x:(r6)+
	jsr	(r1)
; **************************************************
;   MANT = f_mult(BP[2], S->DQ3);
; **************************************************
	move	#5,n2
	move	#18,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	#2,n0
	move	x:(r2+n2),y0
	move	(r6)-
	move	y0,x:(r6)+
	move	x:(r0+n0),y0
	move	y0,x:(r6)+
	jsr	(r1)
; **************************************************
;   A1S  = f_mult(BP[3], S->DQ4);
; **************************************************
	move	#6,n2
	move	#14,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	#3,n0
	move	x:(r2+n2),y0
	move	(r6)-
	move	y0,x:(r6)+
	move	x:(r0+n0),y0
	move	y0,x:(r6)+
	jsr	(r1)
; **************************************************
;   AP   = f_mult(BP[4], S->DQ5);
; **************************************************
	move	#7,n2
	move	#10,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	#4,n0
	move	x:(r2+n2),y0
	move	(r6)-
	move	y0,x:(r6)+
	move	x:(r0+n0),y0
	move	y0,x:(r6)+
	jsr	(r1)
; **************************************************
;   EXP  = f_mult(BP[5], S->DQ6);
; **************************************************
	move	#5,n0
	move	#8,n2
	move	a1,x1
	move	x:(r2+n2),y0
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
;   S->SEZ = DQS >> 1;
; **************************************************
; **************************************************
;   S_E = S->SE = ((DQS + WA2 + WA1) & 65535) >> 1;
; **************************************************
; **************************************************
;   
;   /* STATE update */
;   S->PK2 = S->PK1;
; **************************************************
; **************************************************
;   S->PK1 = PK0;
; **************************************************
	move	#22,n2
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
	move	b1,b
	asr	b
	move	b1,b
	move	b1,y0
	move	y0,x:(r2+n2)
	move	#21,n2
	move	x:(r0+n0),b
	move	#12,n0
	move	x:(r0+n0),y0
	add	y0,b	#11,n0
	move	x:(r0+n0),y0
	add	y0,b	#9,n0
	and	x0,b
	move	b1,b
	move	b1,b
	asr	b
	move	b1,b
	move	b1,y0
	move	y0,x:(r2+n2)
	move	#10,n2
	move	y0,x:FS_E
	move	x:(r2+n2),y0
	move	#11,n2
	move	y0,x:(r2+n2)
	move	x:(r0+n0),y0
; **************************************************
;   S->SR2 = SR1;
; **************************************************
; **************************************************
;   A_2 = S->A2 = A2R;
; **************************************************
; **************************************************
; /*
;   for (EXP = 0; EXP < 6; EXP++)
; #ifdef __LOOPCOUNT__
;   {
;     Loop6++;
;     S->B[EXP] = BP[EXP];
;   }
; #else
;     S->B[EXP] = BP[EXP];
; #endif
; */
;   S->DQ7 = S->DQ6;
; **************************************************
; **************************************************
;   S->DQ6 = S->DQ5;
; **************************************************
; **************************************************
;   S->DQ5 = S->DQ4;
; **************************************************
; **************************************************
;   S->DQ4 = S->DQ3;
; **************************************************
; **************************************************
;   S->DQ3 = S->DQ2;
; **************************************************
; **************************************************
;   S->DQ2 = DQSEZ;
; **************************************************
; **************************************************
; }
; **************************************************
	move	#20,n0
	move	#10,n2
	move	(r6)-
	move	y0,x:(r2+n2)
	move	#12,n2
	move	y1,x:(r2+n2)
	move	#14,n2
	move	x:(r0+n0),y0
	move	#19,n0
	move	y0,x:(r2+n2)
	move	#8,n2
	move	y0,x:FA_2
	move	x:(r2+n2),y0
	move	#9,n2
	move	y0,x:(r2+n2)
	move	#7,n2
	move	x:(r2+n2),y0
	move	#8,n2
	move	y0,x:(r2+n2)
	move	#6,n2
	move	x:(r2+n2),y0
	move	#7,n2
	move	y0,x:(r2+n2)
	move	#5,n2
	move	x:(r2+n2),y0
	move	#6,n2
	move	y0,x:(r2+n2)
	move	#4,n2
	move	x:(r2+n2),y0
	move	#5,n2
	move	y0,x:(r0+n0)
	move	#8,n0
	move	y0,x:(r2+n2)
	move	#4,n2
	move	x:(r0+n0),y0
	move	y0,x:(r2+n2)
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

