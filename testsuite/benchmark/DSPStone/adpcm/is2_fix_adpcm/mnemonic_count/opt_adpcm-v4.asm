;
;	annotate version 1.03
;
; **************************************************
; **************************************************
	section	opt_adpcm_v4_c
	opt so,nomd
	page 132,66,3,3
;*** DSP56000/1 Motorola 1.03 GNU 1.37.1
	org	p:
	global	FE_STATES
	org	x:
FE_STATES
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	global	FD_STATES
FD_STATES
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	dc 0
	org	p:
	global	Ff_mult
Ff_mult
; **************************************************
; /*
;  * opt_adpcm.c
;  * Handwritten version of an optimized ADPCM transcoder applying the
;  * CCITT recommendation G.721
;  *
;  * For our purposes the algorithem has been retimed (a few delays have been
;  * moved around). But apart from that the algorithem matches the CCITT's.
;  * So for further documentation of the source refer to the recommendation.
;  *
;  * Compile flags:
;  * __SIMULATION__ : use to get a file for run56sim
;  * __DOWNLOAD__:    use to get a file for the S56X board
;  * __MNECOUNT__:    use to count mnemonics
;  *
;  * 3-NOV-93 Juan Martinez and Chris Schlaeger
;  * 
;  * VERSION 1.4
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
; /* global signals */
; unsigned int A_2, AL, D, I, SD, S_E, SL, SP, TDP;
; 
; typedef struct
; {
;   int DQ, T, Y, YL, DQ2, DQ3, DQ4, DQ5, DQ6, DQ7, PK1, PK2, SR2,
;       A1, A2, SE, SEZ, t_dr, LAST_TR, DMS, DML, AP, Y_L, LAST_Y;
;   unsigned int B[6];
; } STATES;
; 
; /* ENCODER states */
; STATES E_STATES = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
; 
; /* DECODER states */
; STATES D_STATES = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
; 
; #define LSHIFT(a, b)  ((b) < 0 ? (a) << -(b) : (a) >> (b))
; #define SIGNBIT(a, b) ((a) & (1 << (b)) ? 1 : 0)
; #define MSB(a, b)     { register unsigned int tmp = (a); \
;                         (b) = 0; while(tmp) {tmp >>= 1; (b)++; }}
;  
; unsigned int f_mult(unsigned int An, unsigned int SRn)
; {
;   register int  EXP;
;   register unsigned int WAnMANT;
;   int                   AnS, AnMANT;
;   unsigned int          MAG;
; 
;   AnS = SIGNBIT(An, 15);
; **************************************************
	move	#65533,n0
	move	r0,x:(r6)+
	move	(r6)+
	move	r6,r0
	move	(r6)+
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	#>32768,x0
	move	x1,x:(r6)+
	move	y0,x:(r6)+
	move	y1,x:(r6)+
	move	x:(r0+n0),a
	and	x0,a
	move	a1,a
	move	#0,a2
	tst	a
	jeq	L2
	move	#>1,y1
	jmp	L3
L2
	move	#0,y1
L3
; **************************************************
;   MAG = AnS ? (16384 - (An >> 2)) & 8191 : An >> 2;
; **************************************************
	tfr	y1,b
	tst	b
	jeq	L4
	move	#65533,n0
	move	#>8191,x0
	move	x:(r0+n0),a
	lsr	a	#>16384,b
	lsr	a
	sub	a,b
	and	x0,b
	move	b1,b
	jmp	L18
L4
	move	#65533,n0
	move	x:(r0+n0),b
	lsr	b
	lsr	b
L18
; **************************************************
;   MSB(MAG, EXP);
; **************************************************
	move	#0,x1
	move	b1,x:(r0)
	move	x:(r0),a
	move	#0,a2
	tst	a
	jeq	L17
	move	#>1,x0
L8
	lsr	a	#0,a2
	tfr	x1,b
	add	x0,b
	tst	a	b1,x1
	jne	L8
L17
; **************************************************
;   AnMANT = MAG ? LSHIFT(MAG, EXP - 6) : 1 << 5;
; **************************************************
	move	x:(r0),b
	move	#0,b2
	tst	b
	jeq	L9
	tfr	x1,b	#>16777210,a
	add	a,b
	tfr	b,a
	tst	a
	jge	L11
	neg	a	x:(r0),b
	tst	a
	jeq	L20
	rep	a
	lsl	b
L20
	move	b1,x:(r0)
	jmp	L10
L11
	tfr	x1,b	#>16777210,a
	add	a,b
	tfr	b,a	x:(r0),b
	tst	a
	jeq	L21
	rep	a
	lsr	b
L21
	move	b1,x:(r0)
	jmp	L10
L9
	move	#>32,y0
	move	y0,x:(r0)
L10
; **************************************************
;   EXP += ((SRn >> 6) & 15);
; **************************************************
	move	#65532,n0
	move	x:(r0+n0),a
	rep	#6
	lsr	a
; **************************************************
;   WAnMANT = (((SRn & 63) * AnMANT) + 48) >> 4;
; **************************************************
	tfr	x1,b	x:(r0),y0
	move	#>15,x0
	and	x0,a	#>63,x0
	move	a1,a
	add	a,b	#>48,a
	move	a,a0
	asl	a	b1,x1
	move	x:(r0+n0),b
	and	x0,b
	move	b1,b
	move	b1,x0
	mac	+x0,y0,a
	asr	a
	move	a0,a
	tfr	a,b
	rep	#4
	lsr	b
; **************************************************
;   MAG = LSHIFT(WAnMANT, 19 - EXP) & 32767;
; **************************************************
	move	#>19,a
	sub	x1,a	b1,x:(r0)
	tst	a
	jge	L13
	neg	a
	tst	a
	jeq	L22
	rep	a
	lsl	b
L22
	jmp	L19
L13
	move	x:(r0),b
	move	#>19,a
	sub	x1,a
	tst	a
	jeq	L23
	rep	a
	lsr	b
L23
L19
; **************************************************
;   return ((SRn >> 10) ^ AnS ? (65536 - MAG) & 65535 : MAG);
; **************************************************
	move	#65532,n0
	move	#>32767,x0
	move	b1,x:(r0)
	move	x:(r0),b
	and	x0,b
	move	b1,b
	move	b1,x:(r0)
	move	x:(r0+n0),a
	rep	#10
	lsr	a
	eor	y1,a
	move	a1,a
	move	#0,a2
	tst	a
	jeq	L15
	move	#>65535,x0
	move	x:(r0),y0
	move	#>65536,a
	sub	y0,a
	and	x0,a
	move	a1,a
	jmp	L16
L15
	move	x:(r0),a
L16
; **************************************************
; }
; **************************************************
	tst	a	(r6)-
	move	x:(r6)-,y1
	move	x:(r6)-,y0
	move	x:(r6)-,x1
	move	x:(r6)-,x0
	move	x:(r6)-,b
	move	x:(r6),b0
	move	(r6)-
	move	(r6)-
	move	(r6)-
	move	x:(r6),r0
	rts

	global	Fadpt_predict
Fadpt_predict
	move	r0,x:(r6)+
	move	ssh,x:(r6)+
; **************************************************
; **************************************************
; **************************************************
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
;   register int EXP;
;   register unsigned int MAG;
;   unsigned int DQS, DQI, DQSEZ, PK0; 
;   int SR1;
;   int A1S, AP;
;   int A2R;
;   int WA1, WA2;
;   int MANT; 
;   int BP[6];
; 
;   /* ADDC */
;   DQS = SIGNBIT(S->DQ, 14);
; **************************************************
	move	#65533,n0
	move	#21,n6
	move	r6,r0
	move	(r6)+n6
	move	b0,x:(r6)+
	move	b1,x:(r6)+
	move	x0,x:(r6)+
	move	#>16384,x0
	move	x1,x:(r6)+
	move	y0,x:(r6)+
	move	y1,x:(r6)+
	move	r1,x:(r6)+
	move	r2,x:(r6)+
	move	r3,x:(r6)+
	move	r4,x:(r6)+
	move	x:(r0+n0),r4
	move	x:(r4),a
	and	x0,a
	move	a1,a
	tst	a
	jeq	L25
	move	#>1,y1
	jmp	L115
L25
	move	#0,y1
L115
; **************************************************
;   DQI = DQS ? 65536 - (S->DQ & 16383) : S->DQ;
; **************************************************
	move	#7,n0
	move	y1,x:(r0+n0)
	move	x:(r0+n0),b
	move	#0,b2
	tst	b
	jeq	L27
	move	#8,n0
	move	x:(r4),a
	move	#>65536,b
	move	#>16383,x0
	and	x0,a
	move	a1,a
	sub	a,b
	move	b1,x:(r0+n0)
	jmp	L28
L27
	move	#8,n0
	move	x:(r4),y1
	move	y1,x:(r0+n0)
L28
; **************************************************
;   MAG = S->SEZ & (1 << 14) ? (1 << 15) + S->SEZ : S->SEZ;
; **************************************************
	move	#19,n0
	move	#16,n4
	move	#>16384,x0
	move	x:(r4+n4),y1
	tfr	y1,b	y1,x:(r0+n0)
	and	x0,b
	move	b1,b
	tst	b
	jeq	L29
	move	x:(r0+n0),b
	move	#>32768,y1
	add	y1,b
	move	b1,x:(r0+n0)
	jmp	L30
L29
	move	#19,n0
	move	#16,n4
	move	x:(r4+n4),y1
	move	y1,x:(r0+n0)
L30
; **************************************************
;   DQSEZ = (DQI + MAG) & 65535;
; **************************************************
; **************************************************
;   PK0 = DQSEZ >> 15;
; **************************************************
	move	#8,n0
	move	#>65535,x0
	move	x:(r0+n0),b
	move	#19,n0
	move	x:(r0+n0),y1
	add	y1,b	#9,n0
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	rep	#15
	lsr	b
; **************************************************
;   WA2 = DQSEZ ? 0 : 1;
; **************************************************
	move	#10,n0
	move	b1,x:(r0+n0)
	move	#9,n0
	move	x:(r0+n0),b
	move	#0,b2
	tst	b
	jeq	L31
	move	#0,y1
	jmp	L116
L31
	move	#>1,y1
L116
; **************************************************
;   
;   /* ADDB */
;   DQI = DQS ? 65536 - (S->DQ & 16383) : S->DQ;
; **************************************************
	move	#14,n0
	move	y1,x:(r0+n0)
	move	#7,n0
	move	x:(r0+n0),b
	move	#0,b2
	tst	b
	jeq	L33
	move	#8,n0
	move	x:(r4),a
	move	#>65536,b
	move	#>16383,x0
	and	x0,a
	move	a1,a
	sub	a,b
	move	b1,x:(r0+n0)
	jmp	L34
L33
	move	#8,n0
	move	x:(r4),y1
	move	y1,x:(r0+n0)
L34
; **************************************************
;   MAG = S->SE & (1 << 14) ? (1 << 15) + S->SE : S->SE;
; **************************************************
	move	#19,n0
	move	#15,n4
	move	#>16384,x0
	move	x:(r4+n4),y1
	tfr	y1,b	y1,x:(r0+n0)
	and	x0,b
	move	b1,b
	tst	b
	jeq	L35
	move	x:(r0+n0),b
	move	#>32768,y1
	add	y1,b
	move	b1,x:(r0+n0)
	jmp	L36
L35
	move	#19,n0
	move	#15,n4
	move	x:(r4+n4),y1
	move	y1,x:(r0+n0)
L36
; **************************************************
;   DQSEZ = (DQI + MAG) & 65535;
; **************************************************
; **************************************************
;   
;   /* FLOATB */
;   MAG = SIGNBIT(DQSEZ, 15) ? (65536 - DQSEZ) & 32767 : DQSEZ;
; **************************************************
	move	#8,n0
	move	#>65535,x0
	move	x:(r0+n0),b
	move	#19,n0
	move	x:(r0+n0),y1
	add	y1,b	#9,n0
	and	x0,b	#>32768,x0
	move	b1,b
	and	x0,b	b1,x:(r0+n0)
	move	b1,b
	move	#0,b2
	tst	b
	jeq	L37
	move	x:(r0+n0),y1
	move	#19,n0
	move	#>32767,x0
	move	#>65536,b
	sub	y1,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L38
L37
	move	#9,n0
	move	x:(r0+n0),y1
	move	#19,n0
	move	y1,x:(r0+n0)
L38
; **************************************************
;   MSB(MAG, EXP);
; **************************************************
	move	#19,n0
	move	x:(r0+n0),y1
	move	#20,n0
	move	y1,x:(r0+n0)
	move	#6,n0
	move	#0,y1
	move	y1,x:(r0+n0)
	move	#19,n0
	move	x:(r0+n0),b
	move	#0,b2
	tst	b
	jeq	L114
	move	#>1,x0
L43
	move	#20,n0
	move	x:(r0+n0),b
	lsr	b
	move	b1,x:(r0+n0)
	move	#6,n0
	move	x:(r0+n0),b
	add	x0,b
	move	b1,x:(r0+n0)
	move	#20,n0
	move	x:(r0+n0),b
	move	#0,b2
	tst	b
	jne	L43
L114
; **************************************************
;   MANT = MAG ? (MAG << 6) >> EXP : 1 << 5;
; **************************************************
	move	#19,n0
	move	x:(r0+n0),b
	move	#0,b2
	tst	b
	jeq	L44
	rep	#6
	lsl	b
	move	#6,n0
	move	x:(r0+n0),a
	tst	a
	jeq	L127
	rep	a
	lsr	b
L127
	move	#15,n0
	move	b1,x:(r0+n0)
	jmp	L45
L44
	move	#15,n0
	move	#>32,y1
	move	y1,x:(r0+n0)
L45
; **************************************************
;   SR1 = ((DQSEZ >> 15) << 10) + (EXP << 6) + MANT;
; **************************************************
	move	#9,n0
	move	x:(r0+n0),b
	rep	#15
	lsr	b
	rep	#10
	lsl	b
	move	#20,n0
	move	b1,x:(r0+n0)
	move	#6,n0
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
	move	#10,n4
	move	#19,n0
	move	b1,b
	move	b1,y1
	move	b1,x:(r0+n0)
	move	#20,n0
	move	x:(r0+n0),b
	add	y1,b	#15,n0
	move	x:(r0+n0),y1
	add	y1,b	#11,n0
	move	b1,x:(r0+n0)
	move	#10,n0
	move	x:(r4+n4),x0
	move	#11,n4
	move	x:(r0+n0),b
	eor	x0,b	#13,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#10,n0
	move	x:(r4+n4),x0
	move	x:(r0+n0),b
	eor	x0,b
	move	b1,b
	move	#0,b2
	tst	b
	jeq	L46
	move	#>114688,y1
	jmp	L117
L46
	move	#>16384,y1
L117
; **************************************************
;   A1S = SIGNBIT(S->A1, 15);
; **************************************************
	move	#13,n4
	move	#19,n0
	move	#>32768,x0
	move	y1,x:(r0+n0)
	move	x:(r4+n4),a
	and	x0,a
	move	a1,a
	tst	a
	jeq	L48
	move	#>1,y1
	jmp	L118
L48
	move	#0,y1
L118
; **************************************************
;   EXP = A1S ? (S->A1 >= 57345 ? (S->A1 << 2) & 131071 : 24577 << 2) :
;               (S->A1 <= 8191 ? S->A1 << 2 : 8191 << 2); 
; **************************************************
	move	#12,n0
	move	y1,x:(r0+n0)
	move	x:(r0+n0),b
	tst	b
	jeq	L50
	move	#16,n0
	move	#13,n4
	move	x:(r4+n4),y1
	move	y1,x:(r0+n0)
	move	x:(r0+n0),b
	move	#>57344,y1
	cmp	y1,b
	jle	L52
	asl	b	#6,n0
	asl	b	#>131071,x0
	move	b1,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L51
L52
	move	#>98308,y1
	jmp	L119
L50
	move	#13,n4
	move	x:(r4+n4),y1
	tfr	y1,b	#>8191,y1
	cmp	y1,b
	jgt	L54
	asl	b	#6,n0
	asl	b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L51
L54
	move	#>32764,y1
L119
	move	#6,n0
	move	y1,x:(r0+n0)
L51
; **************************************************
;   EXP = WA1 == 1 ? EXP : (131072 - EXP) & 131071;
; **************************************************
	move	#13,n0
	move	#>1,y1
	move	x:(r0+n0),b
	cmp	y1,b
	jeq	L57
	move	#6,n0
	move	#>131071,x0
	move	x:(r0+n0),y1
	move	#>131072,b
	sub	y1,b
	and	x0,b	b1,x:(r0+n0)
	move	b1,b
	move	b1,x:(r0+n0)
L57
; **************************************************
;   MAG = ((MAG + EXP) & 131071) >> 7;
; **************************************************
	move	#19,n0
	move	#>131071,x0
	move	x:(r0+n0),b
	move	#6,n0
	move	x:(r0+n0),y1
	add	y1,b
	and	x0,b
	move	b1,b
	rep	#7
	lsr	b
; **************************************************
;   EXP = WA2 == 1 ? 0 : (MAG & (1 << 9) ? MAG + 64512 : MAG);
; **************************************************
	move	#19,n0
	move	#>1,y1
	move	b1,x:(r0+n0)
	move	#14,n0
	move	x:(r0+n0),b
	cmp	y1,b
	jne	L58
	move	#0,y1
	jmp	L120
L58
	move	#19,n0
	move	#>512,x0
	move	x:(r0+n0),a
	and	x0,a
	move	a1,a
	move	#0,a2
	tst	a
	jeq	L60
	move	x:(r0+n0),b
	move	#6,n0
	move	#>64512,y1
	add	y1,b
	move	b1,x:(r0+n0)
	jmp	L59
L60
	move	#19,n0
	move	x:(r0+n0),y1
L120
	move	#6,n0
	move	y1,x:(r0+n0)
L59
; **************************************************
;   MANT = SIGNBIT(S->A2, 15) ? 65536 - ((S->A2 >> 7) + 65024) :
;                              65536 - (S->A2 >> 7);
; **************************************************
	move	#19,n0
	move	#14,n4
	move	#>32768,x0
	move	x:(r4+n4),y1
	tfr	y1,b	y1,x:(r0+n0)
	and	x0,b
	move	b1,b
	tst	b
	jeq	L62
	move	x:(r0+n0),a
	move	a1,a
	rep	#7
	asr	a
	move	#>512,b
	move	a1,a
	jmp	L121
L62
	move	#14,n4
	move	x:(r4+n4),a
	move	a1,a
	rep	#7
	asr	a
	move	#>65536,b
	move	a1,a
L121
; **************************************************
;   EXP = (S->A2 + EXP + MANT) & 65535;
; **************************************************
; **************************************************
; 
;   /* LIMC */
;   AP = 32768 <= EXP && EXP <= 53248 ? 53248 :
;        12288 <= EXP && EXP <= 32767 ? 12288 : EXP;
; **************************************************
	sub	a,b	#14,n4
	move	#15,n0
	move	#>65535,x0
	move	b1,x:(r0+n0)
	move	#20,n0
	move	x:(r4+n4),y1
	move	y1,x:(r0+n0)
	move	#6,n0
	move	x:(r0+n0),b
	add	y1,b	#15,n0
	move	x:(r0+n0),y1
	add	y1,b	#6,n0
	and	x0,b	#>32767,y1
	move	b1,b
	cmp	y1,b	b1,x:(r0+n0)
	jle	L66
	move	#>53248,x0
	cmp	x0,b
	jgt	L66
	move	x0,x1
	jmp	L67
L66
	move	#6,n0
	move	#>12287,y1
	move	x:(r0+n0),b
	cmp	y1,b
	jle	L68
	move	#>32767,y1
	cmp	y1,b
	jgt	L68
	move	#>12288,x1
	jmp	L67
L68
	move	#6,n0
	move	x:(r0+n0),x1
L67
; **************************************************
;   
;   /* TRIGB */
;   A2R = S->T ? 0 : AP;
; **************************************************
	move	(r4)+
	move	x:(r4)-,a
	tst	a
	jeq	L70
	move	#0,y0
	jmp	L71
L70
	move	x1,y0
L71
; **************************************************
;   
;   /* UPA1 */
;   MANT = A1S ? (65536 - ((S->A1 >> 8) + 65280)):
;                (65536 - (S->A1 >> 8));
; **************************************************
	move	#12,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L72
	move	#13,n4
	move	x:(r4+n4),a
	move	a1,a
	rep	#8
	asr	a
	move	#>256,b
	move	a1,a
	jmp	L122
L72
	move	#13,n4
	move	x:(r4+n4),a
	move	a1,a
	rep	#8
	asr	a
	move	#>65536,b
	move	a1,a
L122
; **************************************************
;   EXP = (S->A1 + ((WA2 == 1 ? 0 : (WA1 ? 65344 : 192)) + MANT)) & 65535;
; **************************************************
	sub	a,b	#15,n0
	move	#>1,y1
	move	b1,x:(r0+n0)
	move	x:(r0+n0),x0
	move	#14,n0
	move	x:(r0+n0),b
	cmp	y1,b
	jeq	L74
	move	#13,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L75
	move	#>65344,y1
	jmp	L123
L75
	move	#>192,y1
L123
	tfr	x0,b	#20,n0
	move	y1,x:(r0+n0)
	move	x:(r0+n0),y1
	add	y1,b
	move	b1,x0
L74
; **************************************************
;   
;   /* FMULT */
;   WA2 = f_mult(A2R, S->SR2); 
; **************************************************
	tfr	x0,b	#6,n0
	move	#13,n4
	move	#>65535,x0
	move	x:(r4+n4),y1
	add	y1,b	#12,n4
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	#20,n0
	move	x:(r4+n4),y1
	move	y1,x:(r0+n0)
	move	y1,x:(r6)+
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
	move	#13,n4
	move	#14,n0
	move	#>50176,y1
	move	#>80896,b
	sub	x1,b	a1,x:(r0+n0)
	and	x0,b	#19,n0
	move	b1,b
	tfr	x1,b	b1,x:(r0+n0)
	add	y1,b	#18,n0
	and	x0,b	(r6)-
	move	b1,b
	move	b1,y1
	move	b1,x:(r0+n0)
	move	#6,n0
	move	y1,x:(r4+n4)
	move	x:(r0+n0),b
	move	#>32767,y1
	cmp	y1,b	(r6)-
	jle	L77
	move	#18,n0
	move	x:(r0+n0),y1
	cmp	y1,b
	jgt	L77
	move	x:(r0+n0),x1
	jmp	L78
L77
	move	#19,n0
	move	x:(r0+n0),b
	move	#6,n0
	move	#0,b2
	move	x:(r0+n0),a
	move	#0,a2
	cmp	a,b
	jgt	L79
	move	#>32767,y1
	cmp	y1,a
	jgt	L79
	move	#19,n0
	move	x:(r0+n0),x1
	jmp	L78
L79
	move	#6,n0
	move	x:(r0+n0),x1
L78
; **************************************************
;   
;   /* TRIGB */
;   S->A1 = S->T ? 0 : AP;
; **************************************************
	move	(r4)+
	move	x:(r4)-,a
	tst	a
	jeq	L81
	move	#0,x0
	jmp	L82
L81
	move	x1,x0
L82
; **************************************************
;   
;   /* FMULT */
;   WA1 = f_mult(S->A1, SR1);
; **************************************************
	move	#11,n0
	move	#13,n4
	move	x0,x:(r4+n4)
	move	x:(r0+n0),y1
	move	y1,x:(r6)+
	move	x0,x:(r6)+
	jsr	Ff_mult
; **************************************************
;   
;   /* FLOATA */
;   MAG = S->DQ & 16383;
; **************************************************
; **************************************************
;   MSB(MAG, EXP);
; **************************************************
	move	#13,n0
	move	#>16383,x0
	move	a1,x:(r0+n0)
	move	#19,n0
	move	x:(r4),y1
	tfr	y1,b	(r6)-
	and	x0,b	(r6)-
	move	b1,b
	move	b1,y1
	move	#0,b2
	tst	b	b1,x:(r0+n0)
	move	#20,n0
	move	y1,x:(r0+n0)
	move	#6,n0
	move	#0,y1
	move	y1,x:(r0+n0)
	jeq	L113
	move	#>1,x0
L85
	move	#20,n0
	move	x:(r0+n0),b
	lsr	b
	move	b1,x:(r0+n0)
	move	#6,n0
	move	x:(r0+n0),b
	add	x0,b
	move	b1,x:(r0+n0)
	move	#20,n0
	move	x:(r0+n0),b
	move	#0,b2
	tst	b
	jne	L85
L113
; **************************************************
;   MANT = MAG ? (MAG << 6) >> EXP : 1 << 5;
; **************************************************
	move	#19,n0
	move	x:(r0+n0),b
	move	#0,b2
	tst	b
	jeq	L86
	rep	#6
	lsl	b
	move	#6,n0
	move	x:(r0+n0),a
	tst	a
	jeq	L128
	rep	a
	lsr	b
L128
	move	#15,n0
	move	b1,x:(r0+n0)
	jmp	L87
L86
	move	#15,n0
	move	#>32,y1
	move	y1,x:(r0+n0)
L87
; **************************************************
;   DQSEZ = (DQS << 10) + (EXP << 6) + MANT;
; **************************************************
	move	#7,n0
	move	x:(r0+n0),b
	rep	#10
	lsl	b
	move	#20,n0
	move	b1,x:(r0+n0)
	move	#6,n0
	move	x:(r0+n0),b
	rep	#6
	asl	b
; **************************************************
;   
;   /* XOR */
;   BP[0] = DQS ^ (S->DQ2 >> 10);
; **************************************************
	move	#4,n4
	move	#19,n0
	move	b1,b
	move	b1,y1
	move	b1,x:(r0+n0)
	move	#20,n0
	move	x:(r0+n0),b
	add	y1,b	#15,n0
	move	x:(r0+n0),y1
	add	y1,b	#9,n0
	move	b1,x:(r0+n0)
	move	x:(r4+n4),y1
	tfr	y1,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[1] = DQS ^ (S->DQ3 >> 10);
; **************************************************
	move	#5,n4
	move	#7,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b
	move	b1,b
	move	b1,y1
	move	y1,x:(r0)
	move	x:(r4+n4),y1
	tfr	y1,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[2] = DQS ^ (S->DQ4 >> 10);
; **************************************************
	move	#6,n4
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	(r0)+
	move	b1,b
	move	b1,y1
	move	y1,x:(r0)-
	move	x:(r4+n4),y1
	tfr	y1,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[3] = DQS ^ (S->DQ5 >> 10);
; **************************************************
	move	#7,n4
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#2,n0
	move	b1,b
	move	b1,y1
	move	y1,x:(r0+n0)
	move	x:(r4+n4),y1
	tfr	y1,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[4] = DQS ^ (S->DQ6 >> 10);
; **************************************************
	move	#8,n4
	move	#7,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#3,n0
	move	b1,b
	move	b1,y1
	move	y1,x:(r0+n0)
	move	x:(r4+n4),y1
	tfr	y1,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   BP[5] = DQS ^ (S->DQ7 >> 10);
; **************************************************
	move	#9,n4
	move	#7,n0
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#4,n0
	move	b1,b
	move	b1,y1
	move	y1,x:(r0+n0)
	move	x:(r4+n4),y1
	tfr	y1,b
	move	b1,b
	rep	#10
	asr	b
; **************************************************
;   
;   /* UPB */
;   MANT = S->DQ & 16383;
; **************************************************
; **************************************************
;   for (EXP = 0; EXP < 6; EXP++)
; **************************************************
	move	#24,n3
	move	#7,n0
	move	r0,r2
	move	r4,r3
	move	#>65535,x1
	move	b1,b
	move	b1,x0
	move	x:(r0+n0),b
	eor	x0,b	#5,n0
	move	b1,b
	move	b1,y1
	move	#>16383,x0
	move	y1,x:(r0+n0)
	move	#20,n0
	move	x:(r4),y1
	tfr	y1,b	y1,x:(r0+n0)
	and	x0,b	#15,n0
	move	b1,b
	move	#>1,x0
	move	#0,y1
	move	b1,x:(r0+n0)
	move	#6,n0
	move	(r3)+n3
	move	y1,x:(r0+n0)
	do	#6,L112
L97
; **************************************************
;   {
;     DQS = MANT ? (BP[EXP] ? 65408 : 128) : 0;
; **************************************************
	move	#15,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L91
	move	x:(r2),a
	tst	a
	jeq	L93
	move	#>65408,y1
	jmp	L124
L93
	move	#>128,y1
L124
	move	#20,n0
	move	y1,x:(r0+n0)
	move	x:(r0+n0),y1
	jmp	L125
L91
	move	#0,y1
L125
; **************************************************
;     DQS += S->B[EXP] & (1 << 15) ? (65536 - ((S->B[EXP] >> 8) + 65280)) :
;                                    (65536 - (S->B[EXP] >> 8));
; **************************************************
	move	#24,n1
	move	#20,n0
	move	r4,b
	move	y1,x:(r0+n0)
	move	x:(r0+n0),y1
	move	#7,n0
	move	y1,x:(r0+n0)
	move	#6,n0
	move	x:(r0+n0),y1
	add	y1,b	#19,n0
	move	b1,r1
	move	x:(r1+n1),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	#>32768,y1
	and	y1,b
	move	b1,b
	move	#0,b2
	tst	b
	jeq	L95
	move	x:(r0+n0),b
	rep	#8
	lsr	b
	move	b1,y1
	move	#>256,b
	jmp	L126
L95
	move	x:(r3),y1
	tfr	y1,b
	rep	#8
	lsr	b
	move	b1,y1
	move	#>65536,b
L126
; **************************************************
;     BP[EXP] = (S->B[EXP] + DQS) & 65535;
; **************************************************
; **************************************************
; **************************************************
	sub	y1,b	#20,n0
	move	b1,x:(r0+n0)
	move	#7,n0
	move	x:(r0+n0),b
	move	#20,n0
	move	x:(r0+n0),y1
	add	y1,b	#7,n0
	move	b1,x:(r0+n0)
	move	#20,n0
	move	x:(r3),y1
	add	y1,b
	and	x1,b
	move	b1,b
	move	b1,y1
	move	b1,x:(r0+n0)
	move	#6,n0
	move	r3,b
	add	x0,b	y1,x:(r2)
	move	b1,r3
	move	r2,b
	add	x0,b
	move	b1,r2
	move	x:(r0+n0),b
	add	x0,b
	move	b1,x:(r0+n0)
L112
; **************************************************
;   }
;   
;   /* TRIGB */
;   if (S->T)
; **************************************************
	move	(r4)+
	move	x:(r4)-,a
	tst	a
	jeq	L98
; **************************************************
;     for (EXP = 0; EXP < 6; EXP++)
; **************************************************
	move	r0,r1
	move	#0,x0
	do	#6,L110
L102
; **************************************************
;       BP[EXP] = 0;
; **************************************************
; **************************************************
; **************************************************
	move	x0,x:(r1)+
L110
	nop
L98
; **************************************************
;   
;   /* FMULT */
;   DQI  = f_mult(BP[0], DQSEZ);
; **************************************************
	move	#9,n0
	move	#Ff_mult,r1
	move	x:(r0+n0),y1
	move	y1,x:(r6)+
	move	x:(r0),y1
	move	y1,x:(r6)+
	jsr	(r1)
; **************************************************
;   MAG  = f_mult(BP[1], S->DQ2);
; **************************************************
	move	#4,n4
	move	#8,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	x:(r0+n0),b
	move	x:(r4+n4),y1
	move	(r0)+
	move	(r6)-
	move	y1,x:(r6)+
	move	x:(r0)-,y1
	move	y1,x:(r6)+
	jsr	(r1)
; **************************************************
;   MANT = f_mult(BP[2], S->DQ3);
; **************************************************
	move	#5,n4
	move	#19,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	#2,n0
	move	x:(r4+n4),y1
	move	(r6)-
	move	y1,x:(r6)+
	move	x:(r0+n0),y1
	move	y1,x:(r6)+
	jsr	(r1)
; **************************************************
;   A1S  = f_mult(BP[3], S->DQ4);
; **************************************************
	move	#6,n4
	move	#15,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	#3,n0
	move	x:(r4+n4),y1
	move	(r6)-
	move	y1,x:(r6)+
	move	x:(r0+n0),y1
	move	y1,x:(r6)+
	jsr	(r1)
; **************************************************
;   AP   = f_mult(BP[4], S->DQ5);
; **************************************************
	move	#7,n4
	move	#12,n0
	move	(r6)-
	move	a1,x:(r0+n0)
	move	#4,n0
	move	x:(r4+n4),y1
	move	(r6)-
	move	y1,x:(r6)+
	move	x:(r0+n0),y1
	move	y1,x:(r6)+
	jsr	(r1)
; **************************************************
;   EXP  = f_mult(BP[5], S->DQ6);
; **************************************************
	move	#5,n0
	move	#8,n4
	move	a1,x1
	move	x:(r4+n4),y1
	move	(r6)-
	move	(r6)-
	move	y1,x:(r6)+
	move	x:(r0+n0),y1
	move	y1,x:(r6)+
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
	move	#16,n4
	move	#6,n0
	move	#>65535,x0
	move	a1,x:(r0+n0)
	move	#19,n0
	move	(r6)-
	move	x:(r0+n0),y1
	add	y1,b	#15,n0
	move	(r6)-
	move	x:(r0+n0),y1
	add	y1,b	#12,n0
	move	x:(r0+n0),y1
	add	y1,b	#6,n0
	add	x1,b
	move	x:(r0+n0),y1
	add	y1,b	#7,n0
	and	x0,b
	move	b1,b
	lsr	b	b1,x:(r0+n0)
	move	b1,y1
	move	y1,x:(r4+n4)
	move	#15,n4
	move	x:(r0+n0),b
	move	#14,n0
	move	x:(r0+n0),y1
	add	y1,b	#13,n0
	move	x:(r0+n0),y1
	add	y1,b	#20,n0
	and	x0,b
	move	b1,b
	lsr	b
	move	b1,y1
	move	y1,x:(r4+n4)
	move	#10,n4
	move	y1,x:FS_E
	move	x:(r4+n4),y1
	move	#11,n4
	move	y1,x:(r0+n0)
	move	#10,n0
	move	y1,x:(r4+n4)
	move	#10,n4
	move	x:(r0+n0),y1
	move	y1,x:(r4+n4)
; **************************************************
;   S->SR2 = SR1;
; **************************************************
; **************************************************
;   A_2 = S->A2 = A2R;
; **************************************************
; **************************************************
; 
;   for (EXP = 0; EXP < 6; EXP++)
; **************************************************
	move	#24,n2
	move	#12,n4
	move	#11,n0
	move	r0,r1
	move	r4,r2
	move	x:(r0+n0),y1
	move	#>1,x0
	move	y1,x:(r4+n4)
	move	#14,n4
	move	(r2)+n2
	move	y0,x:(r4+n4)
	move	y0,x:FA_2
	do	#6,L108
L106
; **************************************************
;     S->B[EXP] = BP[EXP];
; **************************************************
; **************************************************
; **************************************************
	move	#20,n0
	move	r2,b
	add	x0,b	x:(r1)+,y1
	move	y1,x:(r0+n0)
	move	y1,x:(r2)
	move	b1,r2
	nop
L108
; **************************************************
; 
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
	move	#8,n4
	move	(r6)-
	move	x:(r4+n4),y1
	move	#9,n4
	move	y1,x:(r4+n4)
	move	#7,n4
	move	x:(r4+n4),y1
	move	#8,n4
	move	y1,x:(r4+n4)
	move	#6,n4
	move	x:(r4+n4),y1
	move	#7,n4
	move	y1,x:(r4+n4)
	move	#5,n4
	move	x:(r4+n4),y1
	move	#6,n4
	move	y1,x:(r4+n4)
	move	#4,n4
	move	x:(r4+n4),y1
	move	#5,n4
	move	y1,x:(r0+n0)
	move	#9,n0
	move	y1,x:(r4+n4)
	move	#4,n4
	move	x:(r0+n0),y1
	move	y1,x:(r4+n4)
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
	move	y0,x:(r6)+
	move	r1,x:(r6)+
	move	#F___qtab0,r1
	move	r2,x:(r6)+
	move	x:(r0+n0),r2
	move	x:FI,a
	move	x:(r2+n2),y0
	move	r1,b
	add	a,b
	tfr	y0,b	b1,r1
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
	jeq	L130
	clr	a
	jmp	L131
L130
	move	#>128,y0
	move	x:(r0),a
	move	#>127,x0
	and	x0,a
	move	a1,a
	add	y0,a
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
	move	b1,y0
	move	#>14,b
	sub	y0,b
	tst	b
	jeq	L132
	move	a1,a
	rep	b
	asr	a
	move	a1,a
L132
L131
; **************************************************
; }
; **************************************************
	move	(r6)-
	move	(r0)+
	move	x:(r0)-,b
	add	a,b	(r0)-
	tfr	b,a	(r0)-
	move	a1,x:(r2)
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

	global	Ftone_detector
Ftone_detector
; **************************************************
; **************************************************
; **************************************************
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
	move	#0,a2
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
	move	#0,b2
	cmp	a,b
	jle	L134
	move	#0,b2
	move	#>53760,a
	move	#0,a2
	cmp	a,b
	jgt	L134
	move	#>1,a
	jmp	L135
L134
	clr	a
L135
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
	move	#17,n1
	move	a1,x:FTDP
	move	x:(r1+n1),x1
	move	#18,n1
	move	(r0)+
	move	x1,x:(r0)-
	move	x:(r1+n1),a
	tst	a
	jeq	L136
	clr	a
	jmp	L137
L136
	move	x:FTDP,a
L137
; **************************************************
;   
;   S->LAST_TR = S->YL >> 15; /* (*LAST_TR)  is used here as a temporary variable */
; **************************************************
	move	#17,n1
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
	move	#18,n1
	move	#>8,b
	move	a1,a
	cmp	b,a	a1,x:(r1+n1)
	jle	L138
	move	#>15872,x1
	move	x1,x:(r0)
	jmp	L139
L138
	move	#18,n1
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
	jeq	L142
	rep	b
	asl	a
	move	a1,a
L142
	move	a1,x:(r0)
L139
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
	jle	L140
	move	#>1,a
	move	(r0)+
	move	x:(r0)-,b
	cmp	a,b
	jeq	L141
L140
	clr	a
L141
; **************************************************
; }
; **************************************************
	move	#18,n1
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
;   register unsigned int FI, tmp ; 
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
	rep	#3
	lsr	a
	move	#0,a2
	tst	a
	jeq	L144
	move	#>15,a
	sub	y0,a
	jmp	L145
L144
	move	x:FI,a
L145
; **************************************************
; 
;   /* FILTA */
;   tmp = ((FI << 9) + 8192 - S->DMS) & 8191;              /* tmp => DIF */
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
	lsl	b
; **************************************************
;   tmp >>= 5;
; **************************************************
	move	#19,n2
	move	#>16769024,x0
	move	x:(r2+n2),a
	add	x0,a	#>8191,x0
	sub	a,b
	and	x0,b
	move	b1,b
	rep	#5
	lsr	b
; **************************************************
;   if (tmp & (1 << 7))
; **************************************************
	tfr	b,a	#>128,x0
	and	x0,a	(r0)+
	move	a1,a
	move	#0,a2
	tst	a	b1,x:(r0)-
	jeq	L146
; **************************************************
;     tmp = tmp + 3840;                                    /* tmp => DIFSX */
; **************************************************
	move	#>3840,a
	add	a,b	(r0)+
	move	b1,x:(r0)-
L146
; **************************************************
;   S->DMS = (tmp + S->DMS) & 4095;
; **************************************************
; **************************************************
; 
;   /* FILTB */
;   tmp = ((FI << 11) + 32768 - S->DML) & 32767;           /* tmp => DIF */
; **************************************************
	move	#19,n2
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
	lsl	b
; **************************************************
;   tmp >>= 7;
; **************************************************
	move	#20,n2
	move	#>16744448,x0
	move	x:(r2+n2),a
	add	x0,a	#>32767,x0
	sub	a,b
	and	x0,b
	move	b1,b
	rep	#7
	lsr	b
; **************************************************
;   if (tmp & (1 << 7))
; **************************************************
	tfr	b,a	#>128,x0
	and	x0,a	(r0)+
	move	a1,a
	move	#0,a2
	tst	a	b1,x:(r0)-
	jeq	L147
; **************************************************
;     tmp = tmp + 16128;                                   /* tmp => DIFSX */
; **************************************************
	move	#>16128,a
	add	a,b	(r0)+
	move	b1,x:(r0)-
L147
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
	move	#20,n2
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
	move	#19,n2
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
	move	#0,a2
	tst	a
	jeq	L148
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
L148
; **************************************************
;   FI = ((S->Y > 1535) && (tmp < (S->DML >> 3)) && (TDP == 0)) ? 0 : 1;
; **************************************************
	move	#2,n2
	move	#>1535,b
	move	x:(r2+n2),a
	cmp	b,a
	jle	L149
	move	#20,n2
	move	x:(r2+n2),a
	move	a1,a
	rep	#3
	asr	a
	move	a1,a
	move	#0,a2
	move	(r0)+
	move	x:(r0)-,b
	move	#0,b2
	cmp	a,b
	jge	L149
	move	x:FTDP,a
	move	#0,a2
	tst	a
	jne	L149
	move	#0,y0
	jmp	L157
L149
	move	#>1,y0
L157
; **************************************************
; 
;   /* FILTC */
;   tmp = ((FI << 9) + 2048 - S->AP) & 2047;               /* tmp => DIF */ 
; **************************************************
	move	y0,x:(r0)
	move	x:(r0),b
	rep	#9
	lsl	b
; **************************************************
;   tmp = (tmp >> 4) + (tmp >> 10 ? 896 : 0);              /* tmp => DIFSX */
; **************************************************
	move	#21,n2
	move	#>16775168,x0
	move	x:(r2+n2),a
	add	x0,a	#>2047,x0
	sub	a,b	(r0)+
	and	x0,b
	move	b1,b
	tfr	b,a	b1,x:(r0)-
	rep	#4
	lsr	b
	move	#2,n0
	move	b1,x:(r0+n0)
	rep	#10
	lsr	a
	move	#0,a2
	tst	a
	jeq	L151
	move	#>896,a
	jmp	L152
L151
	clr	a
L152
; **************************************************
;   tmp = (tmp + S->AP) & 1023;                            /* tmp => APP */
; **************************************************
; **************************************************
; 
;   /* TRIGA */
;   S->AP = S->T ? 256 : tmp; 
; **************************************************
	move	#21,n2
	move	#2,n0
	move	#>1023,x0
	move	x:(r0+n0),b
	add	a,b	x:(r2+n2),a
	add	a,b	(r2)+
	and	x0,b	(r0)+
	move	b1,b
	move	b1,x:(r0)-
	move	x:(r2)-,a
	tst	a
	jeq	L153
	move	#>256,y0
	jmp	L158
L153
	move	(r0)+
	move	x:(r0)-,y0
L158
; **************************************************
; 
;   /* LIMA */
;   AL = (S->AP > 255) ? 64 : S->AP >> 2; 
; **************************************************
	move	#21,n2
	move	#2,n0
	move	#>255,a
	move	y0,x:(r0+n0)
	move	x:(r0+n0),y0
	tfr	y0,b	y0,x:(r2+n2)
	cmp	a,b
	jle	L155
	move	#>64,a
	jmp	L156
L155
	move	#21,n2
	move	x:(r2+n2),a
	move	a1,a
	asr	a
	asr	a
	move	a1,a
L156
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
;   register unsigned int TMP, YUP, YLP;
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
	rep	#3
	lsr	a
	move	#0,a2
	tst	a
	jeq	L160
	move	#>15,a
	sub	x1,a
	jmp	L171
L160
	move	x:FI,a
L171
; **************************************************
;   
;   /* FILTD */
;   TMP = ((TMP << 5) + 131072 - S->LAST_Y) & 131071;
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
	lsl	b
; **************************************************
;   TMP >>= 5;
; **************************************************
	move	#23,n2
	move	#>16646144,x0
	move	x:(r2+n2),a
	add	x0,a	#>131071,x0
	sub	a,b
	and	x0,b
	move	b1,b
	rep	#5
	lsr	b
; **************************************************
;   YUP = (TMP >> 11) ? TMP + 4096 : TMP;
; **************************************************
	tfr	b,a	#2,n0
	move	b1,x:(r0+n0)
	rep	#11
	lsr	a
	move	#0,a2
	tst	a
	jeq	L162
	move	#>4096,a
	add	a,b
	move	b1,x:(r0)
	jmp	L163
L162
	move	#2,n0
	move	x:(r0+n0),x1
	move	x1,x:(r0)
L163
; **************************************************
;   YUP = (YUP + S->LAST_Y) & 8191;
; **************************************************
; **************************************************
;   
;   /* LIMB */
;   if ((((YUP + 11264) & 16383) >> 13) == 0)
; **************************************************
	move	#23,n2
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
	rep	#13
	lsr	a
	move	#0,a2
	tst	a
	jne	L164
; **************************************************
;     YUP = 5120;
; **************************************************
	move	#>5120,x1
	move	x1,x:(r0)
L164
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
	rep	#13
	lsr	a
	move	#0,a2
	tst	a
	jeq	L165
; **************************************************
;     YUP = 544;
; **************************************************
	move	#>544,x1
	move	x1,x:(r0)
L165
; **************************************************
;   
;   /* FILTE */
;   TMP = (YUP + ((1048576 - S->Y_L) >> 6)) & 16383;
; **************************************************
	move	#22,n2
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
	move	#0,a2
	tst	a
	jeq	L166
; **************************************************
;     TMP = TMP + 507904;
; **************************************************
	move	#>507904,a
	add	a,b
	move	b1,x:(r0+n0)
L166
; **************************************************
;   YLP = (TMP + S->Y_L) & 524287;
; **************************************************
; **************************************************
;   
;   /* MIX */
;   TMP = (YUP + 16384 - (S->Y_L >> 6)) & 16383;
; **************************************************
	move	#2,n0
	move	#22,n2
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
;   S->LAST_Y = SIGNBIT(TMP, 13);
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
	move	#0,a2
	tst	a
	jeq	L167
	move	#>1,a
	jmp	L168
L167
	clr	a
L168
; **************************************************
;   if (S->LAST_Y)
; **************************************************
	tst	a	#23,n2
	move	a1,x:(r2+n2)
	jeq	L169
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
L169
; **************************************************
;   TMP = (TMP * AL) >> 6;
; **************************************************
	move	#2,n0
	move	x:FAL,x0
	move	x:(r0+n0),x1
	mpy	+x0,x1,b
	asr	b
	move	b0,b
	rep	#6
	lsr	b
; **************************************************
;   if (S->LAST_Y)
; **************************************************
	move	#23,n2
	move	b1,x:(r0+n0)
	move	x:(r2+n2),a
	tst	a
	jeq	L170
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
L170
; **************************************************
;   
;   S->LAST_Y = S->Y = ((S->Y_L >> 6) + TMP) & 8191; 
; **************************************************
	move	#22,n2
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
	move	#23,n2
	move	a1,x:(r2+n2)
	move	#22,n2
	move	x:(r2+n2),a
	move	#3,n2
	move	a1,x:(r2+n2)
	move	#22,n2
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

	global	Finit_disk_play
Finit_disk_play
; **************************************************
; **************************************************
; **************************************************
; 
; #ifndef __MNECOUNT__
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
	jle	L174
; **************************************************
;     ExitFlag = TRUE;
; **************************************************
	move	#>1,a
	move	a1,x:FExitFlag
L174
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
	move	r0,x:(r6)+
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
; #endif
; 
; main()
; {
; #ifndef __MNECOUNT__
; #ifdef __DOWNLOAD__
;   run56k_start();
; #endif
; 
;   init_disk_play();
; **************************************************
	move	#3,n6
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
	clr	a	#FD_STATES,r1
	move	#Fiadpt_quant,r3
	move	#FE_STATES,r2
	move	a1,x:FExitFlag
L231
; **************************************************
;   {
; #endif
;     disk_play();
; **************************************************
	jsr	Fdisk_play
; **************************************************
;     if (ExitFlag)
; **************************************************
	move	x:FExitFlag,a
	tst	a
	jne	L181
; **************************************************
;       break;
; **************************************************
; **************************************************
; 
;     /*
;      * ENCODER
;      */
;     adpt_predict(&E_STATES);
; **************************************************
	move	r2,x:(r6)+
	jsr	Fadpt_predict
; **************************************************
;     {
;       /* SUBTA (inlined) */
;       D = ((SL & (1 << 13) ? 49152 | SL : SL) + 65536 -
;           (S_E & (1 << 14) ? 32768 | S_E : S_E)) & 65535;
; **************************************************
	move	#2,n0
	move	x:FSL,y1
	tfr	y1,a	#>8192,x0
	and	x0,a	y1,x:(r0+n0)
	move	a1,a
	move	#0,a2
	tst	a	(r6)-
	jeq	L183
	move	x:(r0+n0),b
	move	#>49152,x0
	or	x0,b
	move	b1,b
	move	b1,x:(r0)
	jmp	L184
L183
	move	x:FSL,y1
	move	y1,x:(r0)
L184
	move	#2,n0
	move	x:FS_E,y1
	tfr	y1,a	y1,x:(r0+n0)
	move	#>16384,y1
	and	y1,a
	move	a1,a
	move	#0,a2
	tst	a
	jeq	L185
	move	x:(r0+n0),b
	move	#>32768,y1
	or	y1,b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L186
L185
	move	#2,n0
	move	x:FS_E,y1
	move	y1,x:(r0+n0)
L186
; **************************************************
;     }
;     {
;       /* adaptive predictor (inlined) */
;       register int DS, DQM, DL, DLN, EXP;
; 
;       /* LOG */
;       DS = D & (1 << 15);
; **************************************************
; **************************************************
;       DQM = DS ? (65536 - D) & 32767 : D;
; **************************************************
	move	#2,n0
	move	#>65535,x0
	move	x:(r0+n0),b
	move	#>16711680,a
	add	a,b
	tfr	b,a	x:(r0),b
	sub	a,b
	tfr	b,a
	and	x0,a
	move	a1,a
	tfr	a,b	a1,y1
	move	a1,x:(r0+n0)
	move	y1,x:FD
	move	#>32768,y1
	and	y1,b	(r0)+
	move	b1,b
	tst	b	b1,x:(r0)-
	jeq	L187
	move	#>32767,x0
	move	x:(r0+n0),y1
	move	#>65536,a
	sub	y1,a
	and	x0,a
	move	a1,a
	jmp	L188
L187
	move	x:FD,a
L188
; **************************************************
;       {
; 	register int dqm = DQM;
; **************************************************
; **************************************************
; 
; 	for (EXP = 1; dqm >>= 1; EXP++)
; **************************************************
	move	#>1,y0
	move	a1,x1
	jmp	L234
L191
	tfr	y0,b	#>1,y1
	add	y1,b
	move	b1,y0
L234
	move	a1,a
	asr	a
	move	a1,a
	tst	a
	jne	L191
; **************************************************
; 	  ;
;       }
;       EXP--;
; **************************************************
; **************************************************
;       DL = (EXP * (1 << 7)) + (((DQM * (1 << 7)) >> EXP) & 127);
; **************************************************
	tfr	y0,b	#>128,x0
	move	#>16777215,a
	add	a,b
	mpy	+x0,x1,a	b1,y0
	asr	a
	tst	b	a0,a
	jeq	L236
	move	a1,a
	rep	b
	asr	a
	move	a1,a
L236
; **************************************************
; 
;       /* SUBTB */
;       DLN = (DL + 4096 - (E_STATES.Y >> 2)) & 4095;
; **************************************************
; **************************************************
; 
;       /* QUAN */
;       if (DLN > 3971)
; **************************************************
	move	#2,n0
	move	x:(FE_STATES+2),y1
	tfr	y1,b	#>127,x1
	and	x1,a	b1,b
	asr	b	a1,a
	asr	b	a,a0
	asl	a	b1,b
	mac	+y0,x0,a	#>16773120,x0
	add	x0,b	#>4095,x0
	asr	a	b1,y1
	move	a0,a
	sub	y1,a
	and	x0,a
	move	a1,a
	tfr	a,b	a1,x:(r0+n0)
	move	#>3971,a
	cmp	a,b
	jle	L193
; **************************************************
; 	I = DS ? 0xE : 0x1;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jne	L235
	jmp	L223
L193
; **************************************************
;       else if (DLN > 2047)
; **************************************************
	move	#2,n0
	move	#>2047,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L197
; **************************************************
; 	I = 0xF;
; **************************************************
	move	#>15,a
	jmp	L224
L197
; **************************************************
;       else if (DLN > 399)
; **************************************************
	move	#2,n0
	move	#>399,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L199
; **************************************************
; 	I = DS ? 0x8 : 0x7;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L200
	move	#>8,a
	jmp	L224
L200
	move	#>7,a
	jmp	L224
L199
; **************************************************
;       else if (DLN > 348)
; **************************************************
	move	#2,n0
	move	#>348,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L203
; **************************************************
; 	I = DS ? 0x9 : 0x6;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L204
	move	#>9,a
	jmp	L224
L204
	move	#>6,a
	jmp	L224
L203
; **************************************************
;       else if (DLN > 299)
; **************************************************
	move	#2,n0
	move	#>299,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L207
; **************************************************
; 	I = DS ? 0xA : 0x5;	
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L208
	move	#>10,a
	jmp	L224
L208
	move	#>5,a
	jmp	L224
L207
; **************************************************
;       else if (DLN > 245)
; **************************************************
	move	#2,n0
	move	#>245,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L211
; **************************************************
; 	I = DS ? 0xB : 0x4;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L212
	move	#>11,a
	jmp	L224
L212
	move	#>4,a
	jmp	L224
L211
; **************************************************
;       else if (DLN > 177)
; **************************************************
	move	#2,n0
	move	#>177,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L215
; **************************************************
; 	I = DS ? 0xC : 0x3;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L216
	move	#>12,a
	jmp	L224
L216
	move	#>3,a
	jmp	L224
L215
; **************************************************
;       else if (DLN > 79)
; **************************************************
	move	#2,n0
	move	#>79,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L219
; **************************************************
; 	I = DS ? 0xD : 0x2;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L220
	move	#>13,a
	jmp	L224
L220
	move	#>2,a
	jmp	L224
L219
; **************************************************
;       else
; 	I = DS ? 0xE : 0x1;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L223
L235
	move	#>14,a
	jmp	L224
L223
	move	#>1,a
L224
; **************************************************
;     }
;     iadpt_quant(&E_STATES);
; **************************************************
	move	a1,x:FI
	move	r2,x:(r6)+
	jsr	(r3)
; **************************************************
;     tone_detector(&E_STATES);
; **************************************************
	move	(r6)-
	move	r2,x:(r6)+
	jsr	Ftone_detector
; **************************************************
;     speed_control(&E_STATES);
; **************************************************
	move	(r6)-
	move	r2,x:(r6)+
	jsr	Fspeed_control
; **************************************************
;     scale_factor(&E_STATES);
; **************************************************
	move	(r6)-
	move	r2,x:(r6)+
	jsr	Fscale_factor
; **************************************************
; 
;     /*
;      * DECODER
;      */
;     adpt_predict(&D_STATES);
; **************************************************
	move	(r6)-
	move	r1,x:(r6)+
	jsr	Fadpt_predict
; **************************************************
;     iadpt_quant(&D_STATES);
; **************************************************
	move	(r6)-
	move	r1,x:(r6)+
	jsr	(r3)
; **************************************************
;     {
;       /* ADD (inlined) */
;       register int dq, se;
; 
;       /* 15 SM auf 16 TC */
;       dq = (D_STATES.DQ & (1 << 14)) ? D_STATES.DQ == (1 << 14) ? 0: 
;            (((D_STATES.DQ ^ 0x3FFF) + 0x1) & 0x3FFF) + 0xC000 : D_STATES.DQ ; 
; **************************************************
	move	#2,n0
	move	x:(r1),y1
	tfr	y1,a	y1,x:(r0+n0)
	move	#>16384,y1
	and	y1,a	(r6)-
	move	a1,a
	tst	a
	jeq	L225
	move	x:(r0+n0),b
	cmp	y1,b
	jne	L227
	clr	a
	jmp	L226
L227
	move	#>1,y1
	move	#>16383,x0
	move	x:(r1),a
	eor	x0,a
	move	a1,a
	add	y1,a	#>49152,y1
	and	x0,a
	move	a1,a
	add	y1,a
	jmp	L226
L225
	move	x:(r1),a
L226
; **************************************************
; 
;       /* 15 TC auf 16 TC */
;       se = S_E & (1 << 14) ? 0x8000 | S_E : S_E ; 
; **************************************************
	move	#2,n0
	move	x:FS_E,y1
	tfr	y1,a	a1,x0
	move	y1,x:(r0+n0)
	move	#>16384,y1
	and	y1,a
	move	a1,a
	move	#0,a2
	tst	a
	jeq	L229
	move	#>32768,y1
	move	x:(r0+n0),a
	or	y1,a
	move	a1,a
	jmp	L230
L229
	move	x:FS_E,a
L230
; **************************************************
;   
;       /* perform add operation at 16 TC */
;       SP  = (dq + se) & 0xFFFF;
; **************************************************
; **************************************************
;     }
;     {
;       /* coding_adjector (inlined) */
;       /* Just passing a SP (16TC) through */
;       /* rest of the signals is ignored */
;       SD = SP;
; **************************************************
; **************************************************
;     }
;     tone_detector(&D_STATES);
; **************************************************
	tfr	x0,b	#>65535,x0
	add	a,b
	tfr	b,a
	and	x0,a
	move	a1,a
	move	a1,x:FSP
	move	a1,x:FSD
	move	r1,x:(r6)+
	jsr	Ftone_detector
; **************************************************
;     speed_control(&D_STATES);
; **************************************************
	move	(r6)-
	move	r1,x:(r6)+
	jsr	Fspeed_control
; **************************************************
;     scale_factor(&D_STATES);
; **************************************************
	move	(r6)-
	move	r1,x:(r6)+
	jsr	Fscale_factor
; **************************************************
; #ifndef __MNECOUNT__
;     disk_record();
; **************************************************
	move	(r6)-
	jsr	Fdisk_record
	move	x:FExitFlag,a
	tst	a
	jeq	L231
L181
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
;   return (0);
; **************************************************
; **************************************************
; #endif
; } 
; **************************************************
	clr	a	(r0)-
	move	(r6)-
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

