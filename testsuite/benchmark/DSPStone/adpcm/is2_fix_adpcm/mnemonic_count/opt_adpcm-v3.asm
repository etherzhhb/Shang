;
;	annotate version 1.03
;
; **************************************************
; **************************************************
	section	opt_adpcm_v3_c
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
;  * __LOOPCOUNT__:   use to count loop cycles
;  * __MNECOUNT__:    use to count mnemonics
;  *
;  * 3-NOV-93 Chris Schlaeger
;  * 
;  * VERSION 1.3
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
; typedef struct
; {
;   int DQ, T, Y, YL, DQ2, DQ3, DQ4, DQ5, DQ6, DQ7, PK1, PK2, SR2,
;       A1, A2, B[6], SE, SEZ, t_dr, LAST_TR, DMS, DML, AP, Y_L, LAST_Y;
; } STATES;
; 
; /* ENCODER states */
; STATES E_STATES = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
; 
; /* DECODER states */
; STATES D_STATES = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
; 
; #define LSHIFT(a, b) ((b) < 0 ? (a) << -(b) : (a) >> (b))
;  
; int f_mult(int An, int SRn)
; {
;   register int  EXP;
;   register int  WAnMANT;
;   int           AnS, MAG, AnMANT;
; 
;   AnS = An & (1 << 15) ? 1 : 0;
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
	move	#>32768,x0
	move	x1,x:(r6)+
	move	y0,x:(r6)+
	move	y1,x:(r6)+
	move	x:(r0+n0),a
	and	x0,a
	move	a1,a
	tst	a
	jeq	L2
	move	#>1,y0
	jmp	L19
L2
	move	#0,y0
L19
; **************************************************
;   MAG = AnS? (16384 - (An >> 2)) & 8191 : An >> 2;
; **************************************************
	move	#2,n0
	move	y0,x:(r0+n0)
	move	x:(r0+n0),b
	tst	b
	jeq	L4
	move	#65533,n0
	move	#>8191,x0
	move	x:(r0+n0),a
	move	a1,a
	asr	a	#>16384,b
	asr	a
	move	a1,a
	sub	a,b
	and	x0,b
	move	b1,b
	jmp	L20
L4
	move	#65533,n0
	move	x:(r0+n0),b
	move	b1,b
	asr	b
	asr	b
	move	b1,b
L20
; **************************************************
;   {
;     register int mag = MAG << 1;
; **************************************************
; **************************************************
; 
;     for (EXP = 0; mag >>= 1; EXP++)
; **************************************************
	move	#0,x1
	move	(r0)+
	move	b1,x:(r0)-
	move	(r0)+
	move	x:(r0)-,a
	asl	a
	move	a1,a
	move	a1,a
	asr	a
	move	a1,a
	tst	a
	jeq	L18
	move	#>1,x0
L8
	tfr	x1,b	a1,a
	asr	a
	add	x0,b	a1,a
	tst	a	b1,x1
	jne	L8
L18
; **************************************************
; #ifdef __LOOPCOUNT__
;       Loop1++;
; #else
;         ;
; #endif
;   } 
; 
;   AnMANT = MAG ? LSHIFT(MAG, EXP - 6) : 1 << 5;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L10
	tfr	x1,b	#>16777210,a
	add	a,b
	tfr	b,a
	tst	a
	jge	L12
	neg	a	(r0)+
	tst	a	x:(r0)-,b
	jeq	L22
	rep	a
	asl	b
	move	b1,b
L22
	move	b1,x:(r0)
	jmp	L11
L12
	tfr	x1,b	#>16777210,a
	add	a,b	(r0)+
	tfr	b,a	x:(r0)-,b
	tst	a
	jeq	L23
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L23
	move	b1,x:(r0)
	jmp	L11
L10
	move	#>32,y0
	move	y0,x:(r0)
L11
; **************************************************
;   EXP += ((SRn >> 6) & 15);
; **************************************************
	move	#65532,n0
	move	x:(r0+n0),a
	move	a1,a
	rep	#6
	asr	a
; **************************************************
;   WAnMANT = (((SRn & 63) * AnMANT) + 48) >> 4;
; **************************************************
	tfr	x1,b	x:(r0),y0
	move	#>15,x0
	move	a1,a
	and	x0,a	#>63,x0
	move	a1,a
	add	a,b	#>48,a
	move	a,a0
	asl	a	b1,x1
	move	x:(r0+n0),b
	and	x0,b
	move	b1,b
	move	b1,y1
	mac	+y1,y0,a
	asr	a
	move	a0,a
	tfr	a,b
	move	b1,b
	rep	#4
	asr	b
; **************************************************
;   MAG = LSHIFT(WAnMANT, 19 - EXP) & 32767;
; **************************************************
	move	#>19,a
	sub	x1,a	b1,b
	tst	a	(r0)+
	move	b1,x:(r0)-
	jge	L14
	neg	a
	tst	a
	jeq	L24
	rep	a
	asl	b
	move	b1,b
L24
	jmp	L21
L14
	move	#>19,a
	sub	x1,a	(r0)+
	tst	a	x:(r0)-,b
	jeq	L25
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L25
L21
; **************************************************
;   return ((SRn >> 10) ^ AnS ? (65536 - MAG) & 65535 : MAG);
; **************************************************
	move	#65532,n0
	move	#>32767,x0
	move	(r0)+
	move	b1,x:(r0)-
	move	(r0)+
	move	x:(r0)-,b
	and	x0,b	(r0)+
	move	b1,b
	move	b1,x:(r0)-
	move	x:(r0+n0),a
	move	a1,a
	rep	#10
	asr	a
	move	#2,n0
	move	a1,a
	move	x:(r0+n0),y0
	eor	y0,a
	move	a1,a
	tst	a
	jeq	L16
	move	#>65535,x0
	move	#>65536,a
	move	(r0)+
	move	x:(r0)-,y0
	sub	y0,a
	and	x0,a
	move	a1,a
	jmp	L17
L16
	move	(r0)+
	move	x:(r0)-,a
L17
; **************************************************
; }
; **************************************************
	tst	a	(r0)-
	move	(r0)-
	move	(r6)-
	move	x:(r6)-,y1
	move	x:(r6)-,y0
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
	move	#22,n6
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
	move	x:(r0+n0),r4
	move	#20,n0
	move	x:(r4),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	b1,b
	rep	#14
	asr	b
; **************************************************
;   DQI = DQS ? 65536 - (S->DQ & 16383) : S->DQ;
; **************************************************
	move	#6,n0
	move	b1,b
	tst	b	b1,x:(r0+n0)
	jeq	L27
	move	#20,n0
	move	#>65536,b
	move	x:(r0+n0),a
	move	#7,n0
	move	#>16383,x0
	and	x0,a
	move	a1,a
	sub	a,b
	move	b1,x:(r0+n0)
	jmp	L28
L27
	move	#7,n0
	move	x:(r4),y1
	move	y1,x:(r0+n0)
L28
; **************************************************
;   MAG = S->SEZ & (1 << 14) ? (1 << 15) + S->SEZ : S->SEZ;
; **************************************************
	move	#19,n0
	move	#22,n4
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
	move	#22,n4
	move	x:(r4+n4),y1
	move	y1,x:(r0+n0)
L30
; **************************************************
;   DQSEZ = (DQI + MAG) & 65535;
; **************************************************
; **************************************************
;   PK0 = DQSEZ >> 15;
; **************************************************
	move	#7,n0
	move	#>65535,x0
	move	x:(r0+n0),b
	move	#19,n0
	move	x:(r0+n0),y1
	add	y1,b	#8,n0
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
	jeq	L31
	move	#0,y1
	jmp	L113
L31
	move	#>1,y1
L113
; **************************************************
;   
;   /* ADDB */
;   DQI = DQS ? 65536 - (S->DQ & 16383) : S->DQ;
; **************************************************
	move	#13,n0
	move	y1,x:(r0+n0)
	move	#6,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L33
	move	#7,n0
	move	x:(r4),a
	move	#>65536,b
	move	#>16383,x0
	and	x0,a
	move	a1,a
	sub	a,b
	move	b1,x:(r0+n0)
	jmp	L34
L33
	move	#7,n0
	move	x:(r4),y1
	move	y1,x:(r0+n0)
L34
; **************************************************
;   MAG = S->SE & (1 << 14) ? (1 << 15) + S->SE : S->SE;
; **************************************************
	move	#19,n0
	move	#21,n4
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
	move	#21,n4
	move	x:(r4+n4),y1
	move	y1,x:(r0+n0)
L36
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
	move	#19,n0
	move	x:(r0+n0),y1
	add	y1,b	#8,n0
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	b1,b
	rep	#15
	asr	b
	move	b1,b
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
	move	#8,n0
	move	x:(r0+n0),y1
	move	#19,n0
	move	y1,x:(r0+n0)
L38
; **************************************************
;   {
;     register int mag = MAG << 1;
; **************************************************
; **************************************************
; 
;     for (EXP = 0; mag >>= 1; EXP++)
; **************************************************
	move	#19,n0
	move	#0,y1
	move	x:(r0+n0),b
	asl	b	#14,n0
	move	b1,b
	move	b1,b
	asr	b	y1,x:(r0+n0)
	move	#20,n0
	move	b1,b
	tst	b	b1,x:(r0+n0)
	jeq	L112
	move	#>1,x0
L41
	move	#14,n0
	move	x:(r0+n0),b
	add	x0,b
	move	b1,x:(r0+n0)
	move	#20,n0
	move	x:(r0+n0),b
	move	b1,b
	asr	b
	move	b1,b
	tst	b	b1,x:(r0+n0)
	jne	L41
L112
; **************************************************
; #ifdef __LOOPCOUNT__ 
;       Loop2++;
; #else
;       ;
; #endif
;   }
;   MANT = MAG ? (MAG << 6) >> EXP : 1 << 5;
; **************************************************
	move	#19,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L43
	rep	#6
	asl	b
	move	#14,n0
	move	b1,b
	move	x:(r0+n0),a
	tst	a
	jeq	L124
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L124
	move	#15,n0
	move	b1,x:(r0+n0)
	jmp	L44
L43
	move	#15,n0
	move	#>32,y1
	move	y1,x:(r0+n0)
L44
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
	move	#20,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#14,n0
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
	add	y1,b	#10,n0
	move	b1,x:(r0+n0)
	move	#9,n0
	move	x:(r4+n4),x0
	move	#11,n4
	move	x:(r0+n0),b
	eor	x0,b	#12,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#9,n0
	move	x:(r4+n4),x0
	move	x:(r0+n0),b
	eor	x0,b
	move	b1,b
	tst	b
	jeq	L45
	move	#>114688,y1
	jmp	L114
L45
	move	#>16384,y1
L114
; **************************************************
;   A1S = S->A1 & (1 << 15) ? 1 : 0;
; **************************************************
	move	#13,n4
	move	#19,n0
	move	#>32768,x0
	move	y1,x:(r0+n0)
	move	x:(r4+n4),a
	and	x0,a
	move	a1,a
	tst	a
	jeq	L47
	move	#>1,y1
	jmp	L115
L47
	move	#0,y1
L115
; **************************************************
;   EXP = A1S ? (S->A1 >= 57345 ? (S->A1 << 2) & 131071 : 24577 << 2) :
;               (S->A1 <= 8191 ? S->A1 << 2 : 8191 << 2); 
; **************************************************
	move	#11,n0
	move	y1,x:(r0+n0)
	move	x:(r0+n0),b
	tst	b
	jeq	L49
	move	#16,n0
	move	#13,n4
	move	x:(r4+n4),y1
	move	y1,x:(r0+n0)
	move	x:(r0+n0),b
	move	#>57344,y1
	cmp	y1,b
	jle	L51
	asl	b	#14,n0
	asl	b	#>131071,x0
	move	b1,b
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L50
L51
	move	#>98308,y1
	jmp	L116
L49
	move	#13,n4
	move	x:(r4+n4),y1
	tfr	y1,b	#>8191,y1
	cmp	y1,b
	jgt	L53
	asl	b	#14,n0
	asl	b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L50
L53
	move	#>32764,y1
L116
	move	#14,n0
	move	y1,x:(r0+n0)
L50
; **************************************************
;   EXP = WA1 == 1 ? EXP : (131072 - EXP) & 131071;
; **************************************************
	move	#12,n0
	move	#>1,y1
	move	x:(r0+n0),b
	cmp	y1,b
	jeq	L56
	move	#14,n0
	move	#>131071,x0
	move	x:(r0+n0),y1
	move	#>131072,b
	sub	y1,b
	and	x0,b	b1,x:(r0+n0)
	move	b1,b
	move	b1,x:(r0+n0)
L56
; **************************************************
;   MAG = ((MAG + EXP) & 131071) >> 7;
; **************************************************
	move	#19,n0
	move	#>131071,x0
	move	x:(r0+n0),b
	move	#14,n0
	move	x:(r0+n0),y1
	add	y1,b
	and	x0,b
	move	b1,b
	move	b1,b
	rep	#7
	asr	b
; **************************************************
;   EXP = WA2 == 1 ? 0 : (MAG & (1 << 9) ? MAG + 64512 : MAG);
; **************************************************
	move	#19,n0
	move	#>1,y1
	move	b1,b
	move	b1,x:(r0+n0)
	move	#13,n0
	move	x:(r0+n0),b
	cmp	y1,b
	jne	L57
	move	#0,y1
	jmp	L117
L57
	move	#19,n0
	move	#>512,x0
	move	x:(r0+n0),a
	and	x0,a
	move	a1,a
	tst	a
	jeq	L59
	move	x:(r0+n0),b
	move	#14,n0
	move	#>64512,y1
	add	y1,b
	move	b1,x:(r0+n0)
	jmp	L58
L59
	move	#19,n0
	move	x:(r0+n0),y1
L117
	move	#14,n0
	move	y1,x:(r0+n0)
L58
; **************************************************
;   MANT = S->A2 & (1 << 15) ? 65536 - ((S->A2 >> 7) + 65024) :
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
	jeq	L61
	move	x:(r0+n0),a
	move	a1,a
	rep	#7
	asr	a
	move	#>512,b
	move	a1,a
	jmp	L118
L61
	move	#14,n4
	move	x:(r4+n4),a
	move	a1,a
	rep	#7
	asr	a
	move	#>65536,b
	move	a1,a
L118
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
	move	#14,n0
	move	x:(r0+n0),b
	add	y1,b	#15,n0
	move	x:(r0+n0),y1
	add	y1,b	#14,n0
	and	x0,b	#>32767,y1
	move	b1,b
	cmp	y1,b	b1,x:(r0+n0)
	jle	L63
	move	#>53248,x0
	cmp	x0,b
	jgt	L63
	move	x0,x1
	jmp	L64
L63
	move	#14,n0
	move	#>12287,y1
	move	x:(r0+n0),b
	cmp	y1,b
	jle	L65
	move	#>32767,y1
	cmp	y1,b
	jgt	L65
	move	#>12288,x1
	jmp	L64
L65
	move	#14,n0
	move	x:(r0+n0),x1
L64
; **************************************************
;   
;   /* TRIGB */
;   A2R = S->T ? 0 : AP;
; **************************************************
	move	(r4)+
	move	x:(r4)-,a
	tst	a
	jeq	L67
	move	#0,y0
	jmp	L68
L67
	move	x1,y0
L68
; **************************************************
;   
;   /* UPA1 */
;   MANT = A1S ? (65536 - ((S->A1 >> 8) + 65280)):
;                (65536 - (S->A1 >> 8));
; **************************************************
	move	#11,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L69
	move	#13,n4
	move	x:(r4+n4),a
	move	a1,a
	rep	#8
	asr	a
	move	#>256,b
	move	a1,a
	jmp	L119
L69
	move	#13,n4
	move	x:(r4+n4),a
	move	a1,a
	rep	#8
	asr	a
	move	#>65536,b
	move	a1,a
L119
; **************************************************
;   EXP = (S->A1 + ((WA2 == 1 ? 0 : (WA1 ? 65344 : 192)) + MANT)) & 65535;
; **************************************************
	sub	a,b	#15,n0
	move	#>1,y1
	move	b1,x:(r0+n0)
	move	x:(r0+n0),x0
	move	#13,n0
	move	x:(r0+n0),b
	cmp	y1,b
	jeq	L71
	move	#12,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L72
	move	#>65344,y1
	jmp	L120
L72
	move	#>192,y1
L120
	tfr	x0,b	#21,n0
	move	y1,x:(r0+n0)
	move	x:(r0+n0),y1
	add	y1,b
	move	b1,x0
L71
; **************************************************
;   
;   /* FMULT */
;   WA2 = f_mult(A2R, S->SR2); 
; **************************************************
	tfr	x0,b	#14,n0
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
	move	#13,n0
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
	move	#14,n0
	move	y1,x:(r4+n4)
	move	x:(r0+n0),b
	move	#>32767,y1
	cmp	y1,b	(r6)-
	jle	L74
	move	#18,n0
	move	x:(r0+n0),y1
	cmp	y1,b
	jgt	L74
	move	x:(r0+n0),x1
	jmp	L75
L74
	move	#19,n0
	move	x:(r0+n0),b
	move	#14,n0
	move	x:(r0+n0),y1
	cmp	y1,b
	jgt	L76
	move	#>32767,y1
	move	x:(r0+n0),b
	cmp	y1,b
	jgt	L76
	move	#19,n0
	move	x:(r0+n0),x1
	jmp	L75
L76
	move	#14,n0
	move	x:(r0+n0),x1
L75
; **************************************************
;   
;   /* TRIGB */
;   S->A1 = S->T ? 0 : AP;
; **************************************************
	move	(r4)+
	move	x:(r4)-,a
	tst	a
	jeq	L78
	move	#0,x0
	jmp	L79
L78
	move	x1,x0
L79
; **************************************************
;   
;   /* FMULT */
;   WA1 = f_mult(S->A1, SR1);
; **************************************************
	move	#10,n0
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
;   {
;     register int mag = MAG << 1;
; **************************************************
; **************************************************
; 
;     for (EXP = 0; mag >>= 1; EXP++)
; **************************************************
	move	#12,n0
	move	#>16383,x0
	move	a1,x:(r0+n0)
	move	#19,n0
	move	x:(r4),y1
	tfr	y1,b	(r6)-
	and	x0,b	#0,y1
	move	b1,b
	asl	b	b1,x:(r0+n0)
	move	#14,n0
	move	b1,b
	move	b1,b
	asr	b	y1,x:(r0+n0)
	move	#20,n0
	move	b1,b
	tst	b	b1,x:(r0+n0)
	move	(r6)-
	jeq	L111
	move	#>1,x0
L82
	move	#14,n0
	move	x:(r0+n0),b
	add	x0,b
	move	b1,x:(r0+n0)
	move	#20,n0
	move	x:(r0+n0),b
	move	b1,b
	asr	b
	move	b1,b
	tst	b	b1,x:(r0+n0)
	jne	L82
L111
; **************************************************
; #ifdef __LOOPCOUNT__ 
;       Loop3++;
; #else
;       ;
; #endif
;   }
;   MANT = MAG ? (MAG << 6) >> EXP : 1 << 5;
; **************************************************
	move	#19,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L84
	rep	#6
	asl	b
	move	#14,n0
	move	b1,b
	move	x:(r0+n0),a
	tst	a
	jeq	L125
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L125
	move	#15,n0
	move	b1,x:(r0+n0)
	jmp	L85
L84
	move	#15,n0
	move	#>32,y1
	move	y1,x:(r0+n0)
L85
; **************************************************
;   DQSEZ = (DQS << 10) + (EXP << 6) + MANT;
; **************************************************
	move	#6,n0
	move	x:(r0+n0),b
	rep	#10
	asl	b
	move	#20,n0
	move	b1,b
	move	b1,x:(r0+n0)
	move	#14,n0
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
	add	y1,b	#8,n0
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
	move	#6,n0
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
	move	#6,n0
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
	move	#6,n0
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
	move	#15,n3
	move	#6,n0
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
	move	#14,n0
	move	(r3)+n3
	move	y1,x:(r0+n0)
	do	#6,L110
L95
; **************************************************
;   {
; #ifdef __LOOPCOUNT__ 
;     Loop4++;
; #endif
;     DQS = MANT ? (BP[EXP] ? 65408 : 128) : 0;
; **************************************************
	move	#15,n0
	move	x:(r0+n0),b
	tst	b
	jeq	L89
	move	x:(r2),a
	tst	a
	jeq	L91
	move	#>65408,y1
	jmp	L121
L91
	move	#>128,y1
L121
	move	#20,n0
	move	y1,x:(r0+n0)
	move	x:(r0+n0),y1
	jmp	L122
L89
	move	#0,y1
L122
; **************************************************
;     DQS += S->B[EXP] & (1 << 15) ? (65536 - ((S->B[EXP] >> 8) + 65280)) :
;                                    (65536 - (S->B[EXP] >> 8));
; **************************************************
	move	#15,n1
	move	#20,n0
	move	r4,b
	move	y1,x:(r0+n0)
	move	x:(r0+n0),y1
	move	#6,n0
	move	y1,x:(r0+n0)
	move	#14,n0
	move	x:(r0+n0),y1
	add	y1,b	#19,n0
	move	b1,r1
	move	x:(r1+n1),y1
	tfr	y1,b	y1,x:(r0+n0)
	move	#>32768,y1
	and	y1,b
	move	b1,b
	tst	b
	jeq	L93
	move	x:(r0+n0),b
	move	b1,b
	rep	#8
	asr	b
	move	b1,b
	move	b1,y1
	move	#>256,b
	jmp	L123
L93
	move	x:(r3),y1
	tfr	y1,b
	move	b1,b
	rep	#8
	asr	b
	move	b1,b
	move	b1,y1
	move	#>65536,b
L123
; **************************************************
;     BP[EXP] = (S->B[EXP] + DQS) & 65535;
; **************************************************
; **************************************************
; **************************************************
	sub	y1,b	#20,n0
	move	b1,x:(r0+n0)
	move	#6,n0
	move	x:(r0+n0),b
	move	#20,n0
	move	x:(r0+n0),y1
	add	y1,b	#6,n0
	move	b1,x:(r0+n0)
	move	#20,n0
	move	x:(r3),y1
	add	y1,b
	and	x1,b
	move	b1,b
	move	b1,y1
	move	b1,x:(r0+n0)
	move	#14,n0
	move	r3,b
	add	x0,b	y1,x:(r2)
	move	b1,r3
	move	r2,b
	add	x0,b
	move	b1,r2
	move	x:(r0+n0),b
	add	x0,b
	move	b1,x:(r0+n0)
L110
; **************************************************
;   }
;   
;   /* TRIGB */
;   if (S->T)
; **************************************************
	move	(r4)+
	move	x:(r4)-,a
	tst	a
	jeq	L96
; **************************************************
;     for (EXP = 0; EXP < 6; EXP++)
; **************************************************
	move	r0,r1
	move	#0,x0
	do	#6,L108
L100
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
L108
	nop
L96
; **************************************************
; #endif
;   
;   /* FMULT */
;   DQI  = f_mult(BP[0], DQSEZ);
; **************************************************
	move	#8,n0
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
	move	#7,n0
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
	move	#11,n0
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
	move	#22,n4
	move	#14,n0
	move	#>65535,x0
	move	a1,x:(r0+n0)
	move	#19,n0
	move	(r6)-
	move	x:(r0+n0),y1
	add	y1,b	#15,n0
	move	(r6)-
	move	x:(r0+n0),y1
	add	y1,b	#11,n0
	move	x:(r0+n0),y1
	add	y1,b	#14,n0
	add	x1,b
	move	x:(r0+n0),y1
	add	y1,b	#6,n0
	and	x0,b
	move	b1,b
	move	b1,x:(r0+n0)
	move	b1,b
	asr	b
	move	b1,b
	move	b1,y1
	move	y1,x:(r4+n4)
	move	#21,n4
	move	x:(r0+n0),b
	move	#13,n0
	move	x:(r0+n0),y1
	add	y1,b	#12,n0
	move	x:(r0+n0),y1
	add	y1,b	#20,n0
	and	x0,b
	move	b1,b
	move	b1,b
	asr	b
	move	b1,b
	move	b1,y1
	move	y1,x:(r4+n4)
	move	#10,n4
	move	y1,x:FS_E
	move	x:(r4+n4),y1
	move	#11,n4
	move	y1,x:(r0+n0)
	move	y1,x:(r4+n4)
; **************************************************
;   S->PK1 = PK0;
; **************************************************
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
	move	#15,n2
	move	#10,n4
	move	#9,n0
	move	r0,r1
	move	r4,r2
	move	x:(r0+n0),y1
	move	#10,n0
	move	#>1,x0
	move	y1,x:(r4+n4)
	move	#12,n4
	move	x:(r0+n0),y1
	move	y1,x:(r4+n4)
	move	#14,n4
	move	(r2)+n2
	move	y0,x:(r4+n4)
	move	y0,x:FA_2
	do	#6,L106
L104
; **************************************************
; #ifdef __LOOPCOUNT__
;   {
;     Loop6++;
;     S->B[EXP] = BP[EXP];
;   }
; #else
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
L106
; **************************************************
; #endif
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
	move	#8,n0
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
	jle	L130
	move	#>53760,a
	cmp	a,b
	jgt	L130
	move	#>1,a
	jmp	L131
L130
	clr	a
L131
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
	jeq	L132
	clr	a
	jmp	L133
L132
	move	x:FTDP,a
L133
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
	jle	L134
	move	#>15872,x1
	move	x1,x:(r0)
	jmp	L135
L134
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
	jeq	L138
	rep	b
	asl	a
	move	a1,a
L138
	move	a1,x:(r0)
L135
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
	jle	L136
	move	#>1,a
	move	(r0)+
	move	x:(r0)-,b
	cmp	a,b
	jeq	L137
L136
	clr	a
L137
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
	jeq	L140
	move	#>15,a
	sub	y0,a
	jmp	L141
L140
	move	x:FI,a
L141
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
	jeq	L142
; **************************************************
;     tmp = tmp + 3840;                                    /* tmp => DIFSX */
; **************************************************
	move	#>3840,a
	add	a,b	(r0)+
	move	b1,x:(r0)-
L142
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
	jeq	L143
; **************************************************
;     tmp = tmp + 16128;                                   /* tmp => DIFSX */
; **************************************************
	move	#>16128,a
	add	a,b	(r0)+
	move	b1,x:(r0)-
L143
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
	jeq	L144
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
L144
; **************************************************
;   FI = ((S->Y > 1535) && (tmp < (S->DML >> 3)) && (TDP == 0)) ? 0 : 1;
; **************************************************
	move	#2,n2
	move	#>1535,b
	move	x:(r2+n2),a
	cmp	b,a
	jle	L145
	move	#26,n2
	move	x:(r2+n2),a
	move	a1,a
	rep	#3
	asr	a
	move	a1,a
	move	(r0)+
	move	x:(r0)-,b
	cmp	a,b
	jge	L145
	move	x:FTDP,a
	tst	a
	jne	L145
	move	#0,y0
	jmp	L152
L145
	move	#>1,y0
L152
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
	jeq	L147
	move	#>896,a
	add	a,b
	move	b1,x:(r0+n0)
L147
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
	jeq	L148
	move	#>256,y0
	jmp	L153
L148
	move	(r0)+
	move	x:(r0)-,y0
L153
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
	jle	L150
	move	#>64,a
	jmp	L151
L150
	move	#27,n2
	move	x:(r2+n2),a
	move	a1,a
	asr	a
	asr	a
	move	a1,a
L151
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
	jeq	L155
	move	#>15,a
	sub	x1,a
	jmp	L166
L155
	move	x:FI,a
L166
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
	jeq	L157
	move	#>4096,a
	add	a,b
	move	b1,x:(r0)
	jmp	L158
L157
	move	#2,n0
	move	x:(r0+n0),x1
	move	x1,x:(r0)
L158
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
	jne	L159
; **************************************************
;     YUP = 5120;
; **************************************************
	move	#>5120,x1
	move	x1,x:(r0)
L159
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
	jeq	L160
; **************************************************
;     YUP = 544;
; **************************************************
	move	#>544,x1
	move	x1,x:(r0)
L160
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
	jeq	L161
; **************************************************
;     TMP = TMP + 507904;
; **************************************************
	move	#>507904,a
	add	a,b
	move	b1,x:(r0+n0)
L161
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
	jeq	L162
	move	#>1,a
	jmp	L163
L162
	clr	a
L163
; **************************************************
;   if (S->LAST_Y)
; **************************************************
	tst	a	#29,n2
	move	a1,x:(r2+n2)
	jeq	L164
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
L164
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
	jeq	L165
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
L165
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
	jle	L169
; **************************************************
;     ExitFlag = TRUE;
; **************************************************
	move	#>1,a
	move	a1,x:FExitFlag
L169
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
L226
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
	jne	L176
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
	tst	a	(r6)-
	jeq	L178
	move	x:(r0+n0),b
	move	#>49152,x0
	or	x0,b
	move	b1,b
	move	b1,x:(r0)
	jmp	L179
L178
	move	x:FSL,y1
	move	y1,x:(r0)
L179
	move	#2,n0
	move	x:FS_E,y1
	tfr	y1,a	y1,x:(r0+n0)
	move	#>16384,y1
	and	y1,a
	move	a1,a
	tst	a
	jeq	L180
	move	x:(r0+n0),b
	move	#>32768,y1
	or	y1,b
	move	b1,b
	move	b1,x:(r0+n0)
	jmp	L181
L180
	move	#2,n0
	move	x:FS_E,y1
	move	y1,x:(r0+n0)
L181
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
	jeq	L182
	move	#>32767,x0
	move	x:(r0+n0),y1
	move	#>65536,a
	sub	y1,a
	and	x0,a
	move	a1,a
	jmp	L183
L182
	move	x:FD,a
L183
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
	jmp	L229
L186
	tfr	y0,b	#>1,y1
	add	y1,b
	move	b1,y0
L229
	move	a1,a
	asr	a
	move	a1,a
	tst	a
	jne	L186
; **************************************************
; #ifdef __LOOPCOUNT__
; 	  Loop7++;
; #else
; 	  ;
; #endif
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
	jeq	L231
	move	a1,a
	rep	b
	asr	a
	move	a1,a
L231
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
	jle	L188
; **************************************************
; 	I = DS ? 0xE : 0x1;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jne	L230
	jmp	L218
L188
; **************************************************
;       else if (DLN > 2047)
; **************************************************
	move	#2,n0
	move	#>2047,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L192
; **************************************************
; 	I = 0xF;
; **************************************************
	move	#>15,a
	jmp	L219
L192
; **************************************************
;       else if (DLN > 399)
; **************************************************
	move	#2,n0
	move	#>399,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L194
; **************************************************
; 	I = DS ? 0x8 : 0x7;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L195
	move	#>8,a
	jmp	L219
L195
	move	#>7,a
	jmp	L219
L194
; **************************************************
;       else if (DLN > 348)
; **************************************************
	move	#2,n0
	move	#>348,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L198
; **************************************************
; 	I = DS ? 0x9 : 0x6;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L199
	move	#>9,a
	jmp	L219
L199
	move	#>6,a
	jmp	L219
L198
; **************************************************
;       else if (DLN > 299)
; **************************************************
	move	#2,n0
	move	#>299,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L202
; **************************************************
; 	I = DS ? 0xA : 0x5;	
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L203
	move	#>10,a
	jmp	L219
L203
	move	#>5,a
	jmp	L219
L202
; **************************************************
;       else if (DLN > 245)
; **************************************************
	move	#2,n0
	move	#>245,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L206
; **************************************************
; 	I = DS ? 0xB : 0x4;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L207
	move	#>11,a
	jmp	L219
L207
	move	#>4,a
	jmp	L219
L206
; **************************************************
;       else if (DLN > 177)
; **************************************************
	move	#2,n0
	move	#>177,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L210
; **************************************************
; 	I = DS ? 0xC : 0x3;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L211
	move	#>12,a
	jmp	L219
L211
	move	#>3,a
	jmp	L219
L210
; **************************************************
;       else if (DLN > 79)
; **************************************************
	move	#2,n0
	move	#>79,a
	move	x:(r0+n0),b
	cmp	a,b
	jle	L214
; **************************************************
; 	I = DS ? 0xD : 0x2;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L215
	move	#>13,a
	jmp	L219
L215
	move	#>2,a
	jmp	L219
L214
; **************************************************
;       else
; 	I = DS ? 0xE : 0x1;
; **************************************************
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L218
L230
	move	#>14,a
	jmp	L219
L218
	move	#>1,a
L219
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
	jeq	L220
	move	x:(r0+n0),b
	cmp	y1,b
	jne	L222
	clr	a
	jmp	L221
L222
	move	#>1,y1
	move	#>16383,x0
	move	x:(r1),a
	eor	x0,a
	move	a1,a
	add	y1,a	#>49152,y1
	and	x0,a
	move	a1,a
	add	y1,a
	jmp	L221
L220
	move	x:(r1),a
L221
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
	tst	a
	jeq	L224
	move	#>32768,y1
	move	x:(r0+n0),a
	or	y1,a
	move	a1,a
	jmp	L225
L224
	move	x:FS_E,a
L225
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
	jeq	L226
L176
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

