;
;	annotate version 1.03
;
; **************************************************
; **************************************************
	section	fmult_c
	opt so,nomd
	page 132,66,3,3
;*** DSP56000/1 Motorola 1.03 GNU 1.37.1
	org	p:
	global	Ff_mult
Ff_mult
; **************************************************
; 
; #define LSHIFT(a, b) ((b) < 0 ? (a) << -(b) : (a) >> (b))
; #define SIGNBIT(a, b) ((a) & (1 << (b)) ? 1 : 0)
;  
; int f_mult(int An, int SRn)
; {
;   register int  EXP;
;   register int  WAnMANT;
;   int           AnS, MAG, AnMANT;
; 
;   AnS = SIGNBIT(An, 15);
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
	jmp	L14
L2
	move	#0,y0
L14
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
	jmp	L15
L4
	move	#65533,n0
	move	x:(r0+n0),b
	move	b1,b
	asr	b
	asr	b
	move	b1,b
L15
; **************************************************
; /*  {
;     register int mag = MAG << 1;
;  
;     for (EXP = 0; mag >>= 1; EXP++)
;         ;
;   } */
; 
;   AnMANT = MAG ? LSHIFT(MAG, EXP - 6) : 1 << 5;
; **************************************************
	move	(r0)+
	move	b1,x:(r0)-
	move	(r0)+
	move	x:(r0)-,b
	tst	b
	jeq	L6
	tfr	x1,b	#>16777210,a
	add	a,b
	tfr	b,a
	tst	a
	jge	L8
	neg	a	(r0)+
	tst	a	x:(r0)-,b
	jeq	L17
	rep	a
	asl	b
	move	b1,b
L17
	move	b1,x:(r0)
	jmp	L7
L8
	tfr	x1,b	#>16777210,a
	add	a,b	(r0)+
	tfr	b,a	x:(r0)-,b
	tst	a
	jeq	L18
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L18
	move	b1,x:(r0)
	jmp	L7
L6
	move	#>32,y0
	move	y0,x:(r0)
L7
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
	jge	L10
	neg	a
	tst	a
	jeq	L19
	rep	a
	asl	b
	move	b1,b
L19
	jmp	L16
L10
	move	#>19,a
	sub	x1,a	(r0)+
	tst	a	x:(r0)-,b
	jeq	L20
	move	b1,b
	rep	a
	asr	b
	move	b1,b
L20
L16
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
	jeq	L12
	move	#>65535,x0
	move	#>65536,a
	move	(r0)+
	move	x:(r0)-,y0
	sub	y0,a
	and	x0,a
	move	a1,a
	jmp	L13
L12
	move	(r0)+
	move	x:(r0)-,a
L13
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


	endsec

; 

