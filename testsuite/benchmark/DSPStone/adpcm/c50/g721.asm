*******************************************************
*  TMS320C2x/C5x ANSI C Codegen    Version 6.40       *
*******************************************************
;	dspac -v50 -d__TMS320C50__ g721.c g721.if 
;	dspopt -v50 -s -O2 g721.if g721.opt 
;	dspcg -o -n -v50 g721.opt g721.asm g721.tmp 
	.mmregs
	.file	"g721.c"
	.file	"g721.h"
	.globl	_LAW
	.globl	_reset_encoder
	.globl	_encoder
	.globl	_reset_decoder
	.globl	_decoder
	.file	"g721.c"

	.stag	.fake0,416
	.member	_B,0,62,8,128,,8
	.member	_DQ,128,62,8,128,,8
	.member	_PK1,256,14,8,16
	.member	_PK2,272,14,8,16
	.member	_AP,288,14,8,16
	.member	_DMS,304,14,8,16
	.member	_DML,320,14,8,16
	.member	_YU,336,14,8,16
	.member	_TD,352,14,8,16
	.member	_YL6,368,14,8,16
	.member	_YL,384,15,8,32
	.eos
	.sym	_STATES,0,8,13,416,.fake0

	.sect	".cinit"
	.word	IS1,_A_LAW_table
	.word	4784
	.word	4752
	.word	4848
	.word	4816
	.word	4656
	.word	4624
	.word	4720
	.word	4688
	.word	5040
	.word	5008
	.word	5104
	.word	5072
	.word	4912
	.word	4880
	.word	4976
	.word	4944
	.word	4440
	.word	4424
	.word	4472
	.word	4456
	.word	4376
	.word	4360
	.word	4408
	.word	4392
	.word	4568
	.word	4552
	.word	4600
	.word	4584
	.word	4504
	.word	4488
	.word	4536
	.word	4520
	.word	6848
	.word	6720
	.word	7104
	.word	6976
	.word	6336
	.word	6208
	.word	6592
	.word	6464
	.word	7872
	.word	7744
	.word	8128
	.word	8000
	.word	7360
	.word	7232
	.word	7616
	.word	7488
	.word	5472
	.word	5408
	.word	5600
	.word	5536
	.word	5216
	.word	5152
	.word	5344
	.word	5280
	.word	5984
	.word	5920
	.word	6112
	.word	6048
	.word	5728
	.word	5664
	.word	5856
	.word	5792
	.word	4139
	.word	4137
	.word	4143
	.word	4141
	.word	4131
	.word	4129
	.word	4135
	.word	4133
	.word	4155
	.word	4153
	.word	4159
	.word	4157
	.word	4147
	.word	4145
	.word	4151
	.word	4149
	.word	4107
	.word	4105
	.word	4111
	.word	4109
	.word	4099
	.word	4097
	.word	4103
	.word	4101
	.word	4123
	.word	4121
	.word	4127
	.word	4125
	.word	4115
	.word	4113
	.word	4119
	.word	4117
	.word	4268
	.word	4260
	.word	4284
	.word	4276
	.word	4236
	.word	4228
	.word	4252
	.word	4244
	.word	4332
	.word	4324
	.word	4348
	.word	4340
	.word	4300
	.word	4292
	.word	4316
	.word	4308
	.word	4182
	.word	4178
	.word	4190
	.word	4186
	.word	4166
	.word	4162
	.word	4174
	.word	4170
	.word	4214
	.word	4210
	.word	4222
	.word	4218
	.word	4198
	.word	4194
	.word	4206
	.word	4202
	.word	688
	.word	656
	.word	752
	.word	720
	.word	560
	.word	528
	.word	624
	.word	592
	.word	944
	.word	912
	.word	1008
	.word	976
	.word	816
	.word	784
	.word	880
	.word	848
	.word	344
	.word	328
	.word	376
	.word	360
	.word	280
	.word	264
	.word	312
	.word	296
	.word	472
	.word	456
	.word	504
	.word	488
	.word	408
	.word	392
	.word	440
	.word	424
	.word	2752
	.word	2624
	.word	3008
	.word	2880
	.word	2240
	.word	2112
	.word	2496
	.word	2368
	.word	3776
	.word	3648
	.word	4032
	.word	3904
	.word	3264
	.word	3136
	.word	3520
	.word	3392
	.word	1376
	.word	1312
	.word	1504
	.word	1440
	.word	1120
	.word	1056
	.word	1248
	.word	1184
	.word	1888
	.word	1824
	.word	2016
	.word	1952
	.word	1632
	.word	1568
	.word	1760
	.word	1696
	.word	43
	.word	41
	.word	47
	.word	45
	.word	35
	.word	33
	.word	39
	.word	37
	.word	59
	.word	57
	.word	63
	.word	61
	.word	51
	.word	49
	.word	55
	.word	53
	.word	11
	.word	9
	.word	15
	.word	13
	.word	3
	.word	1
	.word	7
	.word	5
	.word	27
	.word	25
	.word	31
	.word	29
	.word	19
	.word	17
	.word	23
	.word	21
	.word	172
	.word	164
	.word	188
	.word	180
	.word	140
	.word	132
	.word	156
	.word	148
	.word	236
	.word	228
	.word	252
	.word	244
	.word	204
	.word	196
	.word	220
	.word	212
	.word	86
	.word	82
	.word	94
	.word	90
	.word	70
	.word	66
	.word	78
	.word	74
	.word	118
	.word	114
	.word	126
	.word	122
	.word	102
	.word	98
	.word	110
	.word	106
IS1	.set	256
	.text

	.sym	_A_LAW_table,_A_LAW_table,62,3,4096,,256
	.bss	_A_LAW_table,256,1

	.sect	".cinit"
	.word	IS2,_u_LAW_table
	.word	16223
	.word	15967
	.word	15711
	.word	15455
	.word	15199
	.word	14943
	.word	14687
	.word	14431
	.word	14175
	.word	13919
	.word	13663
	.word	13407
	.word	13151
	.word	12895
	.word	12639
	.word	12383
	.word	12191
	.word	12063
	.word	11935
	.word	11807
	.word	11679
	.word	11551
	.word	11423
	.word	11295
	.word	11167
	.word	11039
	.word	10911
	.word	10783
	.word	10655
	.word	10527
	.word	10399
	.word	10271
	.word	10175
	.word	10111
	.word	10047
	.word	9983
	.word	9919
	.word	9855
	.word	9791
	.word	9727
	.word	9663
	.word	9599
	.word	9535
	.word	9471
	.word	9407
	.word	9343
	.word	9279
	.word	9215
	.word	9167
	.word	9135
	.word	9103
	.word	9071
	.word	9039
	.word	9007
	.word	8975
	.word	8943
	.word	8911
	.word	8879
	.word	8847
	.word	8815
	.word	8783
	.word	8751
	.word	8719
	.word	8687
	.word	8663
	.word	8647
	.word	8631
	.word	8615
	.word	8599
	.word	8583
	.word	8567
	.word	8551
	.word	8535
	.word	8519
	.word	8503
	.word	8487
	.word	8471
	.word	8455
	.word	8439
	.word	8423
	.word	8411
	.word	8403
	.word	8395
	.word	8387
	.word	8379
	.word	8371
	.word	8363
	.word	8355
	.word	8347
	.word	8339
	.word	8331
	.word	8323
	.word	8315
	.word	8307
	.word	8299
	.word	8291
	.word	8285
	.word	8281
	.word	8277
	.word	8273
	.word	8269
	.word	8265
	.word	8261
	.word	8257
	.word	8253
	.word	8249
	.word	8245
	.word	8241
	.word	8237
	.word	8233
	.word	8229
	.word	8225
	.word	8222
	.word	8220
	.word	8218
	.word	8216
	.word	8214
	.word	8212
	.word	8210
	.word	8208
	.word	8206
	.word	8204
	.word	8202
	.word	8200
	.word	8198
	.word	8196
	.word	8194
	.word	0
	.word	8031
	.word	7775
	.word	7519
	.word	7263
	.word	7007
	.word	6751
	.word	6495
	.word	6239
	.word	5983
	.word	5727
	.word	5471
	.word	5215
	.word	4959
	.word	4703
	.word	4447
	.word	4191
	.word	3999
	.word	3871
	.word	3743
	.word	3615
	.word	3487
	.word	3359
	.word	3231
	.word	3103
	.word	2975
	.word	2847
	.word	2719
	.word	2591
	.word	2463
	.word	2335
	.word	2207
	.word	2079
	.word	1983
	.word	1919
	.word	1855
	.word	1791
	.word	1727
	.word	1663
	.word	1599
	.word	1535
	.word	1471
	.word	1407
	.word	1343
	.word	1279
	.word	1215
	.word	1151
	.word	1087
	.word	1023
	.word	975
	.word	943
	.word	911
	.word	879
	.word	847
	.word	815
	.word	783
	.word	751
	.word	719
	.word	687
	.word	655
	.word	623
	.word	591
	.word	559
	.word	527
	.word	495
	.word	471
	.word	455
	.word	439
	.word	423
	.word	407
	.word	391
	.word	375
	.word	359
	.word	343
	.word	327
	.word	311
	.word	295
	.word	279
	.word	263
	.word	247
	.word	231
	.word	219
	.word	211
	.word	203
	.word	195
	.word	187
	.word	179
	.word	171
	.word	163
	.word	155
	.word	147
	.word	139
	.word	131
	.word	123
	.word	115
	.word	107
	.word	99
	.word	93
	.word	89
	.word	85
	.word	81
	.word	77
	.word	73
	.word	69
	.word	65
	.word	61
	.word	57
	.word	53
	.word	49
	.word	45
	.word	41
	.word	37
	.word	33
	.word	30
	.word	28
	.word	26
	.word	24
	.word	22
	.word	20
	.word	18
	.word	16
	.word	14
	.word	12
	.word	10
	.word	8
	.word	6
	.word	4
	.word	2
	.word	0
IS2	.set	256
	.text

	.sym	_u_LAW_table,_u_LAW_table,62,3,4096,,256
	.bss	_u_LAW_table,256

	.sym	_adapt_quant,_adapt_quant,32,3,0

	.func	147
******************************************************
* FUNCTION DEF : _adapt_quant
******************************************************
_adapt_quant:

LF1	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,5
	LAR	AR0,*0+,AR2

	RSXM
*
*  ---  'tmp' shares AUTO storage with 'DLN'
*  ---  'tmp' shares AUTO storage with 'S$1'
*
*** 17	-----------------------    DQM = (DS = D&32768u) ? -D&32767u : D;
	.sym	_DS,1,14,1,16
	.sym	_DQM,2,14,1,16
	.sym	_tmp,3,14,1,16
	.sym	_EXP,4,4,1,16
	.line	17
	LDPK	_D
	ZALS	_D
	ANDK	32768
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	ANDK	0FFFFh
	BZ	LL3
	LAC	_D
	NEG
	BD	LL4
	ANDK	32767
***	B	LL4 OCCURS
LL3:
	LAC	_D
LL4:
	MAR	*+
	SACL	* 
*** 18	-----------------------    tmp = DQM>>1;
	.line	18
	RSXM
	LAC	*+,14
	SACH	*+,1
*** 19	-----------------------    EXP = 0;
	.line	19
	LACK	0
	SACL	*-
*** 19	-----------------------    if ( !tmp ) goto g3;
	LAC	* 
	BZ	L3
	MAR	*+
L2:
***	-----------------------g2:
*** 19	-----------------------    ++EXP;
	LAC	* 
	ADDK	1
	SACL	*-
*** 19	-----------------------    if ( tmp >>= 1 ) goto g2;
	LAC	* ,14
	SACH	*+,1
	ANDK	0FFFFh,15
	SFL
	BNZ	L2
L3:
***	-----------------------g3:
*** 20	-----------------------    S$1 = (EXP < 7) ? DQM<<7-EXP : DQM>>EXP-7;
	.line	20
	SSXM
	LARK	AR2,4
	MAR	*0+
	LAC	* 
	SUBK	7
	BGEZ	LL5
	LACK	7
	SUB	* ,AR0
	SACL	* 
	LT	* ,AR2
	SBRK	2
	LACT	* 
	BD	LL6
	ANDK	0FFFFh
***	B	LL6 OCCURS
LL5:
	LAC	* ,AR0
	SUBK	7
	SACL	* 
	LT	* ,AR2
	SBRK	2
	ZALS	* 
	SATL
	ANDK	0FFFFh
LL6:
	MAR	*+
	SACL	* 
*** 20	-----------------------    I = ((DLN = (unsigned)(EXP*128)+(S$1&127u)-(Y>>2)&4095u) > 299u) ? ((DLN > 2047u) ? ((DLN > 39...
	ZALS	*+
	ANDK	127
	SACB
	LAC	*-,7,AR1
	ADDB
	SACL	* ,AR0
	RSXM
	LDPK	_Y
	LAC	_Y,13
	SACH	* ,1,AR1
	ZALS	* ,AR0
	SUBS	* ,AR2
	ANDK	4095
	SACL	* 
	ANDK	0FFFFh
	SUBK	299
	BLEZ	LL7
	ZALS	* 
	SUBK	2047
	BLEZ	LL8
	ZALS	* 
	SUBK	3971
	BLEZ	LL9
	SBRK	2
	LAC	* 
	BZ	LL10
	BD	LL11
	LACK	14
	NOP
***	B	LL11 OCCURS
LL10:
	LACK	1
LL11:
	B	LL12
LL9:
	LACK	15
LL12:
	B	LL13
LL8:
	ZALS	* 
	SUBK	399
	BLEZ	LL14
	SBRK	2
	LAC	* 
	BZ	LL15
	BD	LL16
	LACK	8
	NOP
***	B	LL16 OCCURS
LL15:
	LACK	7
LL16:
	B	LL17
LL14:
	ZALS	* 
	SUBK	348
	BLEZ	LL18
	SBRK	2
	LAC	* 
	BZ	LL19
	BD	LL20
	LACK	9
	NOP
***	B	LL20 OCCURS
LL19:
	LACK	6
LL20:
	B	LL21
LL18:
	SBRK	2
	LAC	* 
	BZ	LL22
	BD	LL23
	LACK	10
	NOP
***	B	LL23 OCCURS
LL22:
	LACK	5
LL23:
LL21:
LL17:
LL13:
	B	LL24
LL7:
	ZALS	* 
	SUBK	177
	BLEZ	LL25
	ZALS	* 
	SUBK	245
	BLEZ	LL26
	SBRK	2
	LAC	* 
	BZ	LL27
	BD	LL28
	LACK	11
	NOP
***	B	LL28 OCCURS
LL27:
	LACK	4
LL28:
	B	LL29
LL26:
	SBRK	2
	LAC	* 
	BZ	LL30
	BD	LL31
	LACK	12
	NOP
***	B	LL31 OCCURS
LL30:
	LACK	3
LL31:
LL29:
	B	LL32
LL25:
	ZALS	* 
	SUBK	79
	BLEZ	LL33
	SBRK	2
	LAC	* 
	BZ	LL34
	BD	LL35
	LACK	13
	NOP
***	B	LL35 OCCURS
LL34:
	LACK	2
LL35:
	B	LL36
LL33:
	SBRK	2
	LAC	* 
	BZ	LL37
	BD	LL38
	LACK	14
	NOP
***	B	LL38 OCCURS
LL37:
	LACK	1
LL38:
LL36:
LL32:
LL24:
	SACL	_I
*** 30	-----------------------    return;
	.line	30
EPI0_1:
	.line	52
	MAR	* ,AR1
	RETD
	SBRK	6
	LAR	AR0,*
***	RET OCCURS

	.endfunc	198,000000000H,5

	.sym	_adpt_predict_1,_adpt_predict_1,32,3,0

	.func	200
******************************************************
* FUNCTION DEF : _adpt_predict_1
******************************************************
_adpt_predict_1:

LF2	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,11
	LAR	AR0,*0+
;	<save register vars>
	SAR	AR6,*+
	SAR	AR7,*+,AR2

*
* AR5	assigned to variable 'tmp1'
* AR6	assigned to variable 'tmp2'
* AR7	assigned to temp var 'L$3'
*  ---  'tmp' shares AUTO storage with 'WAnMANT'
*  ---  'tmp' shares AUTO storage with 'WAnMAG'
*  ---  'tmp' shares AUTO storage with 'AnMANT'
*  ---  'WAnEXP' shares AUTO storage with 'AnEXP'
*  ---  'tmp' shares AUTO storage with 'S$1'
*  ---  'AnMAG' shares AUTO storage with 'C$5'
*
*** 11	-----------------------    x = X;
	.sym	S$2,1,15,1,32
	.sym	C$4,3,4,1,16
	.sym	_WAnEXP,4,4,1,16
	.sym	_tmp,5,14,1,16
	.sym	_AnMAG,6,14,1,16
	.sym	_x,7,24,1,16,.fake0
	.sym	_SEZI,8,14,1,16
	.sym	_i,9,4,1,16
	.sym	_AnS,10,14,1,16
	.line	11
	LDPK	_X
	LAC	_X
	LARK	AR2,7
	MAR	*0+
	SACL	*+
*** 16	-----------------------    SEZI = 0u;
	.line	16
	LACK	0
	SACL	*-
*** 18	-----------------------    tmp1 = &x->B;
	.line	18
	LAR	AR5,* 
*** 19	-----------------------    tmp2 = &x->DQ;
	.line	19
	LAC	* ,AR0
	ADDK	8
	SACL	* 
	LAR	AR6,* ,AR2
*** 21	-----------------------    i = 0;
	.line	21
	LACK	0
	ADRK	2
	SACL	* ,AR5
***  	-----------------------    L$3 = 7;
	LARK	AR7,7
***	-----------------------g2:
***	-----------------------g22:
L15:
*** 29	-----------------------    AnMAG = (AnS = (unsigned)((*tmp1&32768u) != 0u)) ? -(*tmp1++>>2)&8191u : *tmp1++>>2;
	.line	29
	ZALS	* 
	ANDK	32768
	LARK	AR4,1
	BNZ	LL42
	LARK	AR4,0
LL42:
	MAR	* ,AR2
	LARK	AR2,10
	MAR	*0+
	SAR	AR4,* ,AR4
	BANZ	$+4,*
	B	LL41
	RSXM
	MAR	* ,AR5
	LAC	*+,13,AR1
	SACH	* ,1
	LAC	* 
	NEG
	BD	LL43
	ANDK	8191
***	B	LL43 OCCURS
LL41:
	RSXM
	MAR	* ,AR5
	LAC	*+,13,AR1
	SACH	* ,1
	LAC	* 
LL43:
	MAR	* ,AR2
	SBRK	4
	SACL	*-
*** 29	-----------------------    tmp = AnMAG;
	SACL	*-
*** 31	-----------------------    AnEXP = 0;
	.line	31
	LACK	0
	SACL	*+
*** 31	-----------------------    if ( !tmp ) goto g6;
	LAC	* 
	BZ	L9
	MAR	*-
L7:
***	-----------------------g4:
*** 31	-----------------------    ++AnEXP;
	LAC	* 
	ADDK	1
	SACL	*+
*** 31	-----------------------    if ( tmp >>= 1 ) goto g4;
	LAC	* ,14
	SACH	*-,1
	ANDK	0FFFFh,15
	SFL
	BNZ	L7
*** 32	-----------------------    if ( AnMAG ) goto g7;
	.line	32
	ADRK	2
	LAC	* 
	ANDK	0FFFFh
	BNZ	L10
L9:
***	-----------------------g6:
*** 32	-----------------------    AnMANT = 32u;
	LACK	32
	LARK	AR2,5
	BD	L11
	MAR	*0+
	SACL	* 
*** 32	-----------------------    goto g8;
***	B	L11 OCCURS
L10:
***	-----------------------g7:
*** 32	-----------------------    AnMANT = (AnEXP < 6) ? AnMAG<<6-AnEXP : AnMAG>>AnEXP-6;
	SSXM
	SBRK	2
	LAC	* 
	SUBK	6
	BGEZ	LL44
	LACK	6
	SUB	* ,AR0
	SACL	* 
	LT	* ,AR2
	ADRK	2
	LACT	* 
	BD	LL45
	ANDK	0FFFFh
***	B	LL45 OCCURS
LL44:
	LAC	* ,AR0
	SUBK	6
	SACL	* 
	LT	* ,AR2
	ADRK	2
	ZALS	* 
	SATL
	ANDK	0FFFFh
LL45:
	MAR	*-
	SACL	* 
L11:
***	-----------------------g8:
*** 33	-----------------------    C$5 = *tmp2;
	.line	33
	MAR	*+,AR6
	LAC	* ,AR2
	SACL	* 
*** 33	-----------------------    WAnMANT = (C$5&63u)*AnMANT+48u>>4;
	ZALS	*-,AR0
	ANDK	63
	SACL	* 
	LT	* ,AR2
	MPYU	* ,AR0
	PAC
	ADDK	48
	SACL	* 
	RSXM
	LAC	* ,11,AR2
	SACH	*+,1
*** 34	-----------------------    WAnEXP = (int)((C$5>>6&15u)+(unsigned)AnEXP);
	.line	34
	LAC	* ,9
	ANDK	15,15
	SBRK	2
	ADD	* ,15
	SACH	* ,1
*** 35	-----------------------    S$2 = ((C$4 = 19-WAnEXP) < 0) ? (unsigned long)WAnMANT<<WAnEXP-19 : (unsigned long)WAnMANT>>C$4;
	.line	35
	LACK	19
	SSXM
	SUB	*-
	SACL	* 
	BGEZ	LL46
	MAR	*+
	LAC	*+
	SUBK	19
	SACB
	ZALS	* ,AR1
	SACL	*+
	SACH	*+
	CALLD	L$$SL
	LACB
	SACL	* 
***	CALL	L$$SL OCCURS
	B	LL47
LL46:
	ADRK	2
	ZALS	* 
	SBRK	2
	LT	* 
	SATH
	SATL
LL47:
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
	SACH	*-
*** 35	-----------------------    WAnMAG = (unsigned)(S$2&32767uL);
	LAC	* 
	ANDK	32767
	ADRK	4
	SACL	* ,AR6
*** 36	-----------------------    S$1 = ((unsigned)((*tmp2++&1024u) != 0u)^AnS) ? -WAnMAG : WAnMAG;
	.line	36
	ZALS	*+
	ANDK	1024
	LARK	AR3,1
	BNZ	LL49
	LARK	AR3,0
LL49:
	MAR	* ,AR0
	SAR	AR3,*
	ZALS	* ,AR2
	ADRK	5
	XOR	* 
	ANDK	0FFFFh
	BZ	LL48
	SBRK	5
	BD	LL50
	LAC	* 
	NEG
***	B	LL50 OCCURS
LL48:
	SBRK	5
	LAC	* 
LL50:
	SACL	* 
*** 36	-----------------------    SEZI += S$1;
	ADRK	3
	ADD	* 
	SACL	*+
*** 38	-----------------------    if ( i != 5 ) goto g10;
	.line	38
	LAC	* 
	SUBK	5
	BNZ	L13
*** 39	-----------------------    SEZ = (SEZI&65535u)>>1;
	.line	39
	MAR	*-
	ZALS	* ,AR0
	ANDK	65535
	SACL	* 
	RSXM
	LAC	* ,14
	LDPK	_SEZ
	SACH	_SEZ,1
L13:
***	-----------------------g10:
*** 21	-----------------------    ++i;
	.line	21
	MAR	* ,AR2
	LARK	AR2,9
	MAR	*0+
	LAC	* 
	ADDK	1
	SACL	* ,AR7
*** 21	-----------------------    if ( --L$3 >= 0 ) goto g12;
	BANZ	L15,*-,AR5
*** 42	-----------------------    SE = (SEZI&65535u)>>1;
	.line	42
	MAR	* ,AR2
	MAR	*-
	ZALS	* ,AR0
	ANDK	65535
	SACL	* 
	RSXM
	LAC	* ,14
	LDPK	_SE
	SACH	_SE,1
***  	-----------------------    return;
EPI0_2:
	.line	43
	MAR	* ,AR1
;	<restore register vars>
	MAR	*-
	LAR	AR7,*-
	LAR	AR6,* 
	RETD
	SBRK	12
	LAR	AR0,*
***	RET OCCURS

	.endfunc	242,0000000c0H,11

	.sym	_adpt_predict_2,_adpt_predict_2,32,3,0

	.func	244
******************************************************
* FUNCTION DEF : _adpt_predict_2
******************************************************
_adpt_predict_2:

LF3	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,21
	LAR	AR0,*0+
;	<save register vars>
	SAR	AR6,*+
	SAR	AR7,*+,AR2

*
* AR5	assigned to variable 'tmp1'
* AR5	assigned to temp var 'U$172'
* AR5	assigned to variable 'a2'
* AR5	assigned to temp var 'C$19'
* AR6	assigned to variable 'tmp2'
* AR6	assigned to temp var 'K$171'
* AR7	assigned to temp var 'A$24'
* BRCR	assigned to temp var 'L$16'
* BRCR	assigned to temp var 'L$17'
* BRCR	assigned to temp var 'L$18'
*  ---  'DQM' shares AUTO storage with 'tmp'
*  ---  'MAG' shares AUTO storage with 'DQS'
*  ---  'DQM' shares AUTO storage with 'A1LL'
*  ---  'MAG' shares AUTO storage with 'A1UL'
*  ---  'DQM' shares AUTO storage with 'PKS'
*  ---  'DQM' shares AUTO storage with 'ULA2'
*  ---  'MAG' shares AUTO storage with 'UGA2'
*  ---  'MAG' shares AUTO storage with 'UGA2B'
*  ---  'DQM' shares AUTO storage with 'PKS1'
*  ---  'DQM' shares AUTO storage with 'tmp'
*  ---  'MAG' shares AUTO storage with 'MAG'
*  ---  'EXP' shares AUTO storage with 'EXP'
*  ---  'SRS' shares AUTO storage with 'SR0'
*  ---  'DQM' shares AUTO storage with 'A2T'
*  ---  'DQSEZ' shares AUTO storage with 'A1T'
*  ---  'tmp' shares AUTO storage with 'a1'
*  ---  'DQM' shares AUTO storage with 'S$15'
*  ---  'MAG' shares AUTO storage with 'S$14'
*  ---  'DQM' shares AUTO storage with 'S$13'
*  ---  'MAG' shares AUTO storage with 'S$12'
*  ---  'DQM' shares AUTO storage with 'S$11'
*  ---  'FA1' shares AUTO storage with 'S$9'
*  ---  'DQM' shares AUTO storage with 'S$7'
*  ---  'MAG' shares AUTO storage with 'S$6'
*  ---  'DQSEZ' shares AUTO storage with 'S$5'
*  ---  'DQSEZ' shares AUTO storage with 'S$4'
*  ---  'S$3' shares AUTO storage with 'S$2'
*  ---  'S$10' shares AUTO storage with 'S$1'
*  ---  'MAG' shares AUTO storage with 'C$23'
*  ---  'S$3' shares AUTO storage with 'C$22'
*  ---  'S$3' shares AUTO storage with 'C$21'
*  ---  'tmp' shares AUTO storage with 'C$20'
*
*** 11	-----------------------    x = X;
	.sym	_SRS,1,14,1,16
	.sym	S$8,2,15,1,32
	.sym	_FA1,4,15,1,32
	.sym	U$12,6,14,1,16
	.sym	_PK0,7,14,1,16
	.sym	_x,8,24,1,16,.fake0
	.sym	_tmp,9,30,1,16
	.sym	U$3,10,30,1,16
	.sym	U$17,11,14,1,16
	.sym	_MAG,12,14,1,16
	.sym	S$3,13,14,1,16
	.sym	_DQSEZ,14,14,1,16
	.sym	_DQM,15,14,1,16
	.sym	_EXP,16,4,1,16
	.sym	S$10,17,4,1,16
	.line	11
	LDPK	_X
	LAC	_X
	LARK	AR2,8
	MAR	*0+
	SACL	* 
*** 12	-----------------------    a1 = (U$3 = &x->B[0])+6;
	.line	12
	ADRK	2
	SACL	*-
	ADDK	6
	SACL	* ,AR0
*** 13	-----------------------    a2 = a1+1;
	.line	13
	ADDK	1
	SACL	* 
	LAR	AR5,* ,AR2
*** 19	-----------------------    if ( U$12 = DQ&16384u ) goto g2;
	.line	19
	ZALS	_DQ
	ANDK	16384
	SBRK	3
	SACL	* 
	ANDK	0FFFFh
	BNZ	L17
*** 19	-----------------------    S$14 = DQ;
	LAC	_DQ
	ADRK	6
	SACL	*-
***  	-----------------------    U$17 = S$14&16383u;
	ANDK	16383
	SACL	* 
*** 19	-----------------------    if ( !(SEZ&16384u) ) goto g3;
	ZALS	_SEZ
	ANDK	16384
	ANDK	0FFFFh
	BZ	L18
*** 19	-----------------------    goto g4;
	B	L19
L17:
***	-----------------------g2:
*** 19	-----------------------    S$14 = -(U$17 = DQ&16383u);
	LAC	_DQ
	ANDK	16383
	ADRK	5
	SACL	*+
	NEG
	SACL	* 
*** 19	-----------------------    if ( SEZ&16384u ) goto g4;
	ZALS	_SEZ
	ANDK	16384
	ANDK	0FFFFh
	BNZ	L19
L18:
***	-----------------------g3:
*** 19	-----------------------    S$15 = SEZ;
	LAC	_SEZ
	LARK	AR2,15
	BD	L20
	MAR	*0+
	SACL	* 
*** 19	-----------------------    goto g5;
***	B	L20 OCCURS
L19:
***	-----------------------g4:
*** 19	-----------------------    S$15 = SEZ+32768u;
	LAC	_SEZ
	ADDK	-32768
	LARK	AR2,15
	MAR	*0+
	SACL	* 
L20:
***	-----------------------g5:
*** 19	-----------------------    DQSEZ = S$14+S$15&65535u;
	LAC	* 
	SBRK	3
	ADD	* 
	ANDK	65535
	ADRK	2
	SACL	* 
*** 22	-----------------------    PK0 = (unsigned)((DQSEZ&32768u) != 0u);
	.line	22
	ZALS	* 
	ANDK	32768
	LARK	AR4,1
	BNZ	LL53
	LARK	AR4,0
LL53:
	SBRK	7
	SAR	AR4,*-
*** 26	-----------------------    S$12 = U$12 ? -U$17 : DQ;
	.line	26
	LAC	* 
	BZ	LL54
	ADRK	5
	BD	LL55
	LAC	* 
	NEG
***	B	LL55 OCCURS
LL54:
	LAC	_DQ
LL55:
	LARK	AR2,12
	MAR	*0+
	SACL	* 
*** 26	-----------------------    S$13 = (SE&16384u) ? SE+32768u : SE;
	ZALS	_SE
	ANDK	16384
	ANDK	0FFFFh
	BZ	LL56
	LAC	_SE
	BD	LL57
	ADDK	-32768
***	B	LL57 OCCURS
LL56:
	LAC	_SE
LL57:
	ADRK	3
	SACL	* 
*** 26	-----------------------    C$23 = S$12+S$13&65535u;
	SBRK	3
	ADD	* 
	ANDK	65535
	SACL	* 
*** 26	-----------------------    SR = C$23;
	SACL	_SR
*** 35	-----------------------    MAG = (SRS = C$23&32768u) ? -SR&32767u : C$23;
	.line	35
	ZALS	* 
	ANDK	32768
	SBRK	11
	SACL	* 
	ANDK	0FFFFh
	BZ	LL58
	LAC	_SR
	NEG
	BD	LL59
	ANDK	32767
***	B	LL59 OCCURS
LL58:
	ADRK	11
	LAC	* 
LL59:
	LARK	AR2,12
	MAR	*0+
	SACL	* 
*** 36	-----------------------    tmp = MAG;
	.line	36
	ADRK	3
	SACL	*+
*** 37	-----------------------    EXP = 0;
	.line	37
	LACK	0
	SACL	*-
*** 37	-----------------------    if ( !tmp ) goto g8;
	LAC	* 
	BZ	L23
	MAR	*+
	RSXM ;;;
L22:
***	-----------------------g7:
*** 37	-----------------------    ++EXP;
	LAC	* 
	ADDK	1
	SACL	*-
*** 37	-----------------------    if ( tmp >>= 1 ) goto g7;
	LAC	* ,14
	SACH	*+,1
	ANDK	0FFFFh,15
	SFL
	BNZ	L22
L23:
***	-----------------------g8:
*** 38	-----------------------    S$10 = SRS ? 1024 : 0;
	.line	38
	LARK	AR2,1
	MAR	*0+
	LAC	* 
	BZ	LL60
	BD	LL61
	LACK	1024
***	B	LL61 OCCURS
LL60:
	LACK	0
LL61:
	ADRK	16
	SACL	* 
*** 38	-----------------------    S$11 = MAG ? ((EXP < 6) ? MAG<<6-EXP : MAG>>EXP-6) : 32u;
	SBRK	5
	LAC	* 
	BZ	LL62
	SSXM
	ADRK	4
	LAC	* 
	SUBK	6
	BGEZ	LL63
	LACK	6
	SUB	* ,AR0
	SACL	* 
	LT	* ,AR2
	SBRK	4
	LACT	* 
	BD	LL64
	ANDK	0FFFFh
***	B	LL64 OCCURS
LL63:
	LAC	* ,AR0
	SUBK	6
	SACL	* 
	LT	* ,AR2
	SBRK	4
	ZALS	* 
	SATL
	ANDK	0FFFFh
LL64:
	B	LL65
LL62:
	LACK	32
LL65:
	ADRK	3
	SACL	*+
*** 38	-----------------------    SR0 = (unsigned)(EXP*64+S$10)+S$11;
	LAC	*+,6
	ADD	* 
	SBRK	2
	ADD	* 
	SBRK	14
	SACL	* 
*** 47	-----------------------    PKS1 = x->PK1^PK0;
	.line	47
	ADRK	7
	LAR	AR3,*-
	LAC	* ,AR3
	ADRK	16
	XOR	* ,AR2
	ADRK	8
	SACL	* 
*** 48	-----------------------    C$22 = *a1;
	.line	48
	SBRK	6
	LAR	AR4,* ,AR4
	LAC	* ,AR2
	ADRK	4
	SACL	* 
*** 48	-----------------------    FA1 = (C$22&32768u) ? ((*a1 >= 57345u) ? (unsigned long)*a1*4uL&131071uL : 98308uL) : (C$22 <=...
	ZALS	* 
	ANDK	32768
	ANDK	0FFFFh
	BZ	LL66
	SBRK	4
	LAR	AR3,* ,AR3
	ZALS	* 
	RSXM
	SUBK	-8191
	BLZ	LL67
	MAR	* ,AR2
	LAR	AR4,* ,AR4
	ZALS	* ,AR1
	SACL	*+
	SACH	*+
	CALLD	L$$SL
	LACK	2
	SACL	* 
***	CALL	L$$SL OCCURS
	MAR	* ,AR2
	ADRK	10
	SACH	*-
	ANDK	65535
	SACL	*+
	ZALS	* 
	ANDK	1
	SACL	*-
	BD	LL69
	ZALS	*+
	ADDH	* 
***	B	LL69 OCCURS
LL67:
	LALK	3,15
	BD	LL69
	ORK	4
***	B	LL69 OCCURS
LL66:
	ZALS	* 
	SUBK	8191
	BGZ	LL70
	SBRK	4
	LAR	AR3,* ,AR3
	LAC	* ,2,AR2
	ADRK	11
	BD	LL69
	SACL	* 
	ZALS	* 
***	B	LL69 OCCURS
LL70:
	LACK	32764
LL69:
	MAR	* ,AR2
	LARK	AR2,4
	MAR	*0+
	SACL	*+
	SACH	* 
*** 49	-----------------------    S$8 = (x->PK2^PK0) ? 114688uL : 16384uL;
	.line	49
	ADRK	3
	LAR	AR4,*-
	ZALS	* ,AR4
	ADRK	17
	XOR	* 
	ANDK	0FFFFh
	BZ	LL72
	LALK	3,15
	BD	LL73
	ORK	16384
***	B	LL73 OCCURS
LL72:
	LACK	16384
LL73:
	MAR	* ,AR2
	SBRK	5
	SACL	*+
	SACH	* 
*** 54	-----------------------    S$9 = PKS1 ? FA1 : -FA1&131071uL;
	.line	54
	ADRK	12
	LAC	* 
	BZ	LL74
	SBRK	11
	BD	LL75
	ZALS	*+
	ADDH	* 
***	B	LL75 OCCURS
LL74:
	SBRK	11
	ZALS	*+
	ADDH	* 
	NEG
	ADRK	14
	SACH	*-
	ANDK	65535
	SACL	*+
	ZALS	* 
	ANDK	1
	SACL	*-
	ZALS	*+
	ADDH	* 
LL75:
	LARK	AR2,4
	MAR	*0+
	SACL	*+
	SACH	*-
*** 54	-----------------------    UGA2B = S$8+S$9>>7&1023u;
	ZALS	*+
	ADDH	* 
	SBRK	3
	ADDS	*+
	ADDH	* 
	LDPK	0
	SPLK	7,TREG1
	RSXM
	SATL
	ANDK	1023
	ADRK	9
	SACL	* 
*** 57	-----------------------    UGA2 = DQSEZ ? ((UGA2B&512u) ? UGA2B-1024u : UGA2B) : 0u;
	.line	57
	ADRK	2
	LAC	* 
	BZ	LL76
	SBRK	2
	ZALS	* 
	ANDK	512
	ANDK	0FFFFh
	BZ	LL77
	LAC	* 
	BD	LL78
	SUBK	1024
***	B	LL78 OCCURS
LL77:
	LAC	* 
LL78:
	B	LL79
LL76:
	LACK	0
LL79:
	LARK	AR2,12
	MAR	*0+
	SACL	* ,AR5
*** 57	-----------------------    ULA2 = (*a2&32768u) ? 512u-(*a2>>7) : -(*a2>>7);
	ZALS	* 
	ANDK	32768
	ANDK	0FFFFh
	BZ	LL80
	LAC	* ,8,AR0
	SACH	* ,1
	LACK	512
	BD	LL81
	SUB	* 
	NOP
***	B	LL81 OCCURS
LL80:
	LAC	* ,8,AR1
	SACH	* ,1
	LAC	* 
	NEG
LL81:
	MAR	* ,AR2
	ADRK	3
	SACL	* ,AR5
*** 59	-----------------------    A2T = *a2+UGA2+ULA2&65535u;
	.line	59
	LAC	* ,AR2
	SBRK	3
	ADD	* 
	ADRK	3
	ADD	* 
	ANDK	65535
	SACL	* 
*** 64	-----------------------    if ( TR ) goto g12;
	.line	64
	LDPK	_TR
	LAC	_TR
	ANDK	0FFFFh
	BNZ	L27
*** 74	-----------------------    A2P = (A2T <= 32767u) ? ((A2T >= 12288u) ? 12288u : A2T) : (A2T <= 53248u) ? 53248u : A2T;
	.line	74
	ZALS	* 
	SUBK	32767
	BGZ	LL82
	ZALS	* 
	SUBK	12288
	BLZ	LL83
	BD	LL84
	LACK	12288
***	B	LL84 OCCURS
LL83:
	LAC	* 
LL84:
	B	LL85
LL82:
	ZALS	* 
	SUBK	-12288
	BGZ	LL86
	BD	LL87
	LACK	53248
***	B	LL87 OCCURS
LL86:
	LAC	* 
LL87:
LL85:
	SACL	_A2P
*** 74	-----------------------    *a2 = A2P;
	MAR	* ,AR5
	SACL	* ,AR2
*** 82	-----------------------    PKS = x->PK1^PK0;
	.line	82
	SBRK	7
	LAR	AR3,*-
	LAC	* ,AR3
	ADRK	16
	XOR	* ,AR2
	ADRK	8
	SACL	*-
*** 83	-----------------------    S$6 = DQSEZ ? (PKS ? 65344u : 192u) : 0u;
	.line	83
	LAC	* 
	BZ	LL88
	MAR	*+
	LAC	* 
	BZ	LL89
	BD	LL90
	LACK	65344
***	B	LL90 OCCURS
LL89:
	LACK	192
LL90:
	B	LL91
LL88:
	LACK	0
LL91:
	LARK	AR2,12
	MAR	*0+
	SACL	* 
*** 83	-----------------------    S$7 = (*a1&32768u) ? 256u-(*a1>>8) : -(*a1>>8);
	SBRK	3
	LAR	AR4,* ,AR4
	ZALS	* 
	ANDK	32768
	ANDK	0FFFFh
	BZ	LL92
	MAR	* ,AR2
	LAR	AR3,* ,AR3
	LAC	* ,7,AR0
	SACH	* ,1
	LACK	256
	BD	LL93
	SUB	* 
	NOP
***	B	LL93 OCCURS
LL92:
	MAR	* ,AR2
	LAR	AR4,* ,AR4
	LAC	* ,7,AR1
	SACH	* ,1
	LAC	* 
	NEG
LL93:
	MAR	* ,AR2
	ADRK	6
	SACL	* 
*** 83	-----------------------    A1T = *a1+S$6+S$7&65535u;
	SBRK	6
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	3
	ADD	* 
	ADRK	3
	ADD	*-
	ANDK	65535
	SACL	*-,AR5
*** 92	-----------------------    C$21 = *a2;
	.line	92
	LAC	* ,AR2
	SACL	*-
*** 92	-----------------------    A1UL = (unsigned)(80896uL-(unsigned long)C$21&65535uL);
	NEG
	ADDK	15360
	ANDK	65535
	SACL	*+
*** 93	-----------------------    A1LL = C$21-15360u&65535u;
	.line	93
	LAC	* 
	SUBK	15360
	ANDK	65535
	ADRK	2
	SACL	*-
*** 94	-----------------------    S$5 = (A1T <= 32767u) ? ((A1T >= A1UL) ? A1UL : A1T) : (A1T <= A1LL) ? A1LL : A1T;
	.line	94
	ZALS	* 
	SUBK	32767
	BGZ	LL94
	ZALS	* 
	SBRK	2
	SUBS	* 
	BLZ	LL95
	BD	LL96
	LAC	* 
	NOP
***	B	LL96 OCCURS
LL95:
	ADRK	2
	LAC	* 
LL96:
	B	LL97
LL94:
	ZALS	*+
	SUBS	* 
	BGZ	LL98
	BD	LL99
	LAC	* 
	NOP
***	B	LL99 OCCURS
LL98:
	MAR	*-
	LAC	* 
LL99:
LL97:
	LARK	AR2,14
	MAR	*0+
	SACL	* 
*** 94	-----------------------    *a1 = S$5;
	SBRK	5
	LAR	AR5,*+,AR5
	SACL	* ,AR2
*** 102	-----------------------    tmp1 = U$3;
	.line	102
	LAR	AR5,* 
*** 103	-----------------------    tmp2 = &x->DQ;
	.line	103
	SBRK	2
	LAC	* ,AR0
	ADDK	8
	SACL	* 
	LAR	AR6,* ,AR2
*** 106	-----------------------    DQS = (unsigned)(U$12 != 0u);
	.line	106
	SBRK	2
	LAC	* 
	LARK	AR4,1
	BNZ	LL100
	LARK	AR4,0
LL100:
	ADRK	6
	SAR	AR4,*-
*** 107	-----------------------    DQM = U$17;
	.line	107
	LAC	* 
	ADRK	4
	SACL	* 
***  	-----------------------    L$16 = 5;
	LACK	5
	SAMM	BRCR
***	-----------------------g11:
***	-----------------------g100:
	RPTB	L39-1
*** 109	-----------------------    S$3 = DQM ? (((unsigned)((*tmp2++&1024u) != 0u)^DQS) ? 65408u : 128u) : 0u;
	.line	109
	LAC	* 
	BZ	LL101
	MAR	* ,AR6
	ZALS	*+
	ANDK	1024
	LARK	AR3,1
	BNZ	LL103
	LARK	AR3,0
LL103:
	MAR	* ,AR0
	SAR	AR3,*
	ZALS	* ,AR2
	SBRK	3
	XOR	* 
	ANDK	0FFFFh
	BZ	LL102
	BD	LL104
	LACK	65408
***	B	LL104 OCCURS
LL102:
	LACK	128
LL104:
	B	LL105
LL101:
	LACK	0
LL105:
	LARK	AR2,13
	MAR	*0+
	SACL	* ,AR5
*** 109	-----------------------    S$4 = (*tmp1&32768u) ? 256u-(*tmp1>>8) : -(*tmp1>>8);
	ZALS	* 
	ANDK	32768
	ANDK	0FFFFh
	BZ	LL106
	RSXM
	LAC	* ,7,AR0
	SACH	* ,1
	LACK	256
	BD	LL107
	SUB	* 
	NOP
***	B	LL107 OCCURS
LL106:
	RSXM
	LAC	* ,7,AR1
	SACH	* ,1
	LAC	* 
	NEG
LL107:
	MAR	* ,AR2
	MAR	*+
	SACL	*-
*** 109	-----------------------    A$24 = *tmp1+S$3+S$4&65535u;
	LAC	*+,AR5
	ADD	* ,AR2
	ADD	*+,AR0
	ANDK	65535
	SACL	* 
	LAR	AR7,* ,AR5
*** 109	-----------------------    *tmp1++ = A$24;
	SAR	AR7,*+,AR2
*** 108	-----------------------    if ( --L$16 >= 0 ) goto g24;
	.line	108
L39:
*** 108	-----------------------    goto g15;
	B	L30
L27:
***	-----------------------g12:
***  	-----------------------    U$172 = U$3;
	SBRK	5
	LAR	AR5,* ,AR5
***  	-----------------------    K$171 = 0u;
	LARK	AR6,0
***  	-----------------------    L$17 = 7;
	LACK	7
	SAMM	BRCR
***	-----------------------g14:
***	-----------------------g109:
	RPTB	L38-1
*** 69	-----------------------    *U$172++ = K$171;
	.line	69
	SAR	AR6,*+
*** 68	-----------------------    if ( --L$17 >= 0 ) goto g23;
	.line	68
	NOP
	NOP
L38:
L30:
***	-----------------------g15:
*** 126	-----------------------    tmp = &x->DQ+5;
	.line	126
	MAR	* ,AR2
	LARK	AR2,8
	MAR	*0+
	LAC	*+
	ADDK	13
	SACL	* 
***  	-----------------------    L$18 = 4;
	LACK	4
	SAMM	BRCR
***	-----------------------g17:
***	-----------------------g111:
	RPTB	L37-1
*** 129	-----------------------    *tmp-- = tmp[-(+1)];
	.line	129
	LAR	AR5,* ,AR5
	MAR	*-,AR2
	LAR	AR4,* ,AR5
	LAC	* ,AR4
	SACL	*-,AR2
	SAR	AR4,* 
*** 128	-----------------------    if ( --L$18 >= 0 ) goto g22;
	.line	128
L37:
*** 132	-----------------------    MAG = U$17;
	.line	132
	ADRK	2
	LAC	*+
	SACL	* 
*** 133	-----------------------    tmp = MAG;
	.line	133
	ADRK	3
	SACL	*+
*** 133	-----------------------    EXP = 0;
	LACK	0
	SACL	*-
*** 133	-----------------------    if ( !tmp ) goto g21;
	LAC	* 
	BZ	L36
	MAR	*+
	RSXM ;;;
L35:
***	-----------------------g20:
*** 133	-----------------------    ++EXP;
	LAC	* 
	ADDK	1
	SACL	*-
*** 133	-----------------------    if ( tmp >>= 1 ) goto g20;
	LAC	* ,14
	SACH	*+,1
	ANDK	0FFFFh,15
	SFL
	BNZ	L35
L36:
***	-----------------------g21:
*** 134	-----------------------    S$1 = U$12 ? 1024 : 0;
	.line	134
	LARK	AR2,6
	MAR	*0+
	LAC	* 
	BZ	LL108
	BD	LL109
	LACK	1024
***	B	LL109 OCCURS
LL108:
	LACK	0
LL109:
	ADRK	11
	SACL	* 
*** 134	-----------------------    S$2 = MAG ? ((EXP < 6) ? MAG<<6-EXP : MAG>>EXP-6) : 32u;
	SBRK	5
	LAC	* 
	BZ	LL110
	SSXM
	ADRK	4
	LAC	* 
	SUBK	6
	BGEZ	LL111
	LACK	6
	SUB	* ,AR0
	SACL	* 
	LT	* ,AR2
	SBRK	4
	LACT	* 
	BD	LL112
	ANDK	0FFFFh
***	B	LL112 OCCURS
LL111:
	LAC	* ,AR0
	SUBK	6
	SACL	* 
	LT	* ,AR2
	SBRK	4
	ZALS	* 
	SATL
	ANDK	0FFFFh
LL112:
	B	LL113
LL110:
	LACK	32
LL113:
	MAR	*+
	SACL	* 
*** 134	-----------------------    C$20 = &x->DQ[0];
	SBRK	5
	LAC	*+
	ADDK	8
	SACL	* 
*** 134	-----------------------    *C$20 = (unsigned)(EXP*64+S$1)+S$2;
	ADRK	7
	LAC	*+,6
	ADD	* 
	SBRK	4
	ADD	* 
	SBRK	4
	LAR	AR3,*-,AR3
	SACL	* ,AR2
*** 138	-----------------------    x->PK2 = x->PK1;
	.line	138
	LAR	AR5,* 
	LAR	AR4,* ,AR5
	ADRK	16
	LAC	* ,AR4
	ADRK	17
	SACL	* ,AR2
*** 139	-----------------------    x->PK1 = PK0;
	.line	139
	LAR	AR3,*-
	LAC	* ,AR3
	ADRK	16
	SACL	* ,AR2
*** 140	-----------------------    C$19 = &C$20[6];
	.line	140
	ADRK	2
	LAC	* ,AR0
	ADDK	6
	SACL	* 
	LAR	AR5,* ,AR2
*** 140	-----------------------    C$20[7] = *C$19;
	LAR	AR4,* ,AR4
	ADRK	7
	MAR	* ,AR5
	LAC	* ,AR4
	SACL	* ,AR2
*** 141	-----------------------    *C$19 = SR0;
	.line	141
	SBRK	8
	LAC	* ,AR5
	SACL	* 
***  	-----------------------    return;
EPI0_3:
	.line	142
	MAR	* ,AR1
;	<restore register vars>
	MAR	*-
	LAR	AR7,*-
	LAR	AR6,* 
	RETD
	SBRK	22
	LAR	AR0,*
***	RET OCCURS

	.endfunc	385,0000000c0H,21

	.sym	_coding_adjustment,_coding_adjustment,32,3,0

	.func	387
******************************************************
* FUNCTION DEF : _coding_adjustment
******************************************************
_coding_adjustment:

LF4	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,5
	LAR	AR0,*0+,AR2

	RSXM
*
*  ---  'ID' shares AUTO storage with 'tmp'
*  ---  'IM' shares AUTO storage with 'DQM'
*  ---  'ID' shares AUTO storage with 'DLN'
*  ---  'ID' shares AUTO storage with 'S$1'
*  ---  'DS' shares AUTO storage with 'C$2'
*
*** 17	-----------------------    DQM = (DS = D&32768u) ? -D&32767u : D;
	.sym	_DS,1,14,1,16
	.sym	_IM,2,14,1,16
	.sym	_ID,3,14,1,16
	.sym	_EXP,4,4,1,16
	.line	17
	LDPK	_D
	ZALS	_D
	ANDK	32768
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	ANDK	0FFFFh
	BZ	LL116
	LAC	_D
	NEG
	BD	LL117
	ANDK	32767
***	B	LL117 OCCURS
LL116:
	LAC	_D
LL117:
	MAR	*+
	SACL	* 
*** 18	-----------------------    tmp = DQM>>1;
	.line	18
	RSXM
	LAC	*+,14
	SACH	*+,1
*** 19	-----------------------    EXP = 0;
	.line	19
	LACK	0
	SACL	*-
*** 19	-----------------------    if ( !tmp ) goto g3;
	LAC	* 
	BZ	L42
	MAR	*+
L41:
***	-----------------------g2:
*** 19	-----------------------    ++EXP;
	LAC	* 
	ADDK	1
	SACL	*-
*** 19	-----------------------    if ( tmp >>= 1 ) goto g2;
	LAC	* ,14
	SACH	*+,1
	ANDK	0FFFFh,15
	SFL
	BNZ	L41
L42:
***	-----------------------g3:
*** 20	-----------------------    S$1 = (EXP < 7) ? DQM<<7-EXP : DQM>>EXP-7;
	.line	20
	SSXM
	LARK	AR2,4
	MAR	*0+
	LAC	* 
	SUBK	7
	BGEZ	LL118
	LACK	7
	SUB	* ,AR0
	SACL	* 
	LT	* ,AR2
	SBRK	2
	LACT	* 
	BD	LL119
	ANDK	0FFFFh
***	B	LL119 OCCURS
LL118:
	LAC	* ,AR0
	SUBK	7
	SACL	* 
	LT	* ,AR2
	SBRK	2
	ZALS	* 
	SATL
	ANDK	0FFFFh
LL119:
	MAR	*+
	SACL	*+
*** 20	-----------------------    DLN = (unsigned)(EXP*128)+(S$1&127u)-(Y>>2)&4095u;
	ANDK	127
	SACB
	LAC	*-,7,AR1
	ADDB
	SACL	* ,AR0
	RSXM
	LDPK	_Y
	LAC	_Y,13
	SACH	* ,1,AR1
	LAC	* ,AR0
	SUB	* ,AR2
	ANDK	4095
	SACL	* 
*** 27	-----------------------    IM = (I&8u) ? I&7u : I+8u;
	.line	27
	ZALS	_I
	ANDK	8
	ANDK	0FFFFh
	BZ	LL120
	LAC	_I
	BD	LL121
	ANDK	7
***	B	LL121 OCCURS
LL120:
	LAC	_I
	ADDK	8
LL121:
	MAR	*-
	SACL	*+
*** 27	-----------------------    ID = (DLN > 299u) ? ((DLN > 2047u) ? ((DLN > 3971u) ? (DS ? 6u : 9u) : 7u) : (DLN > 399u) ? (D...
	ZALS	* 
	SUBK	299
	BLEZ	LL122
	ZALS	* 
	SUBK	2047
	BLEZ	LL123
	ZALS	* 
	SUBK	3971
	BLEZ	LL124
	SBRK	2
	LAC	* 
	BZ	LL125
	BD	LL126
	LACK	6
	NOP
***	B	LL126 OCCURS
LL125:
	LACK	9
LL126:
	B	LL127
LL124:
	LACK	7
LL127:
	B	LL128
LL123:
	ZALS	* 
	SUBK	399
	BLEZ	LL129
	SBRK	2
	LAC	* 
	BZ	LL130
	BD	LL131
	LACK	0
	NOP
***	B	LL131 OCCURS
LL130:
	LACK	15
LL131:
	B	LL132
LL129:
	ZALS	* 
	SUBK	348
	BLEZ	LL133
	SBRK	2
	LAC	* 
	BZ	LL134
	BD	LL135
	LACK	1
	NOP
***	B	LL135 OCCURS
LL134:
	LACK	14
LL135:
	B	LL136
LL133:
	SBRK	2
	LAC	* 
	BZ	LL137
	BD	LL138
	LACK	2
	NOP
***	B	LL138 OCCURS
LL137:
	LACK	13
LL138:
LL136:
LL132:
LL128:
	B	LL139
LL122:
	ZALS	* 
	SUBK	177
	BLEZ	LL140
	ZALS	* 
	SUBK	245
	BLEZ	LL141
	SBRK	2
	LAC	* 
	BZ	LL142
	BD	LL143
	LACK	3
	NOP
***	B	LL143 OCCURS
LL142:
	LACK	12
LL143:
	B	LL144
LL141:
	SBRK	2
	LAC	* 
	BZ	LL145
	BD	LL146
	LACK	4
	NOP
***	B	LL146 OCCURS
LL145:
	LACK	11
LL146:
LL144:
	B	LL147
LL140:
	ZALS	* 
	SUBK	79
	BLEZ	LL148
	SBRK	2
	LAC	* 
	BZ	LL149
	BD	LL150
	LACK	5
	NOP
***	B	LL150 OCCURS
LL149:
	LACK	10
LL150:
	B	LL151
LL148:
	SBRK	2
	LAC	* 
	BZ	LL152
	BD	LL153
	LACK	6
	NOP
***	B	LL153 OCCURS
LL152:
	LACK	9
LL153:
LL151:
LL147:
LL139:
	LARK	AR2,3
	MAR	*0+
	SACL	* 
*** 31	-----------------------    if ( LAW ) goto g8;
	.line	31
	LAC	_LAW
	BNZ	L47
*** 77	-----------------------    SD = S;
	.line	77
	LAC	_S
	SACL	_SD
*** 78	-----------------------    if ( ID > IM ) goto g7;
	.line	78
	ZALS	*-
	SUBS	* 
	BGZ	L46
*** 87	-----------------------    if ( ID >= IM ) goto g13;
	.line	87
	MAR	*+
	ZALS	*-
	SUBS	* 
	BGEZ	EPI0_4
*** 89	-----------------------    SD = (S <= 126u) ? SD+1u : (S <= 128u || S > 255u) ? ((S == 127u) ? 254u : 128u) : SD-1u;
	.line	89
	ZALS	_S
	SUBK	126
	BGZ	LL154
	BD	LL155
	LAC	_SD
	ADDK	1
***	B	LL155 OCCURS
LL154:
	ZALS	_S
	SUBK	128
	BLEZ	LL157
	ZALS	_S
	SUBK	255
	BLEZ	LL156
LL157:
	LAC	_S
	SUBK	127
	BNZ	LL158
	BD	LL159
	LACK	254
	NOP
***	B	LL159 OCCURS
LL158:
	LACK	128
LL159:
	B	LL160
LL156:
	LAC	_SD
	SUBK	1
LL160:
LL155:
	BD	EPI0_4
	SACL	_SD
***  	-----------------------    goto g13;
	NOP
***	B	EPI0_4 OCCURS
L46:
***	-----------------------g7:
*** 80	-----------------------    SD = (S == 0u || S > 127u) ? ((S < 128u || S > 254u) ? (S ? 126u : 0u) : SD+1u) : SD-1u;
	.line	80
	LAC	_S
	ANDK	0FFFFh
	BZ	LL162
	ZALS	_S
	SUBK	127
	BLEZ	LL161
LL162:
	ZALS	_S
	SUBK	128
	BLZ	LL164
	ZALS	_S
	SUBK	254
	BLEZ	LL163
LL164:
	LAC	_S
	BZ	LL165
	BD	LL166
	LACK	126
	NOP
***	B	LL166 OCCURS
LL165:
	LACK	0
LL166:
	B	LL167
LL163:
	LAC	_SD
	ADDK	1
LL167:
	B	LL168
LL161:
	LAC	_SD
	SUBK	1
LL168:
	BD	EPI0_4
	SACL	_SD
*** 85	-----------------------    goto g13;
	.line	85
	NOP
***	B	EPI0_4 OCCURS
L47:
***	-----------------------g8:
*** 56	-----------------------    C$2 = S^85u;
	.line	56
	LAC	_S
	XORK	85
	SBRK	2
	SACL	* 
*** 56	-----------------------    SD = C$2;
	SACL	_SD
*** 57	-----------------------    if ( ID > IM ) goto g11;
	.line	57
	ADRK	2
	ZALS	*-
	SUBS	* 
	BGZ	L50
*** 66	-----------------------    if ( ID >= IM ) goto g12;
	.line	66
	MAR	*+
	ZALS	*-
	SUBS	* 
	BGEZ	L51
*** 68	-----------------------    SD = (C$2 <= 127u) ? (SD ? SD-1u : 128u) : (C$2 == 255u) ? 255u : C$2+1u;
	.line	68
	MAR	*-
	ZALS	* 
	SUBK	127
	BGZ	LL169
	LAC	_SD
	BZ	LL170
	BD	LL171
	SUBK	1
	NOP
***	B	LL171 OCCURS
LL170:
	LACK	128
LL171:
	B	LL172
LL169:
	LAC	* 
	SUBK	255
	BNZ	LL173
	BD	LL174
	LACK	255
	NOP
***	B	LL174 OCCURS
LL173:
	LAC	* 
	ADDK	1
LL174:
LL172:
	BD	L51
	SACL	_SD
*** 69	-----------------------    goto g12;
	.line	69
	NOP
***	B	L51 OCCURS
L50:
***	-----------------------g11:
*** 59	-----------------------    SD = (SD <= 126u) ? SD+1u : (SD >= 129u) ? SD-1u : (SD == 128u) ? 0u : 127u;
	.line	59
	ZALS	_SD
	SUBK	126
	BGZ	LL175
	BD	LL176
	LAC	_SD
	ADDK	1
***	B	LL176 OCCURS
LL175:
	ZALS	_SD
	SUBK	129
	BLZ	LL177
	BD	LL178
	LAC	_SD
	SUBK	1
***	B	LL178 OCCURS
LL177:
	LAC	_SD
	SUBK	128
	BNZ	LL179
	BD	LL180
	LACK	0
	NOP
***	B	LL180 OCCURS
LL179:
	LACK	127
LL180:
LL178:
LL176:
	SACL	_SD
L51:
***	-----------------------g12:
*** 73	-----------------------    SD ^= 85u;
	.line	73
	LAC	_SD
	XORK	85
	SACL	_SD
***	-----------------------g13:
***  	-----------------------    return;
EPI0_4:
	.line	98
	MAR	* ,AR1
	RETD
	SBRK	6
	LAR	AR0,*
***	RET OCCURS

	.endfunc	484,000000000H,5

	.sym	_diff_computation,_diff_computation,32,3,0

	.func	486
******************************************************
* FUNCTION DEF : _diff_computation
******************************************************
_diff_computation:

LF5	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,3
	LAR	AR0,*0+

*** 12	-----------------------    SLI = (SL&8192u) ? SL-16384u : SL;
	.sym	_SLI,1,14,1,16
	.sym	_SEI,2,14,1,16
	.line	12
	LDPK	_SL
	ZALS	_SL
	ANDK	8192
	ANDK	0FFFFh
	BZ	LL183
	LAC	_SL
	BD	LL184
	SUBK	16384
***	B	LL184 OCCURS
LL183:
	LAC	_SL
LL184:
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
*** 12	-----------------------    SEI = (SE&16384u) ? SE+32768u : SE;
	ZALS	_SE
	ANDK	16384
	ANDK	0FFFFh
	BZ	LL185
	LAC	_SE
	BD	LL186
	ADDK	-32768
***	B	LL186 OCCURS
LL185:
	LAC	_SE
LL186:
	MAR	*+
	SACL	*-
*** 13	-----------------------    D = SLI-SEI&65535u;
	.line	13
	NEG
	ADD	* 
	ANDK	65535
	SACL	_D
***  	-----------------------    return;
EPI0_5:
	.line	15
	MAR	* ,AR1
	RETD
	SBRK	4
	LAR	AR0,*
***	RET OCCURS

	.endfunc	500,000000000H,3

	.sym	_iadpt_quant,_iadpt_quant,32,3,0

	.func	502
******************************************************
* FUNCTION DEF : _iadpt_quant
******************************************************
_iadpt_quant:

LF6	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,5
	LAR	AR0,*0+

	RSXM

	.sect	".cinit"
	.word	IS3,STATIC_1
	.word	2048
	.word	4
	.word	135
	.word	213
	.word	273
	.word	323
	.word	373
	.word	425
	.word	425
	.word	373
	.word	323
	.word	273
	.word	213
	.word	135
	.word	4
	.word	2048
IS3	.set	16
	.text

	.sym	_qtab,STATIC_1,62,3,256,,16
	.bss	STATIC_1,16
*
*  ---  'DQL' shares AUTO storage with 'S$2'
*  ---  'DEX' shares AUTO storage with 'U$29'
*
*** 18	-----------------------    DQL = qtab[I]+(Y>>2);
	.sym	S$1,1,14,1,16
	.sym	_DQL,2,14,1,16
	.sym	_DQT,3,14,1,16
	.sym	_DEX,4,4,1,16
	.line	18
	LDPK	_Y
	LAC	_Y,13
	SACH	* ,1,AR0
	LAC	_I
	ADLK	STATIC_1+0,0
	SACL	* 
	LAR	AR5,* ,AR5
	LAC	* ,AR1
	ADD	* ,AR2
	LARK	AR2,2
	MAR	*0+
	SACL	* 
*** 25	-----------------------    DEX = (int)(DQL>>7&15u);
	.line	25
	LAC	* ,8
	ANDK	15,15
	ADRK	2
	SACH	* ,1
*** 26	-----------------------    DQT = (DQL&127u)+128u;
	.line	26
	SBRK	2
	LAC	*+
	ANDK	127
	ADDK	128
	SACL	* 
*** 27	-----------------------    S$1 = (I&8u) ? 16384u : 0u;
	.line	27
	ZALS	_I
	ANDK	8
	ANDK	0FFFFh
	BZ	LL189
	BD	LL190
	LACK	16384
***	B	LL190 OCCURS
LL189:
	LACK	0
LL190:
	SBRK	2
	SACL	*+
*** 27	-----------------------    S$2 = (DQL&2048u) ? 0u : ((U$29 = 7-DEX) < 0) ? DQT<<-U$29 : DQT>>U$29;
	ZALS	* 
	ANDK	2048
	ANDK	0FFFFh
	BZ	LL191
	BD	LL192
	LACK	0
	NOP
***	B	LL192 OCCURS
LL191:
	LACK	7
	SSXM
	ADRK	2
	SUB	* 
	SACL	* 
	BGEZ	LL193
	LAC	*-,AR0
	NEG
	SACL	* 
	LT	* ,AR2
	LACT	* 
	BD	LL194
	ANDK	0FFFFh
***	B	LL194 OCCURS
LL193:
	LT	*-
	ZALS	* 
	SATL
	ANDK	0FFFFh
LL194:
LL192:
	LARK	AR2,2
	MAR	*0+
	SACL	*-
*** 27	-----------------------    DQ = S$1+S$2;
	ADD	* 
	SACL	_DQ
***  	-----------------------    return;
EPI0_6:
	.line	30
	MAR	* ,AR1
	RETD
	SBRK	6
	LAR	AR0,*
***	RET OCCURS

	.endfunc	531,000000000H,5

	.sym	_input_conversion,_input_conversion,32,3,0

	.func	533
******************************************************
* FUNCTION DEF : _input_conversion
******************************************************
_input_conversion:

LF7	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,3
	LAR	AR0,*0+

*
*  ---  'C$2' shares AUTO storage with 'C$1'
*
*** 12	-----------------------    if ( LAW ) goto g2;
	.sym	C$2,1,14,1,16
	.sym	_SSQ,2,14,1,16
	.line	12
	LDPK	_LAW
	LAC	_LAW
	BNZ	L54
*** 22	-----------------------    C$2 = u_LAW_table[S];
	.line	22
	LAC	_S
	ADLK	_u_LAW_table+0,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR5,* ,AR5
	LAC	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
*** 22	-----------------------    SSQ = C$2&8191u;
	ANDK	8191
	SACL	*-
*** 22	-----------------------    if ( !(C$2&8192u) ) goto g3;
	ZALS	* 
	ANDK	8192
	ANDK	0FFFFh
	BZ	L55
*** 22	-----------------------    goto g4;
	B	L56
L54:
***	-----------------------g2:
*** 16	-----------------------    C$1 = A_LAW_table[S];
	.line	16
	LAC	_S
	ADLK	_A_LAW_table+0,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR4,* ,AR4
	LAC	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+,AR0
*** 16	-----------------------    SSQ = (C$1&4095u)*2u;
	ANDK	4095
	SACL	* 
	LAC	* ,1,AR2
	SACL	*-
***  	-----------------------    if ( C$1&4096u ) goto g4;
	ZALS	* 
	ANDK	4096
	ANDK	0FFFFh
	BNZ	L56
L55:
***	-----------------------g3:
*** 24	-----------------------    SL = SSQ;
	.line	24
	MAR	*+
	BD	EPI0_7
	LAC	* 
	SACL	_SL
*** 24	-----------------------    goto g5;
***	B	EPI0_7 OCCURS
L56:
***	-----------------------g4:
*** 24	-----------------------    SL = -SSQ&16383u;
	MAR	*+
	LAC	* 
	NEG
	ANDK	16383
	SACL	_SL
***	-----------------------g5:
***  	-----------------------    return;
EPI0_7:
	.line	25
	MAR	* ,AR1
	RETD
	SBRK	4
	LAR	AR0,*
***	RET OCCURS

	.endfunc	557,000000000H,3

	.sym	_output_conversion,_output_conversion,32,3,0

	.func	559
******************************************************
* FUNCTION DEF : _output_conversion
******************************************************
_output_conversion:

LF8	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,5
	LAR	AR0,*0+,AR2

	RSXM
*
*  ---  'SEG' shares AUTO storage with 'tmp'
*  ---  'IMS' shares AUTO storage with 'IMS'
*  ---  'MASK' shares AUTO storage with 'SEG'
*  ---  'SEG' shares AUTO storage with 'IM'
*  ---  'MASK' shares AUTO storage with 'IS'
*  ---  'SEG' shares AUTO storage with 'S$1'
*
*** 11	-----------------------    IM = (IS = SR&32768u) ? -SR&32767u : SR;
	.sym	_MASK,1,14,1,16
	.sym	_MASK,2,14,1,16
	.sym	_SEG,3,14,1,16
	.sym	_IMS,4,14,1,16
	.line	11
	LDPK	_SR
	ZALS	_SR
	ANDK	32768
	LARK	AR2,2
	MAR	*0+
	SACL	* 
	ANDK	0FFFFh
	BZ	LL198
	LAC	_SR
	NEG
	BD	LL199
	ANDK	32767
***	B	LL199 OCCURS
LL198:
	LAC	_SR
LL199:
	MAR	*+
	SACL	* 
*** 12	-----------------------    if ( LAW ) goto g7;
	.line	12
	LAC	_LAW
	BNZ	L64
*** 35	-----------------------    MASK = IS ? 127u : 255u;
	.line	35
	MAR	*-
	LAC	* 
	BZ	LL200
	BD	LL201
	LACK	127
	NOP
***	B	LL201 OCCURS
LL200:
	LACK	255
LL201:
	SACL	*+
*** 35	-----------------------    if ( (IMS = IM+33u) > 8191u ) goto g6;
	ZALS	*+
	ADDK	33
	SACL	* 
	ANDK	0FFFFh
	SUBK	8191
	BGZ	L63
*** 41	-----------------------    SEG = 5u;
	.line	41
	LACK	5
	MAR	*-
	SACL	*+
*** 41	-----------------------    if ( !(IMS>>5u) ) goto g5;
	RSXM
	LAC	* ,10
	ANDK	0FFFFh,15
	SFL
	BZ	L62
	MAR	*-
L61:
***	-----------------------g4:
*** 41	-----------------------    if ( IMS>>(++SEG) ) goto g4;
	LAC	* 
	ADDK	1
	SACL	*+,AR0
	SACL	* 
	LT	* ,AR2
	ZALS	*-
	SATL
	ANDK	0FFFFh
	BNZ	L61
L62:
***	-----------------------g5:
*** 43	-----------------------    SEG -= 6u;
	.line	43
	LARK	AR2,3
	MAR	*0+
	LAC	* 
	SUBK	6
	SACL	*+,AR0
*** 44	-----------------------    S = (SEG*16u|IMS>>SEG+1u&15u)^MASK;
	.line	44
	ADDK	1
	SACL	* 
	LT	* ,AR2
	ZALS	*-
	SATL
	ANDK	15
	SACB
	LAC	*-,4
	ORB
	XOR	* 
	BD	EPI0_8
	LDPK	_S
	SACL	_S
*** 44	-----------------------    goto g13;
***	B	EPI0_8 OCCURS
L63:
***	-----------------------g6:
*** 38	-----------------------    S = MASK^127u;
	.line	38
	SBRK	2
	LAC	* 
	XORK	127
	BD	EPI0_8
	SACL	_S
***  	-----------------------    goto g13;
	NOP
***	B	EPI0_8 OCCURS
L64:
***	-----------------------g7:
*** 18	-----------------------    MASK = IS ? 85u : 213u;
	.line	18
	MAR	*-
	LAC	* 
	BZ	LL202
	BD	LL203
	LACK	85
	NOP
***	B	LL203 OCCURS
LL202:
	LACK	213
LL203:
	MAR	*-
	SACL	*+
*** 18	-----------------------    if ( (IMS = IS ? (IM+1u>>1)-1u : IM>>1) > 4095u ) goto g12;
	LAC	* 
	BZ	LL204
	MAR	*+
	ZALS	* ,AR0
	ADDK	1
	SACL	* 
	RSXM
	LAC	* ,14
	SBLK	1,15
	BD	LL205
	SACH	* ,1
	ZALS	* 
***	B	LL205 OCCURS
LL204:
	RSXM
	MAR	*+
	LAC	* ,14,AR1
	SACH	* ,1
	ZALS	* 
LL205:
	MAR	* ,AR2
	MAR	*+
	SACL	* 
	SUBK	4095
	BGZ	L69
*** 26	-----------------------    SEG = 0u;
	.line	26
	LACK	0
	SBRK	2
	SACL	* 
*** 26	-----------------------    if ( !(tmp = IMS>>4>>1) ) goto g11;
	ADRK	2
	LAC	*-,10
	SACH	* ,1
	ANDK	0FFFFh,15
	SFL
	BZ	L68
	MAR	*-
L67:
***	-----------------------g10:
*** 26	-----------------------    ++SEG;
	LAC	* 
	ADDK	1
	SACL	*+
*** 26	-----------------------    if ( tmp >>= 1 ) goto g10;
	LAC	* ,14
	SACH	*-,1
	ANDK	0FFFFh,15
	SFL
	BNZ	L67
L68:
***	-----------------------g11:
*** 28	-----------------------    S$1 = SEG ? IMS>>SEG : IMS>>1;
	.line	28
	LARK	AR2,2
	MAR	*0+
	LAC	* 
	BZ	LL206
	LT	* 
	ADRK	2
	BD	LL207
	ZALS	* 
	SATL
***	B	LL207 OCCURS
LL206:
	ADRK	2
	LAC	* ,14,AR1
	SACH	* ,1
	LAC	* 
LL207:
	MAR	* ,AR2
	MAR	*-
	SACL	*-
*** 28	-----------------------    S = (SEG*16u|S$1&15u)^MASK;
	ANDK	15
	SACB
	LAC	*-,4
	ORB
	XOR	* 
	BD	EPI0_8
	LDPK	_S
	SACL	_S
*** 28	-----------------------    goto g13;
***	B	EPI0_8 OCCURS
L69:
***	-----------------------g12:
*** 21	-----------------------    S = MASK^127u;
	.line	21
	SBRK	3
	LAC	* 
	XORK	127
	SACL	_S
***	-----------------------g13:
***  	-----------------------    return;
EPI0_8:
	.line	47
	MAR	* ,AR1
	RETD
	SBRK	6
	LAR	AR0,*
***	RET OCCURS

	.endfunc	605,000000000H,5

	.sym	_reset_states,_reset_states,32,3,0

	.func	607
******************************************************
* FUNCTION DEF : _reset_states
******************************************************
_reset_states:

LF9	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,3
	LAR	AR0,*0+
;	<save register vars>
	SAR	AR6,*+
	SAR	AR7,*+,AR2

	RSXM
*
* AR5	assigned to temp var 'U$8'
* AR5	assigned to temp var 'U$17'
* AR6	assigned to temp var 'K$5'
* AR7	assigned to temp var 'K$14'
* BRCR	assigned to temp var 'L$2'
* BRCR	assigned to temp var 'L$1'
*
*** 3	-----------------------    x = X;
	.sym	C$3,1,30,1,16
	.sym	_x,2,24,1,16,.fake0
	.line	3
	LDPK	_X
	LAC	_X
	LARK	AR2,2
	MAR	*0+
	SACL	* ,AR0
***  	-----------------------    U$8 = &x->B[0];
	SACL	* 
	LAR	AR5,* ,AR5
***  	-----------------------    K$5 = 0u;
	LARK	AR6,0
***  	-----------------------    L$1 = 7;
	LACK	7
	SAMM	BRCR
***	-----------------------g2:
***	-----------------------g12:
	RPTB	L78-1
*** 7	-----------------------    *U$8++ = K$5;
	.line	7
	SAR	AR6,*+
*** 6	-----------------------    if ( --L$1 >= 0 ) goto g8;
	.line	6
	NOP
	NOP
L78:
***  	-----------------------    U$17 = &x->DQ[0];
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	LAC	* ,AR0
	ADDK	8
	SACL	* 
	LAR	AR5,* ,AR5
***  	-----------------------    K$14 = 32u;
	LARK	AR7,32
***  	-----------------------    L$2 = 5;
	LACK	5
	SAMM	BRCR
***	-----------------------g5:
***	-----------------------g14:
	RPTB	L77-1
*** 9	-----------------------    *U$17++ = K$14;
	.line	9
	SAR	AR7,*+
*** 8	-----------------------    if ( --L$2 >= 0 ) goto g7;
	.line	8
	NOP
	NOP
L77:
*** 10	-----------------------    C$3 = &x->DQ[0];
	.line	10
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	LAC	*-
	ADDK	8
	SACL	* 
*** 10	-----------------------    C$3[6] = C$3[7] = K$14;
	LAR	AR5,* ,AR5
	ADRK	7
	SAR	AR7,* ,AR2
	LAR	AR4,* ,AR4
	ADRK	6
	SAR	AR7,* ,AR2
*** 11	-----------------------    x->PK1 = x->PK2 = K$5;
	.line	11
	MAR	*+
	LAR	AR3,* ,AR3
	ADRK	17
	SAR	AR6,* ,AR2
	LAR	AR5,* ,AR5
	ADRK	16
	SAR	AR6,* ,AR2
*** 12	-----------------------    x->AP = x->DMS = x->DML = K$5;
	.line	12
	LAR	AR4,* ,AR4
	ADRK	20
	SAR	AR6,* ,AR2
	LAR	AR3,* ,AR3
	ADRK	19
	SAR	AR6,* ,AR2
	LAR	AR5,* ,AR5
	ADRK	18
	SAR	AR6,* ,AR2
*** 13	-----------------------    x->YU = 544u;
	.line	13
	LAR	AR4,* ,AR4
	ADRK	21
	SPLK	#544,* ,AR2
*** 14	-----------------------    x->TD = K$5;
	.line	14
	LAR	AR3,* ,AR3
	ADRK	22
	SAR	AR6,* ,AR2
*** 15	-----------------------    x->YL = 34816uL;
	.line	15
	LAR	AR5,* ,AR5
	LACK	34816
	ADRK	24
	SACL	*+
	SACH	*-,AR2
*** 15	-----------------------    x->YL6 = 544u;
	LAR	AR4,* ,AR4
	ADRK	23
	SPLK	#544,* 
***  	-----------------------    return;
EPI0_9:
	.line	16
	MAR	* ,AR1
;	<restore register vars>
	MAR	*-
	LAR	AR7,*-
	LAR	AR6,* 
	RETD
	SBRK	4
	LAR	AR0,*
***	RET OCCURS

	.endfunc	622,0000000c0H,3

	.sym	_scale_factor_1,_scale_factor_1,32,3,0

	.func	624
******************************************************
* FUNCTION DEF : _scale_factor_1
******************************************************
_scale_factor_1:

LF10	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,7
	LAR	AR0,*0+,AR2

	RSXM
*
*  ---  'PRODM' shares AUTO storage with 'DIFM'
*  ---  'PRODM' shares AUTO storage with 'DIF'
*  ---  'PRODM' shares AUTO storage with 'S$1'
*  ---  'U$4' shares AUTO storage with 'C$3'
*
*** 11	-----------------------    x = X;
	.sym	_x,1,24,1,16,.fake0
	.sym	U$4,2,14,1,16
	.sym	C$2,3,14,1,16
	.sym	U$3,4,14,1,16
	.sym	_PRODM,5,14,1,16
	.sym	_DIFS,6,14,1,16
	.line	11
	LDPK	_X
	LAC	_X
	LARK	AR2,1
	MAR	*0+
	SACL	* 
*** 17	-----------------------    C$3 = x->YL6;
	.line	17
	LAR	AR5,* ,AR5
	ADRK	23
	LAC	* ,AR2
	MAR	*+
	SACL	*-
*** 17	-----------------------    C$2 = x->YU;
	LAR	AR4,* ,AR4
	ADRK	21
	LAC	* ,AR2
	ADRK	2
	SACL	*-
*** 17	-----------------------    DIF = C$2-C$3;
	SUB	* 
	ADRK	3
	SACL	*+
*** 18	-----------------------    DIFS = DIF&8192u;
	.line	18
	ANDK	8192
	SACL	* 
*** 19	-----------------------    DIFM = ((U$4 = C$2-(U$3 = C$3))&8192u) ? -DIF&8191u : U$4;
	.line	19
	SBRK	4
	ZALS	* 
	ADRK	2
	SACL	*-
	NEG
	ADDS	*-
	SACL	* 
	ANDK	8192
	ANDK	0FFFFh
	BZ	LL211
	ADRK	3
	LAC	* 
	NEG
	BD	LL212
	ANDK	8191
***	B	LL212 OCCURS
LL211:
	LAC	* 
LL212:
	LARK	AR2,5
	MAR	*0+
	SACL	* 
*** 19	-----------------------    PRODM = (unsigned)((unsigned long)DIFM*(unsigned long)AL>>6);
	LT	_AL
	MPYU	* 
	PAC
	LDPK	0
	SPLK	6,TREG1
	RSXM
	SATL
	SACL	*+
*** 21	-----------------------    S$1 = DIFS ? -PRODM : PRODM;
	.line	21
	LAC	* 
	BZ	LL213
	MAR	*-
	BD	LL214
	LAC	* 
	NEG
***	B	LL214 OCCURS
LL213:
	MAR	*-
	LAC	* 
LL214:
	SACL	*-
*** 21	-----------------------    Y = U$3+S$1&8191u;
	ADD	* 
	ANDK	8191
	LDPK	_Y
	SACL	_Y
***  	-----------------------    return;
EPI0_10:
	.line	23
	MAR	* ,AR1
	RETD
	SBRK	8
	LAR	AR0,*
***	RET OCCURS

	.endfunc	646,000000000H,7

	.sym	_scale_factor_2,_scale_factor_2,32,3,0

	.func	648
******************************************************
* FUNCTION DEF : _scale_factor_2
******************************************************
_scale_factor_2:

LF11	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,10
	LAR	AR0,*0+,AR2

	SSXM

	.sect	".cinit"
	.word	IS4,STATIC_2
	.word	4084
	.word	18
	.word	41
	.word	64
	.word	112
	.word	198
	.word	355
	.word	1122
IS4	.set	8
	.text

	.sym	_W,STATIC_2,62,3,128,,8
	.bss	STATIC_2,8
*
*  ---  'DIF' shares AUTO storage with 'DIF'
*  ---  'DIF' shares AUTO storage with 'YUT'
*  ---  'DIF' shares AUTO storage with 'S$3'
*  ---  'DIF' shares AUTO storage with 'S$2'
*  ---  'S$1' shares AUTO storage with 'U$53'
*  ---  'DIFSX' shares AUTO storage with 'C$4'
*
*** 15	-----------------------    x = X;
	.sym	U$46,1,15,1,32
	.sym	_DIFSX,3,15,1,32
	.sym	_x,5,24,1,16,.fake0
	.sym	S$1,6,14,1,16
	.sym	_DIF,7,14,1,16
	.line	15
	LDPK	_X
	LAC	_X
	LARK	AR2,5
	MAR	*0+
	SACL	* 
*** 22	-----------------------    S$3 = (I>>3) ? 15u-I : I;
	.line	22
	RSXM
	LAC	_I,12
	ANDK	0FFFFh,15
	SFL
	BZ	LL217
	BD	LL218
	LACK	15
	SUB	_I
***	B	LL218 OCCURS
LL217:
	LAC	_I
LL218:
	ADRK	2
	SACL	* ,AR0
*** 22	-----------------------    DIF = (unsigned)((unsigned long)W[(S$3&7u)]*32uL-(unsigned long)Y>>5&4095uL);
	ANDK	7
	ADLK	STATIC_2+0,0
	SACL	* 
	LAR	AR5,* ,AR5
	ZALS	* ,AR1
	SACL	*+
	SACH	*+
	CALLD	L$$SL
	LACK	5
	SACL	* 
***	CALL	L$$SL OCCURS
	SUBS	_Y
	LDPK	0
	SPLK	5,TREG1
	RSXM
	SATL
	ANDK	4095
	MAR	* ,AR2
	SACL	* 
*** 25	-----------------------    S$2 = (DIF&2048u) ? DIF+4096u : DIF;
	.line	25
	ZALS	* 
	ANDK	2048
	ANDK	0FFFFh
	BZ	LL219
	LAC	* 
	BD	LL220
	ADDK	4096
***	B	LL220 OCCURS
LL219:
	LAC	* 
LL220:
	SACL	* 
*** 25	-----------------------    YUT = S$2+Y&8191u;
	LDPK	_Y
	ADD	_Y
	ANDK	8191
	SACL	* 
*** 29	-----------------------    S$1 = (YUT+15840u&8192u) ? 544u : (YUT+11264u&8192u) ? YUT : 5120u;
	.line	29
	ZALS	* 
	ADDK	15840
	ANDK	8192
	ANDK	0FFFFh
	BZ	LL221
	BD	LL222
	LACK	544
***	B	LL222 OCCURS
LL221:
	ZALS	* 
	ADDK	11264
	ANDK	8192
	ANDK	0FFFFh
	BZ	LL223
	BD	LL224
	LAC	* 
	NOP
***	B	LL224 OCCURS
LL223:
	LACK	5120
LL224:
LL222:
	MAR	*-
	SACL	*-
*** 29	-----------------------    x->YU = S$1;
	LAR	AR4,* ,AR4
	ADRK	21
	SACL	* ,AR2
*** 38	-----------------------    C$4 = x->YL;
	.line	38
	LAR	AR3,* ,AR3
	ADRK	24
	ZALS	*+
	ADDH	*-,AR2
	SBRK	2
	SACL	*+
	SACH	*-
*** 38	-----------------------    DIF = (unsigned)(-C$4>>6)+S$1&16383u;
	ZALS	*+
	ADDH	* 
	NEG
	LDPK	0
	SPLK	6,TREG1
	SATL
	ADRK	2
	ADD	*+
	ANDK	16383
	SACL	* 
*** 39	-----------------------    DIFSX = ((U$53 = (unsigned)(-(U$46 = C$4)>>6)+S$1&16383u)&8192u) ? (unsigned long)DIF+507904uL...
	.line	39
	SBRK	4
	ZALS	*+
	ADDH	* 
	SBRK	3
	SACL	*+
	SACH	* 
	NEG
	LDPK	0
	SPLK	6,TREG1
	SATL
	ADRK	4
	ADDS	* 
	ANDK	16383
	SACL	* 
	ANDK	8192
	ANDK	0FFFFh
	BZ	LL225
	LALK	15,15
	ORK	16384
	BD	LL226
	MAR	*+
	ADDS	* 
***	B	LL226 OCCURS
LL225:
	ZALS	* 
LL226:
	LARK	AR2,3
	MAR	*0+
	SACL	*+
	SACH	*-
*** 39	-----------------------    x->YL6 = (unsigned)((x->YL = U$46+DIFSX&524287uL)>>6);
	ZALS	*+
	ADDH	* 
	SBRK	3
	ADDS	*+
	ADDH	* 
	ADRK	7
	SACH	*-
	ANDK	65535
	SACL	*+
	ZALS	* 
	ANDK	7
	SACL	* 
	SBRK	4
	LAR	AR5,* 
	ADRK	3
	ZALS	*+
	ADDH	* ,AR5
	ADRK	24
	SACL	*+
	SACH	*-,AR2
	LDPK	0
	SPLK	6,TREG1
	SATL
	SBRK	4
	LAR	AR4,* ,AR4
	ADRK	23
	SACL	* 
***  	-----------------------    return;
EPI0_11:
	.line	42
	MAR	* ,AR1
	RETD
	SBRK	11
	LAR	AR0,*
***	RET OCCURS

	.endfunc	689,000000000H,10

	.sym	_speed_control_1,_speed_control_1,32,3,0

	.func	691
******************************************************
* FUNCTION DEF : _speed_control_1
******************************************************
_speed_control_1:

LF12	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,2
	LAR	AR0,*0+,AR5

	RSXM
*** 12	-----------------------    AL = ((C$1 = X->AP) > 255u) ? 64u : C$1>>2;
	.sym	C$1,1,14,1,16
	.line	12
	LDPK	_X
	LAR	AR5,_X
	ADRK	18
	ZALS	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	SUBK	255
	BLEZ	LL229
	BD	LL230
	LACK	64
	NOP
***	B	LL230 OCCURS
LL229:
	RSXM
	LAC	* ,13,AR1
	SACH	* ,1
	LAC	* 
LL230:
	SACL	_AL
*** 12	-----------------------    return;
EPI0_12:
	.line	13
	MAR	* ,AR1
	RETD
	SBRK	3
	LAR	AR0,*
***	RET OCCURS

	.endfunc	703,000000000H,2

	.sym	_speed_control_2,_speed_control_2,32,3,0

	.func	705
******************************************************
* FUNCTION DEF : _speed_control_2
******************************************************
_speed_control_2:

LF13	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,6
	LAR	AR0,*0+,AR2

	RSXM

	.sect	".cinit"
	.word	IS5,STATIC_3
	.word	0
	.word	0
	.word	0
	.word	512
	.word	512
	.word	512
	.word	1536
	.word	3584
IS5	.set	8
	.text

	.sym	_F,STATIC_3,62,3,128,,8
	.bss	STATIC_3,8
*
* AR5	assigned to temp var 'S$1'
*  ---  'DIF' shares AUTO storage with 'DIFM'
*  ---  'DIF' shares AUTO storage with 'DIF'
*  ---  'DIF' shares AUTO storage with 'DIF'
*  ---  'DIF' shares AUTO storage with 'DIF'
*  ---  'AX' shares AUTO storage with 'FI'
*  ---  'AX' shares AUTO storage with 'S$5'
*  ---  'DIF' shares AUTO storage with 'S$4'
*  ---  'AX' shares AUTO storage with 'S$3'
*  ---  'AX' shares AUTO storage with 'S$2'
*  ---  'AX' shares AUTO storage with 'U$68'
*  ---  'AX' shares AUTO storage with 'U$48'
*  ---  'U$63' shares AUTO storage with 'U$32'
*  ---  'AX' shares AUTO storage with 'U$37'
*  ---  'U$63' shares AUTO storage with 'U$15'
*  ---  'U$20' shares AUTO storage with 'C$11'
*  ---  'U$20' shares AUTO storage with 'C$10'
*  ---  'AX' shares AUTO storage with 'C$9'
*  ---  'AX' shares AUTO storage with 'C$8'
*  ---  'U$20' shares AUTO storage with 'C$7'
*  ---  'U$20' shares AUTO storage with 'C$6'
*
*** 12	-----------------------    x = X;
	.sym	U$63,1,14,1,16
	.sym	U$20,2,14,1,16
	.sym	_AX,3,14,1,16
	.sym	_DIF,4,14,1,16
	.sym	_x,5,24,1,16,.fake0
	.line	12
	LDPK	_X
	LAC	_X
	LARK	AR2,5
	MAR	*0+
	SACL	* 
*** 16	-----------------------    S$5 = (I>>3) ? 15u-I : I;
	.line	16
	LAC	_I,12
	ANDK	0FFFFh,15
	SFL
	BZ	LL233
	BD	LL234
	LACK	15
	SUB	_I
***	B	LL234 OCCURS
LL233:
	LAC	_I
LL234:
	SBRK	2
	SACL	* ,AR0
*** 16	-----------------------    FI = F[(S$5&7u)];
	ANDK	7
	ADLK	STATIC_3+0,0
	SACL	* 
	LAR	AR5,* ,AR5
	LAC	* ,AR2
	SACL	* 
*** 22	-----------------------    C$11 = x->DMS;
	.line	22
	ADRK	2
	LAR	AR4,* ,AR4
	ADRK	19
	LAC	* ,AR2
	SBRK	3
	SACL	*+
*** 22	-----------------------    DIF = (FI-C$11&8191u)>>5;
	ZALS	*-
	SUBS	* ,AR0
	ANDK	8191
	SACL	* 
	LAC	* ,10,AR2
	ADRK	2
	SACH	* ,1
*** 23	-----------------------    S$4 = ((U$20 = (FI-(U$15 = C$11)&8191u)>>5)&128u) ? DIF+3840u : U$20;
	.line	23
	SBRK	2
	ZALS	*-
	SACL	* 
	NEG
	ADRK	2
	ADDS	*-,AR0
	ANDK	8191
	SACL	* 
	LAC	* ,10,AR2
	SACH	* ,1
	ANDK	128,15
	ANDK	0FFFFh,15
	SFL
	BZ	LL235
	ADRK	2
	LAC	* 
	BD	LL236
	ADDK	3840
***	B	LL236 OCCURS
LL235:
	LAC	* 
LL236:
	LARK	AR2,4
	MAR	*0+
	SACL	* 
*** 23	-----------------------    x->DMS = S$4+U$15&4095u;
	SBRK	3
	ADD	* 
	ANDK	4095
	ADRK	4
	LAR	AR3,* ,AR3
	ADRK	19
	SACL	* ,AR2
*** 30	-----------------------    C$10 = x->DML;
	.line	30
	LAR	AR5,* ,AR5
	ADRK	20
	LAC	* ,AR2
	SBRK	3
	SACL	*+
*** 30	-----------------------    DIF = ((C$9 *= 4u)-C$10&32767u)>>7;
	LAC	* ,2
	SACL	*-
	SUBS	* ,AR0
	ANDK	32767
	SACL	* 
	LAC	* ,8,AR2
	ADRK	2
	SACH	* ,1
*** 31	-----------------------    S$3 = ((U$37 = (C$9-(U$32 = C$10)&32767u)>>7)&128u) ? DIF+16128u : U$37;
	.line	31
	SBRK	2
	ZALS	*-
	SACL	* 
	NEG
	ADRK	2
	ADDS	* ,AR0
	ANDK	32767
	SACL	* 
	LAC	* ,8,AR2
	SACH	* ,1
	ANDK	128,15
	ANDK	0FFFFh,15
	SFL
	BZ	LL237
	MAR	*+
	LAC	* 
	BD	LL238
	ADDK	16128
***	B	LL238 OCCURS
LL237:
	LAC	* 
LL238:
	LARK	AR2,3
	MAR	*0+
	SACL	* 
*** 31	-----------------------    C$8 = S$3+U$32&16383u;
	SBRK	2
	ADD	* 
	ANDK	16383
	ADRK	2
	SACL	* 
*** 31	-----------------------    x->DML = C$8;
	ADRK	2
	LAR	AR4,* ,AR4
	ADRK	20
	SACL	* ,AR2
*** 38	-----------------------    C$7 = x->DMS*4u;
	.line	38
	LAR	AR3,* ,AR3
	ADRK	19
	LAC	* ,2,AR2
	SBRK	3
	SACL	*+
*** 38	-----------------------    DIF = C$7-C$8&32767u;
	SUB	*+
	ANDK	32767
	SACL	*-
*** 39	-----------------------    DIFM = ((U$48 = C$7-(U$32 = C$8)&32767u)&16384u) ? 32768u-DIF&16383u : U$48;
	.line	39
	ZALS	* 
	SBRK	2
	SACL	*+
	NEG
	ADDS	*+
	ANDK	32767
	SACL	* 
	ANDK	16384
	ANDK	0FFFFh
	BZ	LL239
	LALK	1,15
	MAR	*+
	SUB	* 
	BD	LL240
	ANDK	16383
***	B	LL240 OCCURS
LL239:
	LAC	* 
LL240:
	LARK	AR2,4
	MAR	*0+
	SACL	* 
*** 39	-----------------------    AX = (Y < 1536u || (DIFM >= U$32>>3 || TDP)) ? 512u : 0u;
	ZALS	_Y
	SUBK	1536
	BLZ	LL242
	SBRK	3
	LAC	* ,12,AR0
	SACH	* ,1,AR2
	ADRK	3
	ZALS	* ,AR0
	SUBS	* 
	BGEZ	LL242
	LAC	_TDP
	BZ	LL241
LL242:
	BD	LL244
	LACK	512
***	B	LL244 OCCURS
LL241:
	LACK	0
LL244:
	MAR	* ,AR2
	MAR	*-
	SACL	* 
*** 41	-----------------------    C$6 = x->AP;
	.line	41
	ADRK	2
	LAR	AR5,* ,AR5
	ADRK	18
	LAC	* ,AR2
	SBRK	3
	SACL	*+
*** 48	-----------------------    DIF = (AX-C$6&2047u)>>4;
	.line	48
	ZALS	*-
	SUBS	* ,AR0
	ANDK	2047
	SACL	* 
	LAC	* ,11,AR2
	ADRK	2
	SACH	* ,1
*** 49	-----------------------    S$2 = ((U$68 = (AX-(U$63 = C$6)&2047u)>>4)&64u) ? DIF+896u : U$68;
	.line	49
	SBRK	2
	ZALS	*-
	SACL	* 
	NEG
	ADRK	2
	ADDS	* ,AR0
	ANDK	2047
	SACL	* 
	LAC	* ,11,AR2
	SACH	* ,1
	ANDK	64,15
	ANDK	0FFFFh,15
	SFL
	BZ	LL245
	MAR	*+
	LAC	* 
	BD	LL246
	ADDK	896
***	B	LL246 OCCURS
LL245:
	LAC	* 
LL246:
	LARK	AR2,3
	MAR	*0+
	SACL	* 
*** 49	-----------------------    S$1 = TR ? 256u : S$2+U$63&1023u;
	LAC	_TR
	BZ	LL247
	BD	LL248
	LACK	256
***	B	LL248 OCCURS
LL247:
	LAC	* 
	SBRK	2
	ADD	* 
	ANDK	1023
LL248:
	MAR	* ,AR0
	SACL	* 
	LAR	AR5,* ,AR2
*** 53	-----------------------    x->AP = S$1;
	.line	53
	LARK	AR2,5
	MAR	*0+
	LAR	AR4,* ,AR4
	ADRK	18
	SAR	AR5,* 
***  	-----------------------    return;
EPI0_13:
	.line	54
	MAR	* ,AR1
	RETD
	SBRK	7
	LAR	AR0,*
***	RET OCCURS

	.endfunc	758,000000000H,6

	.sym	_tone_detector_1,_tone_detector_1,32,3,0

	.func	760
******************************************************
* FUNCTION DEF : _tone_detector_1
******************************************************
_tone_detector_1:

LF14	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,6
	LAR	AR0,*0+,AR2

	RSXM
*
*  ---  'THR2' shares AUTO storage with 'U$8'
*
*** 11	-----------------------    x = X;
	.sym	S$2,1,4,1,16
	.sym	_DQMAG,2,14,1,16
	.sym	_x,3,24,1,16,.fake0
	.sym	C$3,4,14,1,16
	.sym	_THR2,5,14,1,16
	.line	11
	LDPK	_X
	LAC	_X
	LARK	AR2,3
	MAR	*0+
	SACL	*-
*** 17	-----------------------    DQMAG = DQ&16383u;
	.line	17
	LAC	_DQ
	ANDK	16383
	SACL	*+
*** 20	-----------------------    C$3 = x->YL6;
	.line	20
	LAR	AR5,* ,AR5
	ADRK	23
	LAC	* ,AR2
	MAR	*+
	SACL	* 
*** 20	-----------------------    THR2 = ((U$8 = C$3>>9) > 8u) ? 15872u : (C$3>>4&31u)+32u<<U$8;
	LAC	*+,6
	SACH	* ,1,AR0
	SACH	* ,1
	ZALS	* 
	SUBK	8
	BLEZ	LL251
	BD	LL252
	LACK	15872
***	B	LL252 OCCURS
LL251:
	MAR	* ,AR2
	MAR	*-
	LAC	*+,11
	ANDK	31,15
	ADLK	32,15
	LT	* ,AR0
	SACH	* ,1
	LACT	* 
	ANDK	0FFFFh
LL252:
	MAR	* ,AR2
	SACL	* 
*** 20	-----------------------    S$2 = 0;
	LACK	0
	SBRK	4
	SACL	* 
*** 21	-----------------------    if ( x->TD == 0u || DQMAG <= (THR2>>1)+THR2>>1 ) goto g2;
	.line	21
	ADRK	2
	LAR	AR4,* ,AR4
	ADRK	22
	LAC	* 
	BZ	L80
	MAR	* ,AR2
	ADRK	2
	LAC	* ,14
	ADD	* ,15,AR0
	SACH	* ,1
	LAC	* ,14
	SACH	* ,1,AR2
	SBRK	3
	ZALS	* ,AR0
	SUBS	* 
	BLEZ	L80
*** 21	-----------------------    S$2 = 1;
	LACK	1
	MAR	* ,AR2
	MAR	*-
	SACL	* 
L80:
***	-----------------------g2:
*** 21	-----------------------    TR = (unsigned)S$2;
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* 
	SACL	_TR
***  	-----------------------    return;
EPI0_14:
	.line	23
	MAR	* ,AR1
	RETD
	SBRK	7
	LAR	AR0,*
***	RET OCCURS

	.endfunc	782,000000000H,6

	.sym	_tone_detector_2,_tone_detector_2,32,3,0

	.func	784
******************************************************
* FUNCTION DEF : _tone_detector_2
******************************************************
_tone_detector_2:

LF15	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,2
	LAR	AR0,*0+,AR2

	RSXM
*
* AR5	assigned to temp var 'S$1'
*
*** 12	-----------------------    S$3 = 0;
	.sym	S$3,1,4,1,16
	.line	12
	LACK	0
	LARK	AR2,1
	MAR	*0+
	SACL	* 
*** 12	-----------------------    if ( A2P < 32768u || A2P >= 53760u ) goto g2;
	LDPK	_A2P
	ZALS	_A2P
	SUBK	-32768
	BLZ	L82
	ZALS	_A2P
	SUBK	-11776
	BGEZ	L82
*** 12	-----------------------    S$3 = 1;
	LACK	1
	SACL	* 
L82:
***	-----------------------g2:
*** 12	-----------------------    TDP = (unsigned)S$3;
	LAC	* 
	SACL	_TDP
*** 15	-----------------------    S$1 = TR ? 0u : (unsigned)S$3;
	.line	15
	LAC	_TR
	BZ	LL255
	BD	LL256
	LACK	0
	NOP
***	B	LL256 OCCURS
LL255:
	LAC	* 
LL256:
	MAR	* ,AR0
	SACL	* 
	LAR	AR5,* ,AR4
*** 15	-----------------------    X->TD = S$1;
	LAR	AR4,_X
	ADRK	22
	SAR	AR5,* 
***  	-----------------------    return;
EPI0_15:
	.line	16
	MAR	* ,AR1
	RETD
	SBRK	3
	LAR	AR0,*
***	RET OCCURS

	.endfunc	799,000000000H,2

	.sect	".cinit"
	.word	1,_LAW
	.word	0
	.text

	.sym	_LAW,_LAW,4,2,16
	.globl	_LAW
	.bss	_LAW,1

	.sym	_reset_encoder,_reset_encoder,32,2,0
	.globl	_reset_encoder

	.func	805
******************************************************
* FUNCTION DEF : _reset_encoder
******************************************************
_reset_encoder:

LF16	.set	0

	SAR	AR0,*+
	POPD	*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+

*** 3	-----------------------    X = &E_STATES;
	.line	3
	LALK	_E_STATES+0
	CALLD	_reset_states
	LDPK	_X
	SACL	_X
*** 4	-----------------------    reset_states();
	.line	4
***	CALL	_reset_states OCCURS
***  	-----------------------    return;
EPI0_16:
	.line	5
	SBRK	2
	PSHD	*-
	RETD
	LAR	AR0,*
	NOP
***	RET OCCURS

	.endfunc	809,000000000H,1

	.sym	_encoder,_encoder,46,2,0
	.globl	_encoder

	.func	811
******************************************************
* FUNCTION DEF : _encoder
******************************************************
_encoder:

LF17	.set	0

	SAR	AR0,*+
	POPD	*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+,AR2

	.sym	_pcm,-3+LF17,14,9,16
	.line	2
*** 3	-----------------------    S = pcm;
	.line	3
	LARK	AR2,-3+LF17
	MAR	*0+
	LAC	* ,AR1
	LDPK	_S
	SACL	_S
*** 4	-----------------------    X = &E_STATES;
	.line	4
	LALK	_E_STATES+0
	CALLD	_input_conversion
	SACL	_X
*** 6	-----------------------    input_conversion();
	.line	6
	NOP
***	CALL	_input_conversion OCCURS
*** 7	-----------------------    adpt_predict_1();
	.line	7
	CALL	_adpt_predict_1
*** 8	-----------------------    diff_computation();
	.line	8
	CALL	_diff_computation
*** 9	-----------------------    speed_control_1();
	.line	9
	CALL	_speed_control_1
*** 10	-----------------------    scale_factor_1();
	.line	10
	CALL	_scale_factor_1
*** 11	-----------------------    adapt_quant();
	.line	11
	CALL	_adapt_quant
*** 13	-----------------------    iadpt_quant();
	.line	13
	CALL	_iadpt_quant
*** 14	-----------------------    tone_detector_1();
	.line	14
	CALL	_tone_detector_1
*** 15	-----------------------    adpt_predict_2();
	.line	15
	CALL	_adpt_predict_2
*** 16	-----------------------    tone_detector_2();
	.line	16
	CALL	_tone_detector_2
*** 17	-----------------------    scale_factor_2();
	.line	17
	CALL	_scale_factor_2
*** 18	-----------------------    speed_control_2();
	.line	18
	CALL	_speed_control_2
*** 20	-----------------------    return I;
	.line	20
	LDPK	_I
	ZALS	_I
EPI0_17:
	.line	21
	SBRK	2
	PSHD	*-
	RETD
	LAR	AR0,*
	NOP
***	RET OCCURS

	.endfunc	831,000000000H,1

	.sym	_reset_decoder,_reset_decoder,32,2,0
	.globl	_reset_decoder

	.func	833
******************************************************
* FUNCTION DEF : _reset_decoder
******************************************************
_reset_decoder:

LF18	.set	0

	SAR	AR0,*+
	POPD	*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+

*** 3	-----------------------    X = &D_STATES;
	.line	3
	LALK	_D_STATES+0
	CALLD	_reset_states
	LDPK	_X
	SACL	_X
*** 4	-----------------------    reset_states();
	.line	4
***	CALL	_reset_states OCCURS
***  	-----------------------    return;
EPI0_18:
	.line	5
	SBRK	2
	PSHD	*-
	RETD
	LAR	AR0,*
	NOP
***	RET OCCURS

	.endfunc	837,000000000H,1

	.sym	_decoder,_decoder,46,2,0
	.globl	_decoder

	.func	839
******************************************************
* FUNCTION DEF : _decoder
******************************************************
_decoder:

LF19	.set	0

	SAR	AR0,*+
	POPD	*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+,AR2

	.sym	_adpcm,-3+LF19,14,9,16
	.line	2
*** 4	-----------------------    I = adpcm;
	.line	4
	LARK	AR2,-3+LF19
	MAR	*0+
	LAC	* ,AR1
	LDPK	_I
	SACL	_I
*** 5	-----------------------    X = &D_STATES;
	.line	5
	LALK	_D_STATES+0
	CALLD	_speed_control_1
	SACL	_X
*** 7	-----------------------    speed_control_1();
	.line	7
	NOP
***	CALL	_speed_control_1 OCCURS
*** 8	-----------------------    scale_factor_1();
	.line	8
	CALL	_scale_factor_1
*** 9	-----------------------    iadpt_quant();
	.line	9
	CALL	_iadpt_quant
*** 10	-----------------------    tone_detector_1();
	.line	10
	CALL	_tone_detector_1
*** 11	-----------------------    adpt_predict_1();
	.line	11
	CALL	_adpt_predict_1
*** 12	-----------------------    adpt_predict_2();
	.line	12
	CALL	_adpt_predict_2
*** 13	-----------------------    tone_detector_2();
	.line	13
	CALL	_tone_detector_2
*** 14	-----------------------    scale_factor_2();
	.line	14
	CALL	_scale_factor_2
*** 15	-----------------------    speed_control_2();
	.line	15
	CALL	_speed_control_2
*** 16	-----------------------    output_conversion();
	.line	16
	CALL	_output_conversion
*** 17	-----------------------    input_conversion();
	.line	17
	CALL	_input_conversion
*** 18	-----------------------    diff_computation();
	.line	18
	CALL	_diff_computation
*** 19	-----------------------    coding_adjustment();
	.line	19
	CALL	_coding_adjustment
*** 21	-----------------------    return SD;
	.line	21
	LDPK	_SD
	ZALS	_SD
EPI0_19:
	.line	22
	SBRK	2
	PSHD	*-
	RETD
	LAR	AR0,*
	NOP
***	RET OCCURS

	.endfunc	860,000000000H,1

	.sym	_SEZ,_SEZ,14,3,16
	.bss	_SEZ,1

	.sym	_TDP,_TDP,14,3,16
	.bss	_TDP,1

	.sym	_A2P,_A2P,14,3,16
	.bss	_A2P,1

	.sym	_D,_D,14,3,16
	.bss	_D,1

	.sym	_I,_I,14,3,16
	.bss	_I,1

	.sym	_S,_S,14,3,16
	.bss	_S,1

	.sym	_X,_X,24,3,16,.fake0
	.bss	_X,1

	.sym	_Y,_Y,14,3,16
	.bss	_Y,1

	.sym	_D_STATES,_D_STATES,8,3,416,.fake0
	.bss	_D_STATES,26

	.sym	_AL,_AL,14,3,16
	.bss	_AL,1

	.sym	_DQ,_DQ,14,3,16
	.bss	_DQ,1

	.sym	_SD,_SD,14,3,16
	.bss	_SD,1

	.sym	_SE,_SE,14,3,16
	.bss	_SE,1

	.sym	_SL,_SL,14,3,16
	.bss	_SL,1

	.sym	_SR,_SR,14,3,16
	.bss	_SR,1

	.sym	_TR,_TR,14,3,16
	.bss	_TR,1

	.sym	_E_STATES,_E_STATES,8,3,416,.fake0
	.bss	_E_STATES,26
*****************************************************
* UNDEFINED REFERENCES                              *
*****************************************************
	.ref	L$$SL
	.end
