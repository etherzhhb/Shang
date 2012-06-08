*******************************************************
*  TMS320C2x/C5x ANSI C Codegen    Version 6.40       *
*******************************************************
;	dspac -v50 -d__TMS320C50__ board_test.c board_test.if 
;	dspopt -e -v50 -s -O2 board_test.if board_test.opt 
;	dspcg -o -n -v50 board_test.opt board_test.asm board_test.tmp 
	.mmregs
	.file	"board_test.c"
	.file	"g721.h"
	.globl	_LAW
	.globl	_reset_encoder
	.globl	_encoder
	.globl	_reset_decoder
	.globl	_decoder
	.file	"board_test.c"

	.sect	".cinit"
	.word	IS1,_Input
	.word	255
	.word	167
	.word	25
	.word	146
	.word	15
	.word	145
	.word	24
	.word	163
	.word	77
	.word	43
	.word	154
	.word	19
	.word	143
	.word	17
	.word	150
	.word	32
	.word	190
	.word	175
	.word	28
	.word	148
	.word	16
	.word	144
	.word	21
	.word	158
	.word	54
	.word	54
	.word	158
	.word	21
	.word	144
	.word	16
	.word	148
	.word	28
IS1	.set	32
	.text

	.sym	_Input,_Input,62,2,512,,32
	.globl	_Input
	.bss	_Input,32,1
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

	.sect	".const"
	.sblock	".const"
_A_LAW_table:
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
	.text

	.sym	_A_LAW_table,_A_LAW_table,62,3,4096,,256

	.sect	".const"
_u_LAW_table:
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
	.text

	.sym	_u_LAW_table,_u_LAW_table,62,3,4096,,256

	.sym	_adapt_quant,_adapt_quant,32,3,0

	.func	153
******************************************************
* FUNCTION DEF : _adapt_quant
******************************************************
_adapt_quant:

LF1	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+

	RSXM

	.sym	S$1,STATIC_1,14,3,16
	.bss	STATIC_1,1

	.sym	_DLN,STATIC_2,14,3,16
	.bss	STATIC_2,1

	.sym	_DS,STATIC_3,14,3,16
	.bss	STATIC_3,1

	.sym	_DQM,STATIC_4,14,3,16
	.bss	STATIC_4,1

	.sym	_EXP,STATIC_5,4,3,16
	.bss	STATIC_5,1

	.sym	_tmp,STATIC_6,14,3,16
	.bss	STATIC_6,1
*
*  ---  AUTO variable moved to STATIC storage:  'tmp'
*  ---  AUTO variable moved to STATIC storage:  'EXP'
*  ---  AUTO variable moved to STATIC storage:  'DQM'
*  ---  AUTO variable moved to STATIC storage:  'DS'
*  ---  AUTO variable moved to STATIC storage:  'DLN'
*  ---  Compiler temp allocated STATIC storage: 'S$1'
*
*** 17	-----------------------    DQM = (DS = D&32768u) ? -D&32767u : D;
	.line	17
	LDPK	_D
	ZALS	_D
	ANDK	32768
	LDPK	STATIC_3
	SACL	STATIC_3
	ANDK	0FFFFh
	BZ	LL3
	LDPK	_D
	LAC	_D
	NEG
	BD	LL4
	ANDK	32767
***	B	LL4 OCCURS
LL3:
	LDPK	_D
	LAC	_D
LL4:
	LDPK	STATIC_4
	SACL	STATIC_4
*** 18	-----------------------    tmp = DQM>>1;
	.line	18
	RSXM
	LAC	STATIC_4,14
	SACH	STATIC_6,1
*** 19	-----------------------    EXP = 0;
	.line	19
	LACK	0
	SACL	STATIC_5
*** 19	-----------------------    if ( !tmp ) goto g3;
	LAC	STATIC_6
	BZ	L3
L2:
***	-----------------------g2:
*** 19	-----------------------    ++EXP;
	LDPK	STATIC_5
	LAC	STATIC_5
	ADDK	1
	SACL	STATIC_5
*** 19	-----------------------    if ( tmp >>= 1 ) goto g2;
	LAC	STATIC_6,14
	SACH	STATIC_6,1
	ANDK	0FFFFh,15
	SFL
	BNZ	L2
L3:
***	-----------------------g3:
*** 20	-----------------------    S$1 = (EXP < 7) ? DQM<<7-EXP : DQM>>EXP-7;
	.line	20
	SSXM
	LDPK	STATIC_5
	LAC	STATIC_5
	SUBK	7
	BGEZ	LL5
	LACK	7
	SUB	STATIC_5
	MAR	* ,AR0
	SACL	* 
	LT	* 
	LACT	STATIC_4
	BD	LL6
	ANDK	0FFFFh
***	B	LL6 OCCURS
LL5:
	LAC	STATIC_5
	SUBK	7
	MAR	* ,AR0
	SACL	* 
	LT	* 
	ZALS	STATIC_4
	SATL
	ANDK	0FFFFh
LL6:
	SACL	STATIC_1
*** 20	-----------------------    I = ((DLN = (unsigned)(EXP*128)+(S$1&127u)-(Y>>2)&4095u) > 299u) ? ((DLN > 2047u) ? ((DLN > 39...
	ZALS	STATIC_1
	ANDK	127
	SACB
	LAC	STATIC_5,7
	ADDB
	MAR	* ,AR1
	SACL	* ,AR0
	RSXM
	LDPK	_Y
	LAC	_Y,13
	SACH	* ,1,AR1
	ZALS	* ,AR0
	SUBS	* 
	ANDK	4095
	LDPK	STATIC_2
	SACL	STATIC_2
	ANDK	0FFFFh
	SUBK	299
	BLEZ	LL7
	ZALS	STATIC_2
	SUBK	2047
	BLEZ	LL8
	ZALS	STATIC_2
	SUBK	3971
	BLEZ	LL9
	LAC	STATIC_3
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
	ZALS	STATIC_2
	SUBK	399
	BLEZ	LL14
	LAC	STATIC_3
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
	ZALS	STATIC_2
	SUBK	348
	BLEZ	LL18
	LAC	STATIC_3
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
	LAC	STATIC_3
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
	ZALS	STATIC_2
	SUBK	177
	BLEZ	LL25
	ZALS	STATIC_2
	SUBK	245
	BLEZ	LL26
	LAC	STATIC_3
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
	LAC	STATIC_3
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
	ZALS	STATIC_2
	SUBK	79
	BLEZ	LL33
	LAC	STATIC_3
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
	LAC	STATIC_3
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
	LDPK	_I
	SACL	_I
*** 30	-----------------------    return;
	.line	30
EPI0_1:
	.line	52
	MAR	* ,AR1
	RETD
	SBRK	2
	LAR	AR0,*
***	RET OCCURS

	.endfunc	204,000000000H,1

	.sym	_adpt_predict_1,_adpt_predict_1,32,3,0

	.func	206
******************************************************
* FUNCTION DEF : _adpt_predict_1
******************************************************
_adpt_predict_1:

LF2	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+
;	<save register vars>
	SAR	AR6,*+
	SAR	AR7,*+,AR0


	.sym	C$4,STATIC_7,4,3,16
	.bss	STATIC_7,1

	.sym	C$5,STATIC_8,14,3,16
	.bss	STATIC_8,1

	.sym	S$1,STATIC_9,14,3,16
	.bss	STATIC_9,1

	.sym	S$2,STATIC_10,15,3,32
	.bss	STATIC_10,2,1

	.sym	_x,STATIC_11,24,3,16,.fake0
	.bss	STATIC_11,1

	.sym	_i,STATIC_12,4,3,16
	.bss	STATIC_12,1

	.sym	_SEZI,STATIC_13,14,3,16
	.bss	STATIC_13,1

	.sym	_AnEXP,STATIC_14,4,3,16
	.bss	STATIC_14,1

	.sym	_WAnEXP,STATIC_15,4,3,16
	.bss	STATIC_15,1

	.sym	_AnMAG,STATIC_16,14,3,16
	.bss	STATIC_16,1

	.sym	_AnS,STATIC_17,14,3,16
	.bss	STATIC_17,1

	.sym	_AnMANT,STATIC_18,14,3,16
	.bss	STATIC_18,1

	.sym	_WAnMAG,STATIC_19,14,3,16
	.bss	STATIC_19,1

	.sym	_WAnMANT,STATIC_20,14,3,16
	.bss	STATIC_20,1

	.sym	_tmp,STATIC_21,14,3,16
	.bss	STATIC_21,1
*
* AR5	assigned to variable 'tmp1'
* AR6	assigned to variable 'tmp2'
* AR7	assigned to temp var 'L$3'
*  ---  AUTO variable moved to STATIC storage:  'tmp'
*  ---  AUTO variable moved to STATIC storage:  'WAnMANT'
*  ---  AUTO variable moved to STATIC storage:  'WAnMAG'
*  ---  AUTO variable moved to STATIC storage:  'AnMANT'
*  ---  AUTO variable moved to STATIC storage:  'AnS'
*  ---  AUTO variable moved to STATIC storage:  'AnMAG'
*  ---  AUTO variable moved to STATIC storage:  'WAnEXP'
*  ---  AUTO variable moved to STATIC storage:  'AnEXP'
*  ---  AUTO variable moved to STATIC storage:  'SEZI'
*  ---  AUTO variable moved to STATIC storage:  'i'
*  ---  AUTO variable moved to STATIC storage:  'x'
*  ---  Compiler temp allocated STATIC storage: 'S$2'
*  ---  Compiler temp allocated STATIC storage: 'S$1'
*  ---  Compiler temp allocated STATIC storage: 'C$5'
*  ---  Compiler temp allocated STATIC storage: 'C$4'
*
*** 11	-----------------------    x = X;
	.line	11
	LDPK	_X
	LAC	_X
	LDPK	STATIC_11
	SACL	STATIC_11
*** 16	-----------------------    SEZI = 0u;
	.line	16
	LACK	0
	SACL	STATIC_13
*** 18	-----------------------    tmp1 = &x->B;
	.line	18
	LAR	AR5,STATIC_11
*** 19	-----------------------    tmp2 = &x->DQ;
	.line	19
	LAC	STATIC_11
	ADDK	8
	SACL	* 
	LAR	AR6,* ,AR5
*** 21	-----------------------    i = 0;
	.line	21
	LACK	0
	SACL	STATIC_12
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
	LDPK	STATIC_17
	SAR	AR4,STATIC_17
	MAR	* ,AR4
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
	SACL	STATIC_16
*** 29	-----------------------    tmp = AnMAG;
	SACL	STATIC_21
*** 31	-----------------------    AnEXP = 0;
	.line	31
	LACK	0
	SACL	STATIC_14
*** 31	-----------------------    if ( !tmp ) goto g6;
	LAC	STATIC_21
	BZ	L9
L7:
***	-----------------------g4:
*** 31	-----------------------    ++AnEXP;
	LDPK	STATIC_14
	LAC	STATIC_14
	ADDK	1
	SACL	STATIC_14
*** 31	-----------------------    if ( tmp >>= 1 ) goto g4;
	LAC	STATIC_21,14
	SACH	STATIC_21,1
	ANDK	0FFFFh,15
	SFL
	BNZ	L7
*** 32	-----------------------    if ( AnMAG ) goto g7;
	.line	32
	LAC	STATIC_16
	ANDK	0FFFFh
	BNZ	L10
L9:
***	-----------------------g6:
*** 32	-----------------------    AnMANT = 32u;
	LACK	32
	BD	L11
	LDPK	STATIC_18
	SACL	STATIC_18
*** 32	-----------------------    goto g8;
***	B	L11 OCCURS
L10:
***	-----------------------g7:
*** 32	-----------------------    AnMANT = (AnEXP < 6) ? AnMAG<<6-AnEXP : AnMAG>>AnEXP-6;
	SSXM
	LAC	STATIC_14
	SUBK	6
	BGEZ	LL44
	LACK	6
	SUB	STATIC_14
	MAR	* ,AR0
	SACL	* 
	LT	* 
	LACT	STATIC_16
	BD	LL45
	ANDK	0FFFFh
***	B	LL45 OCCURS
LL44:
	LAC	STATIC_14
	SUBK	6
	MAR	* ,AR0
	SACL	* 
	LT	* 
	ZALS	STATIC_16
	SATL
	ANDK	0FFFFh
LL45:
	SACL	STATIC_18
L11:
***	-----------------------g8:
*** 33	-----------------------    C$5 = *tmp2;
	.line	33
	MAR	* ,AR6
	LAC	* ,AR0
	LDPK	STATIC_8
	SACL	STATIC_8
*** 33	-----------------------    WAnMANT = (C$5&63u)*AnMANT+48u>>4;
	ZALS	STATIC_8
	ANDK	63
	SACL	* 
	LT	* 
	MPYU	STATIC_18
	PAC
	ADDK	48
	SACL	* 
	RSXM
	LAC	* ,11
	SACH	STATIC_20,1
*** 34	-----------------------    WAnEXP = (int)((C$5>>6&15u)+(unsigned)AnEXP);
	.line	34
	LAC	STATIC_8,9
	ANDK	15,15
	ADD	STATIC_14,15
	SACH	STATIC_15,1
*** 35	-----------------------    S$2 = ((C$4 = 19-WAnEXP) < 0) ? (unsigned long)WAnMANT<<WAnEXP-19 : (unsigned long)WAnMANT>>C$4;
	.line	35
	LACK	19
	SSXM
	SUB	STATIC_15
	SACL	STATIC_7
	BGEZ	LL46
	LAC	STATIC_15
	SUBK	19
	SACB
	ZALS	STATIC_20
	MAR	* ,AR1
	SACL	*+
	SACH	*+
	CALLD	L$$SL
	LACB
	SACL	* 
***	CALL	L$$SL OCCURS
	B	LL47
LL46:
	ZALS	STATIC_20
	LT	STATIC_7
	SATH
	SATL
LL47:
	SACL	STATIC_10
	SACH	STATIC_10+1
*** 35	-----------------------    WAnMAG = (unsigned)(S$2&32767uL);
	LAC	STATIC_10
	ANDK	32767
	SACL	STATIC_19
*** 36	-----------------------    S$1 = ((unsigned)((*tmp2++&1024u) != 0u)^AnS) ? -WAnMAG : WAnMAG;
	.line	36
	MAR	* ,AR6
	ZALS	*+
	ANDK	1024
	LARK	AR3,1
	BNZ	LL49
	LARK	AR3,0
LL49:
	MAR	* ,AR0
	SAR	AR3,*
	ZALS	* 
	XOR	STATIC_17
	ANDK	0FFFFh
	BZ	LL48
	BD	LL50
	LAC	STATIC_19
	NEG
***	B	LL50 OCCURS
LL48:
	LAC	STATIC_19
LL50:
	SACL	STATIC_9
*** 36	-----------------------    SEZI += S$1;
	ADD	STATIC_13
	SACL	STATIC_13
*** 38	-----------------------    if ( i != 5 ) goto g10;
	.line	38
	LAC	STATIC_12
	SUBK	5
	BNZ	L13
*** 39	-----------------------    SEZ = (SEZI&65535u)>>1;
	.line	39
	ZALS	STATIC_13
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
	LDPK	STATIC_12
	LAC	STATIC_12
	ADDK	1
	SACL	STATIC_12
*** 21	-----------------------    if ( --L$3 >= 0 ) goto g12;
	MAR	* ,AR7
	BANZ	L15,*-,AR5
*** 42	-----------------------    SE = (SEZI&65535u)>>1;
	.line	42
	ZALS	STATIC_13
	ANDK	65535
	MAR	* ,AR0
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
	SBRK	2
	LAR	AR0,*
***	RET OCCURS

	.endfunc	248,0000000c0H,1

	.sym	_adpt_predict_2,_adpt_predict_2,32,3,0

	.func	250
******************************************************
* FUNCTION DEF : _adpt_predict_2
******************************************************
_adpt_predict_2:

LF3	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,4
	LAR	AR0,*0+
;	<save register vars>
	SAR	AR6,*+
	SAR	AR7,*+


	.sym	C$20,STATIC_22,30,3,16
	.bss	STATIC_22,1

	.sym	C$21,STATIC_23,14,3,16
	.bss	STATIC_23,1

	.sym	C$22,STATIC_24,14,3,16
	.bss	STATIC_24,1

	.sym	C$23,STATIC_25,14,3,16
	.bss	STATIC_25,1

	.sym	U$12,STATIC_26,14,3,16
	.bss	STATIC_26,1

	.sym	U$17,STATIC_27,14,3,16
	.bss	STATIC_27,1

	.sym	U$3,STATIC_28,30,3,16
	.bss	STATIC_28,1

	.sym	S$1,STATIC_29,4,3,16
	.bss	STATIC_29,1

	.sym	S$2,STATIC_30,14,3,16
	.bss	STATIC_30,1

	.sym	S$3,STATIC_31,14,3,16
	.bss	STATIC_31,1

	.sym	S$4,STATIC_32,14,3,16
	.bss	STATIC_32,1

	.sym	S$5,STATIC_33,14,3,16
	.bss	STATIC_33,1

	.sym	S$6,STATIC_34,14,3,16
	.bss	STATIC_34,1

	.sym	S$7,STATIC_35,14,3,16
	.bss	STATIC_35,1

	.sym	S$8,STATIC_36,15,3,32
	.bss	STATIC_36,2,1

	.sym	S$9,STATIC_37,15,3,32
	.bss	STATIC_37,2,1

	.sym	S$10,STATIC_38,4,3,16
	.bss	STATIC_38,1

	.sym	S$11,STATIC_39,14,3,16
	.bss	STATIC_39,1

	.sym	S$12,STATIC_40,14,3,16
	.bss	STATIC_40,1

	.sym	S$13,STATIC_41,14,3,16
	.bss	STATIC_41,1

	.sym	S$14,STATIC_42,14,3,16
	.bss	STATIC_42,1

	.sym	S$15,STATIC_43,14,3,16
	.bss	STATIC_43,1

	.sym	_x,STATIC_44,24,3,16,.fake0
	.bss	STATIC_44,1

	.sym	_a1,STATIC_45,30,3,16
	.bss	STATIC_45,1

	.sym	_A1T,STATIC_46,14,3,16
	.bss	STATIC_46,1

	.sym	_A2T,STATIC_47,14,3,16
	.bss	STATIC_47,1

	.sym	_DQSEZ,STATIC_48,14,3,16
	.bss	STATIC_48,1

	.sym	_PK0,STATIC_49,14,3,16
	.bss	STATIC_49,1

	.sym	_SR0,STATIC_50,14,3,16
	.bss	STATIC_50,1

	.sym	_EXP,STATIC_51,4,3,16
	.bss	STATIC_51,1

	.sym	_SRS,STATIC_52,14,3,16
	.bss	STATIC_52,1

	.sym	_MAG,STATIC_53,14,3,16
	.bss	STATIC_53,1

	.sym	_tmp,STATIC_54,14,3,16
	.bss	STATIC_54,1

	.sym	_PKS1,STATIC_55,14,3,16
	.bss	STATIC_55,1

	.sym	_UGA2B,STATIC_56,14,3,16
	.bss	STATIC_56,1

	.sym	_UGA2,STATIC_57,14,3,16
	.bss	STATIC_57,1

	.sym	_ULA2,STATIC_58,14,3,16
	.bss	STATIC_58,1

	.sym	_FA1,STATIC_59,15,3,32
	.bss	STATIC_59,2,1

	.sym	_PKS,STATIC_60,14,3,16
	.bss	STATIC_60,1

	.sym	_A1UL,STATIC_61,14,3,16
	.bss	STATIC_61,1

	.sym	_A1LL,STATIC_62,14,3,16
	.bss	STATIC_62,1

	.sym	_DQS,STATIC_63,14,3,16
	.bss	STATIC_63,1

	.sym	_MAG,STATIC_64,14,3,16
	.bss	STATIC_64,1

	.sym	_EXP,STATIC_65,4,3,16
	.bss	STATIC_65,1

	.sym	_tmp,STATIC_66,30,3,16
	.bss	STATIC_66,1

	.sym	_tmp,STATIC_67,14,3,16
	.bss	STATIC_67,1

	.sym	_DQM,STATIC_68,14,3,16
	.bss	STATIC_68,1
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
*  ---  AUTO variable moved to STATIC storage:  'DQM'
*  ---  AUTO variable moved to STATIC storage:  'tmp'
*  ---  AUTO variable moved to STATIC storage:  'tmp'
*  ---  AUTO variable moved to STATIC storage:  'EXP'
*  ---  AUTO variable moved to STATIC storage:  'MAG'
*  ---  AUTO variable moved to STATIC storage:  'DQS'
*  ---  AUTO variable moved to STATIC storage:  'A1LL'
*  ---  AUTO variable moved to STATIC storage:  'A1UL'
*  ---  AUTO variable moved to STATIC storage:  'PKS'
*  ---  AUTO variable moved to STATIC storage:  'FA1'
*  ---  AUTO variable moved to STATIC storage:  'ULA2'
*  ---  AUTO variable moved to STATIC storage:  'UGA2'
*  ---  AUTO variable moved to STATIC storage:  'UGA2B'
*  ---  AUTO variable moved to STATIC storage:  'PKS1'
*  ---  AUTO variable moved to STATIC storage:  'tmp'
*  ---  AUTO variable moved to STATIC storage:  'MAG'
*  ---  AUTO variable moved to STATIC storage:  'SRS'
*  ---  AUTO variable moved to STATIC storage:  'EXP'
*  ---  AUTO variable moved to STATIC storage:  'SR0'
*  ---  AUTO variable moved to STATIC storage:  'PK0'
*  ---  AUTO variable moved to STATIC storage:  'DQSEZ'
*  ---  AUTO variable moved to STATIC storage:  'A2T'
*  ---  AUTO variable moved to STATIC storage:  'A1T'
*  ---  AUTO variable moved to STATIC storage:  'a1'
*  ---  AUTO variable moved to STATIC storage:  'x'
*  ---  Compiler temp allocated STATIC storage: 'S$15'
*  ---  Compiler temp allocated STATIC storage: 'S$14'
*  ---  Compiler temp allocated STATIC storage: 'S$13'
*  ---  Compiler temp allocated STATIC storage: 'S$12'
*  ---  Compiler temp allocated STATIC storage: 'S$11'
*  ---  Compiler temp allocated STATIC storage: 'S$10'
*  ---  Compiler temp allocated STATIC storage: 'S$9'
*  ---  Compiler temp allocated STATIC storage: 'S$8'
*  ---  Compiler temp allocated STATIC storage: 'S$7'
*  ---  Compiler temp allocated STATIC storage: 'S$6'
*  ---  Compiler temp allocated STATIC storage: 'S$5'
*  ---  Compiler temp allocated STATIC storage: 'S$4'
*  ---  Compiler temp allocated STATIC storage: 'S$3'
*  ---  Compiler temp allocated STATIC storage: 'S$2'
*  ---  Compiler temp allocated STATIC storage: 'S$1'
*  ---  Compiler temp allocated STATIC storage: 'U$3'
*  ---  Compiler temp allocated STATIC storage: 'U$17'
*  ---  Compiler temp allocated STATIC storage: 'U$12'
*  ---  Compiler temp allocated STATIC storage: 'C$23'
*  ---  Compiler temp allocated STATIC storage: 'C$22'
*  ---  Compiler temp allocated STATIC storage: 'C$21'
*  ---  Compiler temp allocated STATIC storage: 'C$20'
*
*** 11	-----------------------    x = X;
	.line	11
	LDPK	_X
	LAC	_X
	LDPK	STATIC_44
	SACL	STATIC_44
*** 12	-----------------------    a1 = (U$3 = &x->B[0])+6;
	.line	12
	SACL	STATIC_28
	ADDK	6
	SACL	STATIC_45
*** 13	-----------------------    a2 = a1+1;
	.line	13
	ADDK	1
	MAR	* ,AR0
	SACL	* 
	LAR	AR5,* 
*** 19	-----------------------    if ( U$12 = DQ&16384u ) goto g2;
	.line	19
	LDPK	_DQ
	ZALS	_DQ
	ANDK	16384
	LDPK	STATIC_26
	SACL	STATIC_26
	ANDK	0FFFFh
	BNZ	L17
*** 19	-----------------------    S$14 = DQ;
	LDPK	_DQ
	LAC	_DQ
	LDPK	STATIC_42
	SACL	STATIC_42
***  	-----------------------    U$17 = S$14&16383u;
	ANDK	16383
	SACL	STATIC_27
*** 19	-----------------------    if ( !(SEZ&16384u) ) goto g3;
	LDPK	_SEZ
	ZALS	_SEZ
	ANDK	16384
	ANDK	0FFFFh
	BZ	L18
*** 19	-----------------------    goto g4;
	B	L19
L17:
***	-----------------------g2:
*** 19	-----------------------    S$14 = -(U$17 = DQ&16383u);
	LDPK	_DQ
	LAC	_DQ
	ANDK	16383
	LDPK	STATIC_27
	SACL	STATIC_27
	NEG
	SACL	STATIC_42
*** 19	-----------------------    if ( SEZ&16384u ) goto g4;
	LDPK	_SEZ
	ZALS	_SEZ
	ANDK	16384
	ANDK	0FFFFh
	BNZ	L19
L18:
***	-----------------------g3:
*** 19	-----------------------    S$15 = SEZ;
	LAC	_SEZ
	BD	L20
	LDPK	STATIC_43
	SACL	STATIC_43
*** 19	-----------------------    goto g5;
***	B	L20 OCCURS
L19:
***	-----------------------g4:
*** 19	-----------------------    S$15 = SEZ+32768u;
	LAC	_SEZ
	ADDK	-32768
	LDPK	STATIC_43
	SACL	STATIC_43
L20:
***	-----------------------g5:
*** 19	-----------------------    DQSEZ = S$14+S$15&65535u;
	LAC	STATIC_43
	ADD	STATIC_42
	ANDK	65535
	SACL	STATIC_48
*** 22	-----------------------    PK0 = (unsigned)((DQSEZ&32768u) != 0u);
	.line	22
	ZALS	STATIC_48
	ANDK	32768
	LARK	AR4,1
	BNZ	LL53
	LARK	AR4,0
LL53:
	SAR	AR4,STATIC_49
*** 26	-----------------------    S$12 = U$12 ? -U$17 : DQ;
	.line	26
	LAC	STATIC_26
	BZ	LL54
	BD	LL55
	LAC	STATIC_27
	NEG
***	B	LL55 OCCURS
LL54:
	LDPK	_DQ
	LAC	_DQ
LL55:
	LDPK	STATIC_40
	SACL	STATIC_40
*** 26	-----------------------    S$13 = (SE&16384u) ? SE+32768u : SE;
	LDPK	_SE
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
	LDPK	STATIC_41
	SACL	STATIC_41
*** 26	-----------------------    C$23 = S$12+S$13&65535u;
	ADD	STATIC_40
	ANDK	65535
	SACL	STATIC_25
*** 26	-----------------------    SR = C$23;
	LDPK	_SR
	SACL	_SR
*** 35	-----------------------    MAG = (SRS = C$23&32768u) ? -SR&32767u : C$23;
	.line	35
	LDPK	STATIC_25
	ZALS	STATIC_25
	ANDK	32768
	SACL	STATIC_52
	ANDK	0FFFFh
	BZ	LL58
	LDPK	_SR
	LAC	_SR
	NEG
	BD	LL59
	ANDK	32767
***	B	LL59 OCCURS
LL58:
	LAC	STATIC_25
LL59:
	LDPK	STATIC_53
	SACL	STATIC_53
*** 36	-----------------------    tmp = MAG;
	.line	36
	SACL	STATIC_54
*** 37	-----------------------    EXP = 0;
	.line	37
	LACK	0
	SACL	STATIC_51
*** 37	-----------------------    if ( !tmp ) goto g8;
	LAC	STATIC_54
	BZ	L23
	RSXM ;;;
L22:
***	-----------------------g7:
*** 37	-----------------------    ++EXP;
	LDPK	STATIC_51
	LAC	STATIC_51
	ADDK	1
	SACL	STATIC_51
*** 37	-----------------------    if ( tmp >>= 1 ) goto g7;
	LAC	STATIC_54,14
	SACH	STATIC_54,1
	ANDK	0FFFFh,15
	SFL
	BNZ	L22
L23:
***	-----------------------g8:
*** 38	-----------------------    S$10 = SRS ? 1024 : 0;
	.line	38
	LDPK	STATIC_52
	LAC	STATIC_52
	BZ	LL60
	BD	LL61
	LACK	1024
***	B	LL61 OCCURS
LL60:
	LACK	0
LL61:
	SACL	STATIC_38
*** 38	-----------------------    S$11 = MAG ? ((EXP < 6) ? MAG<<6-EXP : MAG>>EXP-6) : 32u;
	LAC	STATIC_53
	BZ	LL62
	SSXM
	LAC	STATIC_51
	SUBK	6
	BGEZ	LL63
	LACK	6
	SUB	STATIC_51
	MAR	* ,AR0
	SACL	* 
	LT	* 
	LACT	STATIC_53
	BD	LL64
	ANDK	0FFFFh
***	B	LL64 OCCURS
LL63:
	LAC	STATIC_51
	SUBK	6
	MAR	* ,AR0
	SACL	* 
	LT	* 
	ZALS	STATIC_53
	SATL
	ANDK	0FFFFh
LL64:
	B	LL65
LL62:
	LACK	32
LL65:
	SACL	STATIC_39
*** 38	-----------------------    SR0 = (unsigned)(EXP*64+S$10)+S$11;
	LAC	STATIC_38
	ADD	STATIC_51,6
	ADD	STATIC_39
	SACL	STATIC_50
*** 47	-----------------------    PKS1 = x->PK1^PK0;
	.line	47
	LAR	AR3,STATIC_44
	LAC	STATIC_49
	MAR	* ,AR3
	ADRK	16
	XOR	* ,AR4
	SACL	STATIC_55
*** 48	-----------------------    C$22 = *a1;
	.line	48
	LAR	AR4,STATIC_45
	LAC	* 
	SACL	STATIC_24
*** 48	-----------------------    FA1 = (C$22&32768u) ? ((*a1 >= 57345u) ? (unsigned long)*a1*4uL&131071uL : 98308uL) : (C$22 <=...
	ZALS	STATIC_24
	ANDK	32768
	ANDK	0FFFFh
	BZ	LL66
	LAR	AR3,STATIC_45
	MAR	* ,AR3
	ZALS	* 
	RSXM
	SUBK	-8191
	BLZ	LL67
	LAR	AR4,STATIC_45
	MAR	* ,AR4
	ZALS	* ,AR1
	SACL	*+
	SACH	*+
	CALLD	L$$SL
	LACK	2
	SACL	* 
***	CALL	L$$SL OCCURS
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
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
	ZALS	STATIC_24
	SUBK	8191
	BGZ	LL70
	LAR	AR3,STATIC_45
	MAR	* ,AR3
	LAC	* ,2,AR2
	LARK	AR2,3
	MAR	*0+
	BD	LL69
	SACL	* 
	ZALS	* 
***	B	LL69 OCCURS
LL70:
	LACK	32764
LL69:
	SACL	STATIC_59
	SACH	STATIC_59+1
*** 49	-----------------------    S$8 = (x->PK2^PK0) ? 114688uL : 16384uL;
	.line	49
	LAR	AR4,STATIC_44
	ZALS	STATIC_49
	MAR	* ,AR4
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
	SACL	STATIC_36
	SACH	STATIC_36+1
*** 54	-----------------------    S$9 = PKS1 ? FA1 : -FA1&131071uL;
	.line	54
	LAC	STATIC_55
	BZ	LL74
	BD	LL75
	ZALS	STATIC_59
	ADDH	STATIC_59+1
***	B	LL75 OCCURS
LL74:
	ZALS	STATIC_59
	ADDH	STATIC_59+1
	NEG
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	SACH	*-
	ANDK	65535
	SACL	*+
	ZALS	* 
	ANDK	1
	SACL	*-
	ZALS	*+
	ADDH	* 
LL75:
	SACL	STATIC_37
	SACH	STATIC_37+1
*** 54	-----------------------    UGA2B = S$8+S$9>>7&1023u;
	ZALS	STATIC_36
	ADDH	STATIC_36+1
	ADDS	STATIC_37
	ADDH	STATIC_37+1
	LDPK	0
	SPLK	7,TREG1
	RSXM
	SATL
	ANDK	1023
	LDPK	STATIC_56
	SACL	STATIC_56
*** 57	-----------------------    UGA2 = DQSEZ ? ((UGA2B&512u) ? UGA2B-1024u : UGA2B) : 0u;
	.line	57
	LAC	STATIC_48
	BZ	LL76
	ZALS	STATIC_56
	ANDK	512
	ANDK	0FFFFh
	BZ	LL77
	LAC	STATIC_56
	BD	LL78
	SUBK	1024
***	B	LL78 OCCURS
LL77:
	LAC	STATIC_56
LL78:
	B	LL79
LL76:
	LACK	0
LL79:
	SACL	STATIC_57
*** 57	-----------------------    ULA2 = (*a2&32768u) ? 512u-(*a2>>7) : -(*a2>>7);
	MAR	* ,AR5
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
	SACL	STATIC_58
*** 59	-----------------------    A2T = *a2+UGA2+ULA2&65535u;
	.line	59
	LAC	STATIC_57
	MAR	* ,AR5
	ADD	* 
	ADD	STATIC_58
	ANDK	65535
	SACL	STATIC_47
*** 64	-----------------------    if ( TR ) goto g12;
	.line	64
	LDPK	_TR
	LAC	_TR
	ANDK	0FFFFh
	BNZ	L27
*** 74	-----------------------    A2P = (A2T <= 32767u) ? ((A2T >= 12288u) ? 12288u : A2T) : (A2T <= 53248u) ? 53248u : A2T;
	.line	74
	LDPK	STATIC_47
	ZALS	STATIC_47
	SUBK	32767
	BGZ	LL82
	ZALS	STATIC_47
	SUBK	12288
	BLZ	LL83
	BD	LL84
	LACK	12288
***	B	LL84 OCCURS
LL83:
	LAC	STATIC_47
LL84:
	B	LL85
LL82:
	ZALS	STATIC_47
	SUBK	-12288
	BGZ	LL86
	BD	LL87
	LACK	53248
***	B	LL87 OCCURS
LL86:
	LAC	STATIC_47
LL87:
LL85:
	LDPK	_A2P
	SACL	_A2P
*** 74	-----------------------    *a2 = A2P;
	SACL	* ,AR3
*** 82	-----------------------    PKS = x->PK1^PK0;
	.line	82
	LDPK	STATIC_44
	LAR	AR3,STATIC_44
	LAC	STATIC_49
	ADRK	16
	XOR	* 
	SACL	STATIC_60
*** 83	-----------------------    S$6 = DQSEZ ? (PKS ? 65344u : 192u) : 0u;
	.line	83
	LAC	STATIC_48
	BZ	LL88
	LAC	STATIC_60
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
	SACL	STATIC_34
*** 83	-----------------------    S$7 = (*a1&32768u) ? 256u-(*a1>>8) : -(*a1>>8);
	LAR	AR4,STATIC_45
	MAR	* ,AR4
	ZALS	* 
	ANDK	32768
	ANDK	0FFFFh
	BZ	LL92
	LAR	AR3,STATIC_45
	MAR	* ,AR3
	LAC	* ,7,AR0
	SACH	* ,1
	LACK	256
	BD	LL93
	SUB	* 
	NOP
***	B	LL93 OCCURS
LL92:
	LAR	AR4,STATIC_45
	LAC	* ,7,AR1
	SACH	* ,1
	LAC	* 
	NEG
LL93:
	SACL	STATIC_35
*** 83	-----------------------    A1T = *a1+S$6+S$7&65535u;
	LAR	AR3,STATIC_45
	LAC	STATIC_34
	MAR	* ,AR3
	ADD	* ,AR5
	ADD	STATIC_35
	ANDK	65535
	SACL	STATIC_46
*** 92	-----------------------    C$21 = *a2;
	.line	92
	LAC	* 
	SACL	STATIC_23
*** 92	-----------------------    A1UL = (unsigned)(80896uL-(unsigned long)C$21&65535uL);
	NEG
	ADDK	15360
	ANDK	65535
	SACL	STATIC_61
*** 93	-----------------------    A1LL = C$21-15360u&65535u;
	.line	93
	LAC	STATIC_23
	SUBK	15360
	ANDK	65535
	SACL	STATIC_62
*** 94	-----------------------    S$5 = (A1T <= 32767u) ? ((A1T >= A1UL) ? A1UL : A1T) : (A1T <= A1LL) ? A1LL : A1T;
	.line	94
	ZALS	STATIC_46
	SUBK	32767
	BGZ	LL94
	ZALS	STATIC_46
	SUBS	STATIC_61
	BLZ	LL95
	BD	LL96
	LAC	STATIC_61
	NOP
***	B	LL96 OCCURS
LL95:
	LAC	STATIC_46
LL96:
	B	LL97
LL94:
	ZALS	STATIC_46
	SUBS	STATIC_62
	BGZ	LL98
	BD	LL99
	LAC	STATIC_62
	NOP
***	B	LL99 OCCURS
LL98:
	LAC	STATIC_46
LL99:
LL97:
	SACL	STATIC_33
*** 94	-----------------------    *a1 = S$5;
	LAR	AR5,STATIC_45
	SACL	* ,AR0
*** 102	-----------------------    tmp1 = U$3;
	.line	102
	LAR	AR5,STATIC_28
*** 103	-----------------------    tmp2 = &x->DQ;
	.line	103
	LAC	STATIC_44
	ADDK	8
	SACL	* 
	LAR	AR6,* 
*** 106	-----------------------    DQS = (unsigned)(U$12 != 0u);
	.line	106
	LAC	STATIC_26
	LARK	AR4,1
	BNZ	LL100
	LARK	AR4,0
LL100:
	SAR	AR4,STATIC_63
*** 107	-----------------------    DQM = U$17;
	.line	107
	LAC	STATIC_27
	SACL	STATIC_68
***  	-----------------------    L$16 = 5;
	LACK	5
	SAMM	BRCR
***	-----------------------g11:
***	-----------------------g100:
	RPTB	L39-1
*** 109	-----------------------    S$3 = DQM ? (((unsigned)((*tmp2++&1024u) != 0u)^DQS) ? 65408u : 128u) : 0u;
	.line	109
	LDPK	STATIC_68
	LAC	STATIC_68
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
	ZALS	* 
	XOR	STATIC_63
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
	SACL	STATIC_31
*** 109	-----------------------    S$4 = (*tmp1&32768u) ? 256u-(*tmp1>>8) : -(*tmp1>>8);
	MAR	* ,AR5
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
	SACL	STATIC_32
*** 109	-----------------------    A$24 = *tmp1+S$3+S$4&65535u;
	LAC	STATIC_31
	MAR	* ,AR5
	ADD	* ,AR0
	ADD	STATIC_32
	ANDK	65535
	SACL	* 
	LAR	AR7,* ,AR5
*** 109	-----------------------    *tmp1++ = A$24;
	SAR	AR7,*+
*** 108	-----------------------    if ( --L$16 >= 0 ) goto g24;
	.line	108
L39:
*** 108	-----------------------    goto g15;
	B	L30
L27:
***	-----------------------g12:
***  	-----------------------    U$172 = U$3;
	LDPK	STATIC_28
	LAR	AR5,STATIC_28
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
	LDPK	STATIC_44
	LAC	STATIC_44
	ADDK	13
	SACL	STATIC_66
***  	-----------------------    L$18 = 4;
	LACK	4
	SAMM	BRCR
***	-----------------------g17:
***	-----------------------g111:
	RPTB	L37-1
*** 129	-----------------------    *tmp-- = tmp[-(+1)];
	.line	129
	LDPK	STATIC_66
	LAR	AR5,STATIC_66
	MAR	*-
	LAR	AR4,STATIC_66
	LAC	* ,AR4
	SACL	*-,AR5
	SAR	AR4,STATIC_66
*** 128	-----------------------    if ( --L$18 >= 0 ) goto g22;
	.line	128
L37:
*** 132	-----------------------    MAG = U$17;
	.line	132
	LAC	STATIC_27
	SACL	STATIC_64
*** 133	-----------------------    tmp = MAG;
	.line	133
	SACL	STATIC_67
*** 133	-----------------------    EXP = 0;
	LACK	0
	SACL	STATIC_65
*** 133	-----------------------    if ( !tmp ) goto g21;
	LAC	STATIC_67
	BZ	L36
	RSXM ;;;
L35:
***	-----------------------g20:
*** 133	-----------------------    ++EXP;
	LDPK	STATIC_65
	LAC	STATIC_65
	ADDK	1
	SACL	STATIC_65
*** 133	-----------------------    if ( tmp >>= 1 ) goto g20;
	LAC	STATIC_67,14
	SACH	STATIC_67,1
	ANDK	0FFFFh,15
	SFL
	BNZ	L35
L36:
***	-----------------------g21:
*** 134	-----------------------    S$1 = U$12 ? 1024 : 0;
	.line	134
	LDPK	STATIC_26
	LAC	STATIC_26
	BZ	LL108
	BD	LL109
	LACK	1024
***	B	LL109 OCCURS
LL108:
	LACK	0
LL109:
	SACL	STATIC_29
*** 134	-----------------------    S$2 = MAG ? ((EXP < 6) ? MAG<<6-EXP : MAG>>EXP-6) : 32u;
	LAC	STATIC_64
	BZ	LL110
	SSXM
	LAC	STATIC_65
	SUBK	6
	BGEZ	LL111
	LACK	6
	SUB	STATIC_65
	MAR	* ,AR0
	SACL	* 
	LT	* 
	LACT	STATIC_64
	BD	LL112
	ANDK	0FFFFh
***	B	LL112 OCCURS
LL111:
	LAC	STATIC_65
	SUBK	6
	MAR	* ,AR0
	SACL	* 
	LT	* 
	ZALS	STATIC_64
	SATL
	ANDK	0FFFFh
LL112:
	B	LL113
LL110:
	LACK	32
LL113:
	SACL	STATIC_30
*** 134	-----------------------    C$20 = &x->DQ[0];
	LAC	STATIC_44
	ADDK	8
	SACL	STATIC_22
*** 134	-----------------------    *C$20 = (unsigned)(EXP*64+S$1)+S$2;
	LAC	STATIC_29
	ADD	STATIC_65,6
	ADD	STATIC_30
	LAR	AR3,STATIC_22
	MAR	* ,AR3
	SACL	* ,AR5
*** 138	-----------------------    x->PK2 = x->PK1;
	.line	138
	LAR	AR5,STATIC_44
	LAR	AR4,STATIC_44
	ADRK	16
	LAC	* ,AR4
	ADRK	17
	SACL	* ,AR3
*** 139	-----------------------    x->PK1 = PK0;
	.line	139
	LAR	AR3,STATIC_44
	LAC	STATIC_49
	ADRK	16
	SACL	* ,AR0
*** 140	-----------------------    C$19 = &C$20[6];
	.line	140
	LAC	STATIC_22
	ADDK	6
	SACL	* 
	LAR	AR5,* ,AR4
*** 140	-----------------------    C$20[7] = *C$19;
	LAR	AR4,STATIC_22
	ADRK	7
	MAR	* ,AR5
	LAC	* ,AR4
	SACL	* ,AR5
*** 141	-----------------------    *C$19 = SR0;
	.line	141
	LAC	STATIC_50
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
	SBRK	5
	LAR	AR0,*
***	RET OCCURS

	.endfunc	391,0000000c0H,4

	.sym	_coding_adjustment,_coding_adjustment,32,3,0

	.func	393
******************************************************
* FUNCTION DEF : _coding_adjustment
******************************************************
_coding_adjustment:

LF4	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+

	RSXM

	.sym	C$2,STATIC_69,14,3,16
	.bss	STATIC_69,1

	.sym	S$1,STATIC_70,14,3,16
	.bss	STATIC_70,1

	.sym	_DLN,STATIC_71,14,3,16
	.bss	STATIC_71,1

	.sym	_DS,STATIC_72,14,3,16
	.bss	STATIC_72,1

	.sym	_EXP,STATIC_73,4,3,16
	.bss	STATIC_73,1

	.sym	_DQM,STATIC_74,14,3,16
	.bss	STATIC_74,1

	.sym	_tmp,STATIC_75,14,3,16
	.bss	STATIC_75,1

	.sym	_IM,STATIC_76,14,3,16
	.bss	STATIC_76,1

	.sym	_ID,STATIC_77,14,3,16
	.bss	STATIC_77,1
*
*  ---  AUTO variable moved to STATIC storage:  'ID'
*  ---  AUTO variable moved to STATIC storage:  'IM'
*  ---  AUTO variable moved to STATIC storage:  'tmp'
*  ---  AUTO variable moved to STATIC storage:  'DQM'
*  ---  AUTO variable moved to STATIC storage:  'EXP'
*  ---  AUTO variable moved to STATIC storage:  'DS'
*  ---  AUTO variable moved to STATIC storage:  'DLN'
*  ---  Compiler temp allocated STATIC storage: 'S$1'
*  ---  Compiler temp allocated STATIC storage: 'C$2'
*
*** 17	-----------------------    DQM = (DS = D&32768u) ? -D&32767u : D;
	.line	17
	LDPK	_D
	ZALS	_D
	ANDK	32768
	LDPK	STATIC_72
	SACL	STATIC_72
	ANDK	0FFFFh
	BZ	LL116
	LDPK	_D
	LAC	_D
	NEG
	BD	LL117
	ANDK	32767
***	B	LL117 OCCURS
LL116:
	LDPK	_D
	LAC	_D
LL117:
	LDPK	STATIC_74
	SACL	STATIC_74
*** 18	-----------------------    tmp = DQM>>1;
	.line	18
	RSXM
	LAC	STATIC_74,14
	SACH	STATIC_75,1
*** 19	-----------------------    EXP = 0;
	.line	19
	LACK	0
	SACL	STATIC_73
*** 19	-----------------------    if ( !tmp ) goto g3;
	LAC	STATIC_75
	BZ	L42
L41:
***	-----------------------g2:
*** 19	-----------------------    ++EXP;
	LDPK	STATIC_73
	LAC	STATIC_73
	ADDK	1
	SACL	STATIC_73
*** 19	-----------------------    if ( tmp >>= 1 ) goto g2;
	LAC	STATIC_75,14
	SACH	STATIC_75,1
	ANDK	0FFFFh,15
	SFL
	BNZ	L41
L42:
***	-----------------------g3:
*** 20	-----------------------    S$1 = (EXP < 7) ? DQM<<7-EXP : DQM>>EXP-7;
	.line	20
	SSXM
	LDPK	STATIC_73
	LAC	STATIC_73
	SUBK	7
	BGEZ	LL118
	LACK	7
	SUB	STATIC_73
	MAR	* ,AR0
	SACL	* 
	LT	* 
	LACT	STATIC_74
	BD	LL119
	ANDK	0FFFFh
***	B	LL119 OCCURS
LL118:
	LAC	STATIC_73
	SUBK	7
	MAR	* ,AR0
	SACL	* 
	LT	* 
	ZALS	STATIC_74
	SATL
	ANDK	0FFFFh
LL119:
	SACL	STATIC_70
*** 20	-----------------------    DLN = (unsigned)(EXP*128)+(S$1&127u)-(Y>>2)&4095u;
	ANDK	127
	SACB
	LAC	STATIC_73,7
	ADDB
	MAR	* ,AR1
	SACL	* ,AR0
	RSXM
	LDPK	_Y
	LAC	_Y,13
	SACH	* ,1,AR1
	LAC	* ,AR0
	SUB	* 
	ANDK	4095
	LDPK	STATIC_71
	SACL	STATIC_71
*** 27	-----------------------    IM = (I&8u) ? I&7u : I+8u;
	.line	27
	LDPK	_I
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
	LDPK	STATIC_76
	SACL	STATIC_76
*** 27	-----------------------    ID = (DLN > 299u) ? ((DLN > 2047u) ? ((DLN > 3971u) ? (DS ? 6u : 9u) : 7u) : (DLN > 399u) ? (D...
	ZALS	STATIC_71
	SUBK	299
	BLEZ	LL122
	ZALS	STATIC_71
	SUBK	2047
	BLEZ	LL123
	ZALS	STATIC_71
	SUBK	3971
	BLEZ	LL124
	LAC	STATIC_72
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
	ZALS	STATIC_71
	SUBK	399
	BLEZ	LL129
	LAC	STATIC_72
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
	ZALS	STATIC_71
	SUBK	348
	BLEZ	LL133
	LAC	STATIC_72
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
	LAC	STATIC_72
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
	ZALS	STATIC_71
	SUBK	177
	BLEZ	LL140
	ZALS	STATIC_71
	SUBK	245
	BLEZ	LL141
	LAC	STATIC_72
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
	LAC	STATIC_72
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
	ZALS	STATIC_71
	SUBK	79
	BLEZ	LL148
	LAC	STATIC_72
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
	LAC	STATIC_72
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
	SACL	STATIC_77
*** 31	-----------------------    if ( LAW ) goto g8;
	.line	31
	LDPK	_LAW
	LAC	_LAW
	BNZ	L47
*** 77	-----------------------    SD = S;
	.line	77
	LAC	_S
	SACL	_SD
*** 78	-----------------------    if ( ID > IM ) goto g7;
	.line	78
	LDPK	STATIC_77
	ZALS	STATIC_77
	SUBS	STATIC_76
	BGZ	L46
*** 87	-----------------------    if ( ID >= IM ) goto g13;
	.line	87
	ZALS	STATIC_77
	SUBS	STATIC_76
	BGEZ	EPI0_4
*** 89	-----------------------    SD = (S <= 126u) ? SD+1u : (S <= 128u || S > 255u) ? ((S == 127u) ? 254u : 128u) : SD-1u;
	.line	89
	LDPK	_S
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
	LDPK	_S
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
	LDPK	STATIC_69
	SACL	STATIC_69
*** 56	-----------------------    SD = C$2;
	LDPK	_SD
	SACL	_SD
*** 57	-----------------------    if ( ID > IM ) goto g11;
	.line	57
	LDPK	STATIC_77
	ZALS	STATIC_77
	SUBS	STATIC_76
	BGZ	L50
*** 66	-----------------------    if ( ID >= IM ) goto g12;
	.line	66
	ZALS	STATIC_77
	SUBS	STATIC_76
	BGEZ	L51
*** 68	-----------------------    SD = (C$2 <= 127u) ? (SD ? SD-1u : 128u) : (C$2 == 255u) ? 255u : C$2+1u;
	.line	68
	ZALS	STATIC_69
	SUBK	127
	BGZ	LL169
	LDPK	_SD
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
	LAC	STATIC_69
	SUBK	255
	BNZ	LL173
	BD	LL174
	LACK	255
	NOP
***	B	LL174 OCCURS
LL173:
	LAC	STATIC_69
	ADDK	1
LL174:
LL172:
	BD	L51
	LDPK	_SD
	SACL	_SD
*** 69	-----------------------    goto g12;
	.line	69
***	B	L51 OCCURS
L50:
***	-----------------------g11:
*** 59	-----------------------    SD = (SD <= 126u) ? SD+1u : (SD >= 129u) ? SD-1u : (SD == 128u) ? 0u : 127u;
	.line	59
	LDPK	_SD
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
	LDPK	_SD
	LAC	_SD
	XORK	85
	SACL	_SD
***	-----------------------g13:
***  	-----------------------    return;
EPI0_4:
	.line	98
	MAR	* ,AR1
	RETD
	SBRK	2
	LAR	AR0,*
***	RET OCCURS

	.endfunc	490,000000000H,1

	.sym	_diff_computation,_diff_computation,32,3,0

	.func	492
******************************************************
* FUNCTION DEF : _diff_computation
******************************************************
_diff_computation:

LF5	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+


	.sym	_SLI,STATIC_78,14,3,16
	.bss	STATIC_78,1

	.sym	_SEI,STATIC_79,14,3,16
	.bss	STATIC_79,1
*
*  ---  AUTO variable moved to STATIC storage:  'SEI'
*  ---  AUTO variable moved to STATIC storage:  'SLI'
*
*** 12	-----------------------    SLI = (SL&8192u) ? SL-16384u : SL;
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
	LDPK	STATIC_78
	SACL	STATIC_78
*** 12	-----------------------    SEI = (SE&16384u) ? SE+32768u : SE;
	LDPK	_SE
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
	LDPK	STATIC_79
	SACL	STATIC_79
*** 13	-----------------------    D = SLI-SEI&65535u;
	.line	13
	NEG
	ADD	STATIC_78
	ANDK	65535
	LDPK	_D
	SACL	_D
***  	-----------------------    return;
EPI0_5:
	.line	15
	RETD
	SBRK	2
	LAR	AR0,*
***	RET OCCURS

	.endfunc	506,000000000H,1

	.sym	_iadpt_quant,_iadpt_quant,32,3,0

	.func	508
******************************************************
* FUNCTION DEF : _iadpt_quant
******************************************************
_iadpt_quant:

LF6	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+

	RSXM

	.sect	".const"
STATIC_80:
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
	.text

	.sym	_qtab,STATIC_80,62,3,256,,16

	.sym	U$29,STATIC_81,4,3,16
	.bss	STATIC_81,1

	.sym	S$1,STATIC_82,14,3,16
	.bss	STATIC_82,1

	.sym	S$2,STATIC_83,14,3,16
	.bss	STATIC_83,1

	.sym	_DQL,STATIC_84,14,3,16
	.bss	STATIC_84,1

	.sym	_DEX,STATIC_85,4,3,16
	.bss	STATIC_85,1

	.sym	_DQT,STATIC_86,14,3,16
	.bss	STATIC_86,1
*
*  ---  AUTO variable moved to STATIC storage:  'DQT'
*  ---  AUTO variable moved to STATIC storage:  'DEX'
*  ---  AUTO variable moved to STATIC storage:  'DQL'
*  ---  Compiler temp allocated STATIC storage: 'S$2'
*  ---  Compiler temp allocated STATIC storage: 'S$1'
*  ---  Compiler temp allocated STATIC storage: 'U$29'
*
*** 18	-----------------------    DQL = qtab[I]+(Y>>2);
	.line	18
	LDPK	_Y
	LAC	_Y,13
	SACH	* ,1,AR0
	LAC	_I
	ADLK	STATIC_80+0,0
	SACL	* 
	LAR	AR5,* ,AR5
	LAC	* ,AR1
	ADD	* 
	LDPK	STATIC_84
	SACL	STATIC_84
*** 25	-----------------------    DEX = (int)(DQL>>7&15u);
	.line	25
	LAC	STATIC_84,8
	ANDK	15,15
	SACH	STATIC_85,1
*** 26	-----------------------    DQT = (DQL&127u)+128u;
	.line	26
	LAC	STATIC_84
	ANDK	127
	ADDK	128
	SACL	STATIC_86
*** 27	-----------------------    S$1 = (I&8u) ? 16384u : 0u;
	.line	27
	LDPK	_I
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
	LDPK	STATIC_82
	SACL	STATIC_82
*** 27	-----------------------    S$2 = (DQL&2048u) ? 0u : ((U$29 = 7-DEX) < 0) ? DQT<<-U$29 : DQT>>U$29;
	ZALS	STATIC_84
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
	SUB	STATIC_85
	SACL	STATIC_81
	BGEZ	LL193
	LAC	STATIC_81
	NEG
	MAR	* ,AR0
	SACL	* 
	LT	* 
	LACT	STATIC_86
	BD	LL194
	ANDK	0FFFFh
***	B	LL194 OCCURS
LL193:
	LT	STATIC_81
	ZALS	STATIC_86
	SATL
	ANDK	0FFFFh
LL194:
LL192:
	SACL	STATIC_83
*** 27	-----------------------    DQ = S$1+S$2;
	ADD	STATIC_82
	LDPK	_DQ
	SACL	_DQ
***  	-----------------------    return;
EPI0_6:
	.line	30
	MAR	* ,AR1
	RETD
	SBRK	2
	LAR	AR0,*
***	RET OCCURS

	.endfunc	537,000000000H,1

	.sym	_input_conversion,_input_conversion,32,3,0

	.func	539
******************************************************
* FUNCTION DEF : _input_conversion
******************************************************
_input_conversion:

LF7	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+


	.sym	C$1,STATIC_87,14,3,16
	.bss	STATIC_87,1

	.sym	C$2,STATIC_88,14,3,16
	.bss	STATIC_88,1

	.sym	_SSQ,STATIC_89,14,3,16
	.bss	STATIC_89,1
*
*  ---  AUTO variable moved to STATIC storage:  'SSQ'
*  ---  Compiler temp allocated STATIC storage: 'C$2'
*  ---  Compiler temp allocated STATIC storage: 'C$1'
*
*** 12	-----------------------    if ( LAW ) goto g2;
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
	LAC	* 
	LDPK	STATIC_88
	SACL	STATIC_88
*** 22	-----------------------    SSQ = C$2&8191u;
	ANDK	8191
	SACL	STATIC_89
*** 22	-----------------------    if ( !(C$2&8192u) ) goto g3;
	ZALS	STATIC_88
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
	LAC	* ,AR0
	LDPK	STATIC_87
	SACL	STATIC_87
*** 16	-----------------------    SSQ = (C$1&4095u)*2u;
	ANDK	4095
	SACL	* 
	LAC	* ,1
	SACL	STATIC_89
***  	-----------------------    if ( C$1&4096u ) goto g4;
	ZALS	STATIC_87
	ANDK	4096
	ANDK	0FFFFh
	BNZ	L56
L55:
***	-----------------------g3:
*** 24	-----------------------    SL = SSQ;
	.line	24
	LDPK	STATIC_89
	LAC	STATIC_89
	BD	EPI0_7
	LDPK	_SL
	SACL	_SL
*** 24	-----------------------    goto g5;
***	B	EPI0_7 OCCURS
L56:
***	-----------------------g4:
*** 24	-----------------------    SL = -SSQ&16383u;
	LDPK	STATIC_89
	LAC	STATIC_89
	NEG
	ANDK	16383
	LDPK	_SL
	SACL	_SL
***	-----------------------g5:
***  	-----------------------    return;
EPI0_7:
	.line	25
	MAR	* ,AR1
	RETD
	SBRK	2
	LAR	AR0,*
***	RET OCCURS

	.endfunc	563,000000000H,1

	.sym	_output_conversion,_output_conversion,32,3,0

	.func	565
******************************************************
* FUNCTION DEF : _output_conversion
******************************************************
_output_conversion:

LF8	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+

	RSXM

	.sym	S$1,STATIC_90,14,3,16
	.bss	STATIC_90,1

	.sym	_IS,STATIC_91,14,3,16
	.bss	STATIC_91,1

	.sym	_IM,STATIC_92,14,3,16
	.bss	STATIC_92,1

	.sym	_MASK,STATIC_93,14,3,16
	.bss	STATIC_93,1

	.sym	_SEG,STATIC_94,14,3,16
	.bss	STATIC_94,1

	.sym	_IMS,STATIC_95,14,3,16
	.bss	STATIC_95,1

	.sym	_tmp,STATIC_96,14,3,16
	.bss	STATIC_96,1

	.sym	_MASK,STATIC_97,14,3,16
	.bss	STATIC_97,1

	.sym	_IMS,STATIC_98,14,3,16
	.bss	STATIC_98,1

	.sym	_SEG,STATIC_99,14,3,16
	.bss	STATIC_99,1
*
*  ---  AUTO variable moved to STATIC storage:  'SEG'
*  ---  AUTO variable moved to STATIC storage:  'IMS'
*  ---  AUTO variable moved to STATIC storage:  'MASK'
*  ---  AUTO variable moved to STATIC storage:  'tmp'
*  ---  AUTO variable moved to STATIC storage:  'IMS'
*  ---  AUTO variable moved to STATIC storage:  'SEG'
*  ---  AUTO variable moved to STATIC storage:  'MASK'
*  ---  AUTO variable moved to STATIC storage:  'IM'
*  ---  AUTO variable moved to STATIC storage:  'IS'
*  ---  Compiler temp allocated STATIC storage: 'S$1'
*
*** 11	-----------------------    IM = (IS = SR&32768u) ? -SR&32767u : SR;
	.line	11
	LDPK	_SR
	ZALS	_SR
	ANDK	32768
	LDPK	STATIC_91
	SACL	STATIC_91
	ANDK	0FFFFh
	BZ	LL198
	LDPK	_SR
	LAC	_SR
	NEG
	BD	LL199
	ANDK	32767
***	B	LL199 OCCURS
LL198:
	LDPK	_SR
	LAC	_SR
LL199:
	LDPK	STATIC_92
	SACL	STATIC_92
*** 12	-----------------------    if ( LAW ) goto g7;
	.line	12
	LDPK	_LAW
	LAC	_LAW
	BNZ	L64
*** 35	-----------------------    MASK = IS ? 127u : 255u;
	.line	35
	LDPK	STATIC_91
	LAC	STATIC_91
	BZ	LL200
	BD	LL201
	LACK	127
	NOP
***	B	LL201 OCCURS
LL200:
	LACK	255
LL201:
	LDPK	STATIC_97
	SACL	STATIC_97
*** 35	-----------------------    if ( (IMS = IM+33u) > 8191u ) goto g6;
	LDPK	STATIC_92
	ZALS	STATIC_92
	ADDK	33
	LDPK	STATIC_98
	SACL	STATIC_98
	ANDK	0FFFFh
	SUBK	8191
	BGZ	L63
*** 41	-----------------------    SEG = 5u;
	.line	41
	LACK	5
	SACL	STATIC_99
*** 41	-----------------------    if ( !(IMS>>5u) ) goto g5;
	RSXM
	LAC	STATIC_98,10
	ANDK	0FFFFh,15
	SFL
	BZ	L62
	MAR	* ,AR0
L61:
***	-----------------------g4:
*** 41	-----------------------    if ( IMS>>(++SEG) ) goto g4;
	LDPK	STATIC_99
	LAC	STATIC_99
	ADDK	1
	SACL	STATIC_99
	SACL	* 
	LT	* 
	ZALS	STATIC_98
	SATL
	ANDK	0FFFFh
	BNZ	L61
L62:
***	-----------------------g5:
*** 43	-----------------------    SEG -= 6u;
	.line	43
	LDPK	STATIC_99
	LAC	STATIC_99
	SUBK	6
	SACL	STATIC_99
*** 44	-----------------------    S = (SEG*16u|IMS>>SEG+1u&15u)^MASK;
	.line	44
	ADDK	1
	MAR	* ,AR0
	SACL	* 
	LT	* 
	ZALS	STATIC_98
	SATL
	ANDK	15
	SACB
	LAC	STATIC_99,4
	ORB
	BD	EPI0_8
	XOR	STATIC_97
	SACL	_S
*** 44	-----------------------    goto g13;
***	B	EPI0_8 OCCURS
L63:
***	-----------------------g6:
*** 38	-----------------------    S = MASK^127u;
	.line	38
	LAC	STATIC_97
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
	LDPK	STATIC_91
	LAC	STATIC_91
	BZ	LL202
	BD	LL203
	LACK	85
	NOP
***	B	LL203 OCCURS
LL202:
	LACK	213
LL203:
	SACL	STATIC_93
*** 18	-----------------------    if ( (IMS = IS ? (IM+1u>>1)-1u : IM>>1) > 4095u ) goto g12;
	LAC	STATIC_91
	BZ	LL204
	ZALS	STATIC_92
	ADDK	1
	MAR	* ,AR0
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
	LAC	STATIC_92,14
	SACH	* ,1
	ZALS	* 
LL205:
	LDPK	STATIC_95
	SACL	STATIC_95
	SUBK	4095
	BGZ	L69
*** 26	-----------------------    SEG = 0u;
	.line	26
	LACK	0
	SACL	STATIC_94
*** 26	-----------------------    if ( !(tmp = IMS>>4>>1) ) goto g11;
	LAC	STATIC_95,10
	SACH	STATIC_96,1
	ANDK	0FFFFh,15
	SFL
	BZ	L68
L67:
***	-----------------------g10:
*** 26	-----------------------    ++SEG;
	LDPK	STATIC_94
	LAC	STATIC_94
	ADDK	1
	SACL	STATIC_94
*** 26	-----------------------    if ( tmp >>= 1 ) goto g10;
	LAC	STATIC_96,14
	SACH	STATIC_96,1
	ANDK	0FFFFh,15
	SFL
	BNZ	L67
L68:
***	-----------------------g11:
*** 28	-----------------------    S$1 = SEG ? IMS>>SEG : IMS>>1;
	.line	28
	LDPK	STATIC_94
	LAC	STATIC_94
	BZ	LL206
	LT	STATIC_94
	BD	LL207
	ZALS	STATIC_95
	SATL
***	B	LL207 OCCURS
LL206:
	LAC	STATIC_95,14
	MAR	* ,AR1
	SACH	* ,1
	LAC	* 
LL207:
	LDPK	STATIC_90
	SACL	STATIC_90
*** 28	-----------------------    S = (SEG*16u|S$1&15u)^MASK;
	ANDK	15
	SACB
	LDPK	STATIC_94
	LAC	STATIC_94,4
	ORB
	LDPK	STATIC_93
	XOR	STATIC_93
	BD	EPI0_8
	LDPK	_S
	SACL	_S
*** 28	-----------------------    goto g13;
***	B	EPI0_8 OCCURS
L69:
***	-----------------------g12:
*** 21	-----------------------    S = MASK^127u;
	.line	21
	LDPK	STATIC_93
	LAC	STATIC_93
	XORK	127
	LDPK	_S
	SACL	_S
***	-----------------------g13:
***  	-----------------------    return;
EPI0_8:
	.line	47
	MAR	* ,AR1
	RETD
	SBRK	2
	LAR	AR0,*
***	RET OCCURS

	.endfunc	611,000000000H,1

	.sym	_reset_states,_reset_states,32,3,0

	.func	613
******************************************************
* FUNCTION DEF : _reset_states
******************************************************
_reset_states:

LF9	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+
;	<save register vars>
	SAR	AR6,*+
	SAR	AR7,*+,AR0

	RSXM

	.sym	C$3,STATIC_100,30,3,16
	.bss	STATIC_100,1

	.sym	_x,STATIC_101,24,3,16,.fake0
	.bss	STATIC_101,1
*
* AR5	assigned to temp var 'U$8'
* AR5	assigned to temp var 'U$17'
* AR6	assigned to temp var 'K$5'
* AR7	assigned to temp var 'K$14'
* BRCR	assigned to temp var 'L$2'
* BRCR	assigned to temp var 'L$1'
*  ---  AUTO variable moved to STATIC storage:  'x'
*  ---  Compiler temp allocated STATIC storage: 'C$3'
*
*** 3	-----------------------    x = X;
	.line	3
	LDPK	_X
	LAC	_X
	SACL	STATIC_101
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
	LDPK	STATIC_101
	LAC	STATIC_101
	ADDK	8
	MAR	* ,AR0
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
	LDPK	STATIC_101
	LAC	STATIC_101
	ADDK	8
	SACL	STATIC_100
*** 10	-----------------------    C$3[6] = C$3[7] = K$14;
	LAR	AR5,STATIC_100
	ADRK	7
	SAR	AR7,* ,AR4
	LAR	AR4,STATIC_100
	ADRK	6
	SAR	AR7,* ,AR3
*** 11	-----------------------    x->PK1 = x->PK2 = K$5;
	.line	11
	LAR	AR3,STATIC_101
	ADRK	17
	SAR	AR6,* ,AR5
	LAR	AR5,STATIC_101
	ADRK	16
	SAR	AR6,* ,AR4
*** 12	-----------------------    x->AP = x->DMS = x->DML = K$5;
	.line	12
	LAR	AR4,STATIC_101
	ADRK	20
	SAR	AR6,* ,AR3
	LAR	AR3,STATIC_101
	ADRK	19
	SAR	AR6,* ,AR5
	LAR	AR5,STATIC_101
	ADRK	18
	SAR	AR6,* ,AR4
*** 13	-----------------------    x->YU = 544u;
	.line	13
	LAR	AR4,STATIC_101
	ADRK	21
	SPLK	#544,* ,AR3
*** 14	-----------------------    x->TD = K$5;
	.line	14
	LAR	AR3,STATIC_101
	ADRK	22
	SAR	AR6,* ,AR5
*** 15	-----------------------    x->YL = 34816uL;
	.line	15
	LAR	AR5,STATIC_101
	LACK	34816
	ADRK	24
	SACL	*+
	SACH	*-,AR4
*** 15	-----------------------    x->YL6 = 544u;
	LAR	AR4,STATIC_101
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
	SBRK	2
	LAR	AR0,*
***	RET OCCURS

	.endfunc	628,0000000c0H,1

	.sym	_scale_factor_1,_scale_factor_1,32,3,0

	.func	630
******************************************************
* FUNCTION DEF : _scale_factor_1
******************************************************
_scale_factor_1:

LF10	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+,AR5

	RSXM

	.sym	C$2,STATIC_102,14,3,16
	.bss	STATIC_102,1

	.sym	C$3,STATIC_103,14,3,16
	.bss	STATIC_103,1

	.sym	U$4,STATIC_104,14,3,16
	.bss	STATIC_104,1

	.sym	U$3,STATIC_105,14,3,16
	.bss	STATIC_105,1

	.sym	S$1,STATIC_106,14,3,16
	.bss	STATIC_106,1

	.sym	_x,STATIC_107,24,3,16,.fake0
	.bss	STATIC_107,1

	.sym	_DIF,STATIC_108,14,3,16
	.bss	STATIC_108,1

	.sym	_DIFS,STATIC_109,14,3,16
	.bss	STATIC_109,1

	.sym	_DIFM,STATIC_110,14,3,16
	.bss	STATIC_110,1

	.sym	_PRODM,STATIC_111,14,3,16
	.bss	STATIC_111,1
*
*  ---  AUTO variable moved to STATIC storage:  'PRODM'
*  ---  AUTO variable moved to STATIC storage:  'DIFM'
*  ---  AUTO variable moved to STATIC storage:  'DIFS'
*  ---  AUTO variable moved to STATIC storage:  'DIF'
*  ---  AUTO variable moved to STATIC storage:  'x'
*  ---  Compiler temp allocated STATIC storage: 'S$1'
*  ---  Compiler temp allocated STATIC storage: 'U$3'
*  ---  Compiler temp allocated STATIC storage: 'U$4'
*  ---  Compiler temp allocated STATIC storage: 'C$3'
*  ---  Compiler temp allocated STATIC storage: 'C$2'
*
*** 11	-----------------------    x = X;
	.line	11
	LDPK	_X
	LAC	_X
	SACL	STATIC_107
*** 17	-----------------------    C$3 = x->YL6;
	.line	17
	LAR	AR5,STATIC_107
	ADRK	23
	LAC	* ,AR4
	SACL	STATIC_103
*** 17	-----------------------    C$2 = x->YU;
	LAR	AR4,STATIC_107
	ADRK	21
	LAC	* 
	SACL	STATIC_102
*** 17	-----------------------    DIF = C$2-C$3;
	SUB	STATIC_103
	SACL	STATIC_108
*** 18	-----------------------    DIFS = DIF&8192u;
	.line	18
	ANDK	8192
	SACL	STATIC_109
*** 19	-----------------------    DIFM = ((U$4 = C$2-(U$3 = C$3))&8192u) ? -DIF&8191u : U$4;
	.line	19
	ZALS	STATIC_103
	SACL	STATIC_105
	NEG
	ADDS	STATIC_102
	SACL	STATIC_104
	ANDK	8192
	ANDK	0FFFFh
	BZ	LL211
	LAC	STATIC_108
	NEG
	BD	LL212
	ANDK	8191
***	B	LL212 OCCURS
LL211:
	LAC	STATIC_104
LL212:
	SACL	STATIC_110
*** 19	-----------------------    PRODM = (unsigned)((unsigned long)DIFM*(unsigned long)AL>>6);
	LT	_AL
	MPYU	STATIC_110
	PAC
	LDPK	0
	SPLK	6,TREG1
	RSXM
	SATL
	LDPK	STATIC_111
	SACL	STATIC_111
*** 21	-----------------------    S$1 = DIFS ? -PRODM : PRODM;
	.line	21
	LAC	STATIC_109
	BZ	LL213
	BD	LL214
	LAC	STATIC_111
	NEG
***	B	LL214 OCCURS
LL213:
	LAC	STATIC_111
LL214:
	SACL	STATIC_106
*** 21	-----------------------    Y = U$3+S$1&8191u;
	ADD	STATIC_105
	ANDK	8191
	SACL	_Y
***  	-----------------------    return;
EPI0_10:
	.line	23
	MAR	* ,AR1
	RETD
	SBRK	2
	LAR	AR0,*
***	RET OCCURS

	.endfunc	652,000000000H,1

	.sym	_scale_factor_2,_scale_factor_2,32,3,0

	.func	654
******************************************************
* FUNCTION DEF : _scale_factor_2
******************************************************
_scale_factor_2:

LF11	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,3
	LAR	AR0,*0+

	SSXM

	.sect	".const"
STATIC_112:
	.word	4084
	.word	18
	.word	41
	.word	64
	.word	112
	.word	198
	.word	355
	.word	1122
	.text

	.sym	_W,STATIC_112,62,3,128,,8

	.sym	C$4,STATIC_113,15,3,32
	.bss	STATIC_113,2,1

	.sym	U$53,STATIC_114,14,3,16
	.bss	STATIC_114,1

	.sym	U$46,STATIC_115,15,3,32
	.bss	STATIC_115,2,1

	.sym	S$1,STATIC_116,14,3,16
	.bss	STATIC_116,1

	.sym	S$2,STATIC_117,14,3,16
	.bss	STATIC_117,1

	.sym	S$3,STATIC_118,14,3,16
	.bss	STATIC_118,1

	.sym	_x,STATIC_119,24,3,16,.fake0
	.bss	STATIC_119,1

	.sym	_YUT,STATIC_120,14,3,16
	.bss	STATIC_120,1

	.sym	_DIF,STATIC_121,14,3,16
	.bss	STATIC_121,1

	.sym	_DIF,STATIC_122,14,3,16
	.bss	STATIC_122,1

	.sym	_DIFSX,STATIC_123,15,3,32
	.bss	STATIC_123,2,1
*
*  ---  AUTO variable moved to STATIC storage:  'DIFSX'
*  ---  AUTO variable moved to STATIC storage:  'DIF'
*  ---  AUTO variable moved to STATIC storage:  'DIF'
*  ---  AUTO variable moved to STATIC storage:  'YUT'
*  ---  AUTO variable moved to STATIC storage:  'x'
*  ---  Compiler temp allocated STATIC storage: 'S$3'
*  ---  Compiler temp allocated STATIC storage: 'S$2'
*  ---  Compiler temp allocated STATIC storage: 'S$1'
*  ---  Compiler temp allocated STATIC storage: 'U$46'
*  ---  Compiler temp allocated STATIC storage: 'U$53'
*  ---  Compiler temp allocated STATIC storage: 'C$4'
*
*** 15	-----------------------    x = X;
	.line	15
	LDPK	_X
	LAC	_X
	SACL	STATIC_119
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
	SACL	STATIC_118
*** 22	-----------------------    DIF = (unsigned)((unsigned long)W[(S$3&7u)]*32uL-(unsigned long)Y>>5&4095uL);
	ANDK	7
	ADLK	STATIC_112+0,0
	MAR	* ,AR0
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
	LDPK	STATIC_121
	SACL	STATIC_121
*** 25	-----------------------    S$2 = (DIF&2048u) ? DIF+4096u : DIF;
	.line	25
	ZALS	STATIC_121
	ANDK	2048
	ANDK	0FFFFh
	BZ	LL219
	LAC	STATIC_121
	BD	LL220
	ADDK	4096
***	B	LL220 OCCURS
LL219:
	LAC	STATIC_121
LL220:
	SACL	STATIC_117
*** 25	-----------------------    YUT = S$2+Y&8191u;
	ADD	_Y
	ANDK	8191
	SACL	STATIC_120
*** 29	-----------------------    S$1 = (YUT+15840u&8192u) ? 544u : (YUT+11264u&8192u) ? YUT : 5120u;
	.line	29
	ZALS	STATIC_120
	ADDK	15840
	ANDK	8192
	ANDK	0FFFFh
	BZ	LL221
	BD	LL222
	LACK	544
***	B	LL222 OCCURS
LL221:
	ZALS	STATIC_120
	ADDK	11264
	ANDK	8192
	ANDK	0FFFFh
	BZ	LL223
	BD	LL224
	LAC	STATIC_120
	NOP
***	B	LL224 OCCURS
LL223:
	LACK	5120
LL224:
LL222:
	SACL	STATIC_116
*** 29	-----------------------    x->YU = S$1;
	LAR	AR4,STATIC_119
	MAR	* ,AR4
	ADRK	21
	SACL	* ,AR3
*** 38	-----------------------    C$4 = x->YL;
	.line	38
	LAR	AR3,STATIC_119
	ADRK	24
	ZALS	*+
	ADDH	*-
	SACL	STATIC_113
	SACH	STATIC_113+1
*** 38	-----------------------    DIF = (unsigned)(-C$4>>6)+S$1&16383u;
	ZALS	STATIC_113
	ADDH	STATIC_113+1
	NEG
	LDPK	0
	SPLK	6,TREG1
	SATL
	LDPK	STATIC_116
	ADD	STATIC_116
	ANDK	16383
	SACL	STATIC_122
*** 39	-----------------------    DIFSX = ((U$53 = (unsigned)(-(U$46 = C$4)>>6)+S$1&16383u)&8192u) ? (unsigned long)DIF+507904uL...
	.line	39
	ZALS	STATIC_113
	ADDH	STATIC_113+1
	SACL	STATIC_115
	SACH	STATIC_115+1
	NEG
	LDPK	0
	SPLK	6,TREG1
	SATL
	LDPK	STATIC_116
	ADDS	STATIC_116
	ANDK	16383
	SACL	STATIC_114
	ANDK	8192
	ANDK	0FFFFh
	BZ	LL225
	LALK	15,15
	ORK	16384
	BD	LL226
	ADDS	STATIC_122
	NOP
***	B	LL226 OCCURS
LL225:
	ZALS	STATIC_114
LL226:
	SACL	STATIC_123
	SACH	STATIC_123+1
*** 39	-----------------------    x->YL6 = (unsigned)((x->YL = U$46+DIFSX&524287uL)>>6);
	ZALS	STATIC_115
	ADDH	STATIC_115+1
	ADDS	STATIC_123
	ADDH	STATIC_123+1
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	SACH	*-
	ANDK	65535
	SACL	*+
	ZALS	* 
	ANDK	7
	SACL	*-
	LAR	AR5,STATIC_119
	ZALS	*+
	ADDH	* ,AR5
	ADRK	24
	SACL	*+
	SACH	*-,AR4
	LDPK	0
	SPLK	6,TREG1
	SATL
	LDPK	STATIC_119
	LAR	AR4,STATIC_119
	ADRK	23
	SACL	* 
***  	-----------------------    return;
EPI0_11:
	.line	42
	MAR	* ,AR1
	RETD
	SBRK	4
	LAR	AR0,*
***	RET OCCURS

	.endfunc	695,000000000H,3

	.sym	_speed_control_1,_speed_control_1,32,3,0

	.func	697
******************************************************
* FUNCTION DEF : _speed_control_1
******************************************************
_speed_control_1:

LF12	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+,AR5

	RSXM

	.sym	C$1,STATIC_124,14,3,16
	.bss	STATIC_124,1
*
*  ---  Compiler temp allocated STATIC storage: 'C$1'
*
*** 12	-----------------------    AL = ((C$1 = X->AP) > 255u) ? 64u : C$1>>2;
	.line	12
	LDPK	_X
	LAR	AR5,_X
	ADRK	18
	ZALS	* 
	SACL	STATIC_124
	SUBK	255
	BLEZ	LL229
	BD	LL230
	LACK	64
	NOP
***	B	LL230 OCCURS
LL229:
	RSXM
	LAC	STATIC_124,13
	MAR	* ,AR1
	SACH	* ,1
	LAC	* 
LL230:
	SACL	_AL
*** 12	-----------------------    return;
EPI0_12:
	.line	13
	MAR	* ,AR1
	RETD
	SBRK	2
	LAR	AR0,*
***	RET OCCURS

	.endfunc	709,000000000H,1

	.sym	_speed_control_2,_speed_control_2,32,3,0

	.func	711
******************************************************
* FUNCTION DEF : _speed_control_2
******************************************************
_speed_control_2:

LF13	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+

	RSXM

	.sect	".const"
STATIC_125:
	.word	0
	.word	0
	.word	0
	.word	512
	.word	512
	.word	512
	.word	1536
	.word	3584
	.text

	.sym	_F,STATIC_125,62,3,128,,8

	.sym	C$6,STATIC_126,14,3,16
	.bss	STATIC_126,1

	.sym	C$7,STATIC_127,14,3,16
	.bss	STATIC_127,1

	.sym	C$8,STATIC_128,14,3,16
	.bss	STATIC_128,1

	.sym	C$9,STATIC_129,14,3,16
	.bss	STATIC_129,1

	.sym	C$10,STATIC_130,14,3,16
	.bss	STATIC_130,1

	.sym	C$11,STATIC_131,14,3,16
	.bss	STATIC_131,1

	.sym	U$20,STATIC_132,14,3,16
	.bss	STATIC_132,1

	.sym	U$15,STATIC_133,14,3,16
	.bss	STATIC_133,1

	.sym	U$37,STATIC_134,14,3,16
	.bss	STATIC_134,1

	.sym	U$32,STATIC_135,14,3,16
	.bss	STATIC_135,1

	.sym	U$48,STATIC_136,14,3,16
	.bss	STATIC_136,1

	.sym	U$68,STATIC_137,14,3,16
	.bss	STATIC_137,1

	.sym	U$63,STATIC_138,14,3,16
	.bss	STATIC_138,1

	.sym	S$2,STATIC_139,14,3,16
	.bss	STATIC_139,1

	.sym	S$3,STATIC_140,14,3,16
	.bss	STATIC_140,1

	.sym	S$4,STATIC_141,14,3,16
	.bss	STATIC_141,1

	.sym	S$5,STATIC_142,14,3,16
	.bss	STATIC_142,1

	.sym	_x,STATIC_143,24,3,16,.fake0
	.bss	STATIC_143,1

	.sym	_FI,STATIC_144,14,3,16
	.bss	STATIC_144,1

	.sym	_AX,STATIC_145,14,3,16
	.bss	STATIC_145,1

	.sym	_DIF,STATIC_146,14,3,16
	.bss	STATIC_146,1

	.sym	_DIF,STATIC_147,14,3,16
	.bss	STATIC_147,1

	.sym	_DIF,STATIC_148,14,3,16
	.bss	STATIC_148,1

	.sym	_DIFM,STATIC_149,14,3,16
	.bss	STATIC_149,1

	.sym	_DIF,STATIC_150,14,3,16
	.bss	STATIC_150,1
*
* AR5	assigned to temp var 'S$1'
*  ---  AUTO variable moved to STATIC storage:  'DIF'
*  ---  AUTO variable moved to STATIC storage:  'DIFM'
*  ---  AUTO variable moved to STATIC storage:  'DIF'
*  ---  AUTO variable moved to STATIC storage:  'DIF'
*  ---  AUTO variable moved to STATIC storage:  'DIF'
*  ---  AUTO variable moved to STATIC storage:  'AX'
*  ---  AUTO variable moved to STATIC storage:  'FI'
*  ---  AUTO variable moved to STATIC storage:  'x'
*  ---  Compiler temp allocated STATIC storage: 'S$5'
*  ---  Compiler temp allocated STATIC storage: 'S$4'
*  ---  Compiler temp allocated STATIC storage: 'S$3'
*  ---  Compiler temp allocated STATIC storage: 'S$2'
*  ---  Compiler temp allocated STATIC storage: 'U$63'
*  ---  Compiler temp allocated STATIC storage: 'U$68'
*  ---  Compiler temp allocated STATIC storage: 'U$48'
*  ---  Compiler temp allocated STATIC storage: 'U$32'
*  ---  Compiler temp allocated STATIC storage: 'U$37'
*  ---  Compiler temp allocated STATIC storage: 'U$15'
*  ---  Compiler temp allocated STATIC storage: 'U$20'
*  ---  Compiler temp allocated STATIC storage: 'C$11'
*  ---  Compiler temp allocated STATIC storage: 'C$10'
*  ---  Compiler temp allocated STATIC storage: 'C$9'
*  ---  Compiler temp allocated STATIC storage: 'C$8'
*  ---  Compiler temp allocated STATIC storage: 'C$7'
*  ---  Compiler temp allocated STATIC storage: 'C$6'
*
*** 12	-----------------------    x = X;
	.line	12
	LDPK	_X
	LAC	_X
	SACL	STATIC_143
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
	SACL	STATIC_142
*** 16	-----------------------    FI = F[(S$5&7u)];
	ANDK	7
	ADLK	STATIC_125+0,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR5,* ,AR5
	LAC	* ,AR4
	SACL	STATIC_144
*** 22	-----------------------    C$11 = x->DMS;
	.line	22
	LAR	AR4,STATIC_143
	ADRK	19
	LAC	* ,AR0
	SACL	STATIC_131
*** 22	-----------------------    DIF = (FI-C$11&8191u)>>5;
	ZALS	STATIC_144
	SUBS	STATIC_131
	ANDK	8191
	SACL	* 
	LAC	* ,10
	SACH	STATIC_146,1
*** 23	-----------------------    S$4 = ((U$20 = (FI-(U$15 = C$11)&8191u)>>5)&128u) ? DIF+3840u : U$20;
	.line	23
	ZALS	STATIC_131
	SACL	STATIC_133
	NEG
	ADDS	STATIC_144
	ANDK	8191
	SACL	* 
	LAC	* ,10
	SACH	STATIC_132,1
	ANDK	128,15
	ANDK	0FFFFh,15
	SFL
	BZ	LL235
	LAC	STATIC_146
	BD	LL236
	ADDK	3840
***	B	LL236 OCCURS
LL235:
	LAC	STATIC_132
LL236:
	SACL	STATIC_141
*** 23	-----------------------    x->DMS = S$4+U$15&4095u;
	ADD	STATIC_133
	ANDK	4095
	LAR	AR3,STATIC_143
	MAR	* ,AR3
	ADRK	19
	SACL	* ,AR5
*** 30	-----------------------    C$10 = x->DML;
	.line	30
	LAR	AR5,STATIC_143
	ADRK	20
	LAC	* ,AR0
	SACL	STATIC_130
*** 30	-----------------------    C$9 = FI*4u;
	LAC	STATIC_144,2
	SACL	STATIC_129
*** 30	-----------------------    DIF = (C$9-C$10&32767u)>>7;
	ZALS	STATIC_129
	SUBS	STATIC_130
	ANDK	32767
	SACL	* 
	LAC	* ,8
	SACH	STATIC_147,1
*** 31	-----------------------    S$3 = ((U$37 = (C$9-(U$32 = C$10)&32767u)>>7)&128u) ? DIF+16128u : U$37;
	.line	31
	ZALS	STATIC_130
	SACL	STATIC_135
	NEG
	ADDS	STATIC_129
	ANDK	32767
	SACL	* 
	LAC	* ,8
	SACH	STATIC_134,1
	ANDK	128,15
	ANDK	0FFFFh,15
	SFL
	BZ	LL237
	LAC	STATIC_147
	BD	LL238
	ADDK	16128
***	B	LL238 OCCURS
LL237:
	LAC	STATIC_134
LL238:
	SACL	STATIC_140
*** 31	-----------------------    C$8 = S$3+U$32&16383u;
	ADD	STATIC_135
	ANDK	16383
	SACL	STATIC_128
*** 31	-----------------------    x->DML = C$8;
	LAR	AR4,STATIC_143
	MAR	* ,AR4
	ADRK	20
	SACL	* ,AR3
*** 38	-----------------------    C$7 = x->DMS*4u;
	.line	38
	LAR	AR3,STATIC_143
	ADRK	19
	LAC	* ,2
	SACL	STATIC_127
*** 38	-----------------------    DIF = C$7-C$8&32767u;
	SUB	STATIC_128
	ANDK	32767
	SACL	STATIC_148
*** 39	-----------------------    DIFM = ((U$48 = C$7-(U$32 = C$8)&32767u)&16384u) ? 32768u-DIF&16383u : U$48;
	.line	39
	ZALS	STATIC_128
	SACL	STATIC_135
	NEG
	ADDS	STATIC_127
	ANDK	32767
	SACL	STATIC_136
	ANDK	16384
	ANDK	0FFFFh
	BZ	LL239
	LALK	1,15
	SUB	STATIC_148
	BD	LL240
	ANDK	16383
***	B	LL240 OCCURS
LL239:
	LAC	STATIC_136
LL240:
	SACL	STATIC_149
*** 39	-----------------------    AX = (Y < 1536u || (DIFM >= U$32>>3 || TDP)) ? 512u : 0u;
	ZALS	_Y
	SUBK	1536
	BLZ	LL242
	LAC	STATIC_135,12
	MAR	* ,AR0
	SACH	* ,1
	ZALS	STATIC_149
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
	SACL	STATIC_145
*** 41	-----------------------    C$6 = x->AP;
	.line	41
	LAR	AR5,STATIC_143
	MAR	* ,AR5
	ADRK	18
	LAC	* ,AR0
	SACL	STATIC_126
*** 48	-----------------------    DIF = (AX-C$6&2047u)>>4;
	.line	48
	ZALS	STATIC_145
	SUBS	STATIC_126
	ANDK	2047
	SACL	* 
	LAC	* ,11
	SACH	STATIC_150,1
*** 49	-----------------------    S$2 = ((U$68 = (AX-(U$63 = C$6)&2047u)>>4)&64u) ? DIF+896u : U$68;
	.line	49
	ZALS	STATIC_126
	SACL	STATIC_138
	NEG
	ADDS	STATIC_145
	ANDK	2047
	SACL	* 
	LAC	* ,11
	SACH	STATIC_137,1
	ANDK	64,15
	ANDK	0FFFFh,15
	SFL
	BZ	LL245
	LAC	STATIC_150
	BD	LL246
	ADDK	896
***	B	LL246 OCCURS
LL245:
	LAC	STATIC_137
LL246:
	SACL	STATIC_139
*** 49	-----------------------    S$1 = TR ? 256u : S$2+U$63&1023u;
	LAC	_TR
	BZ	LL247
	BD	LL248
	LACK	256
***	B	LL248 OCCURS
LL247:
	LAC	STATIC_138
	ADD	STATIC_139
	ANDK	1023
LL248:
	SACL	* 
	LAR	AR5,* ,AR4
*** 53	-----------------------    x->AP = S$1;
	.line	53
	LAR	AR4,STATIC_143
	ADRK	18
	SAR	AR5,* 
***  	-----------------------    return;
EPI0_13:
	.line	54
	MAR	* ,AR1
	RETD
	SBRK	2
	LAR	AR0,*
***	RET OCCURS

	.endfunc	764,000000000H,1

	.sym	_tone_detector_1,_tone_detector_1,32,3,0

	.func	766
******************************************************
* FUNCTION DEF : _tone_detector_1
******************************************************
_tone_detector_1:

LF14	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+,AR5

	RSXM

	.sym	C$3,STATIC_151,14,3,16
	.bss	STATIC_151,1

	.sym	U$8,STATIC_152,14,3,16
	.bss	STATIC_152,1

	.sym	S$2,STATIC_153,4,3,16
	.bss	STATIC_153,1

	.sym	_x,STATIC_154,24,3,16,.fake0
	.bss	STATIC_154,1

	.sym	_DQMAG,STATIC_155,14,3,16
	.bss	STATIC_155,1

	.sym	_THR2,STATIC_156,14,3,16
	.bss	STATIC_156,1
*
*  ---  AUTO variable moved to STATIC storage:  'THR2'
*  ---  AUTO variable moved to STATIC storage:  'DQMAG'
*  ---  AUTO variable moved to STATIC storage:  'x'
*  ---  Compiler temp allocated STATIC storage: 'S$2'
*  ---  Compiler temp allocated STATIC storage: 'U$8'
*  ---  Compiler temp allocated STATIC storage: 'C$3'
*
*** 11	-----------------------    x = X;
	.line	11
	LDPK	_X
	LAC	_X
	SACL	STATIC_154
*** 17	-----------------------    DQMAG = DQ&16383u;
	.line	17
	LAC	_DQ
	ANDK	16383
	SACL	STATIC_155
*** 20	-----------------------    C$3 = x->YL6;
	.line	20
	LAR	AR5,STATIC_154
	ADRK	23
	LAC	* ,AR0
	SACL	STATIC_151
*** 20	-----------------------    THR2 = ((U$8 = C$3>>9) > 8u) ? 15872u : (C$3>>4&31u)+32u<<U$8;
	LAC	STATIC_151,6
	SACH	STATIC_152,1
	SACH	* ,1
	ZALS	* 
	SUBK	8
	BLEZ	LL251
	BD	LL252
	LACK	15872
***	B	LL252 OCCURS
LL251:
	LAC	STATIC_151,11
	ANDK	31,15
	ADLK	32,15
	LT	STATIC_152
	SACH	* ,1
	LACT	* 
	ANDK	0FFFFh
LL252:
	SACL	STATIC_156
*** 20	-----------------------    S$2 = 0;
	LACK	0
	SACL	STATIC_153
*** 21	-----------------------    if ( x->TD == 0u || DQMAG <= (THR2>>1)+THR2>>1 ) goto g2;
	.line	21
	LAR	AR4,STATIC_154
	MAR	* ,AR4
	ADRK	22
	LAC	* 
	BZ	L80
	LAC	STATIC_156,14
	ADD	STATIC_156,15
	MAR	* ,AR0
	SACH	* ,1
	LAC	* ,14
	SACH	* ,1
	ZALS	STATIC_155
	SUBS	* 
	BLEZ	L80
*** 21	-----------------------    S$2 = 1;
	LACK	1
	SACL	STATIC_153
L80:
***	-----------------------g2:
*** 21	-----------------------    TR = (unsigned)S$2;
	LAC	STATIC_153
	SACL	_TR
***  	-----------------------    return;
EPI0_14:
	.line	23
	MAR	* ,AR1
	RETD
	SBRK	2
	LAR	AR0,*
***	RET OCCURS

	.endfunc	788,000000000H,1

	.sym	_tone_detector_2,_tone_detector_2,32,3,0

	.func	790
******************************************************
* FUNCTION DEF : _tone_detector_2
******************************************************
_tone_detector_2:

LF15	.set	1

	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+

	RSXM

	.sym	S$3,STATIC_157,4,3,16
	.bss	STATIC_157,1
*
* AR5	assigned to temp var 'S$1'
*  ---  Compiler temp allocated STATIC storage: 'S$3'
*
*** 12	-----------------------    S$3 = 0;
	.line	12
	LACK	0
	LDPK	STATIC_157
	SACL	STATIC_157
*** 12	-----------------------    if ( A2P < 32768u || A2P >= 53760u ) goto g2;
	ZALS	_A2P
	SUBK	-32768
	BLZ	L82
	ZALS	_A2P
	SUBK	-11776
	BGEZ	L82
*** 12	-----------------------    S$3 = 1;
	LACK	1
	SACL	STATIC_157
L82:
***	-----------------------g2:
*** 12	-----------------------    TDP = (unsigned)S$3;
	LAC	STATIC_157
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
	LAC	STATIC_157
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
	SBRK	2
	LAR	AR0,*
***	RET OCCURS

	.endfunc	805,000000000H,1

	.sect	".cinit"
	.word	1,_LAW
	.word	0
	.text

	.sym	_LAW,_LAW,4,2,16
	.globl	_LAW
	.bss	_LAW,1

	.sym	_reset_encoder,_reset_encoder,32,2,0
	.globl	_reset_encoder

	.func	811
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

	.endfunc	815,000000000H,1

	.sym	_encoder,_encoder,46,2,0
	.globl	_encoder

	.func	817
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

	.endfunc	837,000000000H,1

	.sym	_reset_decoder,_reset_decoder,32,2,0
	.globl	_reset_decoder

	.func	839
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

	.endfunc	843,000000000H,1

	.sym	_decoder,_decoder,46,2,0
	.globl	_decoder

	.func	845
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

	.endfunc	866,000000000H,1
	.file	"board_test.c"

	.sym	_main,_main,36,2,0
	.globl	_main

	.func	13
******************************************************
* FUNCTION DEF : _main
******************************************************
_main:

LF20	.set	0

	SAR	AR0,*+
	POPD	*+
	SAR	AR1,*
	LARK	AR0,2
	CALLD	_reset_encoder
	LAR	AR0,*0+
;	<save register vars>
	SAR	AR6,*+

*
* AR6	assigned to temp var 'U$4'
*
*** 5	-----------------------    reset_encoder();
	.sym	_i,1,4,1,16
	.line	5
***	CALL	_reset_encoder OCCURS
*** 6	-----------------------    reset_decoder();
	.line	6
	CALL	_reset_decoder
*** 8	-----------------------    i = 0;
	.line	8
	LACK	0
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* ,AR6
***  	-----------------------    U$4 = &Input[0];
	LRLK	AR6,_Input
L84:
***	-----------------------g2:
*** 9	-----------------------    decoder(encoder(*U$4++));
	.line	9
	CALLD	_encoder
	LAC	*+,AR1
	SACL	*+
***	CALL	_encoder OCCURS
	CALLD	_decoder
	MAR	*-
	SACL	*+
***	CALL	_decoder OCCURS
	MAR	*-,AR2
*** 8	-----------------------    if ( (unsigned)(++i) < 32u ) goto g2;
	.line	8
	SSXM
	LARK	AR2,1
	MAR	*0+
	LAC	* 
	ADDK	1
	SACL	* ,AR6
	ANDK	0FFFFh
	SUBK	32
	BLZ	L84
*** 11	-----------------------    return 0;
	.line	11
	LACK	0
EPI0_20:
	.line	12
	MAR	* ,AR1
;	<restore register vars>
	MAR	*-
	LAR	AR6,* 
	SBRK	3
	PSHD	*-
	RETD
	LAR	AR0,*
	NOP
***	RET OCCURS

	.endfunc	24,000000040H,2

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
