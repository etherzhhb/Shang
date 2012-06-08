!		Analog Devices ADSP21XX
.MODULE/RAM	_board_test_;
.external ___lib_save_frame;
.external ___lib_restore_frame;
!gcc_compiled


.entry	Input_;
.var	Input_[32];
.init	Input_: 255,167,25,146,15,145,24,163,77,43,154,19,143,17,150,32,190,175,28,
	148,16,144,21,158,54,54,158,21,144,16,148,28;
.var	A_LAW_table_[256];
.init	A_LAW_table_: 4784,4752,4848,4816,4656,4624,4720,4688,5040,5008,5104,5072,4912,4880,4976,4944,4440,4424,4472,
	4456,4376,4360,4408,4392,4568,4552,4600,4584,4504,4488,4536,4520,6848,6720,7104,6976,6336,6208,6592,
	6464,7872,7744,8128,8000,7360,7232,7616,7488,5472,5408,5600,5536,5216,5152,5344,5280,5984,5920,6112,
	6048,5728,5664,5856,5792,4139,4137,4143,4141,4131,4129,4135,4133,4155,4153,4159,4157,4147,4145,4151,
	4149,4107,4105,4111,4109,4099,4097,4103,4101,4123,4121,4127,4125,4115,4113,4119,4117,4268,4260,4284,
	4276,4236,4228,4252,4244,4332,4324,4348,4340,4300,4292,4316,4308,4182,4178,4190,4186,4166,4162,4174,
	4170,4214,4210,4222,4218,4198,4194,4206,4202,688,656,752,720,560,528,624,592,944,912,1008,
	976,816,784,880,848,344,328,376,360,280,264,312,296,472,456,504,488,408,392,440,
	424,2752,2624,3008,2880,2240,2112,2496,2368,3776,3648,4032,3904,3264,3136,3520,3392,1376,1312,1504,
	1440,1120,1056,1248,1184,1888,1824,2016,1952,1632,1568,1760,1696,43,41,47,45,35,33,39,
	37,59,57,63,61,51,49,55,53,11,9,15,13,3,1,7,5,27,25,31,
	29,19,17,23,21,172,164,188,180,140,132,156,148,236,228,252,244,204,196,220,
	212,86,82,94,90,70,66,78,74,118,114,126,122,102,98,110,106;
.var	u_LAW_table_[256];
.init	u_LAW_table_: 16223,15967,15711,15455,15199,14943,14687,14431,14175,13919,13663,13407,13151,12895,12639,12383,12191,12063,11935,
	11807,11679,11551,11423,11295,11167,11039,10911,10783,10655,10527,10399,10271,10175,10111,10047,9983,9919,9855,9791,
	9727,9663,9599,9535,9471,9407,9343,9279,9215,9167,9135,9103,9071,9039,9007,8975,8943,8911,8879,8847,
	8815,8783,8751,8719,8687,8663,8647,8631,8615,8599,8583,8567,8551,8535,8519,8503,8487,8471,8455,8439,
	8423,8411,8403,8395,8387,8379,8371,8363,8355,8347,8339,8331,8323,8315,8307,8299,8291,8285,8281,8277,
	8273,8269,8265,8261,8257,8253,8249,8245,8241,8237,8233,8229,8225,8222,8220,8218,8216,8214,8212,8210,
	8208,8206,8204,8202,8200,8198,8196,8194,0,8031,7775,7519,7263,7007,6751,6495,6239,5983,5727,5471,
	5215,4959,4703,4447,4191,3999,3871,3743,3615,3487,3359,3231,3103,2975,2847,2719,2591,2463,2335,2207,
	2079,1983,1919,1855,1791,1727,1663,1599,1535,1471,1407,1343,1279,1215,1151,1087,1023,975,943,911,
	879,847,815,783,751,719,687,655,623,591,559,527,495,471,455,439,423,407,391,375,
	359,343,327,311,295,279,263,247,231,219,211,203,195,187,179,171,163,155,147,139,
	131,123,115,107,99,93,89,85,81,77,73,69,65,61,57,53,49,45,41,37,
	33,30,28,26,24,22,20,18,16,14,12,10,8,6,4,2,0;


adapt_quant_:
!	FUNCTION PROLOGUE: adapt_quant
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax1;
	dm(i4,m5)=ay0;
!	END FUNCTION PROLOGUE: adapt_quant
	ax1=dm(D_);
	ay1=-32768;
	ar=ax1 and ay1;
	mr1=ar;
	af=pass mr1;
	if eq jump adapt_quantL2_;
	ar= -ax1;
	ay1=32767;
	ar=ar and ay1;
	my1=ar;
	jump adapt_quantL3_;
adapt_quantL2_:
	my1=ax1;
adapt_quantL3_:
	sr1=my1;
	sr=lshift sr1 by -1 (hi);
	si=0;
	ax1=sr1;
	af=pass ax1;
	if eq jump adapt_quantL5_;
adapt_quantL6_:
	sr1=ax1;
	ay0=si;
	sr=lshift sr1 by -1 (hi);
	ar=ay0+1,ax1=sr1;	!!PRLL-2 
	af=pass ax1,si=ar;	!!PRLL-2 
	if ne jump adapt_quantL6_;
adapt_quantL5_:
	ay1=-7;
	ar=si;
	sr=ashift si by 7 (hi);
	ar=ar+ay1,mr0=sr1;	!!PRLL-2 
	ax1=ar;
	af=pass ax1;
	if ge jump adapt_quantL7_;
	ay0=si;
	ax1=7;
	ay1=127;
	ar=ax1-ay0,sr1=my1;	!!PRLL-2 
	ax1=ar;
	se=ax1; sr=lshift sr1 (hi);
	jump adapt_quantL41_;
adapt_quantL7_:
	ay1=127;
	ar= -ax1,sr1=my1;	!!PRLL-2 
	ax1=ar;
	se=ax1; sr=lshift sr1 (hi);
adapt_quantL41_:
	ax1=sr1;
	ar=ax1 and ay1;
	ay1=ar;
	ar=mr0+ay1;
	ax1=ar;
	mr0=dm(Y_);
	sr=lshift mr0 by -2 (hi);
	ay1=sr1;
	ar=ax1-ay1;
	ay1=4095;
	ar=ar and ay1;
	ay1=299;
	ax1=ar;
	af=ax1-ay1;
	if not ac jump adapt_quantL9_;
	if eq jump adapt_quantL9_;
	ay1=2047;
	af=ax1-ay1;
	if not ac jump adapt_quantL10_;
	if eq jump adapt_quantL10_;
	ay1=3971;
	af=ax1-ay1;
	if not ac jump adapt_quantL11_;
	if eq jump adapt_quantL11_;
	af=pass mr1;
	if ne jump adapt_quantL42_;
	jump adapt_quantL39_;
adapt_quantL11_:
	sr1=15;
	dm(I_)=sr1;
	jump adapt_quantL26_;
adapt_quantL10_:
	ay1=399;
	af=ax1-ay1;
	if not ac jump adapt_quantL16_;
	if eq jump adapt_quantL16_;
	af=pass mr1;
	if eq jump adapt_quantL17_;
	ay0=8;
	dm(I_)=ay0;
	jump adapt_quantL26_;
adapt_quantL17_:
	ar=7;
	jump adapt_quantL43_;
adapt_quantL16_:
	ay1=348;
	af=ax1-ay1;
	if not ac jump adapt_quantL20_;
	if eq jump adapt_quantL20_;
	af=pass mr1;
	if eq jump adapt_quantL21_;
	sr1=9;
	dm(I_)=sr1;
	jump adapt_quantL26_;
adapt_quantL21_:
	ay0=6;
	dm(I_)=ay0;
	jump adapt_quantL26_;
adapt_quantL20_:
	af=pass mr1;
	if eq jump adapt_quantL24_;
	ar=10;
	jump adapt_quantL43_;
adapt_quantL24_:
	sr1=5;
	dm(I_)=sr1;
	jump adapt_quantL26_;
adapt_quantL9_:
	ay1=177;
	af=ax1-ay1;
	if not ac jump adapt_quantL27_;
	if eq jump adapt_quantL27_;
	ay1=245;
	af=ax1-ay1;
	if not ac jump adapt_quantL28_;
	if eq jump adapt_quantL28_;
	af=pass mr1;
	if eq jump adapt_quantL29_;
	ay0=11;
	dm(I_)=ay0;
	jump adapt_quantL26_;
adapt_quantL29_:
	ar=4;
	jump adapt_quantL43_;
adapt_quantL28_:
	af=pass mr1;
	if eq jump adapt_quantL32_;
	sr1=12;
	dm(I_)=sr1;
	jump adapt_quantL26_;
adapt_quantL32_:
	ay0=3;
	dm(I_)=ay0;
	jump adapt_quantL26_;
adapt_quantL27_:
	ay1=79;
	af=ax1-ay1;
	if not ac jump adapt_quantL35_;
	if eq jump adapt_quantL35_;
	af=pass mr1;
	if eq jump adapt_quantL36_;
	ar=13;
	jump adapt_quantL43_;
adapt_quantL36_:
	sr1=2;
	dm(I_)=sr1;
	jump adapt_quantL26_;
adapt_quantL35_:
	af=pass mr1;
	if eq jump adapt_quantL39_;
adapt_quantL42_:
	ay0=14;
	dm(I_)=ay0;
	jump adapt_quantL26_;
adapt_quantL39_:
	ar=1;
adapt_quantL43_:
	dm(I_)=ar;
adapt_quantL26_:
!	FUNCTION EPILOGUE: adapt_quant
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax1=dm(i6,m5);
	ay0=dm(i6,m5);
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: adapt_quant
	jump (i6);

.external	__lshldi3_;
.external	__lshrdi3_;
adpt_predict_1_:
!	FUNCTION PROLOGUE: adpt_predict_1
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax0;
	dm(i4,m5)=ax1;
	dm(i4,m5)=ay0;
	dm(i4,m5)=mx0;
	dm(i4,m5)=mx1;
	dm(i4,m5)=my0;
	sr0=i2;
	dm(i4,m5)=sr0;
	sr0=i5;
	dm(i4,m5)=sr0;
	sr0=m0;
	dm(i4,m5)=sr0;
	sr0=m7;
	dm(i4,m5)=sr0;
!	END FUNCTION PROLOGUE: adpt_predict_1
	mx1=0;
	mx0=0;
	m5=8;
	m0=1;
	ax1=dm(X_);
	m7=-1;
	i2=ax1;
	i5=i2;
	modify(i5,m5);
adpt_predict_1L64_:
	mr1=dm(i2,m2);
	sr=lshift mr1 by -15 (hi);
	ax0=sr1;
	af=pass ax0;
	if eq jump adpt_predict_1L48_;
	sr=lshift mr1 by -2 (hi);
	ay1=8191;
	ar= -sr1;
	modify(i2,m0);
	ar=ar and ay1;
	my1=ar;
	jump adpt_predict_1L49_;
adpt_predict_1L48_:
	sr=lshift mr1 by -2 (hi);
	modify(i2,m0);
	my1=sr1;
adpt_predict_1L49_:
	ax1=my1;
	si=0;
	af=pass ax1;
	if eq jump adpt_predict_1L53_;
adpt_predict_1L52_:
	sr1=ax1;
	ay0=si;
	sr=lshift sr1 by -1 (hi);
	ar=ay0+1,ax1=sr1;	!!PRLL-2 
	af=pass ax1,si=ar;	!!PRLL-2 
	if ne jump adpt_predict_1L52_;
	ar=my1;
	af=pass ar;
	if eq jump adpt_predict_1L53_;
	ay1=-6;
	ar=si;
	ar=ar+ay1;
	ax1=ar;
	af=pass ax1;
	if ge jump adpt_predict_1L55_;
	ax1=6;
	ay0=si;
	ar=ax1-ay0,sr1=my1;	!!PRLL-2 
	ax1=ar;
	se=ax1; sr=lshift sr1 (hi);
	ax1=sr1;
	jump adpt_predict_1L54_;
adpt_predict_1L55_:
	ar= -ax1,sr1=my1;	!!PRLL-2 
	ax1=ar;
	se=ax1; sr=lshift sr1 (hi);
	ax1=sr1;
	jump adpt_predict_1L54_;
adpt_predict_1L53_:
	ax1=32;
adpt_predict_1L54_:
	ay0=dm(i5,m6);
	ay1=63;
	ar=ay0;
	ar=ar and ay1,my0=ax1;	!!PRLL-2 
	sr0=ar;
	ay1=48;
	mr=sr0*my0 (ss);
	ax1=mr0;
	ar=ax1+ay1,dm(i5,m6)=ay0;	!!PRLL-2 
	ay1=15;
	mr1=ar;
	sr=lshift mr1 by -4 (hi);
	my1=sr1;
	sr1=ay0;
	sr=lshift sr1 by -6 (hi);
	ax1=sr1;
	ar=ax1 and ay1,ay0=si;	!!PRLL-2 
	ax1=ar;
	ar=ax1+ay0;
	ax1=19;
	mr1=ar;
	ay0=mr1;
	ar=ax1-ay0;
	ay1=ar;
	af=pass ay1;
	if ge jump adpt_predict_1L57_;
	ay1=-19;
	ar=mr1+ay1,ax1=my1;	!!PRLL-2 
	ay1=ar;
	dm(i4,m7)=ay1;
	dm(i4,m7)=ax1;
	dm(i4,m7)=0;
	call __lshldi3_;
	jump adpt_predict_1L65_;
adpt_predict_1L57_:
	ax1=my1;
	dm(i4,m7)=ay1;
	dm(i4,m7)=ax1;
	dm(i4,m7)=0;
	call __lshrdi3_;
adpt_predict_1L65_:
	m5=3;
	ay0=32767;
	ax1=sr0;
	modify(i4,m5);
	ar=ax1 and ay0;
	dm(Register_Spill_Area__+0)=ar;
	m5=m0;
	ay1=1024;
	ax1=dm(i5,m5);
	ar=ax1 and ay1;
	af=pass ar;
	if eq jump adpt_predict_1L61_;
	ay1=1;
	af=ax0-ay1;
	if ne jump adpt_predict_1L62_;
	jump adpt_predict_1L59_;
adpt_predict_1L61_:
	af=pass ax0;
	if eq jump adpt_predict_1L59_;
adpt_predict_1L62_:
	ay0=dm(Register_Spill_Area__+0);
	ar=mx1;
	ar=ar-ay0;
	jump adpt_predict_1L66_;
adpt_predict_1L59_:
	ay0=dm(Register_Spill_Area__+0);
	ar=mx1;
	ar=ar+ay0;
adpt_predict_1L66_:
	mx1=ar;
	ax1=5;
	ay0=mx0;
	af=ay0-ax1;
	if ne jump adpt_predict_1L47_;
	sr1=mx1;
	sr=lshift sr1 by -1 (hi);
	ax1=sr1;
	dm(SEZ_)=ax1;
adpt_predict_1L47_:
	ay0=mx0;
	ax1=7;
	ar=ay0+1;
	mx0=ar;
	ay0=mx0;
	af=ay0-ax1;
	if le jump adpt_predict_1L64_;
	sr1=mx1;
	sr=lshift sr1 by -1 (hi);
	ax1=sr1;
	dm(SE_)=ax1;
!	FUNCTION EPILOGUE: adpt_predict_1
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax0=dm(i6,m5);
	ax1=dm(i6,m5);
	ay0=dm(i6,m5);
	mx0=dm(i6,m5);
	mx1=dm(i6,m5);
	my0=dm(i6,m5);
	ay1=dm(i6,m5);
	i2=ay1;
	ay1=dm(i6,m5);
	i5=ay1;
	ay1=dm(i6,m5);
	m0=ay1;
	ay1=dm(i6,m5);
	m7=ay1;
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: adpt_predict_1
	jump (i6);

.external	__adddi3_;
.external	__negdi2_;
adpt_predict_2_:
!	FUNCTION PROLOGUE: adpt_predict_2
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax0;
	dm(i4,m5)=ax1;
	dm(i4,m5)=ay0;
	dm(i4,m5)=mx0;
	dm(i4,m5)=mx1;
	dm(i4,m5)=my0;
	sr0=i0;
	dm(i4,m5)=sr0;
	sr0=i2;
	dm(i4,m5)=sr0;
	sr0=i3;
	dm(i4,m5)=sr0;
	sr0=m0;
	dm(i4,m5)=sr0;
	sr0=m7;
	dm(i4,m5)=sr0;
!	END FUNCTION PROLOGUE: adpt_predict_2
	m0=dm(X_);
	i2=m0;
	i0=i2;
	m3=6;
	ay1=16384;
	modify(i0,m3);
	ax1=dm(DQ_);
	m3=7;
	i3=i2;
	ar=ax1 and ay1;
	modify(i3,m3);
	af=pass ar;
	if eq jump adpt_predict_2L68_;
	ay1=16383;
	ar=ax1 and ay1;
	ar= -ar;
	mx1=ar;
	jump adpt_predict_2L69_;
adpt_predict_2L68_:
	mx1=ax1;
adpt_predict_2L69_:
	ax1=dm(SEZ_);
	ay1=16384;
	ar=ax1 and ay1;
	af=pass ar;
	if eq jump adpt_predict_2L70_;
	ay1=-32768;
	ar=mx1;
	ar=ar+ay1,ay0=ax1;	!!PRLL-2 
	mr0=ar;
	ar=mr0+ay0;
	jump adpt_predict_2L148_;
adpt_predict_2L70_:
	ay1=ax1;
	ax1=mx1;
	ar=ax1+ay1;
adpt_predict_2L148_:
	dm(Register_Spill_Area__+3)=ar;
	ax1=dm(DQ_);
	sr1=dm(Register_Spill_Area__+3);
	ay1=16384;
	sr=lshift sr1 by -15 (hi);
	ar=ax1 and ay1;
	dm(Register_Spill_Area__+4)=sr1;
	ay1=ar;
	af=pass ay1;
	if eq jump adpt_predict_2L72_;
	ay1=16383;
	ar=ax1 and ay1;
	ar= -ar;
	mx1=ar;
	jump adpt_predict_2L73_;
adpt_predict_2L72_:
	mx1=ax1;
adpt_predict_2L73_:
	ax1=dm(SE_);
	ay1=16384;
	ar=ax1 and ay1;
	af=pass ar;
	if eq jump adpt_predict_2L74_;
	ay1=-32768;
	ar=mx1;
	ar=ar+ay1,ay0=ax1;	!!PRLL-2 
	mr0=ar;
	ar=mr0+ay0;
	jump adpt_predict_2L149_;
adpt_predict_2L74_:
	ay1=ax1;
	ax1=mx1;
	ar=ax1+ay1;
adpt_predict_2L149_:
	mx1=ar;
	ay1=-32768;
	ar=mx1;
	dm(SR_)=mx1;
	ar=ar and ay1;
	ax1=ar;
	af=pass ax1;
	if eq jump adpt_predict_2L76_;
	ar=mx1;
	ar= -ar;
	ay1=32767;
	mr0=ar;
	ar=mr0 and ay1;
	my1=ar;
	jump adpt_predict_2L77_;
adpt_predict_2L76_:
	my1=mx1;
adpt_predict_2L77_:
	mx1=my1;
	sr1=my1;
	mr0=0;
	af=pass sr1;
	if eq jump adpt_predict_2L79_;
adpt_predict_2L80_:
	sr1=mx1;
	ay1=mr0;
	sr=lshift sr1 by -1 (hi);
	ar=ay1+1,mx1=sr1;	!!PRLL-2 
	af=pass sr1,mr0=ar;	!!PRLL-2 
	if ne jump adpt_predict_2L80_;
adpt_predict_2L79_:
	sr=ashift mr0 by 6 (hi);
	af=pass ax1,mx1=sr1;	!!PRLL-2 
	if eq jump adpt_predict_2L81_;
	ay1=1024;
	ar=mx1;
	ar=ar+ay1;
	mx1=ar;
adpt_predict_2L81_:
	sr1=my1;
	af=pass sr1;
	if eq jump adpt_predict_2L82_;
	ay1=-6;
	ar=mr0+ay1;
	ax1=ar;
	af=pass ax1;
	if ge jump adpt_predict_2L84_;
	ax1=6;
	ay0=mr0;
	ar=ax1-ay0,ay0=mx1;	!!PRLL-2 
	ax1=ar;
	se=ax1; sr=lshift sr1 (hi);
	ax1=sr1;
	ar=ax1+ay0;
	jump adpt_predict_2L150_;
adpt_predict_2L84_:
	ar= -ax1,sr1=my1;	!!PRLL-2 
	ax1=ar;
	se=ax1; sr=lshift sr1 (hi);
	ax1=sr1;
	ay1=ax1;
	ax1=mx1;
	ar=ax1+ay1;
	jump adpt_predict_2L150_;
adpt_predict_2L82_:
	ay1=32;
	ar=mx1;
	ar=ar+ay1;
adpt_predict_2L150_:
	dm(Register_Spill_Area__+5)=ar;
	m3=16;
	i2=m0;
	ar=dm(Register_Spill_Area__+4);
	modify(i2,m3);
	ax1=dm(i2,m2);
	ay1=ax1;
	ax1=ar;
	i2=i0;
	ar=ax1 xor ay1;
	dm(Register_Spill_Area__+6)=ar;
	mr0=dm(i2,m2);
	af=pass mr0;
	if ge jump adpt_predict_2L86_;
	ay1=-8192;
	af=mr0-ay1;
	if not ac jump adpt_predict_2L87_;
	if eq jump adpt_predict_2L87_;
	ax1=mr0;
	m5=-1;
	dm(i4,m5)=2;
	dm(i4,m5)=ax1;
	dm(i4,m5)=0;
	call __lshldi3_;
	m5=3;
	ay1=1;
	modify(i4,m5);
	ax1=sr1;
	ar=ax1 and ay1,mx1=sr0;	!!PRLL-2 
	my1=mx1;
	mr0=ar;
	jump adpt_predict_2L89_;
adpt_predict_2L87_:
	mr0=1;
	my1=-32764;
	jump adpt_predict_2L89_;
adpt_predict_2L86_:
	ay1=8191;
	af=mr0-ay1;
	if eq jump adpt_predict_2L157_;
	if ac jump adpt_predict_2L90_;
adpt_predict_2L157_:
	sr=lshift mr0 by 2 (hi);
	mr0=0;
	my1=sr1;
	jump adpt_predict_2L89_;
adpt_predict_2L90_:
	mr0=0;
	my1=32764;
adpt_predict_2L89_:
	m3=17;
	i2=m0;
	ar=dm(Register_Spill_Area__+4);
	modify(i2,m3);
	ax1=dm(i2,m2);
	ay1=ax1;
	ax1=ar;
	af=ax1-ay1;
	if eq jump adpt_predict_2L92_;
	ax0=1;
	my0=-16384;
	jump adpt_predict_2L93_;
adpt_predict_2L92_:
	ax0=0;
	my0=16384;
adpt_predict_2L93_:
	sr1=dm(Register_Spill_Area__+6);
	af=pass sr1;
	if eq jump adpt_predict_2L94_;
	m7=-1;
	dm(i4,m7)=my1;
	dm(i4,m7)=mr0;
	jump adpt_predict_2L151_;
adpt_predict_2L94_:
	m7=-1;
	dm(i4,m7)=my1;
	dm(i4,m7)=mr0;
	call __negdi2_;
	ay1=1;
	ax1=sr1;
	m5=2;
	mx1=sr0;
	modify(i4,m5);
	ar=ax1 and ay1,ay1=mx1;	!!PRLL-2 
	dm(i4,m7)=ay1;
	dm(i4,m7)=ar;
adpt_predict_2L151_:
	dm(i4,m7)=my0;
	dm(i4,m7)=ax0;
	call __adddi3_;
	m5=4;
	modify(i4,m5);
	dm(i4,m7)=7;
	ax1=sr0;
	ay1=sr1;
	dm(i4,m7)=ax1;
	dm(i4,m7)=ay1;
	call __lshrdi3_;
	ay1=1023;
	sr1=dm(Register_Spill_Area__+3);
	ar=sr0 and ay1;
	af=pass sr1,ax1=ar;	!!PRLL-2 
	if eq jump adpt_predict_2L96_;
	mx1=ax1;
	ay1=512;
	ar=mx1;
	ar=ar and ay1;
	ax1=ar;
	af=pass ax1;
	if eq jump adpt_predict_2L97_;
	ay1=-1024;
	ar=mx1;
	ar=ar or ay1;
	mx1=ar;
	jump adpt_predict_2L97_;
adpt_predict_2L96_:
	mx1=0;
adpt_predict_2L97_:
	i2=i3;
	mr0=dm(i2,m2);
	af=pass mr0;
	if ge jump adpt_predict_2L99_;
	sr=lshift mr0 by -7 (hi);
	ay1=-512;
	ar=sr1+ay1;
	ar= -ar;
	jump adpt_predict_2L152_;
adpt_predict_2L99_:
	sr=lshift mr0 by -7 (hi);
	my1=sr1;
	ax1=my1;
	ar= -ax1;
adpt_predict_2L152_:
	my1=ar;
	i2=i3;
	ar=mx1;
	ay1=dm(i2,m2);
	ar=ar+ay1,ay0=my1;	!!PRLL-2 
	ax1=ar;
	ar=ax1+ay0;
	ax1=dm(TR_);
	af=pass ax1,mx1=ar;	!!PRLL-2 
	if eq jump adpt_predict_2L101_;
	ax1=0;
adpt_predict_2L105_:
	cntr= 8; do adpt_predict_2L147_-1 until ce; !! (depth 1)
	i2=m0;
	ay1=ax1;
	m3=ax1;
	ar=ay1+1;
	modify(i2,m3);
	ax1=ar;
	dm(i2,m2)=0;
	      !!  Loop Jump Was Here 
adpt_predict_2L147_:
	jump adpt_predict_2L106_;
adpt_predict_2L101_:
	ax1=mx1;
	af=pass ax1;
	if lt jump adpt_predict_2L107_;
	ay1=12288;
	dm(A2P_)=mx1;
	af=ax1-ay1;
	if not ac jump adpt_predict_2L108_;
	if eq jump adpt_predict_2L108_;
	dm(A2P_)=ay1;
	jump adpt_predict_2L108_;
adpt_predict_2L107_:
	sr1=mx1;
	ay1=-12289;
	dm(A2P_)=mx1;
	af=sr1-ay1;
	if eq jump adpt_predict_2L158_;
	if ac jump adpt_predict_2L108_;
adpt_predict_2L158_:
	ay0=-12288;
	dm(A2P_)=ay0;
adpt_predict_2L108_:
	m3=16;
	i2=m0;
	ax1=dm(A2P_);
	modify(i2,m3);
	dm(i3,m2)=ax1;
	ax1=dm(i2,m2);
	ar=dm(Register_Spill_Area__+4);
	ay1=ax1;
	ax1=ar;
	i2=i0;
	ar=ax1 xor ay1,ax1=dm(i2,m2);	!!PRLL-2 
	sr1=dm(Register_Spill_Area__+3);
	af=pass sr1,ay1=ar;	!!PRLL-2 
	if eq jump adpt_predict_2L111_;
	af=pass ay1;
	if eq jump adpt_predict_2L113_;
	ay1=-192;
	jump adpt_predict_2L153_;
adpt_predict_2L113_:
	ay1=192;
adpt_predict_2L153_:
	ar=ax1+ay1;
	ax1=ar;
adpt_predict_2L111_:
	i2=i0;
	mr0=dm(i2,m2);
	af=pass mr0;
	if ge jump adpt_predict_2L115_;
	ay1=256;
	sr=lshift mr0 by -8 (hi);
	ar=ax1+ay1,ay0=sr1;	!!PRLL-2 
	ar=ar-ay0;
	jump adpt_predict_2L154_;
adpt_predict_2L115_:
	sr=lshift mr0 by -8 (hi);
	ay1=sr1;
	ar=ax1-ay1;
adpt_predict_2L154_:
	mr1=ar;
	i2=i3;
	ax1=15360;
	mr0=dm(i2,m2);
	ay0=mr0;
	ay1=-15360;
	ar=ax1-ay0;
	my1=ar;
	ar=mr0+ay1,ax1=mr1;	!!PRLL-2 
	af=pass ax1,mx1=ar;	!!PRLL-2 
	if lt jump adpt_predict_2L117_;
	ay1=my1;
	af=ax1-ay1,dm(i0,m2)=mr1;	!!PRLL-2 
	if not ac jump adpt_predict_2L118_;
	if eq jump adpt_predict_2L118_;
	dm(i0,m2)=my1;
	jump adpt_predict_2L118_;
adpt_predict_2L117_:
	ax1=mr1;
	ay1=mx1;
	af=ax1-ay1,dm(i0,m2)=mr1;	!!PRLL-2 
	if ac jump adpt_predict_2L118_;
	if eq jump adpt_predict_2L118_;
	dm(i0,m2)=mx1;
adpt_predict_2L118_:
	si=-32768;
	ax0=128;
	i2=m0;
	mx0=-128;
	i1=i2;
	ay1=1;
	mr0=dm(DQ_);
	m3=8;
	sr=lshift mr0 by -14 (hi);
	modify(i1,m3);
	my1=sr1;
	ar=my1;
	my0=1;
	ar=ar and ay1;
	ay1=16383;
	i3=1;
	my1=ar;
	i0=m0;
	ar=mr0 and ay1;
	mr1=ar;
adpt_predict_2L132_:
	cntr= 6; do adpt_predict_2L106_-1 until ce; !! (depth 1)
	i2=i0;
	af=pass mr1,mx1=dm(i2,m2);	!!PRLL-2 
	if eq jump adpt_predict_2L124_;
	ax1=dm(i1,m2);
	ay0=1024;
	i2=i1;
	i6=i3;
	m3=i6;
	ar=ax1 and ay0;
	modify(i1,m3);
	af=pass ar;
	if eq jump adpt_predict_2L128_;
	ay1=my0;
	ax1=my1;
	af=ax1-ay1;
	if ne jump adpt_predict_2L129_;
	jump adpt_predict_2L126_;
adpt_predict_2L128_:
	ar=my1;
	af=pass ar;
	if eq jump adpt_predict_2L126_;
adpt_predict_2L129_:
	ay1=mx0;
	jump adpt_predict_2L155_;
adpt_predict_2L126_:
	ay1=ax0;
adpt_predict_2L155_:
	ax1=mx1;
	ar=ax1+ay1;
	ax1=ar;
	jump adpt_predict_2L125_;
adpt_predict_2L124_:
	ax1=mx1;
adpt_predict_2L125_:
	i2=i0;
	ay0=si;
	mr0=dm(i2,m2);
	ar=mr0 and ay0;
	af=pass ar;
	if eq jump adpt_predict_2L130_;
	ay0=256;
	sr=lshift mr0 by -8 (hi);
	ar=ax1+ay0,ay1=sr1;	!!PRLL-2 
	ar=ar-ay1;
	jump adpt_predict_2L156_;
adpt_predict_2L130_:
	sr=lshift mr0 by -8 (hi);
	ay1=sr1;
	ar=ax1-ay1;
adpt_predict_2L156_:
	ax1=ar;
	dm(i0,m2)=ax1;
	i2=i0;
	m3=1;
	modify(i0,m3);
	      !!  Loop Jump Was Here 
adpt_predict_2L146_:
adpt_predict_2L106_:
	i2=m0;
	m3=13;
	modify(i2,m3);
adpt_predict_2L136_:
	cntr= 5; do adpt_predict_2L145_-1 until ce; !! (depth 1)
	i6=i2;
	m3=-1;
	modify(i2,m3);
	m3=-1;
	ar=dm(i2,m2);
	i2=i6;
	modify(i2,m3);
	dm(i6,m6)=ar;
	      !!  Loop Jump Was Here 
adpt_predict_2L145_:
	ay1=16383;
	ax1=dm(DQ_);
	mx1=0;
	ar=ax1 and ay1;
	mr0=ar;
	af=pass mr0,ax1=mr0;	!!PRLL-2 
	if eq jump adpt_predict_2L138_;
adpt_predict_2L139_:
	sr1=ax1;
	ay1=mx1;
	sr=lshift sr1 by -1 (hi);
	ar=ay1+1,ax1=sr1;	!!PRLL-2 
	af=pass ax1,mx1=ar;	!!PRLL-2 
	if ne jump adpt_predict_2L139_;
adpt_predict_2L138_:
	ay1=16384;
	sr1=mx1;
	m3=8;
	sr=ashift sr1 by 6 (hi);
	i2=m0;
	ax1=sr1;
	my1=ax1;
	ax1=dm(DQ_);
	modify(i2,m3);
	ar=ax1 and ay1;
	ax1=ar;
	af=pass ax1;
	if eq jump adpt_predict_2L140_;
	ay1=1024;
	ar=my1;
	ar=ar+ay1;
	my1=ar;
adpt_predict_2L140_:
	af=pass mr0;
	if eq jump adpt_predict_2L141_;
	ay1=-6;
	ar=mx1;
	ar=ar+ay1;
	ax1=ar;
	af=pass ax1;
	if ge jump adpt_predict_2L143_;
	ax1=6;
	ay0=mx1;
	ar=ax1-ay0,ay0=my1;	!!PRLL-2 
	se=ar; sr=lshift mr0 (hi);
	ar=sr1+ay0;
	dm(i2,m2)=ar;
	jump adpt_predict_2L142_;
adpt_predict_2L143_:
	ar= -ax1,ax1=my1;	!!PRLL-2 
	se=ar; sr=lshift mr0 (hi);
	ay1=sr1;
	ar=ax1+ay1;
	dm(i2,m2)=ar;
	jump adpt_predict_2L142_;
adpt_predict_2L141_:
	ay1=32;
	ar=my1;
	ar=ar+ay1;
	ax1=ar;
	dm(i2,m2)=ax1;
adpt_predict_2L142_:
	i2=m0;
	m3=17;
	i0=i2;
	modify(i0,m3);
	m3=16;
	i6=i2;
	m5=15;
	modify(i2,m3);
	m3=14;
	ar=dm(i2,m2);
	dm(i0,m2)=ar;
	sr1=dm(Register_Spill_Area__+4);
	dm(i2,m2)=sr1;
	i2=i6;
	modify(i6,m5);
	modify(i2,m3);
	ar=dm(i2,m2);
	dm(i6,m6)=ar;
	sr1=dm(Register_Spill_Area__+5);
	dm(i2,m2)=sr1;
!	FUNCTION EPILOGUE: adpt_predict_2
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax0=dm(i6,m5);
	ax1=dm(i6,m5);
	ay0=dm(i6,m5);
	mx0=dm(i6,m5);
	mx1=dm(i6,m5);
	my0=dm(i6,m5);
	ay1=dm(i6,m5);
	i0=ay1;
	ay1=dm(i6,m5);
	i2=ay1;
	ay1=dm(i6,m5);
	i3=ay1;
	ay1=dm(i6,m5);
	m0=ay1;
	ay1=dm(i6,m5);
	m7=ay1;
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: adpt_predict_2
	jump (i6);

.external	LAW_;
coding_adjustment_:
!	FUNCTION PROLOGUE: coding_adjustment
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax1;
	dm(i4,m5)=ay0;
!	END FUNCTION PROLOGUE: coding_adjustment
	ax1=dm(D_);
	ay1=-32768;
	ar=ax1 and ay1;
	mr1=ar;
	af=pass mr1;
	if eq jump coding_adjustmentL160_;
	ar= -ax1;
	ay1=32767;
	ar=ar and ay1;
	my1=ar;
	jump coding_adjustmentL161_;
coding_adjustmentL160_:
	my1=ax1;
coding_adjustmentL161_:
	sr1=my1;
	sr=lshift sr1 by -1 (hi);
	si=0;
	ax1=sr1;
	af=pass ax1;
	if eq jump coding_adjustmentL163_;
coding_adjustmentL164_:
	sr1=ax1;
	ay1=si;
	sr=lshift sr1 by -1 (hi);
	ar=ay1+1,ax1=sr1;	!!PRLL-2 
	af=pass ax1,si=ar;	!!PRLL-2 
	if ne jump coding_adjustmentL164_;
coding_adjustmentL163_:
	mr0=dm(Y_);
	ay1=-7;
	sr=ashift si by 7 (hi);
	ar=si;
	dm(Register_Spill_Area__+11)=sr1;
	ar=ar+ay1;
	sr=lshift mr0 by -2 (hi);
	ax1=ar;
	dm(Register_Spill_Area__+9)=sr1;
	af=pass ax1;
	if ge jump coding_adjustmentL165_;
	ay0=si;
	ax1=7;
	ay1=127;
	ar=ax1-ay0,sr1=my1;	!!PRLL-2 
	ax1=ar;
	se=ax1; sr=lshift sr1 (hi);
	jump coding_adjustmentL233_;
coding_adjustmentL165_:
	ay1=127;
	ar= -ax1,sr1=my1;	!!PRLL-2 
	ax1=ar;
	se=ax1; sr=lshift sr1 (hi);
coding_adjustmentL233_:
	ax1=sr1;
	ay0=dm(Register_Spill_Area__+11);
	ar=ax1 and ay1;
	ax1=ar;
	ar=ax1 or ay0;
	ay0=dm(Register_Spill_Area__+9);
	ax1=ar;
	ay1=4095;
	ar=ax1-ay0;
	ax1=ar;
	ar=ax1 and ay1;
	mr0=ar;
	ax1=dm(I_);
	ay1=8;
	ar=ax1 and ay1;
	af=pass ar;
	if eq jump coding_adjustmentL167_;
	ay1=7;
	ar=ax1 and ay1;
	jump coding_adjustmentL234_;
coding_adjustmentL167_:
	ar=ax1+ay1;
coding_adjustmentL234_:
	si=ar;
	ay1=299;
	af=mr0-ay1;
	if not ac jump coding_adjustmentL169_;
	if eq jump coding_adjustmentL169_;
	ay1=2047;
	af=mr0-ay1;
	if not ac jump coding_adjustmentL170_;
	if eq jump coding_adjustmentL170_;
	ay1=3971;
	af=mr0-ay1;
	if eq jump coding_adjustmentL238_;
	if ac jump coding_adjustmentL195_;
coding_adjustmentL238_:
	sr0=7;
	jump coding_adjustmentL186_;
coding_adjustmentL170_:
	ay1=399;
	af=mr0-ay1;
	if not ac jump coding_adjustmentL176_;
	if eq jump coding_adjustmentL176_;
	ar=abs mr1;
	ay1=ar;
	ar=ay1-1;
	sr1=ar;
	sr=ashift sr1 by -15 (hi);
	ay1=15;
	dm(Register_Spill_Area__+10)=ar;
	ar=sr1;
	ar=ar and ay1;
	dm(Register_Spill_Area__+10)=sr1;
	sr0=ar;
	jump coding_adjustmentL186_;
coding_adjustmentL176_:
	ay1=348;
	af=mr0-ay1;
	if not ac jump coding_adjustmentL180_;
	if eq jump coding_adjustmentL180_;
	sr0=14;
	af=pass mr1;
	if eq jump coding_adjustmentL186_;
	sr0=1;
	jump coding_adjustmentL186_;
coding_adjustmentL180_:
	sr0=13;
	af=pass mr1;
	if eq jump coding_adjustmentL186_;
	sr0=2;
	jump coding_adjustmentL186_;
coding_adjustmentL169_:
	ay1=177;
	af=mr0-ay1;
	if not ac jump coding_adjustmentL187_;
	if eq jump coding_adjustmentL187_;
	ay1=245;
	af=mr0-ay1;
	if not ac jump coding_adjustmentL188_;
	if eq jump coding_adjustmentL188_;
	sr0=12;
	af=pass mr1;
	if eq jump coding_adjustmentL186_;
	sr0=3;
	jump coding_adjustmentL186_;
coding_adjustmentL188_:
	sr0=11;
	af=pass mr1;
	if eq jump coding_adjustmentL186_;
	sr0=4;
	jump coding_adjustmentL186_;
coding_adjustmentL187_:
	ay1=79;
	af=mr0-ay1;
	if not ac jump coding_adjustmentL195_;
	if eq jump coding_adjustmentL195_;
	sr0=10;
	af=pass mr1;
	if eq jump coding_adjustmentL186_;
	sr0=5;
	jump coding_adjustmentL186_;
coding_adjustmentL195_:
	sr0=9;
	af=pass mr1;
	if eq jump coding_adjustmentL186_;
	sr0=6;
coding_adjustmentL186_:
	ax1=dm(LAW_);
	af=pass ax1;
	if eq jump coding_adjustmentL201_;
	ay1=85;
	ax1=dm(S_);
	ar=ax1 xor ay1,ay1=si;	!!PRLL-2 
	ax1=sr0;
	my1=ar;
	dm(SD_)=my1;
	af=ax1-ay1;
	if not ac jump coding_adjustmentL202_;
	if eq jump coding_adjustmentL202_;
	ay1=126;
	af=ar-ay1;
	if eq jump coding_adjustmentL239_;
	if ac jump coding_adjustmentL203_;
coding_adjustmentL239_:
	ay1=my1;
	ar=ay1+1;
	dm(SD_)=ar;
	jump coding_adjustmentL209_;
coding_adjustmentL203_:
	mr0=128;
	ax1=my1;
	ay1=mr0;
	af=ax1-ay1;
	if not ac jump coding_adjustmentL205_;
	if eq jump coding_adjustmentL205_;
coding_adjustmentL237_:
	ay1=-1;
	ar=my1;
	ar=ar+ay1;
	ax1=ar;
	jump coding_adjustmentL216_;
coding_adjustmentL205_:
	ay0=mr0;
	ar=my1;
	ar=ar xor ay0;
	ax1=ar;
	ar=abs ax1;
	ax1=ar;
	ar= -ax1;
	sr1=ar;
	sr=ashift sr1 by -15 (hi);
	ay1=127;
	dm(Register_Spill_Area__+10)=ar;
	ar=sr1;
	ar=ar and ay1;
	dm(Register_Spill_Area__+10)=sr1;
	ax1=ar;
	jump coding_adjustmentL216_;
coding_adjustmentL202_:
	ay1=si;
	ax1=sr0;
	af=ax1-ay1;
	if ac jump coding_adjustmentL209_;
	if eq jump coding_adjustmentL209_;
	ay1=127;
	sr1=my1;
	af=sr1-ay1;
	if eq jump coding_adjustmentL240_;
	if ac jump coding_adjustmentL211_;
coding_adjustmentL240_:
	af=pass sr1;
	if ne jump coding_adjustmentL237_;
	ax1=128;
	jump coding_adjustmentL216_;
coding_adjustmentL211_:
	ay1=255;
	sr1=my1;
	af=sr1-ay1;
	if eq jump coding_adjustmentL215_;
	ay1=my1;
	ar=ay1+1;
	ax1=ar;
	jump coding_adjustmentL216_;
coding_adjustmentL215_:
	ax1=255;
coding_adjustmentL216_:
	dm(SD_)=ax1;
coding_adjustmentL209_:
	ay1=85;
	ax1=dm(SD_);
	ar=ax1 xor ay1;
	dm(SD_)=ar;
	jump coding_adjustmentL217_;
coding_adjustmentL201_:
	ax1=sr0;
	mr0=dm(S_);
	ay1=si;
	dm(SD_)=mr0;
	af=ax1-ay1;
	if not ac jump coding_adjustmentL218_;
	if eq jump coding_adjustmentL218_;
	af=pass mr0;
	if eq jump coding_adjustmentL219_;
	ay1=127;
	af=mr0-ay1;
	if not ac jump coding_adjustmentL235_;
	if eq jump coding_adjustmentL235_;
coding_adjustmentL219_:
	ax1=dm(S_);
	ay1=-128;
	my1=126;
	ar=ax1+ay1,ay0=my1;	!!PRLL-2 
	af=ar-ay0;
	if eq jump coding_adjustmentL241_;
	if ac jump coding_adjustmentL221_;
coding_adjustmentL241_:
	ax1=dm(SD_);
	ay1=ax1;
	jump coding_adjustmentL236_;
coding_adjustmentL221_:
	ar=abs ax1,ay1=my1;	!!PRLL-2 
	ar= -ar;
	sr=ashift ar by -15 (hi);
	ar=sr1;
	ax1=ar;
	ar=ax1 and ay1;
	dm(SD_)=ar;
	jump coding_adjustmentL217_;
coding_adjustmentL218_:
	ay1=si;
	ax1=sr0;
	af=ax1-ay1;
	if ac jump coding_adjustmentL217_;
	if eq jump coding_adjustmentL217_;
	my1=126;
	ax1=mr0;
	ay1=my1;
	af=ax1-ay1;
	if eq jump coding_adjustmentL242_;
	if ac jump coding_adjustmentL227_;
coding_adjustmentL242_:
	ay1=mr0;
coding_adjustmentL236_:
	ar=ay1+1;
	dm(SD_)=ar;
	jump coding_adjustmentL217_;
coding_adjustmentL227_:
	ay1=-129;
	ar=mr0+ay1,ay0=my1;	!!PRLL-2 
	af=ar-ay0;
	if eq jump coding_adjustmentL243_;
	if ac jump coding_adjustmentL229_;
coding_adjustmentL243_:
coding_adjustmentL235_:
	ay1=-1;
	ar=mr0+ay1;
	dm(SD_)=ar;
	jump coding_adjustmentL217_;
coding_adjustmentL229_:
	ay1=127;
	ax1=128;
	af=mr0-ay1;
	if ne jump coding_adjustmentL231_;
	ax1=254;
coding_adjustmentL231_:
	dm(SD_)=ax1;
coding_adjustmentL217_:
!	FUNCTION EPILOGUE: coding_adjustment
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax1=dm(i6,m5);
	ay0=dm(i6,m5);
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: coding_adjustment
	jump (i6);

diff_computation_:
!	FUNCTION PROLOGUE: diff_computation
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ay0;
!	END FUNCTION PROLOGUE: diff_computation
	my1=dm(SL_);
	ay1=8192;
	ar=my1;
	ar=ar and ay1;
	ay1=ar;
	af=pass ay1;
	if eq jump diff_computationL245_;
	ay1=-16384;
	ar=my1;
	ar=ar+ay1;
	my1=ar;
diff_computationL245_:
	mr0=dm(SE_);
	ay1=16384;
	ar=mr0 and ay1;
	ay1=ar;
	af=pass ay1;
	if eq jump diff_computationL246_;
	ay1=-32768;
	ar=mr0+ay1;
	mr0=ar;
diff_computationL246_:
	ay0=mr0;
	ar=my1;
	ar=ar-ay0;
	ay1=ar;
	dm(D_)=ay1;
!	FUNCTION EPILOGUE: diff_computation
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ay0=dm(i6,m5);
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: diff_computation
	jump (i6);



.var	qtab_12_[16];
.init	qtab_12_: 2048,4,135,213,273,323,373,425,425,373,323,273,213,135,4,2048;


iadpt_quant_:
!	FUNCTION PROLOGUE: iadpt_quant
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax0;
	dm(i4,m5)=ax1;
	dm(i4,m5)=ay0;
	sr0=i2;
	dm(i4,m5)=sr0;
	sr0=m0;
	dm(i4,m5)=sr0;
!	END FUNCTION PROLOGUE: iadpt_quant
	i6=^qtab_12_;
	m0=i6;
	mr0=dm(Y_);
	i2=dm(I_);
	sr=lshift mr0 by -2 (hi);
	modify(i2,m0);
	ax1=sr1;
	ay1=dm(i2,m2);
	ar=ax1+ay1;
	ay1=15;
	mr0=ar;
	sr=lshift mr0 by -7 (hi);
	ax1=sr1;
	ar=ax1 and ay1;
	ay1=127;
	ar=mr0 and ay1,ax0=ar;	!!PRLL-2 
	ay1=128;
	ax1=ar;
	ar=ax1+ay1;
	ay1=8;
	ax1=dm(I_);
	ar=ax1 and ay1,mr1=ar;	!!PRLL-2 
	ax1=ar;
	ay1=16384;
	ar= -ax1;
	ax1=ar;
	sr1=ax1;
	sr=ashift sr1 by -15 (hi);
	ax1=sr1;
	ar=ax1 and ay1;
	ay1=2048;
	ar=mr0 and ay1,my1=ar;	!!PRLL-2 
	ay1=ar;
	af=pass ay1,ax1=my1;	!!PRLL-2 
	if ne jump iadpt_quantL250_;
	ax1=7;
	ay0=ax0;
	ar=ax1-ay0;
	ax1=ar;
	af=pass ax1;
	if ge jump iadpt_quantL252_;
	ay1=-7;
	ar=ax0+ay1;
	ax1=ar;
	ar=my1;
	se=ax1; sr=lshift mr1 (hi);
	jump iadpt_quantL254_;
iadpt_quantL252_:
	ar= -ax1;
	ax1=ar;
	ar=my1;
	se=ax1; sr=lshift mr1 (hi);
iadpt_quantL254_:
	ay1=sr1;
	ar=ar+ay1;
	ax1=ar;
iadpt_quantL250_:
	dm(DQ_)=ax1;
!	FUNCTION EPILOGUE: iadpt_quant
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax0=dm(i6,m5);
	ax1=dm(i6,m5);
	ay0=dm(i6,m5);
	ay1=dm(i6,m5);
	i2=ay1;
	ay1=dm(i6,m5);
	m0=ay1;
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: iadpt_quant
	jump (i6);

input_conversion_:
!	FUNCTION PROLOGUE: input_conversion
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax1;
	sr0=i2;
	dm(i4,m5)=sr0;
	sr0=m0;
	dm(i4,m5)=sr0;
!	END FUNCTION PROLOGUE: input_conversion
	ax1=dm(LAW_);
	af=pass ax1;
	if eq jump input_conversionL256_;
	ay1=4096;
	i6=^A_LAW_table_;
	i2=dm(S_);
	m0=i6;
	modify(i2,m0);
	ax1=dm(i2,m2);
	ar=ax1 and ay1;
	ay1=4095;
	ar=ax1 and ay1,mr0=ar;	!!PRLL-2 
	mr1=ar;
	sr=lshift mr1 by 1 (hi);
	ax1=sr1;
	jump input_conversionL257_;
input_conversionL256_:
	ay1=8192;
	i6=^u_LAW_table_;
	i2=dm(S_);
	m0=i6;
	modify(i2,m0);
	ax1=dm(i2,m2);
	ar=ax1 and ay1;
	ay1=8191;
	ar=ax1 and ay1,mr0=ar;	!!PRLL-2 
	ax1=ar;
input_conversionL257_:
	af=pass mr0;
	if eq jump input_conversionL258_;
	ar= -ax1;
	ay1=16383;
	ar=ar and ay1;
	dm(SL_)=ar;
	jump input_conversionL259_;
input_conversionL258_:
	dm(SL_)=ax1;
input_conversionL259_:
!	FUNCTION EPILOGUE: input_conversion
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax1=dm(i6,m5);
	ay1=dm(i6,m5);
	i2=ay1;
	ay1=dm(i6,m5);
	m0=ay1;
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: input_conversion
	jump (i6);

output_conversion_:
!	FUNCTION PROLOGUE: output_conversion
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax0;
	dm(i4,m5)=ax1;
	dm(i4,m5)=ay0;
!	END FUNCTION PROLOGUE: output_conversion
	ax1=dm(SR_);
	ay1=-32768;
	ar=ax1 and ay1;
	mr0=ar;
	af=pass mr0;
	if eq jump output_conversionL261_;
	ar= -ax1;
	ay1=32767;
	ar=ar and ay1;
	my1=ar;
	jump output_conversionL262_;
output_conversionL261_:
	my1=ax1;
output_conversionL262_:
	ax1=dm(LAW_);
	af=pass ax1;
	if eq jump output_conversionL263_;
	mr1=213;
	af=pass mr0;
	if eq jump output_conversionL266_;
	ay0=my1;
	ay1=-1;
	ar=ay0+1;
	mr0=ar;
	sr=lshift mr0 by -1 (hi);
	mr1=85;
	ar=sr1+ay1;
	ax1=ar;
	jump output_conversionL267_;
output_conversionL266_:
	sr1=my1;
	sr=lshift sr1 by -1 (hi);
	ax1=sr1;
output_conversionL267_:
	ay1=4095;
	af=ax1-ay1;
	if not ac jump output_conversionL268_;
	if eq jump output_conversionL268_;
	ay1=127;
	ar=mr1 xor ay1;
	jump output_conversionL285_;
output_conversionL268_:
	sr1=ax1;
	sr=lshift sr1 by -4 (hi);
	ay1=0;
	mr0=sr1;
	sr=lshift mr0 by -1 (hi);
	mr0=sr1;
	af=pass mr0;
	if eq jump output_conversionL271_;
output_conversionL272_:
	sr=lshift mr0 by -1 (hi);
	ar=ay1+1,mr0=sr1;	!!PRLL-2 
	af=pass mr0,ay1=ar;	!!PRLL-2 
	if ne jump output_conversionL272_;
output_conversionL271_:
	sr1=ay1;
	sr=lshift sr1 by 4 (hi);
	af=pass ay1,mr0=sr1;	!!PRLL-2 
	if eq jump output_conversionL274_;
	ar= -ay1,sr1=ax1;	!!PRLL-2 
	ay1=ar;
	se=ay1; sr=lshift sr1 (hi);
	ay1=15;
	ax1=sr1;
	ar=ax1 and ay1;
	ay1=ar;
	ar=mr0 or ay1,ay0=mr1;	!!PRLL-2 
	jump output_conversionL286_;
output_conversionL274_:
	sr1=ax1;
	sr=lshift sr1 by -1 (hi);
	ay1=15;
	ax1=sr1;
	ar=ax1 and ay1;
	ay1=ar;
	ar=mr0 or ay1,ay0=mr1;	!!PRLL-2 
	jump output_conversionL286_;
output_conversionL263_:
	ax0=255;
	af=pass mr0;
	if eq jump output_conversionL277_;
	ax0=127;
output_conversionL277_:
	ay1=33;
	ar=my1;
	ar=ar+ay1;
	ay1=8191;
	mr0=ar;
	af=mr0-ay1;
	if not ac jump output_conversionL279_;
	if eq jump output_conversionL279_;
	ay1=127;
	ar=ax0 xor ay1;
	dm(S_)=ar;
	jump output_conversionL276_;
output_conversionL279_:
	sr=lshift mr0 by -5 (hi);
	ax1=5;
	af=pass sr1;
	if eq jump output_conversionL282_;
output_conversionL283_:
	ay0=ax1;
	ar=ay0+1;
	ax1=ar;
	ar= -ax1;
	ay1=ar;
	se=ay1; sr=lshift mr0 (hi);
	af=pass sr1;
	if ne jump output_conversionL283_;
output_conversionL282_:
	ay1=-6;
	ar=ax1+ay1;
	ax1=ar;
	ar=not ax1,sr1=ax1;	!!PRLL-2 
	ax1=ar;
	sr=lshift sr1 by 4 (hi);
	ay1=15;
	mr1=sr1;
	se=ax1; sr=lshift mr0 (hi);
	ax1=sr1;
	ar=ax1 and ay1;
	ay1=ar;
	ar=mr1 or ay1,ay0=ax0;	!!PRLL-2 
output_conversionL286_:
	ax1=ar;
	ar=ax1 xor ay0;
output_conversionL285_:
	ax1=ar;
	dm(S_)=ax1;
output_conversionL276_:
!	FUNCTION EPILOGUE: output_conversion
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax0=dm(i6,m5);
	ax1=dm(i6,m5);
	ay0=dm(i6,m5);
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: output_conversion
	jump (i6);

reset_states_:
!	FUNCTION PROLOGUE: reset_states
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax1;
	sr0=i0;
	dm(i4,m5)=sr0;
	sr0=i2;
	dm(i4,m5)=sr0;
	sr0=m0;
	dm(i4,m5)=sr0;
	sr0=m7;
	dm(i4,m5)=sr0;
!	END FUNCTION PROLOGUE: reset_states
	i1=dm(X_);
	ay1=0;
reset_statesL291_:
	cntr= 8; do reset_statesL297_-1 until ce; !! (depth 1)
	i2=i1;
	m0=ay1;
	ar=ay1+1;
	modify(i2,m0);
	ay1=ar;
	dm(i2,m2)=0;
	      !!  Loop Jump Was Here 
reset_statesL297_:
	i0=i1;
	m0=8;
	ay1=0;
	modify(i0,m0);
reset_statesL295_:
	cntr= 6; do reset_statesL296_-1 until ce; !! (depth 1)
	i2=i0;
	m0=ay1;
	ar=ay1+1;
	modify(i2,m0);
	ay1=ar;
	dm(i2,m2)=32;
	      !!  Loop Jump Was Here 
reset_statesL296_:
	i2=i1;
	i6=i1;
	m0=15;
	m7=14;
	modify(i2,m0);
	modify(i6,m7);
	dm(i2,m2)=32;
	i2=i1;
	dm(i6,m6)=32;
	m0=17;
	i6=i1;
	modify(i2,m0);
	m7=16;
	m0=18;
	modify(i6,m7);
	dm(i2,m2)=0;
	dm(i6,m6)=0;
	i6=i1;
	i0=i1;
	i2=i1;
	modify(i0,m0);
	m7=19;
	m0=20;
	modify(i6,m7);
	modify(i2,m0);
	dm(i2,m2)=0;
	i2=i1;
	dm(i6,m6)=0;
	m0=21;
	dm(i0,m2)=0;
	modify(i2,m0);
	i0=i1;
	m0=22;
	i6=i1;
	dm(i2,m2)=544;
	i2=i1;
	modify(i2,m0);
	m0=23;
	dm(i2,m2)=0;
	i2=i1;
	modify(i0,m0);
	m7=24;
	m0=25;
	modify(i6,m7);
	m5=-1;
	dm(i6,m6)=0;
	modify(i2,m0);
	dm(i2,m2)=-30720;
	ax1=dm(i6,m6);
	dm(i4,m5)=6;
	dm(i4,m5)=-30720;
	dm(i4,m5)=ax1;
	call __lshrdi3_;
	ax1=sr0;
	dm(i0,m2)=ax1;
!	FUNCTION EPILOGUE: reset_states
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax1=dm(i6,m5);
	ay1=dm(i6,m5);
	i0=ay1;
	ay1=dm(i6,m5);
	i2=ay1;
	ay1=dm(i6,m5);
	m0=ay1;
	ay1=dm(i6,m5);
	m7=ay1;
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: reset_states
	jump (i6);

scale_factor_1_:
!	FUNCTION PROLOGUE: scale_factor_1
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax0;
	dm(i4,m5)=ax1;
	dm(i4,m5)=ay0;
	dm(i4,m5)=mx0;
	dm(i4,m5)=mx1;
	dm(i4,m5)=my0;
	sr0=i0;
	dm(i4,m5)=sr0;
	sr0=i2;
	dm(i4,m5)=sr0;
	sr0=m0;
	dm(i4,m5)=sr0;
	sr0=m7;
	dm(i4,m5)=sr0;
!	END FUNCTION PROLOGUE: scale_factor_1
	i0=dm(X_);
	m7=23;
	i2=i0;
	m0=21;
	i6=i0;
	modify(i2,m0);
	modify(i6,m7);
	ax1=dm(i2,m2);
	ay1=dm(i6,m6);
	ar=ax1-ay1;
	ay1=8192;
	ax1=ar;
	ar=ax1 and ay1;
	ax0=ar;
	af=pass ax0;
	if eq jump scale_factor_1L299_;
	ar= -ax1;
	ay1=8191;
	ar=ar and ay1;
	ay1=ar;
	jump scale_factor_1L300_;
scale_factor_1L299_:
	ay1=ax1;
scale_factor_1L300_:
	mx1=ay1;
	my1=dm(AL_);
	  mr=mx1*my1 (ss); 
	  ay1=mr1;
	sr1=mx1;
	si=mr0;
	sr=lshift sr1 by -15 (hi);
	my0=mx1;
	mx0=sr1;
	i2=i0;
	sr1=my1;
	m5=-1;
	sr=lshift sr1 by -15 (hi);
	mr0=sr1;
	m0=23;
	mr=mr0*my0 (ss);
	dm(Register_Spill_Area__+12)=sr1;
	ax1=mr0;
	dm(i4,m5)=6;
	mr=mx0*my1 (ss),dm(i4,m5)=si;	!!PRLL-2 
	ar=ax1+ay1,sr0=mr0;	!!PRLL-2 
	ax1=ar;
	ay0=ax1;
	modify(i2,m0);
	ar=sr0+ay0;
	ax1=ar;
	dm(i4,m5)=ax1;
	call __lshrdi3_;
	ax1=dm(i2,m2);
	ay1=sr0;
	af=pass ax0,mx1=ay1;	!!PRLL-2 
	if eq jump scale_factor_1L301_;
	ar=ax1-ay1;
	ay1=8191;
	jump scale_factor_1L303_;
scale_factor_1L301_:
	ay0=mx1;
	ay1=8191;
	ar=ax1+ay0;
scale_factor_1L303_:
	ar=ar and ay1;
	dm(Y_)=ar;
!	FUNCTION EPILOGUE: scale_factor_1
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax0=dm(i6,m5);
	ax1=dm(i6,m5);
	ay0=dm(i6,m5);
	mx0=dm(i6,m5);
	mx1=dm(i6,m5);
	my0=dm(i6,m5);
	ay1=dm(i6,m5);
	i0=ay1;
	ay1=dm(i6,m5);
	i2=ay1;
	ay1=dm(i6,m5);
	m0=ay1;
	ay1=dm(i6,m5);
	m7=ay1;
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: scale_factor_1
	jump (i6);



.var	W_23_[8];
.init	W_23_: 4084,18,41,64,112,198,355,1122;
.external	__subdi3_;


scale_factor_2_:
!	FUNCTION PROLOGUE: scale_factor_2
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax1;
	dm(i4,m5)=ay0;
	dm(i4,m5)=mx0;
	dm(i4,m5)=mx1;
	sr0=i0;
	dm(i4,m5)=sr0;
	sr0=i2;
	dm(i4,m5)=sr0;
	sr0=i3;
	dm(i4,m5)=sr0;
	sr0=i5;
	dm(i4,m5)=sr0;
	sr0=i7;
	dm(i4,m5)=sr0;
	sr0=m0;
	dm(i4,m5)=sr0;
	sr0=m7;
	dm(i4,m5)=sr0;
!	END FUNCTION PROLOGUE: scale_factor_2
	mr0=dm(I_);
	i3=dm(X_);
	sr=lshift mr0 by -3 (hi);
	af=pass sr1;
	if eq jump scale_factor_2L305_;
	ax1=15;
	ay0=mr0;
	ay1=7;
	ar=ax1-ay0;
	ar=ar and ay1;
	jump scale_factor_2L314_;
scale_factor_2L305_:
	ay1=7;
	ar=mr0 and ay1;
scale_factor_2L314_:
	dm(Register_Spill_Area__+15)=ar;
	ay0=dm(Register_Spill_Area__+15);
	i2=^W_23_;
	m7=-1;
	m0=ay0;
	modify(i2,m0);
	ax1=dm(i2,m2);
	dm(i4,m7)=5;
	dm(i4,m7)=ax1;
	dm(i4,m7)=0;
	call __lshldi3_;
	m5=3;
	ax1=dm(Y_);
	modify(i4,m5);
	dm(i4,m7)=ax1;
	dm(i4,m7)=0;
	ay1=sr0;
	mx1=sr1;
	dm(i4,m7)=ay1;
	dm(i4,m7)=mx1;
	call __subdi3_;
	m5=4;
	modify(i4,m5);
	dm(i4,m7)=5;
	ax1=sr0;
	ay1=sr1;
	dm(i4,m7)=ax1;
	dm(i4,m7)=ay1;
	call __lshrdi3_;
	ay1=4095;
	ax1=sr0;
	ar=ax1 and ay1;
	ay1=2048;
	m5=3;
	mr0=ar;
	modify(i4,m5);
	ar=mr0 and ay1;
	ax1=ar;
	af=pass ax1;
	if eq jump scale_factor_2L307_;
	ay1=4096;
	ax1=dm(Y_);
	ar=ax1+ay1,ay0=mr0;	!!PRLL-2 
	ay1=8191;
	ar=ar+ay0;
	ar=ar and ay1;
	jump scale_factor_2L315_;
scale_factor_2L307_:
	ay1=dm(Y_);
	ay0=8191;
	ar=mr0+ay1;
	ar=ar and ay0;
scale_factor_2L315_:
	ax1=ar;
	i2=i3;
	m0=21;
	mx1=8192;
	ay1=15840;
	ar=ax1+ay1,ay0=mx1;	!!PRLL-2 
	modify(i2,m0);
	ar=ar and ay0;
	af=pass ar;
	if ne jump scale_factor_2L309_;
	ay1=11264;
	ar=ax1+ay1;
	ar=ar and ay0;
	af=pass ar;
	if eq jump scale_factor_2L311_;
	dm(i2,m2)=ax1;
	jump scale_factor_2L310_;
scale_factor_2L311_:
	dm(i2,m2)=5120;
	jump scale_factor_2L310_;
scale_factor_2L309_:
	dm(i2,m2)=544;
scale_factor_2L310_:
	i2=i3;
	m0=21;
	i0=i2;
	modify(i0,m0);
	m7=-1;
	m0=25;
	modify(i2,m0);
	m0=24;
	i7=i2;
	ay1=dm(i2,m2);
	i2=i3;
	modify(i2,m0);
	i5=i2;
	ax1=dm(i2,m2);
	dm(i4,m7)=ay1;
	dm(i4,m7)=ax1;
	call __negdi2_;
	m5=2;
	modify(i4,m5);
	dm(i4,m7)=6;
	ax1=sr0;
	ay1=sr1;
	dm(i4,m7)=ax1;
	dm(i4,m7)=ay1;
	call __lshrdi3_;
	ax1=dm(i0,m2);
	ay0=sr0;
	ay1=16383;
	ar=ax1+ay0;
	ax1=ar;
	my1=0;
	ar=ax1 and ay1;
	ay1=ar;
	mx1=ay1;
	ay1=8192;
	m5=3;
	ar=mx1;
	modify(i4,m5);
	ar=ar and ay1;
	ax1=ar;
	af=pass ax1;
	if eq jump scale_factor_2L313_;
	dm(i4,m7)=-16384;
	dm(i4,m7)=7;
	dm(i4,m7)=mx1;
	dm(i4,m7)=my1;
	call __adddi3_;
	ax1=sr1;
	my1=ax1;
	m5=4;
	ay1=sr0;
	modify(i4,m5);
	mx1=ay1;
scale_factor_2L313_:
	i2=i3;
	i0=i7;
	i2=i5;
	ay1=dm(i0,m2);
	ax1=dm(i2,m2);
	dm(i4,m7)=mx1;
	dm(i4,m7)=my1;
	dm(i4,m7)=ay1;
	dm(i4,m7)=ax1;
	call __adddi3_;
	ay1=7;
	m5=4;
	ax1=sr1;
	m0=23;
	mx1=sr0;
	modify(i4,m5);
	ar=ax1 and ay1,ay1=mx1;	!!PRLL-2 
	modify(i3,m0);
	dm(i2,m2)=ar;
	dm(i0,m2)=ay1;
	ax1=dm(i2,m2);
	dm(i4,m7)=6;
	dm(i4,m7)=ay1;
	dm(i4,m7)=ax1;
	call __lshrdi3_;
	ax1=sr0;
	dm(i3,m2)=ax1;
!	FUNCTION EPILOGUE: scale_factor_2
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax1=dm(i6,m5);
	ay0=dm(i6,m5);
	mx0=dm(i6,m5);
	mx1=dm(i6,m5);
	ay1=dm(i6,m5);
	i0=ay1;
	ay1=dm(i6,m5);
	i2=ay1;
	ay1=dm(i6,m5);
	i3=ay1;
	ay1=dm(i6,m5);
	i5=ay1;
	ay1=dm(i6,m5);
	i7=ay1;
	ay1=dm(i6,m5);
	m0=ay1;
	ay1=dm(i6,m5);
	m7=ay1;
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: scale_factor_2
	jump (i6);

speed_control_1_:
!	FUNCTION PROLOGUE: speed_control_1
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	sr0=i2;
	dm(i4,m5)=sr0;
	sr0=m0;
	dm(i4,m5)=sr0;
!	END FUNCTION PROLOGUE: speed_control_1
	m0=18;
	i2=dm(X_);
	ay1=255;
	modify(i2,m0);
	mr0=dm(i2,m2);
	af=mr0-ay1;
	if eq jump speed_control_1L319_;
	if ac jump speed_control_1L317_;
speed_control_1L319_:
	sr=lshift mr0 by -2 (hi);
	jump speed_control_1L318_;
speed_control_1L317_:
	sr1=64;
speed_control_1L318_:
	dm(AL_)=sr1;
!	FUNCTION EPILOGUE: speed_control_1
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ay1=dm(i6,m5);
	i2=ay1;
	ay1=dm(i6,m5);
	m0=ay1;
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: speed_control_1
	jump (i6);



.var	F_28_[8];
.init	F_28_: 0,0,0,512,512,512,1536,3584;


speed_control_2_:
!	FUNCTION PROLOGUE: speed_control_2
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax0;
	dm(i4,m5)=ax1;
	dm(i4,m5)=ay0;
	sr0=i2;
	dm(i4,m5)=sr0;
	sr0=m0;
	dm(i4,m5)=sr0;
!	END FUNCTION PROLOGUE: speed_control_2
	mr0=dm(I_);
	mr1=dm(X_);
	sr=lshift mr0 by -3 (hi);
	af=pass sr1;
	if eq jump speed_control_2L321_;
	ax1=15;
	ay0=mr0;
	ay1=7;
	ar=ax1-ay0;
	ar=ar and ay1;
	jump speed_control_2L322_;
speed_control_2L321_:
	ay1=7;
	ar=mr0 and ay1;
speed_control_2L322_:
	i2=^F_28_;
	m0=ar;
	modify(i2,m0);
	m0=19;
	mr0=dm(i2,m2);
	i2=mr1;
	modify(i2,m0);
	i6=i2;
	ax1=dm(i2,m2);
	ay0=ax1;
	ay1=8191;
	ar=mr0-ay0;
	ar=ar and ay1;
	ay1=128;
	sr=lshift ar by -5 (hi);
	ar=sr1 and ay1;
	af=pass ar;
	if eq jump speed_control_2L323_;
	ay1=3840;
	ar=ax1+ay1,ay0=sr1;	!!PRLL-2 
	ay1=4095;
	ar=ar+ay0;
	jump speed_control_2L336_;
speed_control_2L323_:
	ay0=ax1;
	ay1=4095;
	ar=sr1+ay0;
speed_control_2L336_:
	ar=ar and ay1;
	dm(i6,m6)=ar;
	m0=20;
	i2=mr1;
	sr=ashift mr0 by 2 (hi);
	modify(i2,m0);
	i6=i2;
	ax1=dm(i2,m2);
	ay0=ax1;
	ay1=32767;
	ar=sr1-ay0;
	ar=ar and ay1;
	ay1=128;
	sr=lshift ar by -7 (hi);
	ar=sr1 and ay1;
	af=pass ar;
	if eq jump speed_control_2L325_;
	ay1=16128;
	ar=ax1+ay1,ay0=sr1;	!!PRLL-2 
	ay1=16383;
	ar=ar+ay0;
	jump speed_control_2L337_;
speed_control_2L325_:
	ay0=ax1;
	ay1=16383;
	ar=sr1+ay0;
speed_control_2L337_:
	ar=ar and ay1;
	dm(i6,m6)=ar;
	i2=mr1;
	m0=19;
	modify(i2,m0);
	m0=20;
	mr0=dm(i2,m2);
	i2=mr1;
	modify(i2,m0);
	sr=ashift mr0 by 2 (hi);
	ay0=dm(i2,m2);
	ay1=32767;
	ar=sr1-ay0;
	ar=ar and ay1;
	ay1=16384;
	ar=ar and ay1,my1=ar;	!!PRLL-2 
	af=pass ar;
	if eq jump speed_control_2L327_;
	ax1=-32768;
	ay0=my1;
	ay1=16383;
	ar=ax1-ay0;
	ar=ar and ay1;
	jump speed_control_2L328_;
speed_control_2L327_:
	ar=my1;
speed_control_2L328_:
	ay1=1535;
	ax1=dm(Y_);
	af=ax1-ay1;
	if not ac jump speed_control_2L331_;
	if eq jump speed_control_2L331_;
	m0=20;
	i2=mr1;
	ax1=ar;
	modify(i2,m0);
	mr0=dm(i2,m2);
	sr=lshift mr0 by -3 (hi);
	ay1=sr1;
	af=ax1-ay1;
	if ac jump speed_control_2L331_;
	if eq jump speed_control_2L331_;
	ax1=dm(TDP_);
	af=pass ax1;
	if eq jump speed_control_2L329_;
speed_control_2L331_:
	ax1=512;
	jump speed_control_2L330_;
speed_control_2L329_:
	ax1=0;
speed_control_2L330_:
	i2=mr1;
	m0=18;
	modify(i2,m0);
	my1=dm(i2,m2);
	ay0=my1;
	ay1=2047;
	ar=ax1-ay0;
	ar=ar and ay1;
	ay1=64;
	sr=lshift ar by -4 (hi);
	ar=sr1 and ay1;
	af=pass ar;
	if eq jump speed_control_2L332_;
	ay1=896;
	ax0=my1;
	ar=ax0+ay1,ay0=sr1;	!!PRLL-2 
	ay1=1023;
	ar=ar+ay0;
	jump speed_control_2L338_;
speed_control_2L332_:
	ay0=my1;
	ay1=1023;
	ar=sr1+ay0;
speed_control_2L338_:
	ar=ar and ay1;
	m0=18;
	ax1=dm(TR_);
	i2=mr1;
	ay1=256;
	modify(i2,m0);
	af=pass ax1;
	if ne jump speed_control_2L334_;
	ay1=ar;
speed_control_2L334_:
	dm(i2,m2)=ay1;
!	FUNCTION EPILOGUE: speed_control_2
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax0=dm(i6,m5);
	ax1=dm(i6,m5);
	ay0=dm(i6,m5);
	ay1=dm(i6,m5);
	i2=ay1;
	ay1=dm(i6,m5);
	m0=ay1;
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: speed_control_2
	jump (i6);

tone_detector_1_:
!	FUNCTION PROLOGUE: tone_detector_1
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax1;
	dm(i4,m5)=ay0;
	sr0=i2;
	dm(i4,m5)=sr0;
	sr0=m0;
	dm(i4,m5)=sr0;
!	END FUNCTION PROLOGUE: tone_detector_1
	ay1=16383;
	ax1=dm(DQ_);
	i6=dm(X_);
	m0=23;
	i2=i6;
	modify(i2,m0);
	ar=ax1 and ay1,mr0=dm(i2,m2);	!!PRLL-2 
	ay1=31;
	mr1=ar;
	sr=lshift mr0 by -9 (hi);
	ax1=sr1;
	sr=lshift mr0 by -4 (hi);
	mr0=sr1;
	ar=mr0 and ay1;
	ay1=8;
	af=ax1-ay1,mr0=ar;	!!PRLL-2 
	if eq jump tone_detector_1L343_;
	if ac jump tone_detector_1L340_;
tone_detector_1L343_:
	ay1=32;
	ar=mr0+ay1;
	se=ax1; sr=lshift ar (hi);
	mr0=sr1;
	jump tone_detector_1L341_;
tone_detector_1L340_:
	mr0=15872;
tone_detector_1L341_:
	i2=i6;
	m0=22;
	my1=0;
	modify(i2,m0);
	ax1=dm(i2,m2);
	af=pass ax1;
	if eq jump tone_detector_1L342_;
	sr=lshift mr0 by -1 (hi);
	ay1=sr1;
	ar=mr0+ay1;
	sr=lshift ar by -1 (hi);
	ax1=sr1;
	ay0=ax1;
	af=mr1-ay0;
	if not ac jump tone_detector_1L342_;
	if eq jump tone_detector_1L342_;
	my1=1;
tone_detector_1L342_:
	dm(TR_)=my1;
!	FUNCTION EPILOGUE: tone_detector_1
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax1=dm(i6,m5);
	ay0=dm(i6,m5);
	ay1=dm(i6,m5);
	i2=ay1;
	ay1=dm(i6,m5);
	m0=ay1;
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: tone_detector_1
	jump (i6);

tone_detector_2_:
!	FUNCTION PROLOGUE: tone_detector_2
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax1;
	sr0=i2;
	dm(i4,m5)=sr0;
	sr0=m0;
	dm(i4,m5)=sr0;
!	END FUNCTION PROLOGUE: tone_detector_2
	ax1=dm(A2P_);
	ay1=-32768;
	my1=0;
	ar=ax1+ay1;
	ay1=20991;
	af=ar-ay1;
	if eq jump tone_detector_2L348_;
	if ac jump tone_detector_2L345_;
tone_detector_2L348_:
	my1=1;
tone_detector_2L345_:
	dm(TDP_)=my1;
	m0=22;
	ax1=dm(TR_);
	i2=dm(X_);
	ay1=0;
	modify(i2,m0);
	af=pass ax1;
	if ne jump tone_detector_2L346_;
	ay1=my1;
tone_detector_2L346_:
	dm(i2,m2)=ay1;
!	FUNCTION EPILOGUE: tone_detector_2
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax1=dm(i6,m5);
	ay1=dm(i6,m5);
	i2=ay1;
	ay1=dm(i6,m5);
	m0=ay1;
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: tone_detector_2
	jump (i6);



.entry	LAW_;
.var	LAW_;
.init	LAW_: 0;


.entry	reset_encoder_;
reset_encoder_:
!	FUNCTION PROLOGUE: reset_encoder
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax1;
!	END FUNCTION PROLOGUE: reset_encoder
	ax1=^E_STATES_;
	dm(X_)=ax1;
	call reset_states_;
!	FUNCTION EPILOGUE: reset_encoder
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax1=dm(i6,m5);
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: reset_encoder
	jump (i6);

.entry	encoder_;
encoder_:
!	FUNCTION PROLOGUE: encoder
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax1;
!	END FUNCTION PROLOGUE: encoder
	ax1=ar;
	ay1=^E_STATES_;
	dm(S_)=ax1;
	dm(X_)=ay1;
	call input_conversion_;
	call adpt_predict_1_;
	call diff_computation_;
	call speed_control_1_;
	call scale_factor_1_;
	call adapt_quant_;
	call iadpt_quant_;
	call tone_detector_1_;
	call adpt_predict_2_;
	call tone_detector_2_;
	call scale_factor_2_;
	call speed_control_2_;
	ar=dm(I_);
!	FUNCTION EPILOGUE: encoder
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax1=dm(i6,m5);
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: encoder
	jump (i6);

.entry	reset_decoder_;
reset_decoder_:
!	FUNCTION PROLOGUE: reset_decoder
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax1;
!	END FUNCTION PROLOGUE: reset_decoder
	ax1=^D_STATES_;
	dm(X_)=ax1;
	call reset_states_;
!	FUNCTION EPILOGUE: reset_decoder
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax1=dm(i6,m5);
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: reset_decoder
	jump (i6);

.entry	decoder_;
decoder_:
!	FUNCTION PROLOGUE: decoder
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax1;
!	END FUNCTION PROLOGUE: decoder
	ax1=ar;
	ay1=^D_STATES_;
	dm(I_)=ax1;
	dm(X_)=ay1;
	call speed_control_1_;
	call scale_factor_1_;
	call iadpt_quant_;
	call tone_detector_1_;
	call adpt_predict_1_;
	call adpt_predict_2_;
	call tone_detector_2_;
	call scale_factor_2_;
	call speed_control_2_;
	call output_conversion_;
	call input_conversion_;
	call diff_computation_;
	call coding_adjustment_;
	ar=dm(SD_);
!	FUNCTION EPILOGUE: decoder
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax1=dm(i6,m5);
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: decoder
	jump (i6);

.entry	main_;
main_:
!	FUNCTION PROLOGUE: main
	mr1=toppcstack;	! get return address
	si=m4;
	m4=i4;		! new frame ptr <= old stack ptr
	m5=-1;
	dm(i4,m5)=si;	! save old frame pointer
	dm(i4,m5)=mr1;	! save return address
!		saving registers: 
	dm(i4,m5)=ax0;
	dm(i4,m5)=ax1;
	dm(i4,m5)=ay0;
	sr0=i2;
	dm(i4,m5)=sr0;
	sr0=m0;
	dm(i4,m5)=sr0;
!	END FUNCTION PROLOGUE: main
	ay0=0;
	call reset_encoder_;
	m0=^Input_;
	call reset_decoder_;
	ax0=31;
mainL357_:
	i2=ay0;
	modify(i2,m0);
	ar=dm(i2,m2);
	call encoder_;
	ax1=ar;
	call decoder_;
	ar=ay0+1;
	ay0=ar;
	af=ay0-ax0;
	if not ac jump mainL357_;
	if eq jump mainL357_;
	ar=0;
!	FUNCTION EPILOGUE: main
	i6=m4;
	m5=-1;
	si=dm(i6,m5);	! old frame pointer
	mr1=dm(i6,m5);	! return address
!		restoring registers: 
	ax0=dm(i6,m5);
	ax1=dm(i6,m5);
	ay0=dm(i6,m5);
	ay1=dm(i6,m5);
	i2=ay1;
	ay1=dm(i6,m5);
	m0=ay1;
	i4=m4;	! reset stack pointer
	i6=mr1;
	m4=si;	! reset frame pointer
!	END FUNCTION EPILOGUE: main
	jump (i6);



.var	AL_;
.init	AL_: 0;
.var	A2P_;
.init	A2P_: 0;
.var	D_;
.init	D_: 0;
.var	DQ_;
.init	DQ_: 0;
.var	I_;
.init	I_: 0;
.var	SD_;
.init	SD_: 0;
.var	SE_;
.init	SE_: 0;
.var	SEZ_;
.init	SEZ_: 0;
.var	S_;
.init	S_: 0;
.var	SL_;
.init	SL_: 0;
.var	SR_;
.init	SR_: 0;
.var	TDP_;
.init	TDP_: 0;
.var	TR_;
.init	TR_: 0;
.var	Y_;
.init	Y_: 0;
.var	X_;
.init	X_: 0;
.var	E_STATES_[26];
.init	E_STATES_: 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0;
.var	D_STATES_[26];
.init	D_STATES_: 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0;
.var Register_Spill_Area__[18];
.ENDMOD;
