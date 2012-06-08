h08697
s 00050/00076/00733
d D 1.14 94/11/23 12:49:25 schlaege 14 13
c Final NEC made version (contains long word bug!)
e
s 00085/00105/00724
d D 1.13 94/11/03 14:22:23 schlaege 13 12
c NEC 3-Nov-94
e
s 00606/00643/00223
d D 1.12 94/10/20 10:18:58 schlaege 12 11
c tested NEC version
e
s 00008/00008/00858
d D 1.11 94/10/19 15:17:32 schlaege 11 10
c pre NEC version
e
s 00000/00000/00866
d D 1.10 94/05/25 13:58:33 schlaege 10 9
c the real sa1 version
e
s 00014/00009/00852
d D 1.9 94/05/05 11:38:26 schlaege 9 8
c version used for sa
e
s 00019/00021/00842
d D 1.8 94/04/20 14:44:39 schlaege 8 7
c preliminary final version
e
s 00111/00108/00752
d D 1.7 94/04/19 19:15:20 schlaege 7 6
c f_mult inlined
e
s 00083/00098/00777
d D 1.6 94/04/19 15:21:40 schlaege 6 5
c SIGNBIT partly replaced by NEGATIVE
e
s 00056/00024/00819
d D 1.5 94/04/15 08:13:04 schlaege 5 4
c further optimized
e
s 00025/00033/00818
d D 1.4 94/04/12 08:14:05 schlaege 4 3
c further optimized
e
s 00130/00123/00721
d D 1.3 94/04/11 11:32:41 schlaege 3 2
c parameters replaced with global vars
e
s 00194/00270/00650
d D 1.2 94/03/31 14:24:46 schlaege 2 1
c Passed simple tests on 56k, C50, and 2101
e
s 00920/00000/00000
d D 1.1 94/03/24 11:29:48 dsp 1 0
c date and time created 94/03/24 11:29:48 by dsp
e
u
U
f e 0
t
T
I 1
/*
 * g721.c
D 13
 * ADPCM transcoder applying the CCITT recommendation G.721
E 13
I 13
 * ADPCM transcoder applying the ITU-T recommendation G.721 (now G.726/32)
E 13
 *
D 9
 * It has been successfully tested with all 32kBits reset test sequences.
D 6
 * The tests were made on a Sparc Station 10 with various compilers.
E 6
I 6
 * The tests were made on a SparcStation 10 with various compilers.
E 9
I 9
D 13
 * It has been successfully tested with all 32 kbits reset test sequences.
 * The tests were made on a SPARCstation10 with various compilers.
E 13
I 13
 * It has been successfully tested with all 32 kbit/s reset test sequences.
 * The tests were made on a SPARCstation, an ATARI ST and a IBM-PC with
 * various compilers.
E 13
E 9
E 6
 *
D 9
 * This source code is released under a DISCLAIMER OF WARRANTY by the
E 9
I 9
 * This source code is released under a DISCLAIMER OF WARRANTY and a
D 13
 * Copyright (c) 1994 by the
E 13
I 13
 * Copyright (c) 1994 by
E 13
E 9
 *
D 9
 * Institute for Integrated Systems in Signal Processing
E 9
I 9
D 11
 * Institute for Integrated Systems in Signal Processing, IS2
E 11
I 11
D 12
 * Laboratory for Integrated Systems in Signal Processing, ISS
E 12
I 12
D 13
 * Institute for Integrated Systems in Signal Processing, IS2
E 12
E 11
 * DSP Tools Group
E 9
 * Aachen University of Technology
E 13
I 13
 *     Christian Schlaeger (email: schlaege@ert.rwth-aachen.de)
D 14
 *     Lab. for Integrated Systems in Signal Processing, ISS
E 14
I 14
 *     Institute for Integrated Systems in Signal Processing, ISS
E 14
 *     DSP Tools Group
 *     Aachen University of Technology (RWTH)
 *     ISS-611810 Templergraben 55
 *     D-52056 Aachen, Germany
E 13
 *
I 9
 * IT MAY BE FREELY DUPLICATED BUT ANY COMMERCIAL USE WITHOUT PRIOR WRITTEN
D 11
 * PERMIT OF THE IS2 IS ILLEGAL!
E 11
I 11
D 12
 * PERMIT OF THE ISS IS ILLEGAL!
E 12
I 12
D 13
 * PERMIT OF THE IS2 IS ILLEGAL!
E 13
I 13
 * PERMIT OF THE AUTHOR IS ILLEGAL!
E 13
E 12
E 11
 *
I 13
 * To increase performance the signed magnitude representation of the
 * data was changed to two's complement representation where possible.
 * Modules containing delays where splitted into a pre-state part (*_2) and
 * a post-state part (*_1) to get a consistent data flow.
E 13
E 9
 * For further documentation of the source consult the recommendation.
 *
D 9
 * The implementation was done by Juan Martinez Velarde and Chris Schlaeger
D 5
 * Last modification 24-Mar-94
E 5
I 5
D 6
 * Last modification 14-Apr-94
E 6
I 6
 * Last modification 19-Apr-94
E 9
I 9
D 13
 * The implementation was done by Chris Schlaeger. Thanks to Juan Martinez
 * Velarde for his ideas and help during the optimization.
D 11
 * Last modification 25-Apr-94
E 11
I 11
D 12
 * Last modification 10-Jun-94
E 12
I 12
 *
 * Last modification: 20-Oct-94 Chris Schlaeger
E 13
I 13
D 14
 * Last modification: 3-Nov-94 Chris Schlaeger
E 14
I 14
 * Last modification: 23-Nov-94 Chris Schlaeger
E 14
E 13
E 12
E 11
E 9
E 6
E 5
 */

#include "g721.h"
I 12
D 14
/*#include "log.h"
#include "dump.h"*/
E 14
E 12

#define FALSE 0
#define TRUE (!FALSE)

I 12
/*
 * The format for storing DQn and SRn has been modified. The mantissa,
 * exponent and sign are stored in three consecutive U16BIT. This allows
 * are more efficient handling of this data.
 * DQ[0]: mantissa of DQ1, DQ[1]: exponent of DQ1, DQ[2]: sign of DQ1
 * DQ[18]: mantissa of SR1, ...
 */

E 12
typedef struct
{
D 7
  U16BIT B[6], DQ[6], PK1, PK2, SR1, SR2, A1, A2;
  U16BIT AP, DMS, DML;
E 7
I 7
D 12
  U16BIT B[8], DQ[8], PK1, PK2;  /* A1 = B[6], A2 = B[7] */
  U16BIT AP, DMS, DML;           /* SR1 = DQ[6], SR2 = DQ[7] */
E 7
  U16BIT YU;
  U16BIT TD;
I 3
  U16BIT YL6;  /* YL >> 6 */
E 3
  U24BIT YL;
E 12
I 12
	S16BIT B[8];                        /* A1 = B[6], A2 = B[7] */
	U16BIT DQ[3 * 8], PK1, PK2;
	S16BIT AP;                          /* SR1 = DQ[18], SR2 = DQ[21] */
	S16BIT DMS, DML;
	U16BIT YU;
	U16BIT TD;
	U16BIT YL6;  /* YL >> 6 */
	U24BIT YL;
E 12
} STATES;

/*
 * Prototypes
 */

static void adapt_quant(void);
D 3
static void adpt_predict_1(STATES *S);
static void adpt_predict_2(STATES *S);
E 3
I 3
static void adpt_predict_1(void);
static void adpt_predict_2(void);
E 3
static void coding_adjustment(void);
static void diff_computation(void);
D 9
static U16BIT f_mult(U16BIT An, U16BIT SRn);
E 9
static void iadpt_quant(void);
static void input_conversion(void);
static void output_conversion(void);
D 3
static void scale_factor_1(STATES* S);
static void scale_factor_2(STATES* S);
static void speed_control_1(STATES* S);
static void speed_control_2(STATES* S);
static void tone_detector_1(STATES* S);
static void tone_detector_2(STATES* S);
E 3
I 3
static void reset_states(void);
static void scale_factor_1(void);
static void scale_factor_2(void);
static void speed_control_1(void);
static void speed_control_2(void);
static void tone_detector_1(void);
static void tone_detector_2(void);
E 3

/* global signals */
D 7
static U16BIT AL, A2P, D, DQ, I, SD, SE, SEZ, S, SL, SR, TDP, TR, Y;
E 7
I 7
D 8
static U16BIT AL, A2P, DQ, D, I, SD, SE, SEZ, S, SL, SR, TDP, TR, Y;
E 8
I 8
D 12
static U16BIT AL, A2P, D, DQ, I, SD, SE, SEZ, S, SL, SR, TDP, TR, Y;
E 12
I 12
static U16BIT AL, DQ, SD, S, TDP, TR, Y;
static S16BIT A2P, D, I, SE, SEZ, SL, SR;
E 12
E 8
E 7
I 3
static STATES* X;
E 3

/* ENCODER states */
static STATES E_STATES;
D 2
/*{
  0,0,0,0,0,0,32,32,32,32,32,32,0,0,32,32,0,0,
  0,0,0,
  544,0,(U24BIT) 34816L
}; */
E 2

/* DECODER states */
static STATES D_STATES;
D 2
/*
{
  0,0,0,0,0,0,32,32,32,32,32,32,0,0,32,32,0,0,
  0,0,0,
  544,0,(U24BIT) 34816L
}; */
E 2

D 11
static U16BIT A_LAW_table[] =
E 11
I 11
D 12
static const U16BIT A_LAW_table[] =
E 12
I 12
static U16BIT A_LAW_table[] =
E 12
E 11
{
D 12
   4784,  4752,  4848,  4816,  4656,  4624,  4720,  4688, 
   5040,  5008,  5104,  5072,  4912,  4880,  4976,  4944, 
   4440,  4424,  4472,  4456,  4376,  4360,  4408,  4392, 
   4568,  4552,  4600,  4584,  4504,  4488,  4536,  4520, 
   6848,  6720,  7104,  6976,  6336,  6208,  6592,  6464, 
   7872,  7744,  8128,  8000,  7360,  7232,  7616,  7488, 
   5472,  5408,  5600,  5536,  5216,  5152,  5344,  5280, 
   5984,  5920,  6112,  6048,  5728,  5664,  5856,  5792, 
   4139,  4137,  4143,  4141,  4131,  4129,  4135,  4133, 
   4155,  4153,  4159,  4157,  4147,  4145,  4151,  4149, 
   4107,  4105,  4111,  4109,  4099,  4097,  4103,  4101, 
   4123,  4121,  4127,  4125,  4115,  4113,  4119,  4117, 
   4268,  4260,  4284,  4276,  4236,  4228,  4252,  4244, 
   4332,  4324,  4348,  4340,  4300,  4292,  4316,  4308, 
   4182,  4178,  4190,  4186,  4166,  4162,  4174,  4170, 
   4214,  4210,  4222,  4218,  4198,  4194,  4206,  4202, 
    688,   656,   752,   720,   560,   528,   624,   592, 
    944,   912,  1008,   976,   816,   784,   880,   848, 
    344,   328,   376,   360,   280,   264,   312,   296, 
    472,   456,   504,   488,   408,   392,   440,   424, 
   2752,  2624,  3008,  2880,  2240,  2112,  2496,  2368, 
   3776,  3648,  4032,  3904,  3264,  3136,  3520,  3392, 
   1376,  1312,  1504,  1440,  1120,  1056,  1248,  1184, 
   1888,  1824,  2016,  1952,  1632,  1568,  1760,  1696, 
     43,    41,    47,    45,    35,    33,    39,    37, 
     59,    57,    63,    61,    51,    49,    55,    53, 
     11,     9,    15,    13,     3,     1,     7,     5, 
     27,    25,    31,    29,    19,    17,    23,    21, 
    172,   164,   188,   180,   140,   132,   156,   148, 
    236,   228,   252,   244,   204,   196,   220,   212, 
     86,    82,    94,    90,    70,    66,    78,    74, 
    118,   114,   126,   122,   102,    98,   110,   106
E 12
I 12
	4784,  4752,  4848,  4816,  4656,  4624,  4720,  4688,
	5040,  5008,  5104,  5072,  4912,  4880,  4976,  4944,
	4440,  4424,  4472,  4456,  4376,  4360,  4408,  4392,
	4568,  4552,  4600,  4584,  4504,  4488,  4536,  4520,
	6848,  6720,  7104,  6976,  6336,  6208,  6592,  6464,
	7872,  7744,  8128,  8000,  7360,  7232,  7616,  7488,
	5472,  5408,  5600,  5536,  5216,  5152,  5344,  5280,
	5984,  5920,  6112,  6048,  5728,  5664,  5856,  5792,
	4139,  4137,  4143,  4141,  4131,  4129,  4135,  4133,
	4155,  4153,  4159,  4157,  4147,  4145,  4151,  4149,
	4107,  4105,  4111,  4109,  4099,  4097,  4103,  4101,
	4123,  4121,  4127,  4125,  4115,  4113,  4119,  4117,
	4268,  4260,  4284,  4276,  4236,  4228,  4252,  4244,
	4332,  4324,  4348,  4340,  4300,  4292,  4316,  4308,
	4182,  4178,  4190,  4186,  4166,  4162,  4174,  4170,
	4214,  4210,  4222,  4218,  4198,  4194,  4206,  4202,
	 688,   656,   752,   720,   560,   528,   624,   592,
	 944,   912,  1008,   976,   816,   784,   880,   848,
	 344,   328,   376,   360,   280,   264,   312,   296,
	 472,   456,   504,   488,   408,   392,   440,   424,
	2752,  2624,  3008,  2880,  2240,  2112,  2496,  2368,
	3776,  3648,  4032,  3904,  3264,  3136,  3520,  3392,
	1376,  1312,  1504,  1440,  1120,  1056,  1248,  1184,
	1888,  1824,  2016,  1952,  1632,  1568,  1760,  1696,
	  43,    41,    47,    45,    35,    33,    39,    37,
	  59,    57,    63,    61,    51,    49,    55,    53,
	  11,     9,    15,    13,     3,     1,     7,     5,
	  27,    25,    31,    29,    19,    17,    23,    21,
	 172,   164,   188,   180,   140,   132,   156,   148,
	 236,   228,   252,   244,   204,   196,   220,   212,
	  86,    82,    94,    90,    70,    66,    78,    74,
	 118,   114,   126,   122,   102,    98,   110,   106
E 12
} ;

D 11
static U16BIT u_LAW_table[] =
E 11
I 11
D 12
static const U16BIT u_LAW_table[] =
E 12
I 12
static U16BIT u_LAW_table[] =
E 12
E 11
{
D 12
  16223, 15967, 15711, 15455, 15199, 14943, 14687, 14431, 
  14175, 13919, 13663, 13407, 13151, 12895, 12639, 12383, 
  12191, 12063, 11935, 11807, 11679, 11551, 11423, 11295, 
  11167, 11039, 10911, 10783, 10655, 10527, 10399, 10271, 
  10175, 10111, 10047,  9983,  9919,  9855,  9791,  9727, 
   9663,  9599,  9535,  9471,  9407,  9343,  9279,  9215, 
   9167,  9135,  9103,  9071,  9039,  9007,  8975,  8943, 
   8911,  8879,  8847,  8815,  8783,  8751,  8719,  8687, 
   8663,  8647,  8631,  8615,  8599,  8583,  8567,  8551, 
   8535,  8519,  8503,  8487,  8471,  8455,  8439,  8423, 
   8411,  8403,  8395,  8387,  8379,  8371,  8363,  8355, 
   8347,  8339,  8331,  8323,  8315,  8307,  8299,  8291, 
   8285,  8281,  8277,  8273,  8269,  8265,  8261,  8257, 
   8253,  8249,  8245,  8241,  8237,  8233,  8229,  8225, 
   8222,  8220,  8218,  8216,  8214,  8212,  8210,  8208, 
   8206,  8204,  8202,  8200,  8198,  8196,  8194,     0, 
   8031,  7775,  7519,  7263,  7007,  6751,  6495,  6239, 
   5983,  5727,  5471,  5215,  4959,  4703,  4447,  4191, 
   3999,  3871,  3743,  3615,  3487,  3359,  3231,  3103, 
   2975,  2847,  2719,  2591,  2463,  2335,  2207,  2079, 
   1983,  1919,  1855,  1791,  1727,  1663,  1599,  1535, 
   1471,  1407,  1343,  1279,  1215,  1151,  1087,  1023, 
    975,   943,   911,   879,   847,   815,   783,   751, 
    719,   687,   655,   623,   591,   559,   527,   495, 
    471,   455,   439,   423,   407,   391,   375,   359, 
    343,   327,   311,   295,   279,   263,   247,   231, 
    219,   211,   203,   195,   187,   179,   171,   163, 
    155,   147,   139,   131,   123,   115,   107,    99, 
     93,    89,    85,    81,    77,    73,    69,    65, 
     61,    57,    53,    49,    45,    41,    37,    33, 
     30,    28,    26,    24,    22,    20,    18,    16, 
     14,    12,    10,     8,     6,     4,     2,     0
E 12
I 12
	16223, 15967, 15711, 15455, 15199, 14943, 14687, 14431,
	14175, 13919, 13663, 13407, 13151, 12895, 12639, 12383,
	12191, 12063, 11935, 11807, 11679, 11551, 11423, 11295,
	11167, 11039, 10911, 10783, 10655, 10527, 10399, 10271,
	10175, 10111, 10047,  9983,  9919,  9855,  9791,  9727,
	 9663,  9599,  9535,  9471,  9407,  9343,  9279,  9215,
	 9167,  9135,  9103,  9071,  9039,  9007,  8975,  8943,
	 8911,  8879,  8847,  8815,  8783,  8751,  8719,  8687,
	 8663,  8647,  8631,  8615,  8599,  8583,  8567,  8551,
	 8535,  8519,  8503,  8487,  8471,  8455,  8439,  8423,
	 8411,  8403,  8395,  8387,  8379,  8371,  8363,  8355,
	 8347,  8339,  8331,  8323,  8315,  8307,  8299,  8291,
	 8285,  8281,  8277,  8273,  8269,  8265,  8261,  8257,
	 8253,  8249,  8245,  8241,  8237,  8233,  8229,  8225,
	 8222,  8220,  8218,  8216,  8214,  8212,  8210,  8208,
	 8206,  8204,  8202,  8200,  8198,  8196,  8194,     0,
	 8031,  7775,  7519,  7263,  7007,  6751,  6495,  6239,
	 5983,  5727,  5471,  5215,  4959,  4703,  4447,  4191,
	 3999,  3871,  3743,  3615,  3487,  3359,  3231,  3103,
	 2975,  2847,  2719,  2591,  2463,  2335,  2207,  2079,
	 1983,  1919,  1855,  1791,  1727,  1663,  1599,  1535,
	 1471,  1407,  1343,  1279,  1215,  1151,  1087,  1023,
	  975,   943,   911,   879,   847,   815,   783,   751,
	  719,   687,   655,   623,   591,   559,   527,   495,
	  471,   455,   439,   423,   407,   391,   375,   359,
	  343,   327,   311,   295,   279,   263,   247,   231,
	  219,   211,   203,   195,   187,   179,   171,   163,
	  155,   147,   139,   131,   123,   115,   107,    99,
		93,    89,    85,    81,    77,    73,    69,    65,
		61,    57,    53,    49,    45,    41,    37,    33,
		30,    28,    26,    24,    22,    20,    18,    16,
		14,    12,    10,     8,     6,     4,     2,     0
E 12
} ;

I 2
/*
 * "inline Functions"
 */
E 2
D 6
#define LSHIFT(a, b)  ((b) < 0 ? (a) << -(b) : (a) >> (b))
#define SIGNBIT(a, b) ((a) & (((U16BIT) 1) << (b)) ? 1 : 0)
E 6
I 6
D 12
#define LSHIFT(a, b)   ((b) < 0 ? (a) << -(b) : (a) >> (b))
E 12
I 12
D 13
#define ABS(a)         ((a) < 0 ? (-(a)) : (a))
#define RSHIFT(a, b)   ((b) < 0 ? (a) << -(b) : (a) >> (b))
E 12
#define SIGNBIT(a, b)  ((a) & (((U16BIT) 1) << (b)) ? 1 : 0)
#define NEGATIVE(a, b) ((a) & (((U16BIT) 1) << (b)))
E 6
D 2
#define MSB(a, b)     { register U16BIT tmp = (U16BIT) (a); \
			(b) = 0; while(tmp) {tmp >>= 1; (b)++; }}
E 2
I 2
#define MSB0(a, b)     { register U16BIT tmp = (U16BIT) (a); \
D 6
			 (b) = 0; while(tmp) {tmp >>= 1; (b)++; }}
E 6
I 6
D 12
                        (b) = 0; while(tmp) {tmp >>= 1; (b)++; }}
E 12
I 12
								(b) = 0; while(tmp) {tmp >>= 1; (b)++; }}
E 12
E 6
#define MSB1(a, b)     { register U16BIT tmp = (U16BIT) ((a) >> 1); \
D 12
                         (b) = 0; while(tmp) {tmp >>= 1; (b)++; }}
E 12
I 12
								 (b) = 0; while(tmp) {tmp >>= 1; (b)++; }}
E 13
I 13
D 14
#define ABS(a)          ((a) < 0 ? (-(a)) : (a))
#define ADJUST(a, b, c) { U16BIT d = ((a) + (c)) & 0xFF; \
								  S16BIT e = d & 0x80 ? (-1 << 8) | d : d; \
								  b = e > 0 ? b - 1 : e < 0 ? b + 1 : \
										e ? 126 + (c) : (c); }
#define RSHIFT(a, b)    ((b) < 0 ? (a) << -(b) : (a) >> (b))
#define SIGNBIT(a, b)   ((a) & (((U16BIT) 1) << (b)) ? 1 : 0)
#define NEGATIVE(a, b)  ((a) & (((U16BIT) 1) << (b)))
#define MSB0(a, b)      { register U16BIT tmp = (U16BIT) (a); \
								  (b) = 0; while(tmp) {tmp >>= 1; (b)++; }}
#define MSB1(a, b)      { register U16BIT tmp = (U16BIT) ((a) >> 1); \
								  (b) = 0; while(tmp) {tmp >>= 1; (b)++; }}
E 14
I 14
#define ABS(a)           ((a) < 0 ? (-(a)) : (a))
#define ADJUST1(a, b)    { U16BIT c = ((a) + (b)) & 0xFF; \
									S16BIT d = c & 0x80 ? (-1 << 8) | c : c; \
									a = d > 0 ? a - 1 : d < -1 ? a + 1 : \
										 (d ? 255 + (b) : 128 + (b)) & 0xFF; }
#define ADJUST2(a, b)    { U16BIT c = ((a) + (b)) & 0xFF; \
								  S16BIT d = c & 0x80 ? (-1 << 8) | c : c; \
								  a = d > 0 ? a - 1 : d < -1 ? a + 1 : \
										d ? 126 + (b) : (b); }
#define RSHIFT(a, b)     ((b) < 0 ? (a) << -(b) : (a) >> (b))
#define SIGNBIT(a, b)    ((a) & (((U16BIT) 1) << (b)) ? 1 : 0)
#define NEGATIVE(a, b)   ((a) & (((U16BIT) 1) << (b)))
#define MSB0(a, b)       { register U16BIT tmp = (U16BIT) (a); \
									(b) = 0; while(tmp) {tmp >>= 1; (b)++; }}
#define MSB1(a, b)       { register U16BIT tmp = (U16BIT) ((a) >> 1); \
									(b) = 0; while(tmp) {tmp >>= 1; (b)++; }}
E 14
E 13
E 12
D 6
#define TC(a)         (~(a) + 1)
E 6
I 6
D 7
#define TC(a)          (~(a) + 1)
E 7
E 6
E 2

static void adapt_quant()
{
D 12
  /*
   * adaptive quantizer
   *
   * Input signals:  D, Y
   * Output signals: I
   */
D 2
  U16BIT DS, DL, DLN;
E 2
I 2
D 6
  register U16BIT DLN;
  U16BIT          DS, DL;
E 6
I 6
  register U16BIT DLN, DS;
  U16BIT          DL;
E 12
I 12
	/*
	 * adaptive quantizer
	 *
	 * Input signals:  D, Y
	 * Output signals: I
	 */
I 13
	 static S16BIT TAB[] = { 400, 349, 300, 246, 178, 80, -124 };
	 S16BIT* pt = TAB;
	 int i;
E 13
	 register S16BIT DLN;
D 13
	 U16BIT          DL;
E 13
E 12
E 6
E 2

D 12
  /* LOG */
  {
D 2
    U16BIT DQM, EXP, MANT;
E 2
I 2
    U16BIT DQM;
    S16BIT EXP;
E 12
I 12
D 13
	/* LOG */
E 13
I 13
	/* LOG and SUBTB */
E 13
	{
		U16BIT DQM;
		S16BIT EXP;
E 12
E 2

D 2
    DS = D >> 15;
    DQM = IF_ELSE(DS, ((U16BIT) (((U24BIT) 65536L) - D)) & 32767, D);
    for (EXP = 0; DQM >> (EXP + 1); EXP++)
      ;
    MANT = ((DQM << 7) >> EXP) & 127;
    DL = (EXP << 7) + MANT;
E 2
I 2
D 6
    DS = SIGNBIT(D, 15);
E 6
I 6
D 12
    DS = NEGATIVE(D, 15);
E 6
D 7
    DQM = IF_ELSE(DS, TC(D) & 32767, D);
E 7
I 7
    DQM = IF_ELSE(DS, (-D) & 32767, D);
E 7
    MSB1(DQM, EXP);
    DL = (EXP << 7) + ((LSHIFT(DQM, EXP - 7)) & 127);
E 2
  }
    	
  /* SUBTB */
D 2
  DLN = (DL + 4096 - (Y >> 2)) & 4095;
E 2
I 2
D 7
  DLN = (DL + TC(Y >> 2)) & 4095;
E 7
I 7
  DLN = (DL - (Y >> 2)) & 4095;
E 7
E 2
  
  /* QUAN */
D 2
  if (DLN > 3971)
    I = DS ? 0xE : 0x1;
  else if (DLN > 2047)
    I = 0xF;
  else if (DLN > 399)
    I = DS ? 0x8 : 0x7;
  else if (DLN > 348)
    I = DS ? 0x9 : 0x6;
  else if (DLN > 299)
    I = DS ? 0xA : 0x5;     
  else if (DLN > 245)
    I = DS ? 0xB : 0x4;
  else if (DLN > 177)
    I = DS ? 0xC : 0x3;
  else if (DLN > 79)
    I = DS ? 0xD : 0x2;
E 2
I 2
  if (DLN > 299)
    if (DLN > 2047)
      if (DLN > 3971)
D 6
	I = DS ? 0xE : 0x1;
E 6
I 6
        I = DS ? 0xE : 0x1;
E 6
      else
D 6
	I = 0xF;
E 6
I 6
        I = 0xF;
E 6
    else
      if (DLN > 399)
D 6
	I = DS ? 0x8 : 0x7;
E 6
I 6
        I = DS ? 0x8 : 0x7;
E 6
      else
	if (DLN > 348)
D 6
	  I = DS ? 0x9 : 0x6;
E 6
I 6
          I = DS ? 0x9 : 0x6;
E 6
        else
	  I = DS ? 0xA : 0x5;
E 2
  else
D 2
    I = DS ? 0xE : 0x1;
E 2
I 2
    if (DLN > 177)
      if (DLN > 245)
D 6
	I = DS ? 0xB : 0x4;
E 6
I 6
        I = DS ? 0xB : 0x4;
E 6
      else
D 6
	I = DS ? 0xC : 0x3;
E 6
I 6
        I = DS ? 0xC : 0x3;
E 6
    else
      if (DLN > 79)
D 6
	I = DS ? 0xD : 0x2;
E 6
I 6
        I = DS ? 0xD : 0x2;
E 6
      else
D 6
	I = DS ? 0xE : 0x1;
E 6
I 6
        I = DS ? 0xE : 0x1;
E 12
I 12
		DQM = ABS(D);
		MSB1(DQM, EXP);
D 13
		DL = (EXP << 7) + ((RSHIFT(DQM, EXP - 7)) & 127);
E 13
I 13
		DLN = (EXP << 7) + ((RSHIFT(DQM, EXP - 7)) & 127) - (Y >> 2);
E 13
	}

D 13
	/* SUBTB */
	DLN = DL - (Y >> 2);

E 13
	/* QUAN */
	I = 7;
D 13
	if (DLN < 400)    I--;
	if (DLN < 349)    I--;
	if (DLN < 300)    I--;
	if (DLN < 246)    I--;
	if (DLN < 178)    I--;
	if (DLN < 80)     I--;
	if (DLN < -124)   I--;
E 13
I 13
	for (i = 0; i < 7; i++)
		if (DLN < *pt++)
			I--;
E 13
	if (D < 0)        I = ~I;
	if (I == 0)       I = ~I;
E 12
E 6
E 2
}

D 3
static void adpt_predict_1(STATES *S)
E 3
I 3
static void adpt_predict_1(void)
E 3
{
D 12
  /*
   * adptive predictor
   *
   * Input signals:  none
   * Output signals: SE, SEZ
D 7
   * States:         B[], DQ[], A1, A1
E 7
I 7
   * States:         B[], DQ[], B[6], B[6]
E 7
   */
E 12
I 12
	/*
D 13
	 * adptive predictor
E 13
I 13
	 * post-state part of the adptive predictor
E 13
	 *
D 14
	 * Input signals:  none
	 * Output signals: SE, SEZ
	 * States:         B[], DQ[], B[6], B[6]
E 14
I 14
	 * Input signals:   X->DQn, X->SRn, X->An, X->Bn
	 * Output signals:  SE, SEZ
E 14
	 */
E 12

I 3
D 12
  register STATES* x = X;
E 3
D 2
  int i;
  U16BIT SEZI, SEI;
E 2
I 2
  register int    i;
D 9
  register U16BIT SEZI, SEI;
E 9
I 9
  register U16BIT SEZI;
E 12
I 12
	register STATES* x = X;
	register int    i;
	register S16BIT SEZI;
E 12
E 9
E 2

D 12
  /* FMULT and ACCUM */
  SEZI = 0;
D 5
  for (i = 0; i < 6; i++)
D 2
    SEZI = (U16BIT) ((SEZI + f_mult(S->B[i], S->DQ[i])) & ((U16BIT) 65535L));
  SEI = (SEZI + f_mult(S->A1, S->SR1) + f_mult(S->A2, S->SR2))
E 2
I 2
D 3
    SEZI += f_mult(S->B[i], S->DQ[i]);
  SEI = ((SEZI &= (U16BIT) 65535L) + f_mult(S->A1, S->SR1)
	  + f_mult(S->A2, S->SR2))
E 3
I 3
    SEZI += f_mult(x->B[i], x->DQ[i]);
E 5
I 5
  {
    register U16BIT* tmp1 = x->B;
    register U16BIT* tmp2 = x->DQ;
E 12
I 12
	/* FMULT and ACCUM */
	SEZI = 0;
	{
		register S16BIT* tmp1 = x->B;
		register U16BIT* tmp2 = x->DQ;
E 12

D 7
    for (i = 0; i < 6; i++)
      SEZI += f_mult(*tmp1++, *tmp2++);
E 7
I 7
D 12
    for (i = 0; i < 8; i++)
    {
      {
	register S16BIT AnEXP, WAnEXP;
	register U16BIT AnMAG;
	U16BIT AnS, AnMANT, WAnMAG, WAnMANT;
	
	/* FMULT */
	AnMAG = (AnS = SIGNBIT(*tmp1, 15)) ?
D 8
	         (-(*tmp1 >> 2)) & 8191 : *tmp1 >> 2;
	tmp1++;
E 8
I 8
	         (-(*tmp1++ >> 2)) & 8191 : *tmp1++ >> 2;
E 8
	MSB0(AnMAG, AnEXP);
	AnMANT = AnMAG ? LSHIFT(AnMAG, AnEXP - 6) : 1 << 5;
	WAnMANT = (((*tmp2 & 63) * AnMANT) + 48) >> 4;
	WAnEXP = ((*tmp2 >> 6) & 15) + AnEXP;
	WAnMAG = LSHIFT((U24BIT) WAnMANT, 19 - WAnEXP) & 32767;
D 8
	SEZI += (IF_ELSE(SIGNBIT(*tmp2++, 10) ^ AnS,
		 (-(WAnMAG)) & ((U16BIT) 65535L), WAnMAG));
E 8
I 8
	SEZI += IF_ELSE(SIGNBIT(*tmp2++, 10) ^ AnS, -WAnMAG, WAnMAG);
E 8
      }
      if (i == 5)
	SEZ = (SEZI & (U16BIT) 65535L) >> 1;
    }
E 7
  }
E 5
D 6
  SEI = ((SEZI &= (U16BIT) 65535L) + f_mult(x->A1, x->SR1)
	  + f_mult(x->A2, x->SR2))
E 3
E 2
        & ((U16BIT) 65535L);
  SEZ = SEZI >> 1;
  SE = SEI >> 1;
E 6
I 6
D 7
  SEZ = (SEZI &= (U16BIT) 65535L) >> 1;
  SE = (SEI = (SEZI + f_mult(x->A1, x->SR1) + f_mult(x->A2, x->SR2))
        & ((U16BIT) 65535L)) >> 1;
E 7
I 7
D 9
  SE = (SEI = (SEZI & ((U16BIT) 65535L))) >> 1;
E 9
I 9
  SE = (SEZI & ((U16BIT) 65535L)) >> 1;
E 12
I 12
		for (i = 0; i < 8; i++)
		{
			{
				register S16BIT AnEXP, WAnEXP;
				register U16BIT AnMAG;
				U16BIT AnS, AnMANT, WAnMAG, WAnMANT;

				/* FMULT */
				AnS = *tmp1 < 0 ? 1 : 0;
				AnMAG = ABS(*tmp1 >> 2) & 8191;
				tmp1++;
				MSB0(AnMAG, AnEXP);
				AnMANT = AnMAG ? RSHIFT(AnMAG, AnEXP - 6) : 1 << 5;
				WAnMANT = ((*tmp2++ * AnMANT) + 48) >> 4;
				WAnEXP = *tmp2++ + AnEXP;
				WAnMAG = RSHIFT(WAnMANT, 19 - WAnEXP) & 32767;
				SEZI += IF_ELSE(*tmp2++ ^ AnS, -WAnMAG, WAnMAG);
			}
			if (i == 5)
D 14
				SEZ = SEZI >> 1;
E 14
I 14
				SEZ = (SEZI << (sizeof(S16BIT) * 8 - 16)) >> (sizeof(S16BIT) * 8 - 15);
E 14
		}
	}
D 14
	SE = SEZI >> 1;
E 14
I 14
	SE = (SEZI << (sizeof(S16BIT) * 8 - 16)) >> (sizeof(S16BIT) * 8 - 15);
E 14
E 12
E 9
E 7
E 6
}

D 3
static void adpt_predict_2(STATES *S)
E 3
I 3
static void adpt_predict_2(void)
E 3
{
D 12
  /*
   * adaptive predictor
   *
   * Input signals:  DQ, SE, SEZ, TR
   * Output signals: A2P
D 7
   * States:         B[], DQ[], PK1, PK2, SR1, SR2, A1, A2
E 7
I 7
   * States:         B[], DQ[], PK1, PK2
E 7
   */
E 12
I 12
	/*
D 13
	 * adaptive predictor
E 13
I 13
	 * pre-state part of the adaptive predictor
E 13
	 *
D 14
	 * Input signals:  DQ, SE, SEZ, TR
	 * Output signals: A2P, SR
	 * States:         B[], DQ[], PK1, PK2
E 14
I 14
	 * Input signals:   DQ, SE, SEZ, TR, X->DQn, X->SRn, X->An, X->Bn
	 * Output signals:  A2P, SR
	 * Internal states: PK1, PK2
E 14
	 */
E 12

D 2
  U16BIT A1P, A1T, A2T, BP[6], PK0, SIGPK, SR0, U[6];
E 2
I 2
D 3
  U16BIT A1T, A2T, PK0, SIGPK, SR0, U;
E 3
I 3
D 6
  U16BIT A1T, A2T, DQSEZ, PK0, SR0, U;
E 6
I 6
D 7
  U16BIT A1T, A2T, DQSEZ, PK0, SR0;
E 7
E 6
D 12
  register STATES* x = X;
I 7
  U16BIT *a1 = &x->B[6];
  U16BIT *a2 = a1 + 1;
  U16BIT A1T, A2T, DQSEZ, PK0, SR0;
E 12
I 12
	register STATES* x = X;
	S16BIT *a1 = &x->B[6];
	S16BIT *a2 = a1 + 1;
	U16BIT PK0, SR0M, SR0E, SR0S;
	S16BIT A1T, A2T, DQSEZ, DQTC;
E 12
E 7
E 3
E 2

I 12
	/* ADDC */
	{
		DQTC = IF_ELSE(NEGATIVE(DQ, 14), -(DQ & 16383), DQ);
		DQSEZ = DQTC + SEZ;
		PK0 = DQSEZ < 0 ? 1 : 0;
	}
E 12
I 7

E 7
D 12
  /* ADDC */
  {
D 2
    U16BIT DQS, DQI, SEZS, SEZI, DQSEZ;
E 2
I 2
D 3
    U16BIT register DQSEZ;
E 2

E 3
D 2
    DQS = DQ >> 14;
    DQI = DQS ? 
          (U16BIT) ((((U24BIT) 65536L) - (DQ & 16383)) & ((U16BIT) 65535L))
	  : DQ;
    SEZS = SEZ >> 14;
    SEZI = IF_ELSE(SEZS, (((U16BIT) 1) << 15) + SEZ, SEZ);
    DQSEZ = (DQI + SEZI) & ((U16BIT) 65535L);
    PK0 = DQSEZ >> 15;
E 2
I 2
D 6
    DQSEZ = (IF_ELSE(SIGNBIT(DQ, 14), TC(DQ & 16383) & ((U16BIT) 65535L), DQ) +
             IF_ELSE(SIGNBIT(SEZ, 14), (((U16BIT) 1) << 15) + SEZ, SEZ))
E 6
I 6
D 7
    DQSEZ = (IF_ELSE(NEGATIVE(DQ, 14), TC(DQ & 16383) & ((U16BIT) 65535L),
E 7
I 7
D 8
    DQSEZ = (IF_ELSE(NEGATIVE(DQ, 14), (-(DQ & 16383)) & ((U16BIT) 65535L),
E 7
		     DQ) + IF_ELSE(NEGATIVE(SEZ, 14),
				   (((U16BIT) 1) << 15) + SEZ, SEZ))
E 8
I 8
    DQSEZ = (IF_ELSE(NEGATIVE(DQ, 14), -(DQ & 16383), DQ)
              + IF_ELSE(NEGATIVE(SEZ, 14), (((U16BIT) 1) << 15) + SEZ, SEZ))
E 8
E 6
            & ((U16BIT) 65535L);
    PK0 = SIGNBIT(DQSEZ, 15);
E 2
D 3
    SIGPK = DQSEZ ? 0 : 1;
E 3
  }
E 12
I 12
	/* ADDB */
	SR = DQTC + SE;
E 12

D 12
  /* ADDB */
D 2
  {
    U16BIT DQS, DQI, SES, SEI;

    DQS = DQ >> 14;
    DQI = IF_ELSE(DQS, (U16BIT) ((((U24BIT) 65536L) - (DQ & 16383))
                & ((U16BIT)65535L)), DQ);
    SES = SE >> 14;
    SEI = IF_ELSE(SES, (((U16BIT) 1) << 15) + SE, SE);
    SR = (DQI + SEI) & ((U16BIT) 65535L);
  }
E 2
I 2
D 6
  SR = (IF_ELSE(SIGNBIT(DQ, 14), TC(DQ & 16383) & ((U16BIT)65535L), DQ)
	+ IF_ELSE(SIGNBIT(SE, 14), (((U16BIT) 1) << 15) + SE, SE))
E 6
I 6
D 7
  SR = (IF_ELSE(NEGATIVE(DQ, 14), TC(DQ & 16383) & ((U16BIT)65535L), DQ)
E 7
I 7
D 8
  SR = (IF_ELSE(NEGATIVE(DQ, 14), (-(DQ & 16383)) & ((U16BIT)65535L), DQ)
E 8
I 8
  SR = (IF_ELSE(NEGATIVE(DQ, 14), -(DQ & 16383), DQ)
E 8
E 7
	+ IF_ELSE(NEGATIVE(SE, 14), (((U16BIT) 1) << 15) + SE, SE))
E 6
       & ((U16BIT) 65535L);
E 2
  
  /* FLOATB */
  {
D 2
    U16BIT SRS, MAG, MANT;
    S16BIT EXP;
E 2
I 2
    register S16BIT EXP;
    U16BIT SRS, MAG;
E 12
I 12
	/* FLOATB */
	{
		U16BIT MAG;
E 12
E 2

D 2
    SRS = SR >> 15;
    MAG = IF_ELSE(SRS, (U16BIT) (((((U24BIT) 65536L)) - SR) & 32767), SR);
    MSB(MAG, EXP);
    MANT = MAG ? (MAG << 6) >> EXP : 1 << 5;
    SR0 = (SRS << 10) + (EXP << 6) + MANT;
E 2
I 2
D 6
    SRS = SIGNBIT(SR, 15);
E 6
I 6
D 12
    SRS = NEGATIVE(SR, 15);
E 6
D 7
    MAG = IF_ELSE(SRS, TC(SR) & 32767, SR);
E 7
I 7
    MAG = IF_ELSE(SRS, (-SR) & 32767, SR);
E 7
    MSB0(MAG, EXP);
D 6
    SR0 = (SRS << 10) + (EXP << 6) + (MAG ? LSHIFT(MAG, EXP - 6) : 1 << 5);
E 6
I 6
    SR0 = (SRS ? 1 << 10 : 0) + (EXP << 6) +
          (MAG ? LSHIFT(MAG, EXP - 6) : 1 << 5);
E 6
E 2
  }
E 12
I 12
		SR0S = SR < 0 ? 1 : 0;
		MAG = ABS(SR);
		MSB0(MAG, SR0E);
		SR0M = MAG ? RSHIFT(MAG, ((S16BIT) SR0E) - 6) : 1 << 5;
	}
E 12

D 12
  /* UPA2 */
  {
D 2
    U16BIT PKS1, PKS2, A1S, UGA2S, UGA2, A2S, ULA2, UA2;
    U24BIT UGA2A, FA, FA1, UGA2B;
E 2
I 2
D 4
    U16BIT PKS1, UGA2B, UGA2S, UGA2, ULA2;
E 4
I 4
D 6
    U16BIT PKS1, UGA2B, UGA2, ULA2;
E 6
I 6
    register U16BIT PKS1, UGA2B, UGA2, ULA2;
E 6
E 4
    U24BIT FA1;
E 12
I 12
D 13
	/* UPA2 */
	{
		register S16BIT FA1, UGA2;
E 12
E 2

D 3
    PKS1 = PK0 ^ S->PK1;
D 2
    PKS2 = PK0 ^ S->PK2;
    UGA2A = PKS2 ? (U24BIT) 114688L : (U24BIT) 16384;
    A1S = S->A1 >> 15;
    if (A1S)
E 2
I 2
    if (SIGNBIT(S->A1, 15))
E 2
      FA1 = S->A1 >= (U16BIT) 57345L ?
                     (((U24BIT) S->A1) << 2) & ((U24BIT) 131071L) :
E 3
I 3
D 12
    PKS1 = PK0 ^ x->PK1;
D 6
    if (SIGNBIT(x->A1, 15))
E 6
I 6
D 7
    if (NEGATIVE(x->A1, 15))
E 6
      FA1 = x->A1 >= (U16BIT) 57345L ?
                     (((U24BIT) x->A1) << 2) & ((U24BIT) 131071L) :
E 7
I 7
    if (NEGATIVE(*a1, 15))
      FA1 = *a1 >= (U16BIT) 57345L ?
                     (((U24BIT) *a1) << 2) & ((U24BIT) 131071L) :
E 7
E 3
D 6
		     ((U24BIT) 24577) << 2;
E 6
I 6
                     ((U24BIT) 24577) << 2;
E 6
    else
D 3
      FA1 = S->A1 <= 8191 ? S->A1 << 2 : 8191 << 2;
D 2
    FA = IF_ELSE(PKS1, FA1, (((U24BIT) 131072L) - FA1) & ((U24BIT) 131071L));
    UGA2B = (UGA2A + FA) & ((U24BIT) 131071L);
    UGA2S = (U16BIT) (UGA2B >> 16);
    UGA2 = IF_ELSE(UGA2S == 0 && SIGPK == 0, (U16BIT) (UGA2B >> 7),
           IF_ELSE(UGA2S == 1 && SIGPK == 0, ((U16BIT) (UGA2B >> 7))
                                      + ((U16BIT) 64512L), 0));
    A2S = S->A2 >> 15;
    ULA2 = A2S ? (((U24BIT) 65536L) - ((S->A2 >> 7)
				       + ((U16BIT) 65024L)))
		  & ((U16BIT) 65535L) :
                 (((U24BIT) 65536L) - (S->A2 >> 7)) & ((U16BIT) 65535L);
    UA2 = (UGA2 + ULA2) & ((U16BIT) 65535L);
	  A2T = (S->A2 + UA2) & ((U16BIT) 65535L);
E 2
I 2
    UGA2B = ((U16BIT) (((PK0 ^ S->PK2 ? (U24BIT) 114688L : (U24BIT) 16384)
E 3
I 3
D 7
      FA1 = x->A1 <= 8191 ? x->A1 << 2 : 8191 << 2;
E 7
I 7
      FA1 = *a1 <= 8191 ? *a1 << 2 : 8191 << 2;
E 7
    UGA2B = ((U16BIT) (((PK0 ^ x->PK2 ? (U24BIT) 114688L : (U24BIT) 16384)
E 3
D 6
			+ IF_ELSE(PKS1, FA1, TC(FA1) & ((U24BIT) 131071L)))
		       >> 7)) & 1023;
D 4
    UGA2S = SIGNBIT(UGA2B, 9);
D 3
    UGA2 = IF_ELSE(UGA2S == 0 && SIGPK == 0, UGA2B,
           IF_ELSE(UGA2S && SIGPK == 0, UGA2B + ((U16BIT) 64512L), 0));
    ULA2 = (SIGNBIT(S->A2, 15) ? TC((S->A2 >> 7) + ((U16BIT) 65024L)) :
                                 TC(S->A2 >> 7)) & ((U16BIT) 65535L);
    A2T = (S->A2 + UGA2 + ULA2) & ((U16BIT) 65535L);
E 3
I 3
    UGA2 = IF_ELSE(UGA2S == 0 && DQSEZ, UGA2B,
           IF_ELSE(UGA2S && DQSEZ, UGA2B + ((U16BIT) 64512L), 0));
E 4
I 4
    UGA2 = IF_ELSE(DQSEZ, IF_ELSE(SIGNBIT(UGA2B, 9),
E 6
I 6
D 7
                       + IF_ELSE(PKS1, FA1, TC(FA1) & ((U24BIT) 131071L)))
E 7
I 7
                       + IF_ELSE(PKS1, FA1, (-FA1) & ((U24BIT) 131071L)))
E 7
                       >> 7)) & 1023;
    UGA2 = IF_ELSE(DQSEZ, IF_ELSE(NEGATIVE(UGA2B, 9),
E 6
				  UGA2B + ((U16BIT) 64512L), UGA2B), 0);
E 4
D 6
    ULA2 = (SIGNBIT(x->A2, 15) ? TC((x->A2 >> 7) + ((U16BIT) 65024L)) :
E 6
I 6
D 7
    ULA2 = (NEGATIVE(x->A2, 15) ? TC((x->A2 >> 7) + ((U16BIT) 65024L)) :
E 6
                                 TC(x->A2 >> 7)) & ((U16BIT) 65535L);
    A2T = (x->A2 + UGA2 + ULA2) & ((U16BIT) 65535L);
E 7
I 7
D 8
    ULA2 = (NEGATIVE(*a2, 15) ? -((*a2 >> 7) + ((U16BIT) 65024L)) :
                                -(*a2 >> 7)) & ((U16BIT) 65535L);
E 8
I 8
    ULA2 = NEGATIVE(*a2, 15) ? -((*a2 >> 7) + ((U16BIT) 65024L)) :
                               -(*a2 >> 7);
E 8
    A2T = (*a2 + UGA2 + ULA2) & ((U16BIT) 65535L);
E 7
E 3
E 2
  }
E 12
I 12
		if (DQSEZ)
		{
			FA1 = *a1 < -8191 ? -8191 : *a1 > 8191 ? 8191 : *a1;
			UGA2 = ((PK0 ^ x->PK1 ? FA1 : -FA1) +
					  (PK0 ^ x->PK2 ? -4096 : 4096)) >> 5;
		}
		else
			UGA2 = 0;
		A2T = UGA2 + *a2 - (*a2 >> 7);
	}
E 12

E 13
D 2
  /* LIMC */
  A2P = IF_ELSE(((U16BIT) 32768L) <= A2T && A2T <= ((U16BIT) 53248L),
        ((U16BIT) 53248L),
        IF_ELSE(12288 <= A2T && A2T <= 32767, 12288, A2T));

  /* UPA1 */
E 2
I 2
D 12
  if (TR)
E 2
  {
D 2
    U16BIT PKS, UGA1, A1S, ULA1;
E 2
I 2
    register int i;
E 12
I 12
	if (TR)
	{
		register int i;
E 12
E 2

D 2
    PKS = PK0 ^ S->PK1;
    UGA1 = IF_ELSE(PKS == 0 && SIGPK == 0, 192,
           IF_ELSE(PKS == 1 && SIGPK == 0, (U16BIT) 65344L, 0));
    A1S = S->A1 >> 15;
    ULA1 = (U16BIT) ((A1S ? (((U24BIT) 65536L) - ((S->A1 >> 8)
	         + ((U16BIT) 65280L))) :
                 (((U24BIT) 65536L) - (S->A1 >> 8)))
           & ((U16BIT) 65535L));
    A1T = (S->A1 + UGA1 + ULA1) & ((U16BIT) 65535L);
  }

  /* LIMD */
  {
    U16BIT A1UL, A1LL;

    A1UL = (U16BIT) ((((U24BIT) 80896L) - A2P) & ((U16BIT) 65535L));
    A1LL = (A2P + ((U16BIT) 50176L)) & ((U16BIT) 65535L);
    A1P = ((U16BIT) 32768L) <= A1T && A1T <= A1LL ? A1LL :
          A1UL <= A1T && A1T <= 32767 ? A1UL : A1T;
  }

  /* XOR */
  {
    int i;
    U16BIT DQS;

    DQS = DQ >> 14;
E 2
I 2
D 3
    S->A1 = S->A2 = 0;
E 3
I 3
D 7
    x->A1 = x->A2 = 0;
E 3
E 2
    for (i = 0; i < 6; i++)
E 7
I 7
D 12
    for (i = 0; i < 8; i++)
E 7
D 2
      U[i] = DQS ^ (S->DQ[i] >> 10);
E 2
I 2
D 3
      S->B[i] = 0;
E 3
I 3
      x->B[i] = 0;
E 3
E 2
  }
D 2

  /* UPB */
E 2
I 2
  else
E 2
  {
D 2
    int i;
    U16BIT DQMAG, UGB, BS, ULB, UB;

    for ( i = 0; i < 6; i++)
E 2
I 2
    /* LIMC */
D 3
    S->A2 = A2P = IF_ELSE(((U16BIT) 32768L) <= A2T && A2T <= ((U16BIT) 53248L),
			  ((U16BIT) 53248L),
			  IF_ELSE(12288 <= A2T && A2T <= 32767, 12288, A2T));
E 3
I 3
D 4
    x->A2 = A2P = IF_ELSE(((U16BIT) 32768L) <= A2T,
			  IF_ELSE(A2T <= ((U16BIT) 53248L),
				  ((U16BIT) 53248L), A2T),
			  IF_ELSE(12288 <= A2T, 12288, A2T));
E 4
I 4
D 7
    x->A2 = A2P = IF_ELSE(A2T <= 32767, IF_ELSE(A2T >= 12288, 12288, A2T),
D 6
			 IF_ELSE(A2T <= (U16BIT) 53248L, (U16BIT) 53248L, A2T));
E 6
I 6
			 IF_ELSE(A2T <= (U16BIT) 53248L,
				 (U16BIT) 53248L, A2T));
E 7
I 7
    *a2 = A2P = IF_ELSE(A2T <= 32767, IF_ELSE(A2T >= 12288, 12288, A2T),
			IF_ELSE(A2T <= (U16BIT) 53248L,
			(U16BIT) 53248L, A2T));
E 7
E 6
E 4
E 3
    
    /* UPA1 */
E 2
    {
D 2
      DQMAG = DQ & 16383;
      UGB = IF_ELSE(U[i] == 0 && DQMAG != 0, 128,
	    IF_ELSE(U[i] == 1 && DQMAG != 0, ((U16BIT) 65408L), 0));
      BS = S->B[i] >> 15;
      ULB = (U16BIT) ((BS ?
                      (((U24BIT) 65536L) - ((S->B[i] >> 8)
                        + ((U16BIT) 65280L))) :
	              (((U24BIT) 65536L) - (S->B[i] >> 8)))
		      & ((U16BIT) 65535L));
      UB = (UGB + ULB) & ((U16BIT) 65535L);
      BP[i] = (S->B[i] + UB) & ((U16BIT) 65535L);
E 2
I 2
      U16BIT PKS;
E 12
I 12
		for (i = 0; i < 8; i++)
			x->B[i] = 0;
	}
	else
	{
I 13
		/* UPA2 */
		{
			register S16BIT FA1, UGA2;

			if (DQSEZ)
			{
				FA1 = *a1 < -8191 ? -8191 : *a1 > 8191 ? 8191 : *a1;
				UGA2 = ((PK0 ^ x->PK1 ? FA1 : -FA1) +
						  (PK0 ^ x->PK2 ? -4096 : 4096)) >> 5;
			}
			else
				UGA2 = 0;
			A2T = UGA2 + *a2 - (*a2 >> 7);
		}

E 13
		/* LIMC */
		*a2 = A2P = A2T < -12288 ? -12288 : A2T > 12288 ? 12288 : A2T;
E 12

D 3
      PKS = PK0 ^ S->PK1;
      A1T = (S->A1 +
	     IF_ELSE(PKS == 0 && SIGPK == 0, 192,
		     IF_ELSE(PKS == 1 && SIGPK == 0, (U16BIT) 65344L, 0)) +
	     (SIGNBIT(S->A1, 15) ? TC((S->A1 >> 8) + ((U16BIT) 65280L)) :
	      TC(S->A1 >> 8))) & ((U16BIT) 65535L);
E 3
I 3
D 12
      PKS = PK0 ^ x->PK1;
D 4
      A1T = (x->A1 +
	     IF_ELSE(PKS == 0 && DQSEZ, 192,
		     IF_ELSE(PKS == 1 && DQSEZ, (U16BIT) 65344L, 0)) +
E 4
I 4
D 7
      A1T = (x->A1 + IF_ELSE(DQSEZ, IF_ELSE(PKS, (U16BIT) 65344L, 192), 0) +
E 4
D 6
	     (SIGNBIT(x->A1, 15) ? TC((x->A1 >> 8) + ((U16BIT) 65280L)) :
E 6
I 6
	     (NEGATIVE(x->A1, 15) ? TC((x->A1 >> 8) + ((U16BIT) 65280L)) :
E 6
	      TC(x->A1 >> 8))) & ((U16BIT) 65535L);
E 7
I 7
      A1T = (*a1 + IF_ELSE(DQSEZ, IF_ELSE(PKS, (U16BIT) 65344L, 192), 0) +
	     (NEGATIVE(*a1, 15) ? -((*a1 >> 8) + ((U16BIT) 65280L)) :
	      -(*a1 >> 8))) & ((U16BIT) 65535L);
E 7
E 3
E 2
    }
E 12
I 12
		/* UPA1 */
		A1T = (DQSEZ ? (PK0 ^ x->PK1 ? -192 : 192) : 0) + *a1 - (*a1 >> 8);
E 12
D 2
  }
E 2

D 2
  /* TRIGB */
  {
    int i;

    if (TR)
E 2
I 2
D 12
    /* LIMD */
E 2
    {
D 2
      S->A1 = S->A2 = 0;
      for (i = 0; i < 6; i++)
	S->B[i] = 0;
E 2
I 2
D 6
      U16BIT A1UL, A1LL;
E 6
I 6
      register U16BIT A1UL, A1LL;
E 6
      
D 3
      A1UL = (U16BIT) ((((U24BIT) 80896L) - S->A2) & ((U16BIT) 65535L));
      A1LL = (S->A2 + ((U16BIT) 50176L)) & ((U16BIT) 65535L);
      S->A1 = ((U16BIT) 32768L) <= A1T && A1T <= A1LL ? A1LL :
E 3
I 3
D 7
      A1UL = (U16BIT) ((((U24BIT) 80896L) - x->A2) & ((U16BIT) 65535L));
      A1LL = (x->A2 + ((U16BIT) 50176L)) & ((U16BIT) 65535L);
D 4
      x->A1 = ((U16BIT) 32768L) <= A1T && A1T <= A1LL ? A1LL :
E 3
	      A1UL <= A1T && A1T <= 32767 ? A1UL : A1T;
E 4
I 4
      x->A1 = IF_ELSE(A1T <= 32767,
E 7
I 7
      A1UL = (U16BIT) ((((U24BIT) 80896L) - *a2) & ((U16BIT) 65535L));
      A1LL = (*a2 + ((U16BIT) 50176L)) & ((U16BIT) 65535L);
      *a1 = IF_ELSE(A1T <= 32767,
E 7
		      IF_ELSE(A1T >= A1UL, A1UL, A1T),
		      IF_ELSE(A1T <= A1LL, A1LL, A1T));
E 4
E 2
    }
E 12
I 12
		/* LIMD */
		{
			register S16BIT A1L;
E 12
D 2
    else
E 2
I 2

D 12
    /* UPB, XOR and TRIGB */
E 2
    {
D 2
      S->A1 = A1P;
      S->A2 = A2P;
      for (i = 0; i < 6; i++)
	S->B[i] = BP[i];
E 2
I 2
      register int i;
I 7
D 8
      register U16BIT* tmp = x->B;
E 8
I 8
      register U16BIT* tmp1 = x->B;
E 8
      U16BIT* tmp2 = x->DQ;
      U16BIT DQS, DQM;
E 7
D 4
      U16BIT DQS, DQMAG, UGB, ULB;
E 4
      
D 7
      for ( i = 0; i < 6; i++)
      {
D 4
	DQS = SIGNBIT(DQ, 14);
D 3
	U = DQS ^ (S->DQ[i] >> 10);
E 3
I 3
	U = DQS ^ (x->DQ[i] >> 10);
E 4
I 4
D 6
	U = SIGNBIT(DQ, 14) ^ (x->DQ[i] >> 10);
E 4
E 3
	
D 4
	DQMAG = DQ & 16383;
	UGB = IF_ELSE(U == 0 && DQMAG != 0, 128,
		      IF_ELSE(U == 1 && DQMAG != 0, ((U16BIT) 65408L), 0));
D 3
	ULB = (SIGNBIT(S->B[i],15) ? TC((S->B[i] >> 8) + ((U16BIT) 65280L)) :
	       TC(S->B[i] >> 8)) & ((U16BIT) 65535L);
	S->B[i] = (S->B[i] + UGB + ULB) & ((U16BIT) 65535L);
E 3
I 3
	ULB = (SIGNBIT(x->B[i],15) ? TC((x->B[i] >> 8) + ((U16BIT) 65280L)) :
	       TC(x->B[i] >> 8)) & ((U16BIT) 65535L);
	x->B[i] = (x->B[i] + UGB + ULB) & ((U16BIT) 65535L);
E 4
I 4
	x->B[i] = (x->B[i] +
		   IF_ELSE(DQ & 16383, IF_ELSE(U, (U16BIT) 65408L, 128), 0) +
		   (SIGNBIT(x->B[i],15) ?
		    TC((x->B[i] >> 8) + ((U16BIT) 65280L)) : TC(x->B[i] >> 8)))
E 6
I 6
	register U16BIT* tmp = &x->B[i];

E 7
I 7
      DQS = SIGNBIT(DQ, 14);
      DQM = DQ & 16383;
D 8
      for (i = 0; i < 6; i++, tmp++)
E 7
	*tmp = (*tmp +
D 7
		IF_ELSE(DQ & 16383,
			IF_ELSE((SIGNBIT(DQ, 14) ^ (x->DQ[i] >> 10)),
			(U16BIT) 65408L, 128), 0) +
		(NEGATIVE(*tmp ,15) ?
		 TC((*tmp >> 8) + ((U16BIT) 65280L)) : TC(*tmp >> 8)))
E 6
	          & ((U16BIT) 65535L);
E 4
E 3
      }
E 7
I 7
	        IF_ELSE(DQM,
	                IF_ELSE(DQS ^ SIGNBIT(*tmp2++, 10),
		        (U16BIT) 65408L, 128), 0) +
	        (NEGATIVE(*tmp ,15) ?
		 -((*tmp >> 8) + ((U16BIT) 65280L)) : -(*tmp >> 8)))
	       & ((U16BIT) 65535L);
E 8
I 8
      for (i = 0; i < 6; i++, tmp1++)
	*tmp1 = (*tmp1 +
	         IF_ELSE(DQM,
	                 IF_ELSE(DQS ^ SIGNBIT(*tmp2++, 10),
		                 (U16BIT) 65408L, 128), 0) +
	         (NEGATIVE(*tmp1 ,15) ?
		 -((*tmp1 >> 8) + ((U16BIT) 65280L)) : -(*tmp1 >> 8)))
	        & ((U16BIT) 65535L);
E 8
E 7
E 2
    }
  }
E 12
I 12
			A1L = 15360 - *a2;
			*a1 = A1T < -A1L ? -A1L : A1T > A1L ? A1L : A1T;
		}
E 12

D 12
  /* FLOATA */
  {
D 2
    int i = 0;
    U16BIT DQS, MAG, MANT;
E 2
I 2
    register int i = 0;
D 6
    U16BIT DQS, MAG;
E 6
I 6
    U16BIT MAG;
E 6
E 2
    S16BIT EXP;
E 12
I 12
		/* UPB, XOR and TRIGB */
		{
			register int i;
			register S16BIT* tmp1 = x->B;
E 12

D 5
    for (i = 5; i; i--)
D 3
      S->DQ[i] = S->DQ[i - 1];
E 3
I 3
      x->DQ[i] = x->DQ[i - 1];
E 5
I 5
D 12
    {
      register U16BIT* tmp = &x->DQ[5];
E 12
I 12
			U16BIT* tmp2 = &x->DQ[2];   /* points to sign of DQ1 */
			U16BIT DQS, DQM;
E 12
E 5
E 3

I 5
D 7
      for (i = 5; i; i--)
E 7
I 7
D 12
      for (i = 0; i < 5; i++)
E 7
	*tmp-- = *(tmp - 1);
    }
E 12
I 12
			DQS = SIGNBIT(DQ, 14);
			DQM = DQ & 16383;
			for (i = 0; i < 6; i++, tmp1++, tmp2 += 3)
				*tmp1 = *tmp1 - (*tmp1 >> 8) +
							IF_ELSE(DQM, IF_ELSE(DQS ^ *tmp2, -128, 128), 0);
		}
	}
E 12

E 5
D 2
    DQS = DQ >> 14;
E 2
I 2
D 6
    DQS = SIGNBIT(DQ, 14);
E 6
E 2
D 12
    MAG = DQ & 16383;
D 2
    MSB(MAG, EXP);
    MANT = MAG ? (MAG << 6) >> EXP : 1 << 5;
    S->DQ[0] = (DQS << 10) + (EXP << 6) + MANT;
E 2
I 2
    MSB0(MAG, EXP);
D 3
    S->DQ[0] = (DQS << 10) + (EXP << 6) + (MAG ? LSHIFT(MAG, EXP - 6) : 1 << 5);
E 3
I 3
D 6
    x->DQ[0] = (DQS << 10) + (EXP << 6) + (MAG ? LSHIFT(MAG, EXP - 6) : 1 << 5);
E 6
I 6
    x->DQ[0] = (NEGATIVE(DQ, 14)? 1 << 10 : 0) + (EXP << 6) +
               (MAG ? LSHIFT(MAG, EXP - 6) : 1 << 5);
E 6
E 3
E 2
  }
E 12
I 12
	x->PK2 = x->PK1;
	x->PK1 = PK0;
E 12

D 3
  S->PK2 = S->PK1;
  S->PK1 = PK0;
  S->SR2 = S->SR1;
  S->SR1 = SR0;
E 3
I 3
D 12
  x->PK2 = x->PK1;
  x->PK1 = PK0;
D 7
  x->SR2 = x->SR1;
  x->SR1 = SR0;
E 7
I 7
  x->DQ[7] = x->DQ[6];
  x->DQ[6] = SR0;
E 12
I 12
	/* FLOATA */
	{
		register int i = 0;
		U16BIT MAG;
		S16BIT EXP;

		MAG = DQ & 16383;
		MSB0(MAG, EXP);
		{
			register U16BIT* tmp1 = &x->DQ[17];
			register U16BIT* tmp2 = tmp1 - 3;

			for (i = 0; i < 15; i++)
				*tmp1-- = *tmp2--;
			*tmp1-- = SIGNBIT(DQ, 14);
			*tmp1-- = EXP;
			*tmp1 = MAG ? RSHIFT(MAG, EXP - 6) : 1 << 5;
		}
	}

	{
		register int i;
		register U16BIT* tmp1 = &x->DQ[23];
		register U16BIT* tmp2 = tmp1 - 3;

		for (i = 0; i < 3; i++)
			*tmp1-- = *tmp2--;
		*tmp1-- = SR0S;
		*tmp1-- = SR0E;
		*tmp1 = SR0M;
	}
E 12
E 7
E 3
}

static void coding_adjustment()
{
  /*
D 12
   * synchronous coding adjustment
   *
   * Input signals:  D, SP, Y, LAW, I
   * Output signals: SD
   */
E 12
I 12
	* synchronous coding adjustment
	*
D 13
	* Input signals:  D, SP, Y, LAW, I
E 13
I 13
	* Input signals:  D, S, Y, LAW, I
E 13
	* Output signals: SD
	*/
E 12

D 2
  U16BIT DL, DLN, DS;
E 2
I 2
D 12
  register U16BIT DLN, DS;
E 12
I 12
D 13
  register U16BIT DLN;
E 13
I 13
  register S16BIT DLN;
E 13
E 12
E 2

D 2
  /* LOG */
E 2
I 2
  /* LOG and SUBTB*/
E 2
  {
D 2
    U16BIT DQM, EXP, MANT;
E 2
I 2
D 12
    register S16BIT EXP;
    U16BIT          DQM;
E 12
I 12
D 14
	 register S16BIT EXP;
	 U16BIT          DQM;
E 14
I 14
		register S16BIT EXP;
		U16BIT          DQM;
E 14
E 12
E 2

D 2
    DS = D >> 15;
    DQM = IF_ELSE(DS, (U16BIT) (((U24BIT) 65536L) - D) & 32767, D);
    for (EXP = 0; DQM >> (EXP + 1); EXP++)
      ;
    MANT = ((DQM << 7) >> EXP) & 127;
    DL = (EXP << 7) + MANT;
E 2
I 2
D 6
    DS = SIGNBIT(D, 15);
E 6
I 6
D 12
    DS = NEGATIVE(D, 15);
E 6
D 7
    DQM = IF_ELSE(DS, TC(D) & 32767, D);
E 7
I 7
    DQM = IF_ELSE(DS, (-D) & 32767, D);
E 7
    MSB1(DQM, EXP);
D 7
    DLN = (((EXP << 7) + (LSHIFT(DQM, EXP - 7) & 127)) + TC(Y >> 2)) & 4095;
E 7
I 7
    DLN = (((EXP << 7) + (LSHIFT(DQM, EXP - 7) & 127)) - (Y >> 2)) & 4095;
E 12
I 12
D 14
	 DQM = ABS(D);
	 MSB1(DQM, EXP);
D 13
	 DLN = (((EXP << 7) + (RSHIFT(DQM, EXP - 7) & 127)) - (Y >> 2)) & 4095;
E 13
I 13
	 DLN = ((EXP << 7) + (RSHIFT(DQM, EXP - 7) & 127)) - (Y >> 2);
E 14
I 14
		DQM = ABS(D);
		MSB1(DQM, EXP);
		DLN = ((EXP << 7) + (RSHIFT(DQM, EXP - 7) & 127)) - (Y >> 2);
E 14
E 13
E 12
E 7
E 2
  }
D 12
    	
E 12
I 12

E 12
D 2
  /* SUBTB */
  DLN = (DL + 4096 - (Y >> 2)) & 4095;

E 2
D 13
  /* SYNC */
E 13
I 13
	/* SYNC */
E 13
D 12
  {
D 2
    U16BIT IS, IM, ID;
E 2
I 2
    U16BIT IM, ID;
E 12
I 12
	{
I 13
		int i;
		static S16BIT TAB[] = { 400, 349, 300, 246, 178, 80, -124 };
		S16BIT* tp = TAB;
E 13
		U16BIT IM, ID;
E 12
E 2

I 13
		ID = 7;
		for (i = 0; i < 7; i++)
			if (DLN < *tp++)
				ID--;
		if (D < 0)        ID = ~ID;
		if (ID == 0)      ID = ~ID;

		ID = (ID ^ 8) & 15;
E 13
D 2
    IS = I >> 3;
    IM = IS ? I & 7 : I + 8;
E 2
I 2
D 6
    IM = SIGNBIT(I, 3) ? I & 7 : I + 8;
E 6
I 6
D 12
    IM = NEGATIVE(I, 3) ? I & 7 : I + 8;
E 6
E 2
D 5
    if (DLN < 80)
E 5
I 5
D 7
  if (DLN > 299)
    if (DLN > 2047)
      if (DLN > 3971)
	ID = DS ? 6 : 9;
E 7
I 7
    if (DLN > 299)
      if (DLN > 2047)
	if (DLN > 3971)
	  ID = DS ? 6 : 9;
	else
	  ID = 7;
E 7
      else
D 7
	ID = 7;
E 7
I 7
	if (DLN > 399)
	  ID = DS ? 0 : 15;
	else
	  if (DLN > 348)
	    ID = DS ? 1 : 14;
	  else
	    ID = DS ? 2 : 13;
E 7
    else
D 7
      if (DLN > 399)
	ID = DS ? 0 : 15;
E 7
I 7
      if (DLN > 177)
	if (DLN > 245)
	  ID = DS ? 3 : 12;
	else
	  ID = DS ? 4 : 11;
E 7
      else
D 7
	if (DLN > 348)
	  ID = DS ? 1 : 14;
        else
	  ID = DS ? 2 : 13;
  else
    if (DLN > 177)
      if (DLN > 245)
	ID = DS ? 3 : 12;
      else
	ID = DS ? 4 : 11;
    else
      if (DLN > 79)
	ID = DS ? 5 : 10;
      else
	ID = DS ? 6 : 9;

E 7
I 7
	if (DLN > 79)
	  ID = DS ? 5 : 10;
	else
	  ID = DS ? 6 : 9;
    
E 7
D 6
/*    if (DLN < 80)
E 5
      ID = DS ? 6 : 9;
    else if (DLN < 178)
      ID = DS ? 5 : 10;
    else if (DLN < 246)
      ID = DS ? 4 : 11;
    else if (DLN < 300)
      ID = DS ? 3 : 12;
    else if (DLN < 349)
      ID = DS ? 2 : 13;
    else if (DLN < 400)
      ID = DS ? 1 : 14;
    else if (DLN < 2048)
      ID = DS ? 0 : 15;
    else if (DLN < 3972)
      ID = 7;
    else
D 5
      ID = DS ? 6 : 9;
E 5
I 5
      ID = DS ? 6 : 9; */
E 5

E 6
    if (LAW)
    {
      SD = S ^ 0x55;
      if (ID > IM)
      {
	if (SD <= 126)
	  SD++;
        else if (SD >= 129)
	  SD--;
	else
	  SD = SD == 128 ? 0 : 127;
      }
      else if (ID < IM)
      {
D 4
        if (SD >= 1 && SD <= 127)
	  SD--;
        else if (SD >= 128 && SD <= 254)
	  SD++;
E 4
I 4
	if (SD <= 127)
	  SD = SD ? SD - 1 : 128;
E 4
	else
D 4
	  SD = SD ? 255 : 128;
E 4
I 4
	  SD = SD == 255 ? 255 : SD + 1;
E 4
      }
      SD ^= 0x55;
    }
    else
    {
I 6
      SD = S;
E 6
      if (ID > IM)
      {
	if (1 <= S && S <= 127)
D 6
	  SD = S - 1;
E 6
I 6
	  SD--;
E 6
	else if (128 <= S && S <= 254)
D 6
	  SD = S + 1;
E 6
I 6
	  SD++;
E 6
	else
	  SD = S ? 126 : 0;
      }
      else if (ID < IM)
      {
	if (S <= 126)
D 6
	  SD = S + 1;
E 6
I 6
	  SD++;
E 6
	else if (128 < S && S <= 255)
D 6
	  SD = S - 1;
E 6
I 6
	  SD--;
E 6
	else
	  SD = S == 127 ? 254 : 128;
      }
D 6
      else
	SD = S;
E 6
    }
  }
E 12
I 12
		IM = (I ^ 8) & 15;
D 13
		if (DLN > 299)
			if (DLN > 2047)
				if (DLN > 3971)
					ID = D < 0 ? 6 : 9;
				else
					ID = 7;
			else
				if (DLN > 399)
					ID = D < 0 ? 0 : 15;
				else
					if (DLN > 348)
						ID = D < 0 ? 1 : 14;
					else
						ID = D < 0 ? 2 : 13;
		else
			if (DLN > 177)
				if (DLN > 245)
					ID = D < 0 ? 3 : 12;
				else
					ID = D < 0 ? 4 : 11;
			else
				if (DLN > 79)
					ID = D < 0 ? 5 : 10;
				else
					ID = D < 0 ? 6 : 9;
E 13

		if (LAW)
		{
I 13
			/* A-law */
E 13
			SD = S ^ 0x55;
			if (ID > IM)
D 14
			{
				if (SD <= 126)
					SD++;
				else if (SD >= 129)
					SD--;
				else
					SD = SD == 128 ? 0 : 127;
			}
E 14
I 14
				ADJUST1(SD, 128)
E 14
			else if (ID < IM)
D 14
			{
				if (SD <= 127)
					SD = SD ? SD - 1 : 128;
				else
					SD = SD == 255 ? 255 : SD + 1;
			}
E 14
I 14
				ADJUST1(SD, 0)
E 14
			SD ^= 0x55;
		}
		else
		{
I 13
			/* u-law */
E 13
			SD = S;
			if (ID > IM)
D 13
			{
				if (1 <= S && S <= 127)
					SD--;
				else if (128 <= S && S <= 254)
					SD++;
				else
					SD = S ? 126 : 0;
			}
E 13
I 13
D 14
				ADJUST(S, SD, 0)
E 14
I 14
				ADJUST2(SD, 0)
E 14
E 13
			else if (ID < IM)
D 13
			{
				if (S <= 126)
					SD++;
				else if (128 < S && S <= 255)
					SD--;
				else
					SD = S == 127 ? 254 : 128;
			}
E 13
I 13
D 14
				ADJUST(S, SD, 128);
E 14
I 14
				ADJUST2(SD, 128)
E 14
E 13
		}
	}
E 12
}

static void diff_computation()
{
  /*
D 12
   * difference signal computation
   *
   * Input signals:  SL, SE
   * Output signals: D
   */
D 2
  U16BIT SLS, SLI, SES, SEI;
E 2
I 2
  U16BIT SLI, SEI;
E 12
I 12
	* difference signal computation
	*
	* Input signals:  SL, SE
	* Output signals: D
	*/
E 12
E 2

  /* SUBTA */
D 2
  SLS = SL >> 13;
  SLI = IF_ELSE(SLS, ((U16BIT) 49152L) + SL, SL);
  SES = SE >> 14;
  SEI = IF_ELSE(SES, ((U16BIT) 32768L) + SE, SE);
  D = (SLI + (((U24BIT) 65536L) - SEI)) & ((U16BIT) 65535L);
E 2
I 2
D 6
  SLI = IF_ELSE(SIGNBIT(SL, 13), ((U16BIT) 49152L) + SL, SL);
  SEI = IF_ELSE(SIGNBIT(SE, 14), ((U16BIT) 32768L) + SE, SE);
E 6
I 6
D 12
  SLI = IF_ELSE(NEGATIVE(SL, 13), ((U16BIT) 49152L) + SL, SL);
  SEI = IF_ELSE(NEGATIVE(SE, 14), ((U16BIT) 32768L) + SE, SE);
E 6
D 7
  D = (SLI + TC(SEI)) & ((U16BIT) 65535L);
E 7
I 7
  D = (SLI - SEI) & ((U16BIT) 65535L);
E 12
I 12
  D = SL - SE;
E 12
E 7
E 2
}

D 7
static U16BIT f_mult(U16BIT An, U16BIT SRn)
{
D 2
  U16BIT AnS, AnMAG, AnMANT, SRnS, SRnEXP, SRnMANT, WAnS, WAnEXP, WAnMAG;
  U16BIT AnEXP;
  U24BIT WAnMANT;
E 2
I 2
  register S16BIT AnEXP, WAnEXP;
D 6
  U16BIT AnS, AnMAG, AnMANT, WAnMAG, WAnMANT;
E 6
I 6
  register U16BIT AnMAG;
  U16BIT AnS, AnMANT, WAnMAG, WAnMANT;
E 6
E 2

D 2
  AnS = An >> 15;
  AnMAG = AnS ? (16384 - (An >> 2)) & 8191 : An >> 2;
  MSB(AnMAG, AnEXP);
  AnMANT = AnMAG ? (((U24BIT) AnMAG) << 6) >> AnEXP : 1 << 5;
  SRnS = SRn >> 10;
  SRnEXP = (SRn >> 6) & 15;
  SRnMANT = SRn & 63;
  WAnS = SRnS ^ AnS;
  WAnEXP = SRnEXP + AnEXP;
  WAnMANT = (((U24BIT) SRnMANT * AnMANT) + 48) >> 4;
  WAnMAG = WAnEXP <= 26 ?
           (WAnMANT << 7) >> (26 - WAnEXP) :
	   ((WAnMANT << 7) << (WAnEXP - 26)) & 32767;
  return (IF_ELSE(WAnS, ((((U24BIT) 65536L) - WAnMAG)) & ((U16BIT) 65535L)
	  , WAnMAG));
E 2
I 2
  AnMAG = (AnS = SIGNBIT(An, 15)) ? TC(An >> 2) & 8191 : An >> 2;
  MSB0(AnMAG, AnEXP);
  AnMANT = AnMAG ? LSHIFT(AnMAG, AnEXP - 6) : 1 << 5;
  WAnMANT = (((SRn & 63) * AnMANT) + 48) >> 4;
  WAnEXP = ((SRn >> 6) & 15) + AnEXP;
  WAnMAG = LSHIFT((U24BIT) WAnMANT, 19 - WAnEXP) & 32767;
  return (IF_ELSE(SIGNBIT(SRn, 10) ^ AnS,
                  TC(WAnMAG) & ((U16BIT) 65535L), WAnMAG));
E 2
}

E 7
static void iadpt_quant()
{
D 12
  /*
   * inverse adaptive quantizer
   * 
   * Input signals:  I, Y
   * Output signals: DQ
   */
 
D 11
  static U16BIT qtab[] =
E 11
I 11
  static const U16BIT qtab[] =
E 11
  {
    2048, 4, 135, 213, 273, 323, 373, 425,
    425, 373, 323, 273, 213, 135, 4, 2048,
  } ;
  U16BIT DQL;
E 12
I 12
	/*
	 * inverse adaptive quantizer
	 *
	 * Input signals:  I, Y
	 * Output signals: DQ
	 */
E 12

D 12
  /* RECONST and ADDA */
D 5
  DQL = (qtab[I] + (Y >> 2)) & 4095;
E 5
I 5
  DQL = (qtab[I] + (Y >> 2));
E 12
I 12
	static U16BIT qtab[] =
	{
		2048, 4, 135, 213, 273, 323, 373, 425,
		425, 373, 323, 273, 213, 135, 4, 2048,
	} ;
	U16BIT DQL;
E 12
E 5

D 12
  /* ANTILOG */
  {
D 2
    U16BIT DS, DEX, DMN, DQT, DQMAG;
E 2
I 2
    S16BIT DEX;
    U16BIT DQT;
E 12
I 12
	/* RECONST and ADDA */
	DQL = (qtab[I & 15] + (Y >> 2));
E 12
E 2

D 2
    DS = DQL >> 11;
E 2
D 12
    DEX = (DQL >> 7) & 15;
D 2
    DMN = DQL & 127;
    DQT = (1 << 7) + DMN;
    DQMAG = DS ? 0 : (DQT << 7) >> (14 - DEX);
    DQ = ((I >> 3) << 14) + DQMAG;
E 2
I 2
    DQT = (1 << 7) + (DQL & 127);
D 6
    DQ = IF_ELSE(I & (1 << 3), 1 << 14, 0) 
         + (SIGNBIT(DQL, 11) ? 0 : LSHIFT(DQT, 7 - DEX));
E 6
I 6
    DQ = IF_ELSE(NEGATIVE(I, 3), 1 << 14, 0) 
         + (NEGATIVE(DQL, 11) ? 0 : LSHIFT(DQT, 7 - DEX));
E 6
E 2
  }
E 12
I 12
	/* ANTILOG */
	{
		S16BIT DEX;
		U16BIT DQT;

		DEX = (DQL >> 7) & 15;
		DQT = (1 << 7) + (DQL & 127);
		DQ = IF_ELSE(I < 0, 1 << 14, 0)
			  + (NEGATIVE(DQL, 11) ? 0 : RSHIFT(DQT, 7 - DEX));
	}
E 12
}

static void input_conversion()
{
D 12
  /*
   * convert to uniform PCM
   * Input signals:  S
   * Output signals: SL
   */
E 12
I 12
	/*
D 13
	 * convert to uniform PCM
E 13
I 13
	 * input PCM format conversion
E 13
D 14
	 * Input signals:  S
E 14
I 14
	 * Input signals:  S, LAW
E 14
	 * Output signals: SL
	 */
E 12

D 2
  U16BIT SS, SSS, SSM, SSQ;
E 2
I 2
D 12
  U16BIT SS, SSS, SSQ;
E 12
I 12
	U16BIT SS, SSS, SSQ;
E 12
E 2

D 12
  /* EXPAND */
  if (LAW)
  {
    SS = A_LAW_table[S];
D 2
    SSS = SS >> 12;
    SSM = SS & 4095;
    SSQ = SSM << 1;
E 2
I 2
D 6
    SSS = SIGNBIT(SS, 12);
E 6
I 6
    SSS = NEGATIVE(SS, 12);
E 6
    SSQ = (SS & 4095) << 1;
E 2
  }
  else
  {
    SS = u_LAW_table[S];
D 2
    SSS = SS >> 13;
E 2
I 2
D 6
    SSS = SIGNBIT(SS, 13);
E 6
I 6
    SSS = NEGATIVE(SS, 13);
E 6
E 2
    SSQ = SS & 8191;
  }
D 2
  SL = IF_ELSE(SSS, (16384 - SSQ) & 16383, SSQ);
E 2
I 2
D 7
  SL = IF_ELSE(SSS, TC(SSQ) & 16383, SSQ);
E 7
I 7
  SL = IF_ELSE(SSS, (-SSQ) & 16383, SSQ);
E 12
I 12
	/* EXPAND */
	if (LAW)
	{
		SS = A_LAW_table[S];
		SSS = NEGATIVE(SS, 12);
		SSQ = (SS & 4095) << 1;
	}
	else
	{
		SS = u_LAW_table[S];
		SSS = NEGATIVE(SS, 13);
		SSQ = SS & 8191;
	}
	SL = IF_ELSE(SSS, -SSQ, SSQ);
E 12
E 7
E 2
}

static void output_conversion()
{
D 12
  /*
   * Output PCM format conversion
   *
   * Input signals:  SR
   * Output signals: S
   */
D 2
  U16BIT IS, IM, IMAG;
E 2
I 2
  U16BIT IS, IM;
E 12
I 12
	/*
D 13
	 * Output PCM format conversion
E 13
I 13
	 * output PCM format conversion
E 13
	 *
D 14
	 * Input signals:  SR
E 14
I 14
	 * Input signals:  SR, LAW
E 14
	 * Output signals: S
	 */
	U16BIT IS, IM;
E 12
E 2

I 13
	/* COMPRESS */
E 13
D 2
  IS = SR >> 15;
  IM = IF_ELSE(IS, ((U16BIT) (((U24BIT) 65536L) - SR) & 32767), SR);
  IMAG = IF_ELSE(LAW == 0, IM, IF_ELSE(IS, (IM + 1) >> 1, IM >> 1));
E 2
I 2
D 6
  IS = SIGNBIT(SR, 15);
E 6
I 6
D 12
  IS = NEGATIVE(SR, 15);
E 6
D 7
  IM = IF_ELSE(IS, TC(SR) & 32767, SR);
E 7
I 7
  IM = IF_ELSE(IS, (-SR) & 32767, SR);
E 12
I 12
	IS = SR < 0 ? 1 : 0;
	IM = ABS(SR);
E 12
E 7
E 2
D 13

E 13
D 12
  if (LAW)
  {
    U16BIT MASK, SEG, IMS;
E 12
I 12
	if (LAW)
	{
I 13
		/* A-law */
E 13
		U16BIT MASK, SEG, IMS;
E 12

D 12
    MASK = IS ? 0x55 : 0xD5;
D 2
    IMS = IMAG - (IS ? 1 : 0);
E 2
I 2
D 4
    IMS = IF_ELSE(IS, (IM + 1) >> 1, IM >> 1) - (IS ? 1 : 0);
E 4
I 4
    IMS = IF_ELSE(IS, ((IM + 1) >> 1) - 1, IM >> 1);
E 4
E 2
    if (IMS > 4095)
      S = 0x7F ^ MASK;
    else
    {
D 4
      for (SEG = 5; IMS >> SEG; SEG++)
E 4
I 4
      register U16BIT tmp = IMS >> 4;
E 12
I 12
		MASK = IS ? 0x55 : 0xD5;
		IMS = IF_ELSE(IS, ((IM + 1) >> 1) - 1, IM >> 1);
		if (IMS > 4095)
			S = 0x7F ^ MASK;
		else
		{
			register U16BIT tmp = IMS >> 4;
E 12

D 12
      for (SEG = 0; tmp >>= 1; SEG++)
E 4
	;
D 4
      SEG -= 5;
      S = (SEG << 4 | ((SEG ? IMS >> SEG : IMS >> 1) & 0xF)) ^ MASK;
E 4
I 4
      S = ((SEG << 4) | ((SEG ? IMS >> SEG : IMS >> 1) & 0xF)) ^ MASK;
E 4
    }
  }
  else
  {
    U16BIT MASK, IMS, SEG;
E 12
I 12
			for (SEG = 0; tmp >>= 1; SEG++)
				;
			S = ((SEG << 4) | ((SEG ? IMS >> SEG : IMS >> 1) & 0xF)) ^ MASK;
		}
	}
	else
	{
I 13
		/* u-law */
E 13
		U16BIT MASK, IMS, SEG;
E 12

D 12
    MASK = IS ? 0x7F : 0xFF;
D 2
    IMS = IMAG + 33;
E 2
I 2
    IMS = IM + 33;
E 2
    if (IMS > 8191)
      S = 0x7F ^ MASK;
    else
    {
      for (SEG = 5; IMS >> SEG; SEG++)
	;
      SEG -= 6;
D 4
      S = (SEG << 4 | ((IMS >> SEG + 1) & 0xF)) ^ MASK;
E 4
I 4
D 5
      S = ((SEG << 4) | ((IMS >> SEG + 1) & 0xF)) ^ MASK;
E 5
I 5
      S = ((SEG << 4) | ((IMS >> (SEG + 1)) & 0xF)) ^ MASK;
E 5
E 4
    }
  }
E 12
I 12
		MASK = IS ? 0x7F : 0xFF;
		IMS = IM + 33;
		if (IMS > 8191)
			S = 0x7F ^ MASK;
		else
		{
			for (SEG = 5; IMS >> SEG; SEG++)
				;
			SEG -= 6;
			S = ((SEG << 4) | ((IMS >> (SEG + 1)) & 0xF)) ^ MASK;
		}
	}
E 12
}

D 3
static void scale_factor_1(STATES* S)
E 3
I 3
static void reset_states(void)
E 3
{
I 3
D 12
  register STATES* x = X;
  int i;
E 12
I 12
	register STATES* x = X;
	int i;
E 12

D 7
  for (i = 0; i < 6; i++)
E 7
I 7
D 12
  for (i = 0; i < 8; i++)
E 7
    x->B[i] = 0;
  for (i = 0; i < 6; i++)
    x->DQ[i] = 32;
I 7
  x->DQ[6] = x->DQ[7] = 32;
E 7
  x->PK1 = x->PK2 = 0;
D 7
  x->SR1 = x->SR2 = 32;
  x->A1 = x->A2 = 0;
E 7
  x->AP = x->DMS = x->DML = 0;
  x->YU = 544;
  x->TD = 0;
  x->YL6 = (U16BIT) ((x->YL = (U24BIT) 34816L) >> 6);
E 12
I 12
	for (i = 0; i < 8; i++)
		x->B[i] = 0;
	for (i = 0; i < 8; i++)
	{
		x->DQ[i * 3] = 32;       /* mantissa */
		x->DQ[i * 3 + 1] = 0;    /* exponent */
		x->DQ[i * 3 + 2] = 0;    /* sign */
	}
	x->PK1 = x->PK2 = 0;
	x->AP = x->DMS = x->DML = 0;
	x->YU = 544;
	x->TD = 0;
	x->YL6 = (U16BIT) ((x->YL = (U24BIT) 34816L) >> 6);
E 12
}

static void scale_factor_1(void)
{
E 3
D 12
  /*
   * quantizer scale factor adaptation (part 1)
   * 
   * Input signals:  AL
   * Output signals: Y
D 3
   * States:         YU, YL
E 3
I 3
   * States:         YU, YL6
E 3
   */
E 12
I 12
	/*
D 13
	 * quantizer scale factor adaptation (part 1)
E 13
I 13
	 * post-state part of the quantizer scale factor adaptation
E 13
	 *
D 14
	 * Input signals:  AL
E 14
I 14
	 * Input signals:  AL, X->YU, X->YL6
E 14
	 * Output signals: Y
D 14
	 * States:         YU, YL6
E 14
	 */
E 12

I 3
D 12
  register STATES* x = X;
E 12
I 12
	register STATES* x = X;
E 12

E 3
D 12
  /* MIX */
  {
D 6
    U16BIT DIF, DIFS, DIFM, PRODM, PROD;
E 6
I 6
    U16BIT DIF, DIFS, DIFM, PRODM;
E 12
I 12
	/* MIX */
	{
		S16BIT DIF;
		U16BIT PRODM;
E 12
E 6

D 2
    DIF = (S->YU + 16384 - ((U16BIT) (S->YL >> 6))) & 16383;
    DIFS = DIF >> 13;
    DIFM = IF_ELSE(DIFS, (16384 - DIF) & 8191, DIF);
    PRODM = (DIFM * AL) >> 6;
    PROD = IF_ELSE(DIFS, (16384 - PRODM) & 16383, PRODM);
E 2
I 2
D 3
    DIF = (S->YU + TC((U16BIT) (S->YL >> 6))) & 16383;
E 3
I 3
D 7
    DIF = (x->YU + TC(x->YL6)) & 16383;
E 7
I 7
D 12
    DIF = x->YU + (-(x->YL6));
E 7
E 3
D 6
    DIFS = SIGNBIT(DIF, 13);
E 6
I 6
    DIFS = NEGATIVE(DIF, 13);
E 6
D 7
    DIFM = IF_ELSE(DIFS, TC(DIF) & 8191, DIF);
E 7
I 7
    DIFM = IF_ELSE(DIFS, (-DIF) & 8191, DIF);
E 7
    PRODM = (U16BIT) (((U24BIT) DIFM * AL) >> 6);
D 5
    PROD = IF_ELSE(DIFS, TC(PRODM) & 16383, PRODM);
E 5
I 5
D 6
    PROD = IF_ELSE(DIFS, TC(PRODM), PRODM);
E 5
E 2
D 3
    Y = (((U16BIT) (S->YL >> 6)) + PROD) & 8191;
E 3
I 3
    Y = (x->YL6 + PROD) & 8191;
E 6
I 6
D 7
    Y = (x->YL6 + (IF_ELSE(DIFS, TC(PRODM), PRODM))) & 8191;
E 7
I 7
    Y = (x->YL6 + (IF_ELSE(DIFS, -PRODM, PRODM))) & 8191;
E 7
E 6
E 3
  }
E 12
I 12
		DIF = x->YU - x->YL6;
		PRODM = (U16BIT) (((U24BIT) ABS(DIF) * AL) >> 6);
		Y = (x->YL6 + (IF_ELSE(DIF < 0, -PRODM, PRODM)));
	}
E 12
}

D 3
static void scale_factor_2(STATES* S)
E 3
I 3
static void scale_factor_2(void)
E 3
{
D 12
  /*
D 7
   * quantizer scale factor adaptation
E 7
I 7
   * quantizer scale factor adaptation (part 2)
E 7
   *
   * Input signals:  I
D 3
   * Output signals: Y, S->YL
E 3
I 3
   * Output signals: Y, X->YL, X->YL6
E 3
   * States:         YL, YU
   */
E 12
I 12
	/*
D 13
	 * quantizer scale factor adaptation (part 2)
E 13
I 13
	 * pre-state part of the quantizer scale factor adaptation
E 13
	 *
D 14
	 * Input signals:  I, Y
	 * Output signals: X->YL, X->YL6
	 * States:         YL, YU
E 14
I 14
	 * Input signals:   I, Y
	 * Output signals:  X->YU, X->YL6
	 * Internal states: X->YL
E 14
	 */
E 12

D 11
  static U16BIT W[] = 
E 11
I 11
D 12
  static const U16BIT W[] = 
E 11
  {
    4084, 18, 41, 64, 112, 198, 355, 1122
  } ;
I 3
  register STATES* x = X;
E 3
D 2
  U16BIT WI, YUT;
E 2
I 2
  U16BIT YUT;
E 12
I 12
	static S16BIT W[] =
	{
		-12, 18, 41, 64, 112, 198, 355, 1122
	} ;
	register STATES* x = X;
	U16BIT YUT;
E 12
E 2

D 2
  /* FUNCTW */
  WI = W[(I >> 3) ? (15 - I) & 7 : I & 7];
  
  /* FILTD */
E 2
I 2
D 12
  /* FUNCTW and FILTD */
E 2
  {
D 2
    U16BIT DIFS, DIFSX;
    U24BIT DIF;
E 2
I 2
    U16BIT DIF, DIFS;
E 12
I 12
	/* FUNCTW and FILTD */
	YUT = Y + (S16BIT) ((((S24BIT) (W[(I < 0 ? ~I : I) & 7]) << 5) -
							  ((S24BIT) Y)) >> 5);
E 12
E 2

D 2
    DIF = ((WI << 5) + ((U24BIT) 131072L) - Y) & ((U24BIT) 131071L);
    DIFS = DIF >> 16;
    DIFSX = DIFS ? (DIF >> 5) + 4096 : DIF >> 5;
    YUT = (Y + DIFSX) & 8191;
E 2
I 2
D 5
    DIF = (U16BIT) (((((U24BIT) (W[(I >> 3) ? (15 - I) & 7 : I & 7]) << 5) +
E 5
I 5
D 12
    DIF = (U16BIT) (((((U24BIT) (W[((I >> 3) ? 15 - I : I) & 7]) << 5) +
E 5
D 7
	   TC((U24BIT) Y)) >> 5) & 4095);
E 7
I 7
	   (-((U24BIT) Y))) >> 5) & 4095);
E 7
D 6
    DIFS = SIGNBIT(DIF, 11);
E 6
I 6
    DIFS = NEGATIVE(DIF, 11);
E 6
    YUT = (Y + (IF_ELSE(DIFS, DIF + 4096, DIF))) & 8191;
E 2
  }
E 12
I 12
	/* LIMB */
	x->YU = YUT < 544 ? 544 : YUT > 5120 ? 5120 : YUT;
E 12

D 12
  /* LIMB */
D 2
  {
    U16BIT GEUL, GELL;
E 2
I 2
D 3
  S->YU = IF_ELSE(SIGNBIT((YUT + 15840) & 16383, 13),
E 3
I 3
D 6
  x->YU = IF_ELSE(SIGNBIT((YUT + 15840) & 16383, 13),
E 3
		  544, IF_ELSE(SIGNBIT((YUT + 11264) & 16383, 13), YUT, 5120));
E 6
I 6
  x->YU = IF_ELSE(NEGATIVE((YUT + 15840) & 16383, 13),
		  544, IF_ELSE(NEGATIVE((YUT + 11264) & 16383, 13),
			       YUT, 5120));
E 12
I 12
	/* FILTE */
	{
		S24BIT DIF;
E 12
E 6
E 2

D 2
    GEUL = ((YUT + 11264) & 16383) >> 13;
    GELL = ((YUT + 15840) & 16383) >> 13;
    S->YU = IF_ELSE(GELL == 1, 544, IF_ELSE(GEUL == 0, 5120, YUT));
  }

E 2
D 12
  /* FILTE */
  {
D 2
    U16BIT DIF, DIFS;
E 2
I 2
    U16BIT DIF;
E 2
    U24BIT DIFSX;

D 2
    DIF = (U16BIT) (S->YU + ((((U24BIT) 1048576L) - S->YL) >> 6)) & 16383;
    DIFS = DIF >> 13;
    DIFSX = IF_ELSE(DIFS, DIF + ((U24BIT) 507904L), DIF);
E 2
I 2
D 3
    DIF = (S->YU + (U16BIT) ((((U24BIT) 1048576L) - S->YL) >> 6)) & 16383;
E 3
I 3
D 7
    DIF = (x->YU + ((U16BIT) (TC(x->YL) >> 6))) & 16383;
E 7
I 7
    DIF = (x->YU + ((U16BIT) ((-(x->YL)) >> 6))) & 16383;
E 7
E 3
D 6
    DIFSX = IF_ELSE(SIGNBIT(DIF, 13), DIF + ((U24BIT) 507904L), DIF);
E 6
I 6
    DIFSX = IF_ELSE(NEGATIVE(DIF, 13), DIF + ((U24BIT) 507904L), DIF);
E 6
E 2
D 3
    S->YL = (S->YL + DIFSX) & ((U24BIT) 524287L);
E 3
I 3
    x->YL6 = (U16BIT) ((x->YL = (x->YL + DIFSX) & ((U24BIT) 524287L)) >> 6);
E 3
  }
E 12
I 12
		DIF = x->YU + ((-(x->YL)) >> 6);
		x->YL6 = (U16BIT) ((x->YL = (x->YL + DIF) & ((U24BIT) 524287L)) >> 6);
	}
E 12
}

D 3
static void speed_control_1(STATES* S)
E 3
I 3
static void speed_control_1(void)
E 3
{
D 12
  /*
   * adaption speed control
   *
   * Input signals:  none
   * Output signals: AL
   * States:         AP
   */
E 12
I 12
	/*
D 13
	 * adaption speed control
E 13
I 13
	 * post-state part of the adaption speed control
E 13
	 *
D 14
	 * Input signals:  none
E 14
I 14
	 * Input signals:  X->AP
E 14
	 * Output signals: AL
D 14
	 * States:         AP
E 14
	 */
E 12

D 12
  /* LIMA */
D 3
  AL = S->AP > 255 ? 64 : S->AP >> 2;
E 3
I 3
  AL = X->AP > 255 ? 64 : X->AP >> 2;
E 12
I 12
	/* LIMA */
	AL = X->AP > 255 ? 64 : X->AP >> 2;
E 12
E 3
}

D 3
static void speed_control_2(STATES* S)
E 3
I 3
static void speed_control_2(void)
E 3
{
D 12
  /*
   * adaption speed control
   *
   * Input signals:  TR, TDP, I, Y
   * Output signals: none
   * States:         AP, DMS, DML
   */
E 12
I 12
	/*
D 13
	 * adaption speed control
E 13
I 13
	 * pre-state part of the adaption speed control
E 13
	 *
D 14
	 * Input signals:  TR, TDP, I, Y
	 * Output signals: none
	 * States:         AP, DMS, DML
E 14
I 14
	 * Input signals:   TR, TDP, I, Y
	 * Output signals:  X->AP
	 * Internal states: DMS, DML
E 14
	 */
E 12

D 2
  static U16BIT F[] = { 0, 0, 0, 1, 1, 1, 3, 7 };
E 2
I 2
D 11
  static U16BIT F[] = { 0, 0, 0, 1 << 9, 1 << 9, 1 << 9, 3 << 9, 7 << 9 };
E 11
I 11
D 12
  static const U16BIT F[] = { 0, 0, 0, 1 << 9, 1 << 9, 1 << 9, 3 << 9, 7 << 9 };
E 11
I 3
  register STATES* x = X;
E 3
E 2
  U16BIT FI, AX, APP;
E 12
I 12
	static U16BIT F[] = { 0, 0, 0, 1 << 9, 1 << 9, 1 << 9, 3 << 9, 7 << 9 };
	register STATES* x = X;
	S16BIT AX, APP, FI;
E 12

D 12
  /* FUNTCF */
  FI = F[IF_ELSE(I >> 3, (15 - I), I) & 7] ; 
E 12
I 12
	/* FUNCTF */
	FI = F[(I < 0 ? ~I : I) & 7] ;
E 12

D 12
  /* FILTA */
  {
D 2
    U16BIT DIF, DIFS, DIFSX;
E 2
I 2
D 5
    U16BIT DIF, DIFSX;
E 5
I 5
    U16BIT DIF;
E 12
I 12
	/* FILTA */
	x->DMS += (FI - x->DMS) >> 5;
E 12
E 5
E 2

D 2
    DIF = ((FI << 9) + 8192 - S->DMS) & 8191;
    DIFS = DIF >> 12;
    DIFSX = DIFS ? (DIF >> 5) + 3840 : DIF >> 5;
E 2
I 2
D 3
    DIF = (FI + TC(S->DMS)) & 8191;
E 3
I 3
D 5
    DIF = (FI + TC(x->DMS)) & 8191;
E 3
    DIFSX = SIGNBIT(DIF, 12) ? (DIF >> 5) + 3840 : DIF >> 5;
E 2
D 3
    S->DMS = (DIFSX + S->DMS) & 4095;
E 3
I 3
    x->DMS = (DIFSX + x->DMS) & 4095;
E 5
I 5
D 7
    DIF = ((FI + TC(x->DMS)) & 8191) >> 5;
E 7
I 7
D 12
    DIF = ((FI - x->DMS) & 8191) >> 5;
E 7
D 6
    x->DMS = ((SIGNBIT(DIF, 7) ? DIF + 3840 : DIF) + x->DMS) & 4095;
E 6
I 6
    x->DMS = ((NEGATIVE(DIF, 7) ? DIF + 3840 : DIF) + x->DMS) & 4095;
E 6
E 5
E 3
  }
E 12
I 12
	/* FILTB */
	x->DML += ((FI << 2) - x->DML) >> 7;
E 12

D 12
  /* FILTB */
  {
D 2
    U16BIT DIF, DIFS, DIFSX;
E 2
I 2
D 5
    U16BIT DIF, DIFSX;
E 5
I 5
    U16BIT DIF;
E 12
I 12
	/* SUBTC */
	AX = (Y >= 1536 && ABS((x->DMS << 2) - x->DML) < (x->DML >> 3) && !TDP) ?
		  0 : 1 << 9;
E 12
E 5
E 2

D 2
    DIF = ((FI << 11) + ((U16BIT) 32768L) - S->DML) & 32767;
    DIFS = DIF >> 14;
    DIFSX = DIFS ? (DIF >> 7) + 16128 : DIF >> 7;
E 2
I 2
D 3
    DIF = ((FI << 2) + TC(S->DML)) & 32767;
E 3
I 3
D 5
    DIF = ((FI << 2) + TC(x->DML)) & 32767;
E 3
    DIFSX = SIGNBIT(DIF, 14) ? (DIF >> 7) + 16128 : DIF >> 7;
E 2
D 3
    S->DML = (DIFSX + S->DML) & 16383;
E 3
I 3
    x->DML = (DIFSX + x->DML) & 16383;
E 5
I 5
D 7
    DIF = (((FI << 2) + TC(x->DML)) & 32767) >> 7;
E 7
I 7
D 12
    DIF = (((FI << 2) - x->DML) & 32767) >> 7;
E 7
D 6
    x->DML = ((SIGNBIT(DIF, 7) ? DIF + 16128 : DIF) + x->DML) & 16383;
E 6
I 6
    x->DML = ((NEGATIVE(DIF, 7) ? DIF + 16128 : DIF) + x->DML) & 16383;
E 6
E 5
E 3
  }
E 12
I 12
	/* FILTC */
	APP = x->AP + ((AX - x->AP) >> 4);
E 12

D 12
  /* SUBTC */
  {
D 2
    U16BIT DIF, DIFS, DIFM, DTHR;
E 2
I 2
    U16BIT DIF, DIFM;
E 2

D 2
    DIF = ((S->DMS << 2) + ((U16BIT) 32768L) - S->DML) & 32767;
    DIFS = DIF >> 14;
    DIFM = IF_ELSE(DIFS, (((U16BIT) 32768L) - DIF) & 16383, DIF);
    DTHR = S->DML >> 3;
    AX = (Y >= 1536 && DIFM < DTHR && TDP == 0) ? 0 : 1;
E 2
I 2
D 3
    DIF = ((S->DMS << 2) + TC(S->DML)) & 32767;
E 3
I 3
D 7
    DIF = ((x->DMS << 2) + TC(x->DML)) & 32767;
E 7
I 7
    DIF = ((x->DMS << 2) - x->DML) & 32767;
E 7
E 3
D 6
    DIFM = IF_ELSE(SIGNBIT(DIF, 14), (((U16BIT) 32768L) - DIF) & 16383, DIF);
D 3
    AX = (Y >= 1536 && DIFM < (S->DML >> 3) && TDP == 0) ? 0 : 1 << 9;
E 3
I 3
    AX = (Y >= 1536 && DIFM < (x->DML >> 3) && TDP == 0) ? 0 : 1 << 9;
E 6
I 6
D 8
    DIFM = IF_ELSE(NEGATIVE(DIF, 14), (((U16BIT) 32768L) - DIF) & 16383, DIF);
E 8
I 8
    DIFM = IF_ELSE(NEGATIVE(DIF, 14),
		   (((U16BIT) 32768L) - DIF) & 16383, DIF);
E 8
    AX = (Y >= 1536 && DIFM < (x->DML >> 3) && !TDP) ? 0 : 1 << 9;
E 6
E 3
E 2
  }

  /* FILTC */
  {
D 2
    U16BIT DIF, DIFS, DIFSX;
E 2
I 2
D 5
    U16BIT DIF, DIFSX;
E 5
I 5
    U16BIT DIF;
E 5
E 2

D 2
    DIF = ((AX << 9) + 2048 - S->AP) & 2047;
    DIFS = DIF >> 10;
    DIFSX = DIFS ? (DIF >> 4) + 896 : DIF >> 4;
E 2
I 2
D 3
    DIF = (AX + TC(S->AP)) & 2047;
E 3
I 3
D 5
    DIF = (AX + TC(x->AP)) & 2047;
E 3
    DIFSX = SIGNBIT(DIF, 10) ? (DIF >> 4) + 896 : DIF >> 4;
E 2
D 3
    APP = (DIFSX + S->AP) & 1023;
E 3
I 3
    APP = (DIFSX + x->AP) & 1023;
E 5
I 5
D 7
    DIF = ((AX + TC(x->AP)) & 2047) >> 4;
E 7
I 7
    DIF = ((AX - x->AP) & 2047) >> 4;
E 7
D 6
    APP = ((SIGNBIT(DIF, 6) ? DIF + 896 : DIF) + x->AP) & 1023;
E 6
I 6
    APP = ((NEGATIVE(DIF, 6) ? DIF + 896 : DIF) + x->AP) & 1023;
E 6
E 5
E 3
  }

  /* TRIGA */
D 3
  S->AP = IF_ELSE(TR, 256, APP);
E 3
I 3
  x->AP = IF_ELSE(TR, 256, APP);
E 12
I 12
	/* TRIGA */
	x->AP = IF_ELSE(TR, 256, APP);
E 12
E 3
}

D 3
static void tone_detector_1(STATES* S)
E 3
I 3
static void tone_detector_1(void)
E 3
{
D 12
  /*
   * tone and transition detector
   *
D 3
   * Input signals:  S->YL, DQ
E 3
I 3
   * Input signals:  X->YL, DQ
E 3
   * Output signals: TR
   * STATES:         YL, TD;
   */
E 12
I 12
	/*
D 13
	 * tone and transition detector
E 13
I 13
	 * post-state part of the tone and transition detector
E 13
	 *
D 14
	 * Input signals:  X->YL, DQ
E 14
I 14
	 * Input signals:  DQ, X->YL6, X->TD
E 14
	 * Output signals: TR
D 14
	 * STATES:         YL, TD;
E 14
	 */
E 12

I 3
D 12
  register STATES* x = X;
E 12
I 12
	register STATES* x = X;
E 12

E 3
D 12
  /* TRANS */
  {
D 2
    U16BIT DQMAG, YLINT, YLFRAC, THR1, THR2, DQTHR;
E 2
I 2
    U16BIT DQMAG, YLINT, YLFRAC, THR2;
E 12
I 12
	/* TRANS */
	{
		U16BIT DQMAG, YLINT, YLFRAC, THR2;
E 12
E 2

D 12
    DQMAG = DQ & 16383;
D 3
    YLINT = (U16BIT) (S->YL >> 15);
    YLFRAC = ((U16BIT) (S->YL >> 10)) & 31;
E 3
I 3
    YLINT = x->YL6 >> 9;
    YLFRAC = (x->YL6 >> 4) & 31;
E 3
D 2
    THR1 = (32 + YLFRAC) << YLINT;
    THR2 = IF_ELSE(YLINT > 8, 31 << 9, THR1);
    DQTHR = (THR2 + (THR2 >> 1)) >> 1;
    TR = DQMAG > DQTHR && S->TD == 1 ? 1 : 0;
E 2
I 2
    THR2 = IF_ELSE(YLINT > 8, 31 << 9, (32 + YLFRAC) << YLINT);
D 3
    TR = S->TD == 1 && DQMAG > ((THR2 + (THR2 >> 1)) >> 1) ? 1 : 0;
E 3
I 3
D 4
    TR = x->TD == 1 && DQMAG > ((THR2 + (THR2 >> 1)) >> 1) ? 1 : 0;
E 4
I 4
D 5
    TR = x->TD == 1 && DQMAG > ((THR2 + (THR2 >> 1)) >> 1);
E 5
I 5
    TR = x->TD && DQMAG > ((THR2 + (THR2 >> 1)) >> 1);
E 5
E 4
E 3
E 2
  }
E 12
I 12
		DQMAG = DQ & 16383;
		YLINT = x->YL6 >> 9;
		YLFRAC = (x->YL6 >> 4) & 31;
		THR2 = IF_ELSE(YLINT > 8, 31 << 9, (32 + YLFRAC) << YLINT);
		TR = x->TD && DQMAG > ((THR2 + (THR2 >> 1)) >> 1);
	}
E 12
}

D 3
static void tone_detector_2(STATES* S)
E 3
I 3
static void tone_detector_2(void)
E 3
{
D 12
  /*
   * tone and transition detector
   *
D 3
   * Input signals:  TR, S->A2
E 3
I 3
D 7
   * Input signals:  TR, x->A2
E 7
I 7
   * Input signals:  TR, x->B[7]
E 7
E 3
   * Output signals: TDP;
   * States:         TD;
   */
E 12
I 12
	/*
D 13
	 * tone and transition detector
E 13
I 13
	 * pre-state part of the tone and transition detector
E 13
	 *
	 * Input signals:  TR, A2P
D 14
	 * Output signals: TDP;
	 * States:         TD;
E 14
I 14
	 * Output signals: TDP, X->TD;
E 14
	 */
E 12
D 2

E 2

D 12
  /* TONE */
D 6
  TDP = ((U16BIT) 32768L) <= A2P && A2P < ((U16BIT ) 53760L) ? 1 : 0;
E 6
I 6
  TDP = ((U16BIT) 32768L) <= A2P && A2P < ((U16BIT ) 53760L);
E 12
I 12
	/* TONE */
	TDP = A2P < -11776 ? 1 : 0;
E 12
E 6

D 12
  /* TRIGB */
D 3
  S->TD = IF_ELSE(TR, 0, TDP);
E 3
I 3
  X->TD = IF_ELSE(TR, 0, TDP);
E 12
I 12
	/* TRIGB */
	X->TD = IF_ELSE(TR, 0, TDP);
E 12
E 3
}

D 6
/***************************** public part ***********************************/
E 6
I 6
/*************************** public part *********************************/
E 6

int   LAW = u_LAW;

void reset_encoder(void)
{
D 3
  int i;

  for (i = 0; i < 6; i++)
    E_STATES.B[i] = 0;
  for (i = 0; i < 6; i++)
    E_STATES.DQ[i] = 32;
  E_STATES.PK1 = E_STATES.PK2 = 0;
  E_STATES.SR1 = E_STATES.SR2 = 32;
  E_STATES.A1 = E_STATES.A2 = 0;
  E_STATES.AP = E_STATES.DMS = E_STATES.DML = 0;
  E_STATES.YU = 544;
  E_STATES.TD = 0;
  E_STATES.YL = (U24BIT) 34816L;
E 3
I 3
D 12
  X = &E_STATES;
  reset_states();
E 12
I 12
	X = &E_STATES;
	reset_states();
E 12
E 3
}

U16BIT encoder(U16BIT pcm)
{
D 12
  S = pcm;
I 3
  X = &E_STATES;
E 12
I 12
	S = pcm;
	X = &E_STATES;
E 12
E 3

D 12
  input_conversion();
D 3
  adpt_predict_1(&E_STATES);
E 3
I 3
  adpt_predict_1();
E 3
  diff_computation();
D 3
  speed_control_1(&E_STATES);
  scale_factor_1(&E_STATES);
E 3
I 3
  speed_control_1();
  scale_factor_1();
E 3
  adapt_quant();
 
  iadpt_quant();
D 3
  tone_detector_1(&E_STATES);
  adpt_predict_2(&E_STATES);
  tone_detector_2(&E_STATES);
  scale_factor_2(&E_STATES);
  speed_control_2(&E_STATES);
E 3
I 3
  tone_detector_1();
  adpt_predict_2();
  tone_detector_2();
  scale_factor_2();
  speed_control_2();
E 12
I 12
	input_conversion();
	adpt_predict_1();
	diff_computation();
	speed_control_1();
	scale_factor_1();
	adapt_quant();
E 12
E 3
D 14

E 14
D 12
  return (I);
E 12
I 12
	iadpt_quant();
	tone_detector_1();
	adpt_predict_2();
	tone_detector_2();
	scale_factor_2();
	speed_control_2();

	return (I & 0xF);
E 12
}

void reset_decoder(void)
{
D 3
  int i;

  for (i = 0; i < 6; i++)
    D_STATES.B[i] = 0;
  for (i = 0; i < 6; i++)
    D_STATES.DQ[i] = 32;
  D_STATES.PK1 = D_STATES.PK2 = 0;
  D_STATES.SR1 = D_STATES.SR2 = 32;
  D_STATES.A1 = D_STATES.A2 = 0;
  D_STATES.AP = D_STATES.DMS = D_STATES.DML = 0;
  D_STATES.YU = 544;
  D_STATES.TD = 0;
  D_STATES.YL = (U24BIT) 34816L;
E 3
I 3
D 12
  X = &D_STATES;
  reset_states();
E 12
I 12
	X = &D_STATES;
	reset_states();
E 12
E 3
}

U16BIT decoder(U16BIT adpcm)
{
I 12
	I = adpcm & 8 ? adpcm | (-1 << 4) : adpcm;
	X = &D_STATES;
E 12

D 12
  I = adpcm;
I 3
  X = &D_STATES;
E 12
I 12
	speed_control_1();
	scale_factor_1();
	iadpt_quant();
	tone_detector_1();
	adpt_predict_1();
	adpt_predict_2();
	tone_detector_2();
	scale_factor_2();
	speed_control_2();
	output_conversion();
	input_conversion();
	diff_computation();
	coding_adjustment();
E 12
E 3

D 3
  speed_control_1(&D_STATES);
  scale_factor_1(&D_STATES);
E 3
I 3
D 12
  speed_control_1();
  scale_factor_1();
E 3
  iadpt_quant();
D 3
  tone_detector_1(&D_STATES);
  adpt_predict_1(&D_STATES);
  adpt_predict_2(&D_STATES);
  tone_detector_2(&D_STATES);
  scale_factor_2(&D_STATES);
  speed_control_2(&D_STATES);
E 3
I 3
  tone_detector_1();
  adpt_predict_1();
  adpt_predict_2();
  tone_detector_2();
  scale_factor_2();
  speed_control_2();
E 3
  output_conversion();
  input_conversion();
  diff_computation();
  coding_adjustment();
E 12
I 12
D 14
/*	put(0, 0);
	put("S", S);     put("B1", X->B[0]);     put("A1", X->B[6]);
	put("SD", SD);   put("B2", X->B[1]);     put("A2", X->B[7]);
	put("SE", SE);   put("B3", X->B[2]);     put("PK1", X->PK1);
	put("SEZ", SEZ); put("B4", X->B[3]);     put("TDP", TDP);
	put("D", D);     put("B5", X->B[4]);     put("TD", X->TD);
	put("AL", AL);   put("B6", X->B[5]);     put("YLh", (int) (X->YL >> 16));
	put("Y", Y);     put("DQ1M", X->DQ[0]);  put("YLl", (int) (X->YL & 0xFFFF));
	put("I", I);     put("DQ1E", X->DQ[1]);  put("YU", X->YU);
	put("DQ", DQ);   put("DQ1S", X->DQ[2]);  put("AP", X->AP);
	put("TR", TR);   put("SR1M", X->DQ[18]); put("DMS", X->DMS);
	put("A2P", A2P); put("SR1E", X->DQ[19]); put("DML", X->DML);
D 13
	put("SR", SR);   put("SR1S", X->DQ[20]); put("S", S);*/
E 13
I 13
	put("SR", SR);   put("SR1S", X->DQ[20]); put("S", S); */
E 13
E 12

E 14
D 12
  return (SD);
E 12
I 12
	return (SD);
E 12
}
I 14




E 14
E 1
