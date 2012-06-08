;**************************************************************************
;
;   ADPCME.ASM      32-kbit/s CCITT ADPCM encoder algorithm
;   Version 1.1 -   17 Dec 1993
;
;   This product has been developed at Motorola India Electronics Pvt. Ltd.,
;   Bangalore, INDIA.
;
;   This program implements the algorithm defined in CCITT Recommendation
;   G.721: "32 kbit/s Adaptive Differential Pulse Code Modulation" dated
;   August 1986. It passes all of the Mu-law and A-law test sequences as
;   defined in Appendix II of Recommendation G.721.
;
;   Please refer to the file ADPCM.HLP for further information about
;   this program.
;
;   Version History:
;             Version 1.0       :    28 May 1993
;             Version 1.1       :    17 Dec 1993
;
;       In version 1.1 intermodular optimization was done to reduce the
;       execution time. 
;
;**************************************************************************

           ORG     X:$0000

;************************ X MEMORY CONSTANTS *********************************

RSHFT        DS    27         ;Table with constants for right shifts 1..15
FM_CONS      DS    8          ;Table with constants for FMULT
QUANTAB      DS    8          ;Quantization table used in QUAN
IQUANTAB     DS    8          ;Inverse quantization table used in RECONST
FIBASE       DS    8          ;Table with FI values used in FUNCTF
WIBASE       DS    8          ;Table with WI values used in FUNCTW
SEGTAB       DS    8          ;Table with segment code for COMPRESS  

CONST        DS    35         ;Table of constants. R2 used as pointer
CONST_TRANS  DS    1          ;Table of constants. R2 used as pointer
CONST_ADDC   DS    41         ;Table of constants. R2 used as pointer
DUMMY        DS    1          ;Dummy location (should contain $BABA)
                              ;  used to check for complete constant
                              ;  initialization in X memory.

;************************ X MEMORY VARIABLES **********************************
;************************ ENCODER VARIABLES ***********************************
;----------------------------------------------------------------------------
;  8 word buffer for predictor coefficients B1..B6 A1,A2.
;  R3 is used as pointer.

COEF_T     DS      8
;----------------------------------------------------------------------------
;  24 word modulo buffer for data DQ1..DQ6 SR1,SR2.
;  R1 is used as a pointer.

DATA_T     DSM     24         ;DQ1EXP,DQ1MANT,DQ1S
                              ;DQ2EXP,DQ2MANT,DQ2S
                              ;DQ3EXP,DQ3MANT,DQ3S
                              ;DQ4EXP,DQ4MANT,DQ4S
                              ;DQ5EXP,DQ5MANT,DQ5S
                              ;DQ6EXP,DQ6MANT,DQ6S
                              ;SR1EXP,SR1MANT,SR1S
                              ;SR2EXP,SR2MANT,SR2S
;----------------------------------------------------------------------------
;  8 word buffer for partial products WB1..WB6 WA1,WA2

PP_T       DS      8
;----------------------------------------------------------------------------
I_T        DS      1          ;ADPCM codeword
SEZ_T      DS      1          ;Partial signal estimate
SR_T       DS      1          ;Reconstructed signal
PK_T       DS      2          ;Sign of DQ+SEZ
                              ;Delayed sign of DQ+SEZ
DMS_T      DS      4          ;Short term average of FI
                              ;Long term average of FI
                              ;Tone detect
                              ;Unlimited speed control factor
TD_T       DS      1          ;Delayed tone detect
YU_T       DS      3          ;Fast quantizer scale factor
                              ;Slow quantizer scale factor (2 words)
TR_T       DS      1          ;Transition detect
DATAPTR_T  DS      1          ;Transmit data buffer pointer
SS_T       DS      1          ;PCM word

IMAG       DS      1          ;|I|
DQI        DS      1          ;DQ in 15TC
LAW        DS      1          ;Switch for A-law(!=0) /Mu-law(= 0)   
;********************* PROGRAM STARTS **************************************   

;********************** ENCODER LOOP STARTS *************************************
        ORG     P:$300 

ENCODER
        MOVE    X:DATAPTR_T,R1         ;Set pointer to DQn buffer
        MOVE    #PP_T,R2               ;Set pointer to partial product buffer
        MOVE    #COEF_T,R3             ;Set pointer to Bn buffer

;******************************************************************************
;
;  Module Name     :  FMULT
;  Module Number   :  VII.1.3
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;    SRn or DQm  = (11FL)                               n=1,2 ,m=1,2,...6
;    DQmEXP   = | 0000 0000 | 0000 eeee |  in X:(R1+3*(m-1))     m=1,2,..,6
;    DQmMANT  = | 01mm mmm0 | 0000 0000 |  in X:(R1+3*(m-1)+1)   m=1,2,..,6
;    DQmS     = | sxxx xxxx | xxxx xxxx |  in X:(R1+3*(m-1)+2)   m=1,2,..,6
;
;    SRnEXP   = | 0000 0000 | 0000 eeee |  in X:(R1+18+3*(n-1))     n=1,2
;    SRnMANT  = | 01mm mmm0 | 0000 0000 |  in X:(R1+18+3*(n-1)+1)   n=1,2
;    SRnS     = | sxxx xxxx | xxxx xxxx |  in X:(R1+18+3*(n-1)+2)   n=1,2
;
;    Bm = | si.ff ffff | ffff ffff | (16TCa) in X:COEF_T+(m-1)      m=1,2,...,6
;    An = | si.ff ffff | ffff ffff | (16TCa) in X:COEF_T+6+(n-1)    n=1,2
;    R3 points to COEF_T (B1)
;
; Output:
;   WBm = siii iiii | iiii iii.f (16TCc) in X:PP_T+(m-1)     m=1,2,...,6
;   WAn = siii iiii | iiii iii.f (16TCc) in X:PP_T+6+(n-1)   n=1,2
;   R3 points to COEF_T+8 
;
;************************* MODULE DESCRIPTION *********************************
;
;    Multiply predictor coefficients, bm(k-1) (m=1,2,...,6) or an(k-1) (n=1,2)
;    with corresponding  quantized  difference  signal,  dq(k-m) (m=1,2,...,6)
;    or reconstructed  signal, sr(k-n) (n=1,2).   Multiplication  is  done 
;    using floating point arithmetic.
;
;    Computes  wbm(k-1) = bm(k-1) * dq(k-m)   m=1,2,...,6
;              wan(k-1) = an(k-1) * sr(k-n)   n=1,2
;
;    SYMBOLS USED
;              sr(k-n)   <==>  SRn            n=1,2
;              an(k-1)   <==>  An             n=1,2
;              dq(k-m)   <==>  DQm            m=1,2,...,6
;              bm(k-1)   <==>  Bm             m=1,2,...,6
;              wan(k-1)  <==>  WAn            n=1,2
;              wbm(k-1)  <==>  WBm            m=1,2,...,6
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE **********************
;
;  There are eight floating point multiplications to produce eight outputs.
;  All the multiplications are of numbers in same formats (11FL and 16TCa)
;  and the outputs are in 16TCc format.
;  Here one multiplication is described. The same process has to be repeated
;  eight times.
;
; Flow Description:
;   1. Convert 16TCa An to 13SM AnMAG=|An|
;   2. Convert 13SM AnMAG to 10FL (AnEXP and AnMANT)
;   3. Add SRnEXP to AnEXP to get WAnEXP (5-bits)
;   4. Multiply SRnMANT with AnMANT, add '48', and truncate
;       to 8-bits to get WAnMANT
;   5. Convert 13FL (WAnEXP & WAnMANT) to 15SM WAnMAG
;   6. XOR AnS with SRnS to find WAnS
;   7. Convert 16SM (WAnMAG & WAnS) to 16TCc WAn
;
; Each of the above steps are described below.
;
; Step 1. Convert 16TCa An to 13SM AnMAG (Actually AnMAG*4 is in 13 SM)
;         ----------------------------- 
;      Get An and truncation mask for 13 SM format.
;      Truncate An to 14 bits.
;      Find AnMAG=|An| 
;
; Step 2. Convert  AnMAG (where AnMAG*4 is in 13 SM) to 10FL (AnEXP and AnMANT)
;         ---------------------------------------------------------------------
;         If AnMAG = 0 then
;             Set AnMANT = 1<<5 and AnEXP = 0 
;         Else 
;             Normalize the number i.e. Bring 1 to MSB position (excluding sign
;             bit position) and count the number of shifts. 
;             (13 - number of shifts required) is AnEXP.  Actually AnEXP  has a
;             bias of 12. This bias is used to keep AnEXP positive.  Normalized
;             number is trucated to 6 bits to get AnMANT.
;
; Step 3. Add SRnEXP to AnEXP to get WAnEXP 
;         ---------------------------------
;       Get SRnEXP from memory.
;       Add AnEXP to SRnEXP.
;       
; Step 4. Multiply SRnMANT with AnMANT, add '48', and truncate
;       to 8 bits to get WAnMANT
;     
;      (Note: Adding '48' amounts to adding 1 to 7th bit and 8th bit of 
;             mantissa.  
;             mantissa = 0mmm mmmm mmmm m000 , SRnMANT * AnMANT leads to 
;                                              12 bits of mantissa excluding
;                                              sign bit.
;             '48'     = 0000 0001 1000 0000 )
;     
;         Find WAnMANT = SRnMANT * AnMANT + 48
;         Truncate WAnMANT to 8 bits.
;   
; Step 5. Convert 13FL (WAnEXP & WAnMANT) to 15SM WAnMAG
;         ----------------------------------------------
;
;         Find shift offset WAnEXP-26.
;         26 is subtracted as 12 was the bias in AnEXP (to keep it positive)
;         and WAnMANt has to be shifted to the right by 14 places to get WAnMAG
;         in 15 SM format.
;         If offset>0 then shift left offset times.
;         else if offset<0 shift right |offset| times.
;
; Step 6. XOR AnS with SRnS to find WAnS
;         ------------------------------
;         Get AnS and SRns from memory.
;         Find WAnS = AnS XOR SRnS
;
; Step 7. Convert 16SM (WAnMAG & WAnS) to 16TC WAn
;         ----------------------------------------
;         If WAnS = 0 then WAn = WAnMAG
;         else WAn = -WAnMAG
;               
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 320
; NLOAC       : 50
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   c    -    -   c  c  c  c
;           1   c    -    -   c  c  c  c
;           2   c    -    -   c  c
;           3   c    -    -
;                         
;         -  = not used   u = used & unchanged  c = used & changed
;
;*****************************************************************************
FMULT_T
        MOVE    #FM_CONS+6,R0           ;Set constant pointer
        MOVE    X:(R3),A                ;Get An
        MOVE    X:(R0)+,Y1              ;Get truncation mask ($7FFC) for 13 SM
        AND     Y1,A                    ;Truncate An to 14 bits. (An>>2)
        DO      #8,END_FMULT_T          ;Execute multiplication loop 8 times
;-----------------------------------------------------------------------------
; 1. Convert 16TC An to 13SM AnMAG
;-----------------------------------------------------------------------------
        ABS     A      X:(R0)+,X1       ;Find AnMAG, Get exponent bias 
        AND     Y1,A   X:(R1)+,X0       ;Truncate overflow, if any
                                        ;  Get SRnEXP in X0
        MOVE    X1,R0                   ;Load exponent bias (9) into R0.
;-----------------------------------------------------------------------------
;   A1 = 0i.ff ffff ffff ff00 (A2=A0=0) 
;-----------------------------------------------------------------------------
; 2. Convert 13SM AnMAG to 10FL (AnEXP and AnMANT)
;-----------------------------------------------------------------------------
        BNE     <NORMAN_T               ;Test AnMAG
        MOVE    #$4000,A                ;If AnMAG=0, set AnMANT=1<<5,
        MOVE    #0,R0                   ;  and AnEXP=0
        BRA     <CONTIN_T
NORMAN_T 
        ASL4    A                       ;This ASL4 and 8 NORMs to
        NORM    R0,A                    ;  find MSB
        NORM    R0,A                    
        NORM    R0,A
        NORM    R0,A
        NORM    R0,A
        NORM    R0,A
        NORM    R0,A
        NORM    R0,A
;--------------------------------------------------------------------------
;   A1 = 01?? ???? ???? ??00 = AnMANT
;   R0 = 0000 0000 0000 eeee = AnEXP
;--------------------------------------------------------------------------
CONTIN_T  
        MOVE     R0,B               ;Move AnEXP to B
        MOVE     #FM_CONS,R0        ;Pointer to the constants
        ADD      X0,B    X:(R1)+,Y0 ;Find WAnEXP =SRnEXP+AnEXP,
                                    ;  get SRnMANT in Y0
        TFR      B,X0    X:(R0)+,X1 ;Save WAnEXP in X0
                                    ;  Get mask for truncating AnMANT to 
                                    ;  6 bits.($7E00)
;--------------------------------------------------------------------------
;   B1 = 0000 0000 000e eeee (B2=B0=0)
;--------------------------------------------------------------------------
        AND      X1,A    X:(R0)+,B  ;Truncate AnMANT to 6 bits,
                                    ;  get 48 in B ($0180)
;--------------------------------------------------------------------------
;   A1 = 01mm mmm0 0000 0000 (A2=A0=0)
;--------------------------------------------------------------------------
;--------------------------------------------------------------------------
; 4. Multiply SRnMANT with AnMANT, add '48', and truncate
;       to 8-bits to get WAnMANT
;   A1 = 01mm mmm0 0000 0000 
;   Y0 = 01mm mmm0 0000 0000 
;   B1 = 0000 0001 1000 0000  = '48'
;--------------------------------------------------------------------------
        MAC     A1,Y0,B    X:(R0)+,X1 ;Find WAnMANT, 
                                      ;  get mask for truncating WAnMANT to
                                      ;  8 bits($7F80)
        AND     X1,B       X:(R0)+,A  ;Truncate WAnMANT to 8 bits,
                                      ;  get '26' ($001A)
;--------------------------------------------------------------------------
;   B1 = 0mmm mmmm m000 0000 (B2=B0=0)
;--------------------------------------------------------------------------
;--------------------------------------------------------------------------
; 5. Convert 13FL (WAnEXP & WAnMANT) to 15SM WAnMAG
;   A1 =  0000 0000 0001 1010 = '26'
;   X0 =  0000 0000 000e eeee 
;--------------------------------------------------------------------------
        SUB     X0,A     X:(R0)+,Y0 X:(R3)+,X1
                                    ;Find shift offset -(WAnEXP-26),
                                    ;  get truncation mask for 15SM ($7FFF)
                                    ;  get AnS
        BLE     <LEFT_T             ;If offset<=0 do left shifts
                                    ;Else offset>0, do right shifts
        MOVE    #RSHFT,X0           ;Get base of rshift table.
        ADD     X0,A                ;Get the address of correct right shift
        MOVE    X:(A1),X0           ;Get the shift constant
        MPY     X0,B1,B             ;Shift WAnMANT right offset times
                                    ; upto 26 times.
        BRA     <TRUNC_T
LEFT_T  ABS     A                   ;Find |offset|
        REP     A1                  ;Shift WAnMANT
        LSL     B                   ; left |offset| times
;--------------------------------------------------------------------------
;   B1 = ?mmm mmmm mmmm mmmm  
;--------------------------------------------------------------------------
TRUNC_T  
        AND     Y0,B    X:(R1)+,A   ;Truncate WAnMAG to 15 SM format.
                                    ;  get SRnS
        MOVE    X:(R0)+,B0          ;Remove extra bits of WAnMAG by zeroing B0
;--------------------------------------------------------------------------
; B1 = 0iii iiii | iiii iii.f (B2=B0=0)
;--------------------------------------------------------------------------
        NEG     B   B,Y0            ;Find -WAnMAG, save WAnMAG
;--------------------------------------------------------------------------
; 6. XOR AnS with SRnS to find WAnS
;   X1 = sxxx xxxx xxxx xxxx 
;   A1 = sxxx xxxx xxxx xxxx 
;--------------------------------------------------------------------------
        EOR     X1,A   X:(R0)+,X0   ;Find WAnS (in MSB of A),
                                    ;  Dummy read to advance pointer
;--------------------------------------------------------------------------
; 7. Convert 16SM (WAnMAG & WAnS) to 16TC WAn
;--------------------------------------------------------------------------
        TPL     Y0,B                ;If WAnS=0 use WAnMAG, else use -WAnMAG
;--------------------------------------------------------------------------
;   B1 = siii iiii iiii iii.f (B2=sign ext, B0=0)
;--------------------------------------------------------------------------
        MOVE    X:(R3),A                ;Get An
        AND     Y1,A   B,X:(R2)+        ;Truncate An to 14 bits. (An>>2)
                                        ;Save WAn
END_FMULT_T

;******************************************************************************
;
;  Module Name     :  ACCUM  
;  Module Number   :  VII.1.4
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   WBm = | siii iiii | iiii iii.f |  (16TCc) in X:PP_T+(m-1)     m=1,2,..,6
;   WAn = | siii iiii | iiii iii.f |  (16TCc) in X:PP_T+6+(n-1)   n=1,2
;
; Output:
;   SEZ = | siii iiii | iiii iii.0 |  (15TC) in X:SEZ_T
;   SE  = | siii iiii | iiii iii.0 |  (15TC) in Y1
;
;***************************** MODULE DESCRIPTION ***************************
;
;   Add predictor outputs to form the partial signal estimate
;   and the signal estimate
;
;   sez(k) = wb1(k-1) + wb2(k-1) + ... + wb6(k-1)
;   se(k)  = sez(k)   + wa1(k-1) + wa2(k-1)
;
;   SYMBOLS USED :
;                 wbm(k-1)  <==>  WBm      m=1,2,...,6
;                 wan(k-1)  <==>  WAn      n=1,2
;                 sez(k)    <==>  SEZ      
;                 se(k)     <==>  SE
;
;******************** STRUCTURED ENGLISH OR PSEUDO CODE *******************
;    
;      Get WB1 & WB2.
;      Add WB1 to WB2 and get WB3.
;      Accumulate WBm and get WBp (p=m+1) for m = 3,4,5.
;      Accumulate WB6 to get SEZI and get WA1.
;      Save SEZI and truncate it to 15TC format.
;      Accumulate WA1 and get WA2.
;      Accumulate WA2 to get SEI.
;      Truncate SEI to 15 TC format.
;      Save SEZI and SEI in memory.
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 19
; NLOAC       : 19
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   c  c  c  -
;           1   -    -    -   c  c  c  c
;           2   c    -    u   c  c
;           3   -    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;*****************************************************************************
ACCUM_T 
        MOVE    (R2)+N2             ;Make R2 point to WB1 (N2 = -8)
        MOVE    X:(R2)+,A           ;Get WB1
        MOVE    X:(R2)+,X0          ;Get WB2
        ADD     X0,A    X:(R2)+,X0  ;Accumulate WB1 & WB2, get WB3
        ADD     X0,A    X:(R2)+,X0  ;Accumulate WB3, get WB4
        ADD     X0,A    X:(R2)+,X0  ;Accumulate WB4, get WB5
        ADD     X0,A    X:(R2)+,X0  ;Accumulate WB5, get WB6
        ADD     X0,A    X:(R2)+,X0  ;Accumulate WB6 to get SEZI, get WA1
        TFR     A,B                 ;Copy SEZI
        MOVE    #<-$02,X1           ;Get trunc mask for 15TC format
        AND     X1,B                ;Truncate SEZI to 15 TC
        ADD     X0,A    X:(R2)+,X0  ;Accumulate WA1, get WA2
        MOVE    #CONST,R2           ;Initialize R2 to point to constant table
        ADD     X0,A                ;Accumulate WA2 to get SEI
        MOVE    B1,X:SEZ_T          ;Save SEZ
        AND     X1,A    X:(R2)+,X0  ;Truncate SEI to 15 TC
                                    ;  get `1.0' in 10SM format ($2000)
                                    ;  for LIMA
        MOVE    A,Y1                ;Save SE
END_ACCUM_T

;******************************************************************************
;
;  Module Name     :  LIMA  
;  Module Number   :  V.6
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   AP = | 0ii.f ffff | fff0 0000 |   (10SM)  in  X:DMS_T+3
;
; Output:
;   AL = | 0i.ff ffff | 0000 0000 |   (7SM)   in  B1
;
;***************************** MODULE DESCRIPTION ****************************
;
;       This module limits the unlimited speed control parameter, ap(k),
;       to its limited version, al(k), according to the equation :
;              al(k) = 1          if ap(k-1) > 1
;                    = ap(k-1)    if ap(k-1) <= 1
;
;       SYMBOLS USED :
;                   al(k-1)  <==>   AL
;                   ap(k)    <==>   AP
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;               
;                       Get AP from memory.
;                       If AP > 1
;                          set AL to 1.
;                       Else 
;                          set AL to AP.
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 6
; NLOAC       : 7
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   -  c  c  c
;           1   -    -    -   -  c  c  -
;           2   c    -    -   -  c  
;           3   -    -    -
;
;         -  = not used   u = used & unchanged  c = used & changed
;
;**************************************************************************
LIMA_T
        MOVE    X:DMS_T+3,B         ;Get AP from memory
        CMP     X0,B   X:(R2)+,X1   ;Test if AP>1.
                                    ;   get the mask $7F00 (for 7SM).
        TGT     X0,B                ;If AP>1 set AL=1, else AL=AP.
        ASL     B       X:(R2)+,Y0  ;Shift left to align binary point of AP
                                    ;  to AL
                                    ;  get mask $7FFC (to truncate -> 13SM)
                                    ;  for use in MIX
        AND     X1,B   X:(R2)+,X0   ;Truncate AL to 7SM format.
                                    ;  Get mask $FFFC (to truncate to 14 
                                    ;  bits) for use in MIX
END_LIMA_T

;******************************************************************************
;
;  Module Name     :  MIX
;  Module Number   :  VI.6
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   YL = | 0iii i.fff | ffff ffff | ffff 0000 | 0000 0000 | 
;        |<------- X:YU_T+1------>|<------ X:YU_T+2 ----->|
;                                   (19SM)  in ( X:YU_T +1 | X:YU_T+2 )
;
;   AL = | 0i.ff ffff | 0000 0000 |   (7SM)   in  B1
;   YU = | 0iii i.fff | ffff ff00 |   (13SM)  in  X:YU_T
;
; Output:
;    Y = | 0iii i.fff | ffff ff00 |   (13SM)   in  N3 (for SUBTB, ADDA,
;                                                          SUBTC & FILTD)
;
;***************************** MODULE DESCRIPTION *****************************
;
;      In this module, the overall quantizer scale factor, y(k), is 
;      formed as a linear combination of the fast and slow quantizer scale 
;      factors ( yu(k-1) and yl(k-1) ) according to the equation :
;         y(k)   =   al(k) * yu(k-1) + [1-al(k)] * yl(k-1)
;                =   al(k) * [ yu(k-1) - yl(k-1) ] + yl(k-1)
;
;      SYMBOLS USED :
;                    y(k)    <==>   Y
;                    al(k)   <==>   AL
;                    yu(k-1) <==>   YU
;                    yl(k-1) <==>   YL
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;               Compute the difference DIF = YU - YL.
;               Find DIFM = |DIF|, save the sign of DIF.
;               Form the product  PRODM = DIFM * AL.
;               If DIF<0
;                  PROD = -PRODM.
;               Else
;                  PROD = PRODM.
;               Shift PROD left once to bring it to the correct format.
;               Add YL to the product.
;               
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 17
; NLOAC       : 17
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   c  c  c  c
;           1   -    -    -   c  c  c  -
;           2   c    -    -   c  c
;           3   -    -    c
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;**************************************************************************
MIX_T
        MOVE    X:YU_T+1,A          ;Only the higher order bits of YL need
                                    ;  to be transferred since YL will be
                                    ;  truncated anyway; A0 is zero-filled.
        AND     Y0,A    B,Y0        ;Truncate YL to 13SM format
                                    ;  save AL in Y0
        MOVE    X:YU_T,B            ;Get YU.
        SUB     B,A     A,X1        ;Find -DIF=YL-YU, save YL.
        AND     X0,A    X:(R2)+,X0  ;Truncate DIF to 14 bits,
                                    ;  get mask $7FFE (to truncate to 14 bits).
        ABS     A       A,B         ;Find DIFM=|DIF|, save DIFS.
        MPY     A1,Y0,A             ;Find PRODM=DIFM*AL.
        AND     X0,A                ;Truncate to get 14 bits; 
                                    ;   a later ASL will make it 13SM.
        MOVE    A1,A                ;Zero fill A0
        NEG     A   A,X0            ;Find -PRODM, save PRODM.
        TST     B                   ;Check DIFS for sign.
        TLE     X0,A                ;If DIFS>0 PROD=PRODM,
                                    ;   else PROD=-PRODM.
        ASL     A                   ;Align binary point of PROD with 
                                    ;  that of YL.
        ADD     X1,A                ;Find Y=PROD+YL
        MOVE    A,N3                ;Save Y in N3.
END_MIX_T

;******************************************************************************
;
;  Module Name     :  EXPAND
;  Module Number   :  I
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   LAW = zero or nonzero                             in X:LAW 
;   S   = | psss qqqq | 0000 0000 |  (PCM log format) in X:SS_T
;
; Output:
;   SL =  | siii iiii | iiii ii.00 | (14TC) in A1
;        For Mu-law the 14 MSBs & for A-law 13MSBs of A1 contain the PCM
;        linear output
;
;***************************** MODULE DESCRIPTION *****************************
;
;   Convert from Mu-law/A-law log PCM to Mu-law/A-law linear PCM 
;   (according to Recommendation G.711/G.712).
;
;   The conversion for Mu-law can be summarized in the following table :
;
;         |=================||===========================|
;         | COMPRESSED CODE ||    BIASED LINEAR CODE     |
;         |     S    Q      ||   (magnitude: 13 bits)    |
;         |=================||===========================|
;         |    000  wxyz    ||     0 0000 001w xyz1      |
;         |-----------------||---------------------------|
;         |    001  wxyz    ||     0 0000 01wx yz10      |
;         |-----------------||---------------------------|
;         |    010  wxyz    ||     0 0000 1wxy z100      |
;         |-----------------||---------------------------|
;         |    011  wxyz    ||     0 0001 wxyz 1000      |
;         |-----------------||---------------------------|
;         |    100  wxyz    ||     0 001w xyz1 0000      |
;         |-----------------||---------------------------|
;         |    101  wxyz    ||     0 01wx yz10 0000      |
;         |-----------------||---------------------------|
;         |    110  wxyz    ||     0 1wxy z100 0000      |
;         |-----------------||---------------------------|
;         |    111  wxyz    ||     1 wxyz 1000 0000      |
;         |-----------------||---------------------------|
;
;         The unbiased linear code is obtained by subtacting the bias (33) from
;         the biased linear code.
;         The unbiased linear code is converted to 2's complement (SL) on the
;         basis of the sign bit.
;
;   The conversion for A-law can be summarized in the following table :
;
;         |=================||===========================|
;         | COMPRESSED CODE ||       LINEAR CODE         |
;         |     S    Q      ||   (magnitude: 12 bits)    |
;         |=================||===========================|
;         |    000  wxyz    ||     0000 000w xyz1        |
;         |-----------------||---------------------------|
;         |    001  wxyz    ||     0000 001w xyz1        |
;         |-----------------||---------------------------|
;         |    010  wxyz    ||     0000 01wx yz10        |
;         |-----------------||---------------------------|
;         |    011  wxyz    ||     0000 1wxy z100        |
;         |-----------------||---------------------------|
;         |    100  wxyz    ||     0001 wxyz 1000        |
;         |-----------------||---------------------------|
;         |    101  wxyz    ||     001w xyz1 0000        |
;         |-----------------||---------------------------|
;         |    110  wxyz    ||     01wx yz10 0000        |
;         |-----------------||---------------------------|
;         |    111  wxyz    ||     1wxy z100 0000        |
;         |-----------------||---------------------------|
;
;         The linear code is converted to 2's complement (SL) on the basis
;         of the sign bit.
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;  
;   Get LAW
;   If LAW=0 then {for Mu-law}
;     begin 
;        Invert PCM word
;        Get the quantizer level qqqq << 1 in 13 SM
;        Add the bias of 33 in 13 SM
;        Get the segment number sss
;        Shift the biased qqqq left sss times
;        Subtract the bias 33 in 13 SM
;        Negate the unbiased number if PCM word is negative
;     end
;   Else  {for A-law}
;     begin
;       Find SERB = even bits (0th,2nd,4th and 6th bit) inverted of PCM word
;       Find SEG  = segment code from SERB
;       Find QQ   = quantizer level qqqq<<2 
;       If (SEG = 0 )
;        begin
;          Find |SL| = QQ + 2 in 13SM format
;        end
;       Else
;        begin
;          Find LCODE = QQ+ 66 in 13SM format
;          Find |SL| = LCODE << (SEG-1)
;        end
;       If S>=0 then 
;          SL = |SL|
;       else
;          SL = -|SL|
;     end
;       
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 25
; NLOAC       : 43
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   c  c  c  c 
;           1   -    -    -   c  c  c  -
;           2   c    -    -   c  c
;           3   -    -    -
;
;         -  = not used   u = used & unchanged  c = used & changed
;
;**************************************************************************
EXPAND_T
        MOVE   X:SS_T,A               ;Get PCM word from memory
        MOVE   X:LAW,B                ;Get LAW. LAW = 0 means
                                      ;  mu-law, otherwise A-law
        TST    B        X:(R2)+,X1    ;Check LAW, get mask for 
                                      ;  even bit inversion ($5500) for A-law
        BNE    <A_LAW_EXP_T           ;If LAW != 0, go to A-law EXPAND
        NOT    A        X:(R2)+,B     ;Invert PCM bits
                                      ;  get mask ($7000) for sss
        EXT    A                      ;Sign extend A after NOT operation
        MOVE   A,X1     X:(R2)+,X0    ;Save the PCM word
        AND    X0,A     X:(R2)+,X0    ;Mask out psss
                                      ;  get constant ($0008) for sss >> 12
        AND    X1,B     X:(R2)+,Y0    ;Get sss by masking,
                                      ;  dummy read to skip $0108 used in A-law
        MPY    X0,B1,B  X:(R2)+,Y0    ;Shift sss to 3 LSBs
                                      ;  get constant ($0400) for qqqq >> 5
        MPY    Y0,A1,A  B,Y0          ;Shift qqqq to get qqqq000, save sss
        MOVE   #RSHFT+15,B            ;Set pointer to shift table for left
                                      ;  shifts
        SUB    Y0,B     X:(R2)+,X0    ;Adjust pointer to right value for
                                      ;  left shift
                                      ;  get bias 33 in 13 SM ($0084) 
        EOR    X0,A     X:(B1),Y0     ;XOR 33 with qqqq000 to give 1qqqq100   
                                      ;  get shift constant for left shift 
                                      ;  by segment number sss
        IMPY   Y0,A1,A                ;Shift 1qqqq100 left by sss to give
                                      ;  biased linear code
        SUB    X0,A     (R2)-         ;Subtract bias to give unbiased output
        NEG    A        A,X0          ;Negate A for possible sign inversion if
                                      ;  PCM word is negative, save positive
                                      ;  version
        TST    X1                     ;Check sign bit of PCM word        
        TPL    X0,A                   ;Transfer positive version of linear code
                                      ;  if PCM word is positive
        BRA    <END_EXPAND_T
A_LAW_EXP_T
        EOR    X1,A     X:(R2)+,X0    ;Get SEBR by inverting even bits of S
                                      ;  get mask ($7000) for sss
        AND    X0,A     A,X1          ;Get 0sss in 4 MSBs in A1
                                      ;  save SEBR in X1
        MOVE   X:(R2)+,B              ;Get mask ($0F00) for qqqq
        AND    X1,B     X:(R2)+,Y0    ;B1 = 0000 qqqq 0000 0000
                                      ;  get shift constant for >>12($0008)
        ASR4   B                      ;Find QQ in B1 = 0000 0000 qqqq 0000
        MPY    Y0,A1,A  B,Y0          ;Find SEG= sss in 3 LSBs of A1
                                      ;  save QQ from B1 to Y0
        MOVE   X:(R2)+,X0             ;Get addition constant ($0108)
                                      ;  for sss != 0
        BGT    <SSS_T                 ;If sss != 0, go to SSS_T, else continue
        MOVE   #$0008,A               ;Get addition constant in A for sss=0
        ADD    Y0,A     X:(R2)+,X0    ;Get |SL| in A
                                      ;  dummy read to advance pointer
        BRA    <SLMOD_T               ;Go to find SL
SSS_T   MOVE   #RSHFT+16,B            ;Get base of shift tables, now B1-1
                                      ;  points to zero left shift
        SUB    A,B     X0,A           ;Get correct left shift constant pointer
                                      ;  RSHIFT + 15 - n points to constant for
                                      ;  left shift of n. Left shift of sss - 1
                                      ;  is needed here.
                                      ;  get constant for addition in A for the
                                      ;  case SEG!=0 ($0108) 
        MOVE   X:(R2)+,X0             ;Dummy read to advance pointer
        ADD    Y0,A    X:(B1),Y0      ;Find LCODE = QQ + (66 in 13SM format)
                                      ;  A1 = 0000 0001 qqqq 1000
                                      ;  get apprpriate left shift constant
        IMPY   Y0,A1,A                ;Find |SL|
SLMOD_T
        NEG    A      A,X0            ;Find -|SL| in A, save |SL| in X0
        TST    X1                     ;Check sign of S
        TMI    X0,A                   ;If sign bit = 1 SL = |SL|
                                      ;  else SL = - |SL|
                                      ;  Note that sign bit of S is actually 
                                      ;  inverted.
END_EXPAND_T

;******************************************************************************
;
;  Module Name     :  SUBTA  
;  Module Number   :  II
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;       SL = | siii iiii | iiii ii.00 | (14 TC)  in A1
;       SE = | siii iiii | iiii iii.0 | (15 TC)  in Y1
;
; Output:
;       D  = | siii iiii | iiii iiii |  (16 TCb) in A1
;
;***************************** MODULE DESCRIPTION ****************************
;
;    This module computes the difference signal d(k) by subtracting
;    signal estimate se(k) from input signal sl(k)
;
;               d(k) = sl(k) - se(k)
;
;   SYMBOLS USED :
;                 sl(k)  <==>  SL
;                 se(k)  <==>  SE
;                 d(k)   <==>  D
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;               
;               Find D = SL - SE
;               Get D in 16 TCb
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 3
; NLOAC       : 5
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   c  c  c  -   
;           1   -    -    -   c  c  -  u   
;           2   c    -    -   c  c  
;           3   -    -    -
;
;         -  = not used   u = used & unchanged  c = used & changed
;
;******************************************************************************
SUBTA_T
           ASR     A  X:(R2)+,X0 ;Get SL in 15TC format
                                 ;Dummy read to adjust R2
           SUB     Y1,A          ;Find D = SL - SE
           ASR     A             ;Get D in 16 TCb format
END_SUBTA_T

;******************************************************************************
;
;  Module Name     :  LOG
;  Module Number   :  III.1
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   D  = | siii iiii  | iiii iiii | (16TCb) in A1
;
; Output:
;   DL = | 0iii i.fff | ffff 0000 | (11SM) in A1
;   DS = | sXXX XXXX  | XXXX XXXX | (1TC)  in X1
;
;***************************** MODULE DESCRIPTION *****************************
;
;        Convert difference signal from the linear d(k) to the
;        log domain dl(k). Also obtain the sign ds(k) of d(k).
;          NOTE : The approximation log2(1+x) = x is used for x < 1.0
;            
;        SYMBOLS USED :
;                              d(k)  <==> D
;                              dl(k) <==> DL
;                              ds(k) <==> DS
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;               Save D to get DS
;               Find |D|
;               Normalise |D| to bring leading 1 to the 2nd MSB.
;                 (Upto 14 iterations could be needed)
;               Set exponent = number of iterations needed.
;               Extract 7 bit mantissa from normalized |D| after removing
;                 leading 1.
;               Check if mantissa is zero
;               If mantissa is zero, make exponent zero
;               Combine the exponent and the mantissa to get DL.
;                 (format given 0eee e.mmm mmmm 0000 )
;               
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 20
; NLOAC       : 22
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   c    -    -   c  c  c  c
;           1   -    -    -   c  c  c  -
;           2   c    -    -   c  c
;           3   -    -    -
;
;         -  = not used   u = used & unchanged  c = used & changed   
;
;**************************************************************************
LOG_T
        MOVE    X:(R2)+,R0           ;Get 6 ($0006) to R0 for normalization
        MOVE    A,X1    X:(R2)+,B    ;Save DS in X1, get mask $3F80 for
                                     ;  7 mantissa bits        
        ABS     A                    ;Find DQM=|D|
        ASL4    A                    ;Normalize to find MSB of DQM
        NORM    R0,A
        ASL4    A
        NORM    R0,A               
        NORM    R0,A               
        NORM    R0,A               
        NORM    R0,A               
        NORM    R0,A               
        NORM    R0,A               
        NORM    R0,A               
        MOVE    R0,Y0                ;Move exponent to Y0
        TST     A      A,X0          ;Check if number was zero, save mantissa
                                     ;  in X0
        TNE     Y0,A                 ;If number was not zero, save exponent
                                     ;  in A, else A remains zero which is the 
                                     ;  correct exponent for zero input.
        AND     X0,B   X:(R2)+,Y0    ;Truncate mantissa to 7 bits.
                                     ;  get shift for exponent << 11 ($0800)
;------------------------------------------------------------------------------
;   B1 = 00mm mmmm | m000 0000 |  (B2=B0=0)
;------------------------------------------------------------------------------
        IMPY    Y0,A1,A              ;Shift exponent << 11
        MOVE    X:(R2)+,X0           ;Get shift constant for mantissa >> 3
                                     ;  ($1000)
        MAC     B1,X0,A X:(R2)+,X0   ;Shift mantissa >> 3 & combine with
                                     ;  exponent, get mask $7ff0 (to truncate
                                     ;  -> 11SM). for SUBTB
;------------------------------------------------------------------------------
;   A1 = 0eee e.mmm | mmmm 0000 |  (A2=A0=0)
;      = 0iii i.fff | ffff 0000 |  (A2=A0=0)
;------------------------------------------------------------------------------
END_LOG_T

;******************************************************************************
;
;  Module Name     :  SUBTB
;  Module Number   :  III.2
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   DL  = | 0iii i.fff | ffff 0000 |  (11SM)   in  A1
;   Y   = | 0iii i.fff | ffff ff00 |  (13SM)   in  N3
;
; Output:
;   DLN = | siii i.fff | ffff 0000 |  (12TC)   in  A1
;
;************************ MODULE DESCRIPTION *********************************
;
;    This module scales the log version of the difference signal by
;    subtracting the scale factor.
;
;         dln(k) = dl(k) - y(k)
;         where
;              dl(k) = log version of the difference signal
;              y(k)  = scale factor
;
;    SYMBOLS USED :
;                  dln(k)  <==>   DLN
;                  dl(k)   <==>   DL
;                  y(k)    <==>   Y
;
;******************** STRUCTURED ENGLISH OR PSEUDO CODE **********************
;
;                       Get Y.
;                       Truncate Y to 11SM format.
;                       Compute DLN = DL - Y.
;
;******************************* RESOURCES ***********************************
;
; CYCLE COUNT : 7
; NLOAC       : 7
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   c    -    -   c  c  c  c
;           1   -    -    -   c  c  -  -
;           2   -    -    -   c  c
;           3   -    -    u
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;**************************************************************************
SUBTB_T
        MOVE    N3,B                ;Get Y.
        AND     X0,B                ;Truncate Y to 11SM format.
        MOVE    #QUANTAB,R0         ;Get Quantization table base in  R0.
        MOVE    #QUANTAB+2,Y0       ;Get offset for quan. conversion.
                                    ;Scan the table to check in which range
                                    ;  the given number lies.
        SUB     B,A  X:(R0)+,X0     ;Find DLN=DL-Y.
                                    ;  get first quan. table value
END_SUBTB_T

;******************************************************************************
;
;  Module Name     :  QUAN
;  Module Number   :  III.3
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:                                           
;       DLN = | siii i.fff | ffff 0000 | (12 TCa) in A1
;       DS  = | sxxx xxxx  | xxxx xxxx | (1 TC)   in X1
;
; Output:
;       I   = | siii 0000 | 0000 0000  | (ADPCM format) in B1 & X:I_T
;       Note: IMAG = |I(k)| = | 0000 0000 | 0000 0iii | is put in X:IMAG & X1
;
;***************************** MODULE DESCRIPTION *****************************
;
;         Quantize difference signal in log domain
;       ---------------------------------------------
;         dln(k)                             |I(k)|
;       ---------------------------------------------
;       [3.12, +inf]                            7
;       [2.72, 3.12]                            6
;       [2.34, 2.72]                            5
;       [1.91, 2.34]                            4
;       [1.38, 1.91]                            3
;       [0.62, 1.38]                            2
;       [-0.98,0.62]                            1
;       [-inf,-0.98]                            0    
;       ----------------------------------------------
;       I(k) is the ADPCM output needed
;
;  SYMBOLS USED:  
;              dln(k)  <==>  DLN
;              ds(k)   <==>  DS
;              I(k)    <==>  I
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;               
;       Set pointer to the base address of the
;            quantization table (It will be pointing to -0.98)
;       Find the table position in which range DLN lies, to get IMAG (|I|)
;       Set I = IMAG
;       If IMAG = 0 or sign of DS is negative invert I
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 39
; NLOAC       : 18
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   c    -    -   c  c  c  c
;           1   -    -    -   c  c  c  -
;           2   c    -    -   c  c
;           3   -    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;***************************************************************************
QUAN_T
TSTDLN_T
        CMP     X0,A    X:(R0)+,X0  ;Compare to DLN, get next value.
        BGE     <TSTDLN_T           ;If Value < DLN try next range
        MOVE    R0,B                ;When range found,
                                    ;  subtract pointer from base
        SUB     Y0,B    X1,A        ;  to get IMAG = |I|
                                    ;Move DS to A
;--------------------------------------------------------------------------
;A1 = 0000 0000  0000 0iii (A2 = A0 = 0)
;-------------------------------------------------------------------------
        MOVE    B1,X:IMAG           ;Save IMAG.
        TFR     B,X1    X:(R2)+,X0  ;Save IMAG in X1 (for RECONST)
                                    ;  Get shift constant for << 12 ($1000) 
                                    ;  to move IMAG to MSB position.
        IMPY    X0,B1,B             ;Shift IMAG left by 12 bits
;-------------------------------------------------------------------------
;A1 = 0iii 0000 0000 0000 
;-------------------------------------------------------------------------
        TST B   X:(R2)+,Y0          ;Check  IMAG, Get  invert  mask ($F000)
                                    ;  to invert the first 4 bits
        BEQ     <INVERT_T           ;Invert bits if IMAG = 0
        TST     A                   ;If IMAG is not zero then check DS.
        BPL     <IOUT_T              
INVERT_T                            ;Invert bits if DS = 1
        EOR     Y0,B                ;Required I is in B1.
IOUT_T  MOVE    B1,B                ;Sign extend I
        MOVE    B1,X:I_T            ;Save I in memory
END_QUAN_T

;*************************************************************************
;
;  Module Name     :  RECONST 
;  Module Number   :  IV.1
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   I    = | siii 0000 | 0000 0000 |  in B1
;   IMAG = | 0000 0000 | 0000 0iii |  in X1
;
; Output:
;   DQLN = | siii i.fff | ffff 0000 |  (12TC) in A1
;   DQS  = | sxxx x.xxx | xxxx xxxx |  (1TC) in B1
;
;***************************** MODULE DESCRIPTION ***************************
;               
;  Reconstructs quantized difference signal in the 
;  log domain. Inverse quantize IMAG = |I(k)| to get 
;  dqln(k). The sign of dq(k) (ds(k)) is obtained 
;  from I(k).
;
;           --------------------
;           | IMAG |  dqln(k)  |
;           --------------------
;           |   7  |     3.32  |
;           |   6  |     2.91  |
;           |   5  |     2.52  |
;           |   4  |     2.13  |
;           |   3  |     1.66  |
;           |   2  |     1.05  |
;           |   1  |     0.031 |
;           |   0  |    - inf  |
;           --------------------
;
;    ds(k) = sign bit of I(k)
;
;    SYMBOLS USED :
;                          I(k)    <==>  I
;                          dqln(k) <==>  DQLN         
;                          ds(k)   <==>  DQS
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;               Save I as DQS
;               Get the base of the inverse quantiser table IQUANTAB
;               Add the offset (IMAG) to the base
;               DQLN is read from the table using offset+base as address
;               
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 5
; NLOAC       : 5
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   c  u  -  c
;           1   -    -    -   c  u  u  -
;           2   -    -    -   c  u
;           3   -    -    -
;
;         -  = not used   u = used & unchanged  c = used & changed
;
;**************************************************************************
RECONST_T
        MOVE   #IQUANTAB,A         ;Set lookup table base        
        ADD    X1,A                ;Add offset for table lookup
        MOVE   X:(A1),A            ;Lookup DQLN
END_RECONST_T                

;******************************************************************************
;
;  Module Name     :  ADDA 
;  Module Number   :  IV.2
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;       Y    =  | 0iii i.fff | ffff ff00 | (13 SM)  in N3
;       DQLN =  | siii i.fff | ffff 0000 | (12 TCa) in A1
;
; Output:
;       DQL  =  | siii i.fff | ffff 0000 | (12 TCa) in A1
;
;***************************** MODULE DESCRIPTION **************************
;
;      Add the scale factor y(k) to the log version of the
;      quantized difference signal dqln(k) to get back dql(k).
;
;           dql(k) = dqln(k) + y(k)
;
;      SYMBOLS USED :
;                     y(k)     <==>  Y
;                     dqln(k)  <==>  DQLN
;                     dql(k)   <==>  DQL
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;               
;            Find DQL = DQLN + Y
;            Truncate DQL to 12 TC format.
;       
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT: 3   
; NLOAC      : 5
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   c  -  c  -
;           1   -    -    -   c  -  -  -
;           2   c    -    -   c  -
;           3   -    -    u
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;*****************************************************************************
ADDA_T
       MOVE    N3,X0                    ;Get Y
       ADD     X0,A      X:(R2)+,X0     ;Add Y to DQLN to get DQL,
                                        ;Get trun. mask ($FFF0) for 12 TC.
       AND     X0,A      X:(R2)+,X0     ;Truncate DQL to 12 TC format.
                                        ;  get 0 in X0 for ANTILOG
END_ADDA_T

;******************************************************************************
;
;  Module Name     :  ANTILOG  
;  Module Number   :  IV.3
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;       DQL = | siii i.fff | ffff 0000 |  (12 TCa) in A1
;       DQS = | sxxx xxxx  | xxxx xxxx |  (1 TC)  in  B1
;
; Output:
;       DQ  = | siii iiii | iiii iii.0 |  (15 SM) in A1
;
;       Note: DQMAG = | 0iii iiii | iiii iii.0 |  (15TC) is put in 
;                                                     N0 (for FLOATA & TRANS)
;                                                   & B1 (for ADDB)   
;***************************** MODULE DESCRIPTION ******************************
;               
;       Convert quantized difference signal from log to linear domain.       
;       Use approximation 2**x = 1 + x  for fractional x
;
;       Let us consider a number Y=Yi+Yf where i and f stand for integer and
;       fractional portions respectively.
;       2**Y = 2**Yi * 2**Yf  
;       By using the approximation,
;       2**Yf = 1 + Yf 
;       We get,
;       2**Y = (1+Yf) * 2**Yi  OR Left shift (1+Yf) Yi times 
;
;     SYMBOLS USED : 
;               dql(k)  <==>  DQL
;               dqs(k)  <==>  DQS
;               dq(k)   <==>  DQ
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;               
;               If DQL>=0
;               { 
;                  separate the integer and fractional parts of DQL
;                  Set DEX = integer part of DQL 
;                  Set DMN = fractional part of DQL
;                  Add 1 to DMN to bring it in 01ff ffff f000 0000 format
;                  Shift 1+DMN right 14-DEX times to get DQMAG = |DQ|
;                  At this point one extra right shift is present
;                  Do one left shift and append append sign bit DQS to 
;                       get DQ in 15 SM format.
;               }
;               else
;               {
;                  Store zero as DQMAG
;                  Store DQ with magnitude 0 and sign bit set to DQS in
;                  15 SM format.
;               }
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 20
; NLOAC       : 28
;
; REGISTERS USED:
;      
;               R    M    N   A  B  X  Y
;           0   c    -    c   c  c  c  c
;           1   -    -    -   c  c  c  -
;           2   c    -    -   c  c
;           3   -    -    -
;
;         -  = not used   u = used & unchanged  c = used & changed
;
;******************************************************************************
ANTILOG_T
        TST     A       X:(R2)+,Y0      ;Check DQL, Get shift constant
                                        ;  $0010 for >> 11 in Y0
        BPL     <CONVLOG_T              ;If  DQL>=0 convert DQL
        MOVE    X0,N0                   ;Save DQMAG in N0
        MOVE    #$8000,A                ;Get mask for DQS ($8000)
        MOVE    B,Y0                    ;Get  DQS in Y0
        AND     Y0,A   X:(R2)+,Y0       ;Make MSB of A = DQS, zero other bits
        TFR     X0,B   X:(R2)+,Y0       ;Save DQMAG in B
                                        ;  R2 points to the constants used.
                                        ;  2 constants which are used in
                                        ;  the code below are skipped.
;--------------------------------------------------------------------------
; Now DQ is in 15 SM format with value 0 and sign = DQS.
;--------------------------------------------------------------------------
        BRA     <SAVEDQ_T
CONVLOG_T
        MPY     Y0,A1,A                 ;Shift A right by 11 bits
;---------------------------------------------------------------------------
; Now A1 contains the integer part and A0 contains unsigned fractional
; part of DQL
;----------------------------------------------------------------------------
        MOVE    A1,Y0                   ;Move DEX=integer part of DQL to Y0
        MOVE    #RSHFT+14,A1            ;A1 points to the >>14 constant
                                        ;  in X memory.
        SUB     Y0,A                    ;A1 points to the shift constant
                                        ;  for >> 14-DEX.
        MOVE    A1,R0                   ;Set R0 to point to required shift
                                        ;  constant
        MOVE    A0,A1                   ;Save unsigned fractional part
        LSR     A                       ;Make it signed fraction (DMN)
        LSR     A      X:(R2)+,X1       ;A1 = 00ff ffff f000 0000
                                        ;  get mask ($4000) to add 1 to DMN.
        OR      X1,A    X:(R0)+,Y0      ;A1 = 01ff ffff f000 0000
                                        ;  get shift constant for
                                        ;  >> (14-DEX) to Y0
        MPY     Y0,A1,A  X:(R2)+,Y0     ;A1 contains DQMAG = |DQ| with
                                        ;  one extra right shift
                                        ;  get mask for DQS in Y0 ($8000)
        LSL     A                       ;Get DQMAG=|DQ| in 15SM format.
        MOVE    A1,N0                   ;Save DQMAG in N0
        AND     Y0,B     A,X0           ;Get DQS sign bit in B
                                        ;  save DQMAG in X0
        TFR     X0,B     B,Y0           ;Save DQMAG in B, get DQS to Y0
        OR      Y0,A                    ;Get DQ
SAVEDQ_T
        MOVE    A1,A                    ;Sign extend DQ
END_ANTILOG_T

;******************************************************************************
;
;  Module Name     :  ADDB
;  Module Number   :  VIII
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   DQ    = | siii iiii | iiii iii.0 |   (15SM)  in  A
;   SE    = | siii iiii | iiii iii.0 |   (15TC)  in  Y1
;
;   Note : DQMAG is also used in this module.
;     DQMAG = | 0iii iiii | iiii iii.0 |         in  B
;
; Output:
;   SR    = | siii iiii | iiii iiii. |   (16TCb)  in  B1, X:SR_T
;    Note : DQI is also generated to be used in the next module.
;    Note : DQ & DQMAG are also saved for use the next modules.
;     DQI   = | siii iiii | iiii iii.0 |   (15TC) in  X:DQI
;     DQMAG = | 0iii iiii | iiii iii.0 |          in  Y0 (for XOR/UPB)
;     DQ    = | siii iiii | iiii iii.0 |   (15SM) in  X1 (for XOR/UPB & FLOATA)
;
;***************************** MODULE DESCRIPTION ****************************
;
;     This module forms the reconstructed signal, sr(k), by adding the 
;     quantized difference signal, dq(k), and the signal estimate, se(k).
;              sr(k)  =  se(k) + dq(k)
;     SYMBOLS USED :
;                   sr(k)  <==>  SR
;                   se(k)  <==>  SE
;                   dq(k)  <==>  DQ
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;               
;               Get DQ and DQMAG.
;               Negate DQMAG.
;               Convert DQ to 2's comp by using -DQMAG if DQ<0. Thus form DQI.
;               Add DQI to SE to give SR.
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 9
; NLOAC       : 9
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   u  c  c  c
;           1   c    -    -   u  c  c  u
;           2   -    -    -   u  c
;           3   -    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;**************************************************************************
ADDB_T
        NEG     B      B,Y0         ;If DQ was -ve, B now has DQ in 2's comp.
                                    ;  save DQMAG in Y0
        TST     A      A,X1         ;Test sign of DQ, save DQ in X1
        TPL     A,B                 ;Convert DQ to 15TC format --> DQI.
        MOVE    B,X:DQI             ;Save DQI in X:DQI (for ADDC)
        ADD     Y1,B   X:(R1)+,X0   ;Find SR=DQI+SE in 15TC format.
                                    ;  dummy read to advance R1 to point
                                    ;  to DQ1Mant
;------------------------------------------------------------------------
;   There is a possibility of overflow after addition. But the following
;     ASR instrn takes care of it.
;------------------------------------------------------------------------
        ASR     B     X:(R1)+,X0    ;Convert SR to 16TC format.
                                    ;  dummy read to advance R1 to point
                                    ;  to DQ1S
        MOVE    B1,X:SR_T           ;Save SR.
END_ADDB_T

;******************************************************************************
;
;  Module Name     :  XOR/UPB
;  Module Number   :  VII.6.1/VII.6.2
;
;*************************** INPUT AND OUTPUT *******************************
;
;                       MODULE XOR
;
; Input:
;       DQ   = | siii iiii | iiii iii.0 |  (15 SM)   in X1
;       DQnS = | sxxx xxxx | xxxx xxxx  |            in X:(R1+3*(n-1))
;                                                     n = 1,2,...,6
; Output:
;       Un   = | sxxx xxxx | xxxx xxxx | (1 TC)     in A1
;       
;                       MODULE UPB
; 
; Input:
;      Un   = | sxxx xxxx  | xxxx xxxx  | (1 TC)   in A1      n=1,2,...,6
;      DQ   = | siii iiii  | iiii iii.0 | (15 SM)  in X1
;      Bn   = | si.ff ffff | ffff ffff  | (16 TCa) in X:COEF_T+(n-1)
;                                                             n=1,2,..,6 
;      R3 points to COEF_T+8
;
;      Note: DQMAG=|DQ| = | 0iii iiii | iiii iii.0 |  stored in Y0 is also
;            used as input.
;  
; Output:
;      BnP  = | si.ff ffff | ffff ffff |  (16 TCa) in X:COEF_T+(n-1) 
;                                                             n=1,2,..,6
;      R3 points to COEF_T+6 (A1)
;      Note : DQ  = | siii iiii | iiii iii.0 | (15 SM)  remains in X1 
;                                                               (for FLOATA)
;
;*************************** MODULE DESCRIPTION ******************************
;
;                           XOR
;
;       The Module XOR finds  one  bit  exclusive or of the sign of the 
;       difference signal, dq(k) and the sign of the delayed difference 
;       signal, dq(k-n) (n=1,2,...,6).
;
;       un(k) = sgn[dq(k)] XOR sgn[dq(k-n)]  n=1,2,...,6
;
;      SYMBOLS USED :
;                dq(k)         <==>  DQ
;                sgn(dq(k-n))  <==>  DQnS  n=1,2,...,6
;                un(k)         <==>  Un    n=1,2,...,6
;
;                           UPB
;
;       The module UPB updates the coefficients, bnp(k) (n=1,2,...,6)  of 
;       the sixth order predictor.
;
;       bnp(k) = (1-(2**-8))*bn(k-1) + (2**-7)*un(k)   if dq(k)!=0
;              = (1-(2**-8))*bn(k-1)                   otherwise
;                                                                n=1,2,...,6
;    
;      SYMBOLS USED :
;                bnp(k)       <==>  BnP    n=1,2,...,6
;                bn(k-1)      <==>  Bn     n=1,2,...,6
;                un(k)        <==>  Un     n=1,2,...,6
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;                                       
;   Note: Module UPB and XOR are written together. If DQ is 0 then
;         Un and hence XOR is not needed. Thus, UPB with XOR and
;         UPB without XOR both are written.
;   
;         If (DQMAG=0) then
;         {   
;              do for n=1 to 6
;                  Find BnP = Bn - Bn*2**-8 
;         }
;         else
;         {
;              do for n = 1 to 6
;                 {
;                      Find Un = DQS XOR DQnS
;                      If  Un = 0 then
;                          Set UGBn = 2**-7
;                      Else 
;                          Set UGBn = -2**-7
;                      Get Bn in X1
;                      Find UBn  = UGBn - (Bn*2**-8)
;                      Find BnP  = Bn + UBn
;                   }
;          }
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 58
; NLOAC       : 26
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   c  c  c  c
;           1   c    -    u   c  c  u  c
;           2   c    -    -   c  c
;           3   c    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;******************************************************************************
UPB_T
        TFR     Y0,A   X:(R2)+,X0       ;Get DQMAG = |DQ| to A
                                        ;  Get constant 2**-7 in 16TCa ($0080)
                                        ;  This is reused for >>8
        MOVE    #COEF_T,R3              ;Pointer to B1
        TST     A       X:(R2)+,Y0      ;Check DQMAG
                                        ;  get constant -2**-7 in 16TCa ($FF80)
        BNE     <XORUPB_T               ;If DQMAG != 0 use section with XOR,
                                        ;  else use section with no XOR,
                                        ;  because UGBn = 0 if DQMAG = 0
                                        ;UPB without XOR
        DO      #6,ENDLOOP_T            ;Do UPB for B1-B6.
        MOVE    X:(R2),B                ;Get constant ($FF00) to get first 
                                        ;  eight bits of Bn
        MOVE    X:(R1)+N1,A  X:(R3)+,Y0 ;DQnS not used, it is just for inc. R1.
                                        ;  Get Bn in Y0
        AND     Y0,B      Y0,A          ;Get first 8 bits of Bn in Y0
                                        ;  Save Bn in A
        MAC     -X0,B1,A   (R3)-        ;Get BnP = Bn-(Bn*2**-8)
        MOVE    A1,X:(R3)+              ;Save BnP to Bn
ENDLOOP_T
        MOVE    X:(R2)+,X0              ;Dummy read to adjust constant pointer
        BRA     <END_UPB_T
XORUPB_T                                ;UPB with XOR  
        MOVE    X:(R2)+,B               ;Get constant to get first eight
                                        ;   bits of Bn ($FF00)
        DO      #6,END_UPB_T            ;Do UPB and XOR for B1-B6
        MOVE    X:(R1)+N1,A             ;Get DQnS in A
        EOR     X1,A     X:(R3)+,Y1     ;Find Un = DQS**DQnS(XOR), Get Bn in Y1
        TPL     X0,A                    ;If Un=0 Set UGBn=2**-7
        TMI     Y0,A                    ;If Un=1 Set UGBn=-2**-7
        AND     Y1,B     (R2)-          ;Bn truncated to first 8 bits
                                        ;   Make R2 point to FF00 again.
        MAC     -B1,X0,A (R3)-          ;Find UBn = UGBn-(Bn*2**-8) or -(Bn>>8)
        ADD     Y1,A     X:(R2)+,B      ;Find BnP=Bn+UBn
                                        ;  Get constant to get first eight
                                        ;  bits of Bn ($FF00) for next
                                        ;  execution of loop body. At the
                                        ;  last loop execution this serves as
                                        ;  a dummy read to advance constant
                                        ;  pointer R2.
        MOVE    A1,X:(R3)+              ;Store BnP to Bn
END_UPB_T

;******************************************************************************
;
;  Module Name     :  FLOATA
;  Module Number   :  VII.1.1
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   DQ    =  | siii iiii | iiii iii.0 |  (15SM)  in  X1
;   DQMAG =  | 0iii iiii | iiii iii.0 |          in  N0
;
; Output:
;   DQ1 = ( 11FL : s mmmmmm eeee )
;    DQ1EXP  = | 0000 0000 | 0000 eeee |   in  X:(R1+1)
;    DQ1MANT = | 01mm mmm0 | 0000 0000 |   in  X:(R1+2)
;    DQ1S    = | sXXX XXXX | XXXX XXXX |   in  X:(R1+3)
;
;***************************** MODULE DESCRIPTION ****************************
;
;        The quantized difference signal, dq(k), which is in 15SM
;        format is converted to a floating point format. The sign, 
;        mantissa and exponent are each extracted and stored separately.
;
;        SYMBOLS USED :
;                      dq(k)  (input)  <==>  DQ
;                      dq(k)  (output) <==>  DQ1 (DQ1S,DQ1MANT,DQ1EXP)
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;       Get the magnitude of DQ.
;       Set the Shift Counter to 14.
;       If the magnitude is not 0 
;          Repeat until the magnitude of DQ gets normalized:
;             Left shift DQ and decrement Shift Counter.
;       Else if the magnitude is zero
;          Set mantissa, DQ1MANT, to .100000.
;          Set exponent, DQ1EXP, to 0.
;       Store the exponent, DQ1EXP (the contents of the Shift Counter).
;       Store the mantissa, DQ1MANT (the resulting normalized number).
;       Store the sign of DQ as DQ1S.
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 19
; NLOAC       : 25
;
; REGISTERS USED:
;
;                       R    M    N   A  B  X  Y
;                   0   c    -    u   c  -  c  -
;                   1   c    -    -   c  -  u  -
;                   2   c    -    -   c  -
;                   3   -    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;**************************************************************************
FLOATA_T
        MOVE    N0,A                ;Get MAG=DQMAG
        MOVE    X:(R2)+,R0          ;Load the exponent bias $0006 (= 6).
        TST     A     X:(R2)+,X0    ;Check MAG
                                    ;  Get mask $7E00 (to truncate to 6 bits).
        BNE     <NORMDQ_T           ;If MAG is not 0, normalize
        MOVE    #$4000,A            ;If MAG=0 set MANT=100000,
        MOVE    #$0000,R0           ; and EXP=0
        BRA     <TRUNCDQ_T
NORMDQ_T
        ASL4    A                   ;Normalize A to get MSB of MAG
        NORM    R0,A
        ASL4    A
        NORM    R0,A
        NORM    R0,A
        NORM    R0,A
        NORM    R0,A
        NORM    R0,A
        NORM    R0,A
        NORM    R0,A
;---------------------------------------------------------------------------
;  A1 = | 01xx xxxx | xxxx xxx0 | = normalized MAG (A2=A0=0)
;  R0 = | 0000 0000 | 0000 eeee | = exponent of normalized MAG
;---------------------------------------------------------------------------
TRUNCDQ_T
        AND     X0,A   X:(R1)+,X0   ;Truncate MANT to 6 bits, dummy read
                                    ;  to advance pointer to DQ1EXP
        MOVE    R0,X:(R1)+          ;  save EXP to DQ1EXP
;---------------------------------------------------------------------------
;  A1 = 01mm mmm0 | 0000 0000 | 0000 0000  (A2=A0=0)
;---------------------------------------------------------------------------
        MOVE    A1,X:(R1)+          ;Save MANT to DQ1MANT
        MOVE    X1,X:(R1)+          ;Save DQ to DQ1S
END_FLOATA_T

;******************************************************************************
;
;  Module Name     :  TRANS 
;  Module Number   :  IX.4
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   TD  = | i000 0000  | 0000 0000  | (1TC) in X:TD_T
;   YL  = | 0iii i.fff | ffff ffff  | (19SM) in X:YU_T+1
;                                            (Only first word is needed)
; DQMAG = | siii iiii  | iiii iii.0 | (15SM) in N0
;
; Output:
;   TR  = | i000 0000  | 0000 0000 | (1TC) in X:TR_T
;
;***************************** MODULE DESCRIPTION ****************************
;
;   tr(k) = 1 if((td(k) = 1) & (|dq(k)| > 24 * 2 ** yl(k-1)))
;         = 0 otherwise
;   yl(k-1) = ylint(k-1) + ylfrac(k-1) , where ylfrac(k-1) < 1.0
;
;   24 * 2**yl(k-1)
;       = (2**3 + 2**4) * (2 ** yl(k-1))
;       = (2**3 + 2**4) * (2 ** (ylint(k-1) + ylfrac(k-1)))
;       = (2**3 + 2**4) * (1 + ylfrac(k-1)) * (2**ylint(k-1))
;       = (2**2 * (1 + ylfrac(k-1)) + 2**1 * (1 + ylfrac(k-1))) * 
;                                                        (2**(ylint(k-1)+2))
;
;  If ylint(k-1) is more than 8, threshold value is set to maximum 
;  ( 93.0 * 2**7 ).
;
;             SYMBOLS USED :
;                           td(k)   <==> TD  = Tone detect signal
;                           tr(k)   <==> TR  = Tone transition detect signal
;                           dq(k)   <==> DQ  = Quantized difference signal
;                           yl(k-1) <==> YL  = Slow scale factor
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;   Assign TR = 0
;   Check TD
;   If (TD = 0) stop
;   Else get YL from memory
;   Get integer portion of YL = YLINT by
;     shifting YL by 11 bits to the right
;   Compare YLINT with  8
;   If (YLINT > 8)
;       Set threshold = 'maximum threshold' = $5D00
;       Go to COMPARE
;   Else
;      Get fractional part of YL by masking out the integer part of YL
;      Shift left YLFRAC 4 times
;      Find 6.0 * YLFRAC
;      Add 6.0 to get 6 * (1 + YLFRAC) in |5.11| format
;      Shift above value by (10 - (YLINT+2)) to
;       the right to get threshold = 24 * (1 + YLFRAC) * 2 ** (YLINT) in 15SM
;       format.
;
;COMPARE
;       Get DQMAG from memory
;       Compare threshold with DQMAG 
;       If ( DQMAG > threshold )       
;         Set TR = 1
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 32
; NLOAC       : 29
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    u   c  c  c  c
;           1   -    -    -   c  c  -  c
;           2   c    -    -   c  c
;           3   -    -    -
;
;         -  = not used   u = used & unchanged  c = used & changed
;
;**************************************************************************
TRANS_T
        CLR    A      X:(R2)+,X0        ;Set A to zero
                                        ;  get shift for exponent >> 11 ($0010)
        MOVE   X:TD_T,B                 ;Get TD to B
        TST    B      X:(R2)+,Y0        ;Check TD, get 8 ($0008) to Y0 for 
                                        ;  checking exponent > 8
        BEQ    <SAVE_TR_T               ;If TD=0 goto end of routine
        MOVE   X:YU_T+1,B               ;Else, get YL
        MPY    X0,B1,B  B,A             ;Move YL right 11 times to get exponent
                                        ;  in B1, store original YL in A
        CMP    Y0,B   B,Y1              ;Compare exponent with 8, move exponent
                                        ;  to Y1                
        MOVE   X:(R2)+,B                ;Move maximum value of threshold 
                                        ;  $5D00 to B
        BGT    <COMPARE_T               ;If exponent > 8, jump to compare DQ
                                        ;  and maximum threshold
BELOW_MAX_THR_T                         ;If exponent <= 8, continue check
        MOVE   #RSHFT+9,B               ;Move base address of the necessary 
                                        ;  part of shift constant table to B
        SUB    Y1,B   X:(R2)+,X0        ;Subtract offset = exponent from base
                                        ;  get masking constant ($07C0)
        AND    X0,A   X:(R2)+,Y0        ;Mask YL to 5 fractional bits
                                        ;  get constant ($0003) to perform 
                                        ;  multiplication with 6
        MPY    A1,Y0,A  X:(R2)+,Y0      ;Get 6 * 1.0 ($3000) to add to A
                                        ;  which contains 6 * YLFRAC
        MOVE   A0,A                     ;Move shifted word to A1        
        ADD    Y0,A   X:(B1),Y0         ;Add 6 * 1.0 and 6 * YLFRAC to get
                                        ;  6 * (1 + YLFRAC)
                                        ;  get shift constant from table, using
                                        ;  B1 as address regs
        MPY    Y0,A1,B                  ;Shift appropriately to get threshold
        LSL    B                        ;Get threshold in 15SM format.
COMPARE_T                               ;Comparision of threshold and DQMAG 
        MOVE   #CONST_TRANS,R2          ;Set R2 to point to correct place
                                        ;  to account for different paths
        MOVE   N0,X0                    ;Get DQMAG
        CLR    A                        ;Clear A for TR value
        CMP    X0,B   X:(R2)+,X0        ;Compare DQMAG and threshold
                                        ;Move '1' ($8000) to X0        
        TLT    X0,A                     ;Transfer '1' to A if DQMAG>threshold
SAVE_TR_T        
        MOVE   #CONST_ADDC,R2           ;Update constant pointer R2 to allow
                                        ;  for different paths
        MOVE   A,X:TR_T                 ;Save TR in memory   
END_TRANS_T        

;******************************************************************************
;
;  Module Name     :  ADDC ( with DELAY included)
;  Module Number   :  VII.2.1
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   DQI = | siii iiii | iiii iii.0 |  (15TC)  in  X:DQI
;   SEZ = | siii iiii | iiii iii.0 |  (15TC)  in  X:SEZ_T
;   PK0 = | sXXX XXXX | XXXX XXXX  |  (1TC)   in  X:PK_T
;   PK1 = | sXXX XXXX | XXXX XXXX  |  (1TC)   in  X:PK_T+1
;   
;   Note: (1) DQI is the same as DQ but in a different format (15TC)
;             It is generated in the module ADDB.
;         (2) PK0 is delayed to give PK1. This is the function
;             of DELAY which is incorporated with ADDC.
;
; Output:
;   PK0   = | sXXX XXXX | XXXX XXXX |  (1TC)  in  X:PK_T
;   PK1   = | sXXX XXXX | XXXX XXXX |  (1TC)  in  X:PK_T+1
;   SIGPK = | i000 0000 | 0000 0000 |  (1)    in  X1 (for UPA2 & UPA1)
;   PKS1  = | sXXX XXXX | XXXX XXXX |  (1TC)  in  N0 (for UPA1) & A1 (for UPA2)
;   PKS2  = | sXXX XXXX | XXXX XXXX |  (1TC)  in  B1 (for UPA2)
;
;   Note: (1) PKS1 & PKS2 are strictly not the outputs of
;             ADDC. But it is more convenient to generate
;             it here fore modules UPA1 & UPA2.
;         (2) Outputs PK1 and PK2 are just the delayed versions
;             of inputs PK0 and PK1 respectively.
;
;*************************** MODULE DESCRIPTION ******************************
;
;     This module finds the sign, sgn[p(k)], of the sum of the quantized 
;     difference signal, dq(k), and the partial signal estimate, sez(k).
;     Delayed versions of sgn[p(k)], i.e., sgn[p(k-1)] and sgn[p(k-2)] are
;     also generated.
;
;       p(k) = dq(k) + sez(k)
;
;       sgn[p(k)],sgn[p(k-1)],sgn[p(k-2)] are determined according to 
;       the equation :
;
;               sgn[x]  = +1   if    x>0
;                       = -1   if    x<0
;                       = +1   if    x=0 
;                                       where x=p(k-i) AND i != 0
;                       =  0   if    x=0 
;                                       where x=p(k)
; SYMBOLS USED :
;               dq(k)       <==>   DQ
;               sez(k)      <==>   SEZ
;               p(k)        <==>   DQSEZ         ...( = DQ + SEZ )
;               sgn[p(k)]   <==>   PK0 & SIGPK
;               sgn[p(k-1)] <==>   PK1
;               sgn[p(k-2)] <==>   PK2
;
; IMPORTANT : (1)  `sgn[p(k)]' has 3 possible values : `+1',`0' and `-1'. 
;                 So atleast 2 bits are needed to represent it. The 2 bits 
;                 chosen are the sign bits of variables PK0 and SIGPK.
;                 Values assigned to these sign bits are as follows :
;                  |===============|=======|=======|
;                  | sgn[p(k)]  -->|  PK0  | SIGPK |
;                  |===============|=======|=======|
;                  |   `+1'     -->|   0   |   0   |
;                  |---------------|-------|-------|
;                  |   `-1'     -->|   1   |   0   |
;                  |---------------|-------|-------|
;                  |    `0'     -->|   0   |   1   |
;                  |---------------|-------|-------|
;
;             (2) SIGPK is associated with sgn[p(k)] only. For sgn[(p(k-i)],
;                 i .ne. 0, only PK1 or PK2 is needed. This is because
;                 sgn[p(k-i)] can have one of only 2 values :`+1' and `-1', 
;                 as evident in the equations specified above.
;                 Thus the assignment is :
;                  |================|=======| |================|=======|
;                  | sgn[p(k-1)] -->|  PK1  | | sgn[p(k-2)] -->|  PK2  |
;                  |================|=======| |================|=======|
;                  |   `+1'      -->|   0   | |   `+1'      -->|   0   |
;                  |----------------|-------| |----------------|-------|
;                  |   `-1'      -->|   1   | |   `-1'      -->|   1   |
;                  |----------------|-------| |----------------|-------|
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;                 Find DQSEZ = DQ + SEZ.
;                 If (DQSEZ >= 0)
;                   { Set PK0 = 0.
;                     If (DQSEZ = 0)
;                        Set SIGPK = 1.
;                     Else if (DQSEZ > 0)
;                        Set SIGPK = 0.
;                   }
;                 Else if (DQSEZ < 0)
;                   { Set PK0 = 1.
;                     Set SIGPK = 0.
;                   }
;                 Find PKS1 = PK0 * PK1
;                 Find PKS2 = PK0 * PK2
;                 
;          Note : `*' stands for EOR (Exclusive Or).
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 19
; NLOAC       : 18
;
; REGISTERS USED:
;
;                       R    M    N   A  B  X  Y
;                   0   c    -    c   c  c  c  c
;                   1   -    -    -   c  c  c  -
;                   2   c    -    -   c  c
;                   3   -    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;*************************************************************************
ADDC_T
        MOVE    #PK_T,R0            ;Move the address of PK_T to R0
        MOVE    X:SEZ_T,A           ;Get SEZ.
        MOVE    X:DQI,B             ;Get DQI
        ADD     B,A     X:(R2)+,B   ;Find DQSEZ=DQI+SEZ,
                                    ;  get $0000 (='0' for SIGPK)
        ASR     A                   ;Get DQSEZ in 16TC format.
        TST     A      X:(R2)+,X0   ;Check DQSEZ, get $8000 (= '1' for SIGPK).
        TEQ     X0,B                ;If DQSEZ=0, SIGPK=`1',
                                    ;  else SIGPK=0.
        TFR     B,X1   X:(R0)+,X0   ;Save SIGPK to X1, get PK
        MOVE    X:(R0)-,Y0          ;Get delayed PK
        MOVE    A1,X:(R0)+          ;Save updated PK
        MOVE    X0,X:(R0)           ;Save delayed PK
        EOR     X0,A   A,B          ;Find PKS1=PK0*PK1 ( * = Exclusive Or),
                                    ;  save PK0 in B.
        EOR     Y0,B                ;Find PKS2=PK0*PK2
        MOVE    A1,A                ;Sign extend PKS1
        MOVE    B1,B                ;Sign extend PKS2 
        MOVE    A1,N0               ;Save PKS1 in N0
END_ADDC_T

;******************************************************************************
;
;  Module Name     :  UPA2  
;  Module Number   :  VII.2.3
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   A1    = | si.ff ffff | ffff ffff | (16TCa) in X:COEF_T+6
;   A2    = | si.ff ffff | ffff ffff | (16TCa) in X:COEF_T+7
;   SIGPK = | i000 0000  | 0000 0000 | (1TC) in X1
;   PK0   = | i000 0000  | 0000 0000 | (1TC)
;   PK1   = | i000 0000  | 0000 0000 | (1TC)
;   PK2   = | i000 0000  | 0000 0000 | (1TC)
;   R3 points to COEF_T+6 (A1)
;
;   Note : PK0, PK1, PK2 are not used here, as PKS1 and PKS2 are already
;          available. So PKS1 and PKS2 are used as inputs.
;   PKS1  = | sXXX XXXX  | XXXX XXXX | (1TC) in A1
;   PKS2  = | sXXX XXXX  | XXXX XXXX | (1TC) in B1
;
; Output:
;   A2T   = | si.ff ffff | ffff ffff | (16TCa) in B1
;   Note : SIGPK = | i000 0000  | 0000 0000 | (1TC) is available in X1
;   R3 points to COEF_T+7 (A2)
;
;***************************** MODULE DESCRIPTION ****************************
;
;       Update the coefficient of the second order predictor
;       a2u(k) = [1-(2**-7)] * [a2(k-1)] + (2**-7) * { sgn[p(k)] * sgn[p(k-2)] 
;                                     - f(a1(k-1)) * sgn[p(k) * sgn[p(k-1)] } 
;       
;       where  f(a1) = 4 * a1           |a1| <= 1/2
;                    = 2 * sgn(a1)      |a1| > 1/2
;
;           SYMBOLS USED :
;                          PKS1  <==> PK0**PK1  (** = XOR)
;                          PKS2  <==> PK0**PK2
;                          FA1   <==> f(a1)
;                          A1    <==> a1(k-1)
;                          A2    <==> a2(k-1)
;                          A2T   <==> a2(k) 
;                          SIGPK <==> 1 if pk(0) = 0
;                                     0 if pk(0) != 0
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;               If PKS2 is 1 set UGA2A to -1
;               Else set UGA2A to +1
;               If 'A1' > 1/2 set FA to 1/2
;               Else if 'A1' < -1/2 set FA to -1/2
;               If PKS1 is 1 FA1 = FA
;               Else FA1 = -FA
;               Get UGA2B = UGA2A + FA1
;               Find UGA2 = UGA2B >> 5 
;               If SIGPK is 1 UGA2 = 0
;               Find ULA2 = 'A2' >> 7
;               Find UA2 = UGA2 - ULA2
;               Find A2T = UA2 + A2
;
;******************************** RESOURCES ***********************************
; 
; CYCLE COUNT : 21
; NLOAC       : 23
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   c  c  c  c
;           1   -    -    -   c  c  u  c
;           2   c    -    -   c  c
;           3   c    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed  
;
;**************************************************************************
UPA2_T 
        TST     B       X:(R2)+,X0  ;Check PKS2, get '+1.0' in |4.12| 
                                    ;  format ($1000)
        MOVE    X:(R2)+,Y0          ;Get '-1.0' in |4.12| format ($F000)
        TGE     X0,B                ;If PKS2=0, set UGA2A='+1'
        TMI     Y0,B                ;If PKS2=1, set UGA2A='-1'
        TFR     B,Y1     X:(R2)+,X0 ;Save UGA2A
                                    ;  get +1.99 in |4.12| format ($1FFF)
        MOVE    X:(R3)+,B           ;Get 'A1' in |2.14| format, same as 
                                    ;  4*'A1' in |4.12| format        
        CMP     X0,B    X:(R2)+,Y0  ;Check 'A1' > +1/2 i.e. 4*'A1' > +1.99
                                    ;  get -1.99 in 4.12 format ($E001)
        TGT     X0,B                ;If 'A1' > +1/2, set FA1 = +1.99
        CMP     Y0,B                ;Check 'A1' < -1/2 i.e. 4*'A1' < -1.99
        TLT     Y0,B                ;If 'A1' < -1/2, set FA1 = -1.99
        NEG     B       B,Y0        ;Set FA = -FA1, save FA1 in X0
        TST     A       X:(R2)+,X0  ;Check PKS1
                                    ;  get shift constant >> 5 ($0400)
        TMI     Y0,B                ;If PKS1=1, FA = FA1
        ADD     Y1,B    X1,A        ;Find UGA2B = UGA2A+FA. UGA2B is in |4.12|
                                    ;  Save SIGPK to A
        MPY     B1,X0,B X:(R2)+,X0  ;Find UGA2B>>7, move zero ($0000) to X0 
                                    ;  (used if SIGPK = 1)
        TST     A       X:(R2)+,B0  ;Check SIGPK, get $0000 to A0
        TMI     X0,B                ;If SIGPK=1, set UGA2=0
        MOVE    X:(R3),Y0           ;Get 'A2'
        ADD     Y0,B    X:(R2)+,A   ;Find 'A2' + UGA2
                                    ;  get mask ($FF80) to mask 7 shifted bits.
        AND     Y0,A    X:(R2)+,Y0  ;Mask out 7 bits
                                    ;  get shift >> 7 ($0100)
        MAC     -A1,Y0,B X:(R2)+,X0 ;Find A2T = UGA2 + 'A2' - ULA2
                                    ;  Get -0.75 ($D000) in 16TCa format
                                    ;  for LIMC
END_UPA2_T

;*****************************************************************************
;NOTE:
;       Note that f(a1) <= 2. Equivalently, f(a1) can be limited to 1/2
;       and later multiplied by 4 (shifted left 2 times). This can be combined 
;       with the subsequent 7 right shifts, so later only a right shift of 5
;       will be needed.
;*****************************************************************************

;*****************************************************************************
;
;  Module Name     :  LIMC
;  Module Number   :  VII.3
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;      A2T = | si.ff ffff | ffff ffff | (16 TCa) in B1
;      R3 points to COEF_T+7 (A2)
;
; Output:                                                      
;      A2P = | si.ff ffff | ffff ffff | (16 TCa) in B1 & X:COEF_T+7
;      R3 points to COEF_T+6 (A1)
;
;***************************** MODULE DESCRIPTION ******************************
;
;       Limits the coefficient a2 of the second order predictor
;       |a2(k)| <= 0.75
;
;                        -0.75,      if a2u(k)<-0.75
;           a2p(k) =      0.75,      if a2u(k)>0.75
;                         a2u(k)     otherwise
;
;   SYMBOLS USED :
;             a2u(k)  <==>  A2T
;             a2p(k)  <==>  A2P
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;                                       
;        if A2T is  greater than 0.75 then set A2P  to 0.75
;        if A2T is less than -0.75 then set A2P to -0.75
;        else set A2P to A2T 
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 5
; NLOAC       : 7
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   -  c  u  c
;           1   -    -    -   -  c  -  -
;           2   c    -    -   -  c
;           3   c    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;*****************************************************************************
LIMC_T
            CMP     X0,B    X:(R2)+,Y0      ;Compare A2P to -0.75
                                            ;  Get 0.75 ($3000) in 16TCa format
            TLT     X0,B                    ;If A2T < -0.75, Set A2P=-0.75
            CMP     Y0,B                    ;Compare A2P to 0.75
            TGT     Y0,B                    ;If A2T > 0.75, Set A2T=0.75
            MOVE    B,X:(R3)-               ;Save A2P to A2 and make R3
                                            ;  point to A1.
END_LIMC_T

;******************************************************************************
;
;  Module Name     :  UPA1   
;  Module Number   :  VII.2.2
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input: 
;       A1    = | si.ff ffff | ffff ffff | (16 TCa) in X:COEF_T+6
;       SIGPK = | i000 0000  | 0000 0000 | (1 TC)  in X1
;       PK0   = | sxxx xxxx  | xxxx xxxx | (1 TC)
;       PK1   = | sxxx xxxx  | xxxx xxxx | (1 TC)
;       R3 points to COEF_T+6 (A1)
;
;       Note: PKS1 = PK0 XORed with PK1 is available in N0
;             (So PK0 and PK1 are really  not used)
;
; Output:
;       A1T   = | si.ff ffff | ffff ffff |   (16 TC) in A1
;       R3 points to COEF_T+7 (A2)
;
;***************************** MODULE DESCRIPTION ****************************
;               
;       Update the coefficient a1u(k) of second order predictor
;
;         a1u(k) = [1-(2**-8)] * a1(k-1) +
;                  3 * (2**-8) * sgn[p(k)] * sgn[p(k-1)] 
;
;         SIGPK just indicates whether p(k) is zero or not.
;
;         SIGPK = 1      if p(k) = 0
;               = 0      otherwise
;
;      In computation of a1u(k) second term is absent if SIGPK = 1
;
;      SYMBOLS USED : 
;                a1(k-1)     <==>  A1
;                sgn(p(k-1)  <==>  PK1
;                sgn(p(k))   <==>  PK0  &  SIGPK (refer ADDC)
;                a1u(k)      <==>  A1T
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;                                       
;               If PKS1=1 then
;                  Set UGA1 = -3*2**-8
;               Else
;                  Set UGA1 =  3*2**-8
;               If SIGPK = 1 
;                  Set UGA1 = 0
;               Get A1 in X1   (Note: A1 is a1(k-1) and not the accumulator)
;               Find ULA1M = A1*2**-8
;               Find UGA1PA1 = UGA1 + A1
;               Find ULA1 = -ULA1M
;               Find A1T  = UGA1PA1 + ULA1
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 13
; NLOAC       : 15
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    u   c  c  c  c
;           1   c    -    c   c  c  u  -
;           2   c    -    -   c  c
;           3   c    -    -
;
;         -  = not used   u = used & unchanged  c = used & changed
;
;*************************************************************************************
UPA1_T
          MOVE    N0,B                    ;Get PKS1 = PK0**PK1 from N0.
          TST     B       X:(R2)+,X0      ;Check PKS1, get -3*2**-8 in 16TCa
                                          ;  format ($FF40)
          TFR     X1,B    X:(R2)+,Y0      ;Move SIGPK to B, get 3*2**-8 in 
                                          ;  16TCa format ($00C0)
          TPL     Y0,A                    ;If PKS=0, Set UGA1=3*(2**-8)
          TMI     X0,A                    ;If PKS=1, Set UGA1=-3*(2**-8)
          TST     B       X:(R2)+,X0      ;Check SIGPK, Get 0 in X0 ($0000)
          TMI     X0,A                    ;If SIGPK = 1, set UGA1 = 0
                                          ;  else retain old value of UGA1.
          MOVE    X:(R2)+,N1              ;Load 15 in N1 to adjust pointer
                                          ;  R1 -> SR1 for FLOATB
          MOVE    X:(R2)+,Y0   X:(R3)+,X0 ;Get shift constant for >>8 ($0080)
                                          ;  Get A1 (not accum.) in X0
          MPY     Y0,X0,B  (R1)+N1        ;Find ULA1M =  (A1>>8),
                                          ;  R1->SR1 for FLOATB
          ADD     X0,A     X:(R2)+,B0     ;Find UGA1PA1 = UGA1 + A1
                                          ;  Set B0 to 0.
          NEG     B        X:(R3)+,X0     ;Find ULA1 = -ULA1M
                                          ;  get A2P (for LIMD)
          ADD     B,A      X:(R2)+,B      ;Find A1T = UGA1PA1+ULA1
                                          ;  get OME = (1-2**(-4)) in 
                                          ;  16 TCa $3C00
END_UPA1_T

;******************************************************************************
;
;  Module Name     :  LIMD
;  Module Number   :  VII.4
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   A1T = | si.ff ffff | ffff ffff | (16TCa) in A1
;   A2P = | si.ff ffff | ffff ffff | (16TCa) in X:COEF_T+7
;   R3 points to COEF_T+7 (A2)
;
; Output:
;   A1P = | si.ff ffff | ffff ffff | (16TCa) in A1 & X:COEF_T+6
;   R3 points to COEF_T+7 (A2)
;
;***************************** MODULE DESCRIPTION ****************************
;
;       Limits the coefficient a1(K) of the second order predictor a1u(k)
;       |a1(k)| <= [1-(2**-4)] - a2p(k)
;       NOTE :  |a2p(k)| is restricted to 0.75.
;
;               SYMBOLS USED :
;                              a1(k)  <==> A1T
;                              a2p(k) <==> A2P
;                              a1p(k) <==> A1P        
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;      Set  A1P = A1T
;      Find OME = [1-(2**-4)]
;      Subtract A2P from OME 
;      Compare A1T and (OME - A2P)
;      If A1T > (OME - A2P) then set A1P = OME - A2P
;      If A1T < ( -(OME - A2P)) then set A1P = ( -(OME - A2P)) 
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 6
; NLOAC       : 8
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   c  c  u  c
;           1   -    -    -   c  c  -  -
;           2   -    -    -   c  c
;           3   c    -    -
;
;         -  = not used   u = used & unchanged  c = used & changed   
;
;**************************************************************************
LIMD_T
        SUB     X0,B    (R3)-       ;Find A1UL=OME-A2P
                                    ;  R3 -> A2
        CMP     B,A     B,Y0        ;Check A1T > A1UL, move A1UL to B 
        TGT     Y0,A                ;If A1T>A1UL, set A1P=A1UL
        NEG     B       (R3)-       ;Find A1LL = -A1UL = A2P-OME
                                    ;  R3 -> A1
        CMP     B,A     B,Y0        ;Check A1T again
        TLT     Y0,A                ;If A1T < A1LL, set A1P = A1LL
END_LIMD_T

;******************************************************************************
;
;  Module Name     :  FLOATB
;  Module Number   :  VII.1.2
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   SR = | siii iiii | iiii iiii. |  (16TCb)  in  X:SR_T
;
; Output:
;   SR1 = (11FL : s mmmmmm eeee )
;     SR1EXP  = | 0000 0000 | 0000 eeee |    in  X:(R1)
;     SR1MANT = | 01mm mmm0 | 0000 0000 |    in  X:(R1+1)
;     SR1S    = | sXXX XXXX | XXXX XXXX |    in  X:(R1+2)
;
;************************ MODULE DESCRIPTION *********************************
;
;     This module converts the reconstructed signal, 
;     sr(k), from 16TC format (2's compliment) to a floating
;     point format (11FL).
;     This format has 11 bits: 1 bit for sign, 4 bits for 
;     exponent and 6 bits for mantissa.
;
;         SYMBOLS USED :
;                       sr(k) (input)    <==>   SR
;                       sr(k) (output)   <==>   SR1 (SR1S,SR1MANT,SR1EXP)
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;        Save the sign of SR
;        Find the magnitude, MAG, of SR
;        Test MAG for its value
;        If MAG = 0 :
;           Set the exponent to 0000.
;           Set the mantissa to 100000.
;           Set the sign to 0.
;        Else if MAG is not 0 :
;           Move 15 to the Shift Counter
;           Repeat until MAG is normalized :
;              Left-shift MAG (in B) and decrement Shift Counter
;        Truncate the normalized MAG to 6 bits, store as the mantissa(SR1MANT).
;        Store the contents of the Shift Counter as the exponent(SR1EXP).
;        Store the sign that was saved earlier(SR1S).
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 22
; NLOAC       : 27
;
; REGISTERS USED:
;
;                       R    M    N   A  B  X  Y
;                   0   c    -    -   u  c  c  c
;                   1   c    u    -   u  c  -  -
;                   2   c    -    c   u  c
;                   3   c    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;**************************************************************************
FLOATB_T
        MOVE    X:SR_T,B            ;Get SR.
        MOVE    X:(R2)+,R0          ;Load exponent bias 7.
        ABS     B       B,Y0        ;Find MAG=|SR|, save SRS (Sign of SR).
        TST     B       X:(R2)+,X0  ;Test MAG,
                                    ;  get mask $7E00 (to mask to 6 bits).
        BNE     <NORMSR_T           ;Jump if MAG is not 0.
        MOVE    #$4000,B            ;If MAG=0   * Set MANT to 100000.
        MOVE    #$0000,R0           ;           * Set EXP to 0.
        BRA     <TRUNCSR_T          ;           * Skip Normalization.
NORMSR_T
        ASL4    B                   ;If MAG!=0 normalize         
        NORM    R0,B                ;   to find MSB of MAG.
        ASL4    B
        NORM    R0,B
        NORM    R0,B
        NORM    R0,B
        NORM    R0,B
        NORM    R0,B
        NORM    R0,B
        NORM    R0,B
;------------------------------------------------------------------------
;   B1 = | 01XX XXXX | XXXX XXX0 | = normalized MAG (B2=B0=0)
;   R0 = | 0000 0000 | 0000 eeee | = exponent of normalized MAG
;------------------------------------------------------------------------
TRUNCSR_T
        AND     X0,B   A,X:(R3)+    ;Truncate MANT to 6 bits.
                                    ;  save A1P to A1 for LIMD
        MOVE    R0,X:(R1)+          ;Save EXP in SR1EXP.
        MOVE    B1,X:(R1)+          ;Save MANT in SR1MANT.
        MOVE    X:(R2)+,N1          ;Load offset 3.
        MOVE    Y0,X:(R1)+          ;Save SR in SR1S.
END_FLOATB_T

;******************************************************************************
;
;  Module Name     :  TONE 
;  Module Number   :  IX.1
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   A2P = | si.ff ffff | ffff ffff | (16TCa) in X:COEF_T+7
;   R3 points to COEF_T+7 (A2)
; Output:
;   TDP = | i000 0000  | 0000 0000 | (1TC) in X:DMS_T+2 & A1
;   R3 points to COEF_T+8
;
;***************************** MODULE DESCRIPTION *****************************
;                                                             
;   To obtain tone detect signal td(k)
;
;   td(k) = 1   if(a2(k) < -0.71875)
;         = 0   otherwise
;
;   SYMBOLS USED :
;                          a2(k) <==> A2P = Predictor coefficient
;                          td(k) <==> TDP = Tone detect signal
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;   Set TDP to 0
;   if(A2P < -0.71875)
;       set TDP to 1
;               
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 6
; NLOAC       : 7
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   c  c  c  -
;           1   -    -    -   c  c  -  -
;           2   c    -    -   c  c
;           3   c    -    -
;
;         -  = not used   u = used & unchanged  c = used & changed   
;
;**************************************************************************
TONE_T
        MOVE    X:(R3)+,B           ;Get A2P from memory
        CLR     A     X:(R2)+,X0    ;Set TDP=0
                                    ;  get '-0.71875' in 16TCa $D200 to X0
        CMP     X0,B  X:(R2)+,X0    ;Check A2P < -0.71875;
                                    ;  get '1.0' in 16TCa $8000 TO X0 
        TLT     X0,A                ;If A2P < -0.71875 set TDP=1, else TDP=0
        MOVE    A1,X:DMS_T+2        ;Save TDP
END_TONE_T        

;******************************************************************************
;
;  Module Name     :  TRIGB
;  Module Number   :  IX.2
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   TR  = | i000  0000 | 0000 0000 |   (1TC)   in  X:TR_T
;   TDP = | i000  0000 | 0000 0000 |   (1TC)   in  A1
;   BiP = | si.ff ffff | ffff ffff |  (16TCa)  in  X:COEF_T+(i-1)    i=1,...,6
;   AiP = | si.ff ffff | ffff ffff |  (16TCa)  in  X:COEF_T+6+(i-1)  i=1,2
;   R3 points to COEF_T+8
;
; Output:
;   BiR = | si.ff ffff | ffff ffff |   (16TCa) in  X:COEF_T+(i-1)    i=1,...,6
;   AiR = | si.ff ffff | ffff ffff |   (16TCa) in  X:COEF_T+6+(i-1)  i=1,2
;   TDR = | i000 0000  | 0000 0000 |    (1TC)  in  X:TD_T
;   R3 points to COEF_T+8
;
;***************************** MODULE DESCRIPTION ***************************
;
;        TRIGB checks TR(k) for a transition and, on detection,
;    sets the predictor coefficients and tone detect signal to zero.
;
;    if tr(k) = 1 
;                 (1) ai(k) = 0,      i = 1,2
;                 (2) bi(k) = 0,      i = 1,2,3,4,5,6
;                 (3) td(k) = 0  
;    If tr(k) = 0
;                 all values unaffected.
;    where,
;          tr(k)           = Tone transition detect signal
;          ai(k) and bi(k) = Predictor coefficients,
;                             i = 1,2
;                             i = 1,2,...6
;          td(k)           = Tone detect signal
;
;   SYMBOLS USED :
;                 ai(k) (input)  <==>  AiP
;                 ai(k) (output) <==>  AiR
;                 bi(k) (input)  <==>  BiP
;                 bi(k) (output) <==>  BiR
;                 tr(k)          <==>  TR
;                 td(k)          <==>  TD
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;                
;                
;             If (TR = 1) :
;                        For(i=1 to 6)
;                               BiR = 0.
;                        For(i=1 to 2)
;                               AiR = 0.
;                        TDR = 0.
;               
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 23
; NLOAC       : 19
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   c    -    c   c  c  c  -
;           1   c    -    u   c  c  -  -
;           2   -    -    -   c  c
;           3   c    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;**************************************************************************
TRIGB_T
        MOVE    #FIBASE,R0     ;Base address of FI in P memory moved to R0
        MOVE    X:IMAG,N0      ;|I| value from X memory moved to the offset
                               ;  above two will be used in FUNCTF,
                               ;  put here so that no intervening nop
                               ;  is needed before using N0.
        MOVE    #COEF_T,R3     ;Update R3 to point to B1  
        MOVE    X:TR_T,B       ;Get TR (transition detect signal)
        CLR     A      A,X0    ;Set A to zero, save TD in X0
        TST     B      (R1)+N1 ;Test TR.
                               ;  adjust R1 to point to DQ1.
        TEQ     X0,A           ;Restore TD if TR = 0
        MOVE    A,X:TD_T       ;Save TD in memory.
        BEQ     <END_TRIGB_T   ;If TR=0 skip the rest of the module.
                               ;If TR=1 :
        MOVE    A,X:(R3)+      ;         * Set B1,...,B6 to 0.
        MOVE    A,X:(R3)+      ;         * Set A1,A2 to 0.
        MOVE    A,X:(R3)+
        MOVE    A,X:(R3)+
        MOVE    A,X:(R3)+
        MOVE    A,X:(R3)+
        MOVE    A,X:(R3)+
        MOVE    A,X:(R3)+
END_TRIGB_T

;****************************************************************************
;
;  Module Name     :  FUNCTF
;  Module Number   :  V.1
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   I   = | siii 0000 | 0000 0000 |  (ADPCM format)
;
;   Note : Actually IMAG = |I| = | 0000 0000 | 0000 0iii | (which was saved 
;         in X:IMAG by module QUAN) is directly used. So, I is not used.
;
; Output:
;   FI  = | 0iii. 0000 | 0000 0000 | (3SM) in A1
;
;***************************** MODULE DESCRIPTION ******************************
;
; Map the quantizer output I(k) into FI(k)
;           ----------------------------------------------------------
;           | |I(k)| |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  
;           ---------------------------------------------------------
;           |  FI(k) |  0  |  0  |  0  |  1  |  1  |  1  |  3  |  7  |  
;           ----------------------------------------------------------
;
;   SYMBOLS USED :
;            I(k)   <==>  I
;            FI(k)  <==>  FI
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;    Get IMAG from memory
;    Set pointer to base of table containing FI values
;    Get FI from table using IMAG as offset
;                                                      
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 2
; NLOAC       : 3
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   u    -    u   c  -  -  -
;           1   -    -    -   c  -  -  -
;           2   -    -    -   c  -
;           3   -    -    -
;
;         -  = not used   u = used & unchanged  c = used & changed
;
;******************************************************************************
FUNCTF_T
        MOVE    X:(R0+N0),A    ;FI value for the corresponding I value moved
                               ;  to the accumulator A.
END_FUNCTF_T

;******************************************************************************
;
;  Module Name     :  FILTA
;  Module Number   :  V.2
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   FI   = | 0iii. 0000 | 0000 0000 | (3SM)  in A1
;   DMS  = | 0iii. ffff | ffff f000 | (12SM) in X:DMS_T
;
; Output:
;   DMSP = | 0iii. ffff | ffff f000 | (12SM) in A1 & X:DMS_T
;
;   Note: FI is also saved in B for use by module FILTB.
;
;***************************** MODULE DESCRIPTION *****************************
;
; Update short term average of F[I(k)]   
;
; dms(k) = (1 - (2**-5)) * dms(k-1) + 2**-5 * F[I(k)]
; dms(k) = (2**-5) * (FI(k) - dms(k-1)) + dms(k-1)
;
; F[I(k)]  = Weighted function of I(k).
; dms(k)   = Short term average of F[I(k)].
;
;  SYMBOLS USED : 
;           F[I(k)]   <==>  FI
;           dms(k-1)  <==>  DMS
;           dms(k)    <==>  DMSP
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
; 
;    Find DIFF = FI - DMS
;    Right shift DIFF by 5 bits to get 2**-5 * DIFF
;    Find  DMSP = (2**-5) * DIFF + DMS
;                          
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 8
; NLOAC       : 9
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   c    -    -   c  c  -  c
;           1   -    -    -   c  c  -  c
;           2   c    -    -   c  c
;           3   -    -    -
;
;         -  = not used   u = used & unchanged    c = used & changed
;                  
;******************************************************************************
FILTA_T
        MOVE    #DMS_T,R0                  ;R0 points to DMS
        MOVE    X:(R2)+,Y0                 ;Get shift constant for >>5 ($0400)
        MOVE    X:(R0)+,Y1                 ;Get DMS in reg. Y1
        SUB     Y1,A       A,B             ;Find DIFF = FI - DMS
                                           ;  and save FI in B
        MPY     Y0,A1,A                    ;Find  (2**-5) * DIFF
        ADD     Y1,A       X:(R2)+,Y0      ;Find DMSP = 2**-5*DIFF + DMS
                                           ;  Get mask for 12SM format($FFF8)
        AND     Y0,A                       ;Truncate DMS to 12SM
END_FILTA_T

;******************************************************************************
;
;  Module Name     :  FILTB
;  Module Number   :  V.3
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   FI   = | 0iii. 0000 | 0000 0000 |  (3SM)   in  B1
;   DML  = | 0iii. ffff | ffff fff0 |  (14SM)  in  X:DMS_T+1
;
; Output:
;   DMLP = | 0iii. ffff | ffff fff0 |  (14SM)  in  B1 & X:DMS_T+1
;
;***************************** MODULE DESCRIPTION ****************************
;
;    This module updates dml(k), the long term average of F[I(k)].
;    dml(k) is found out from the equation given below:
;
;       dml(k) = (1 - (2**(-7)) * dml(k-1) + 2**(-7) * F[I(k)]  
;              = 2**(-7)(FI(k) - dml(k-1)) + dml(k-1)
;       where
;             F[I(k)] = Weighted function of I(k).
;             dml(k)  = Long term average of F[I(k)].
;
;    SYMBOLS USED :
;                  dml(k-1)  <==>  DML
;                  dml(k)    <==>  DMLP
;                  F[I(k)]   <==>  FI
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
; 
;    Compute DIF   =  FI - DML
;    Compute DIFSX =  (2**-7)*DIF  by right shifting DIF 7 times.
;    Compute DMLP  =  DIFSX + DML.
;                          
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 5
; NLOAC       : 7
;
; REGISTERS USED:
;
;                       R    M    N   A  B  X  Y
;                   0   c    -    -   -  c  c  c
;                   1   -    -    -   -  c  -  -
;                   2   c    -    -   -  c
;                   3   -    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;******************************************************************************
FILTB_T
        MOVE    X:(R0)-,Y0             ;Get DML from memory
        SUB     Y0,B     X:(R2)+,X0    ;Find DIF=FI-DML,
                                       ;  get >>7 shift constant ($0100).
        MPY     B1,X0,B  X:(R2)+,X0    ;Find DFISX=DIF>>7,
                                       ; get mask for 15 bits ($FFFE).
        AND     X0,B     A,X:(R0)+     ;Truncate DIFSX to 15 bits.
                                       ;  save DMS for FILTA
        ADD     Y0,B                   ;Find DMLP=DIFSX+DML
END_FILTB_T

;******************************************************************************
;
;  Module Name     :  SUBTC
;  Module Number   :  V.4.1
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   Y    = | 0iii i.fff | ffff ff00 |  (13SM)   in  N3
;   DMSP = | 0iii. ffff | ffff f000 |  (12SM)   in  A1
;   DMLP = | 0iii. ffff | ffff fff0 |  (14SM)   in  B1
;   TDP  = | i000  0000 | 0000 0000 |   (1TC)   in  X:DMS_T+2
;
; Output:
;   AX   = | 0i0.0 0000 | 0000 0000 |   (1SM)   in  B1
;
;***************************** MODULE DESCRIPTION ******************************
;                                                   
;   This module computes ax(k) according to the equation :
;
;              ax(k) = 1,   if  (1) y(k) < 3,    OR
;                               (2) td(k) = 1,   OR 
;                               (3) |dms(k) - dml(k)| >= ((2**(-3)) * dml(k)).
;                    = 0,   otherwise.
;   where
;        ax(k)  = Speed control parameter update
;        y(k)   = Adaptation scale factor
;        dms(k) = Short term function of quantizer output
;        dml(k) = Long term function of quantizer output
;        td(k)  = Tone detect signal
;
;   SYMBOLS USED :
;                 y(k)    <==>  Y
;                 dms(k)  <==>  DMSP
;                 dml(k)  <==>  DMLP
;                 td(k)   <==>  TDP
;                 ax(k)   <==>  AX
;   *** Note : ax(k) = 1  is represented by  AX = '2'  in 1SM format.
;              ax(k) = 0  is represented by  AX = '0'  in 1SM format.
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;             Compute DIF  = DMS - DMLP
;             Compute DIFM = |DIF|
;             Compute DTHR = (2**-3) * DMLP
;             Set AX = 0
;             If (DIFM >= DTHR)
;                Set AX = 2
;             If (TDP = 1)
;                Set AX = 2
;             If (Y <= 3) )
;                Set AX = 2
;               
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 15
; NLOAC       : 17
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   c    -    -   c  c  c  c
;           1   -    -    -   c  c  -  -
;           2   c    -    -   c  c
;           3   -    -    u
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;******************************************************************************
SUBTC_T
        MOVE    B1,B                   ;Zero fill B0
        MOVE    A1,A                   ;Zero fill A0
        SUB     B,A      B,X:(R0)+     ;Find DIF=DMSP-DMLP.
                                       ;  save DML (for FILTB)
        ABS     A        X:(R2)+,X0    ;Find DIFM=|DIF|, get $1000 for >>3.
        MPY     B1,X0,B  X:(R2)+,X0    ;Find DTHR=DMLP>>3,
                                       ;  get truncation mask $7FFE for 14SM.
        AND     X0,B                   ;Truncate DTHR to 14SM.
        CLR     B        B,Y0          ;Get 0 in B (one possible value of AX),
                                       ;  save DTHR.
        CMP     Y0,A     X:(R2)+,X0    ;Compare DIFM with DTHR,
                                       ;  get $4000 (`2' in 1SM format).
        TGE     X0,B                   ;If (DIFM >= DTHR) set AX=2.
        MOVE    X:(R0)+,A              ;Get TDP.
        TST     A        X:(R2)+,Y0    ;Check TDP.
                                       ;  get $1800 (`3' in 13SM format).
        MOVE    N3,A                   ;Get Y.
        TNE     X0,B                   ;If (TDP != 0) set AX=1
        CMP     Y0,A    X:(R0)+,Y0     ;Check for Y<=3.
                                       ;  Get AP for FILTC
        TLE     X0,B                   ;If Y<=3 set AX=1
END_SUBTC_T

;******************************************************************************
;
;  Module Name     :  FILTC
;  Module Number   :  V.4.2
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   AX  = | 0i0.0 0000 | 0000 0000 |  (1SM)   in  B1
;   AP  = | 0ii.f ffff | fff0 0000 |  (10SM)  in  Y0
;
; Output:
;   APP = | 0ii.f ffff | fff0 0000 |  (10SM)  in  B1
;
;***************************** MODULE DESCRIPTION ****************************
;
;    This module is a low pass filter of ap(k), the speed control parameter.
;    Filtering is done according to the equation  :
;       app(k) = (1 - 2**(-4)) * ap(k-1)  +  2**(-4) * ax(k)
;              = (2**(-4)) * (ax(k) - ap(k-1))  +  ap(k-1)
;       where
;             app(k) = Filter output
;             ap(k-1)  = Speed control parameter
;             ax(k)    = Shift constant; takes a value 0 or 2.
;
;    SYMBOLS USED :
;                  app(k)  <==>  APP
;                  ap(k-1) <==>  AP
;                  ax(k)   <==>  AX
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;   
;   Compute   DIF   = (AX - AP)
;   Compute   DIFSX = (2**-4) * DIF  by 4 right shifts.
;   Compute   APP   = DIFSX + AP
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 6
; NLOAC       : 7
;
; REGISTERS USED:
;
;                       R    M    N   A  B  X  Y
;                   0   c    -    u   -  c  c  u
;                   1   -    -    -   -  c  -  -
;                   2   c    -    -   -  c
;                   3   -    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;******************************************************************************
FILTC_T                                    
        MOVE    #WIBASE,R0         ;Base address of WI in P memory moved to R0
                                   ;  this will be used in FUNCTW,
                                   ;  put here so that no intervening nop
                                   ;  is needed before using R0.
        SUB     Y0,B   X:(R2)+,X0  ;Find DIF=AX-AP,
                                   ;  get mask $FFE0 for 10SM truncation.
        ASR4    B                  ;Find DIFSX=DIF>>4.
        AND     X0,B   (R0)+N0     ;Truncate DIFSX.
                                   ;  adjust pointer to correct
                                   ;  value of WI
        ADD     Y0,B               ;Find APP=DIFSX+AP.
END_FILTC_T

;******************************************************************************
;
;  Module Name     :  TRIGA
;  Module Number   :  V.4.3
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   TR  = | i000  0000 | 0000 0000 |  (1TC)   in  X:TR_T
;   APP = | 0ii.f ffff | fff0 0000 |  (10SM)  in  B1
;
; Output:
;   APR = | 0ii.f ffff | fff0 0000 |  (10SM)  in  B1 & X:DMS_T+3
;
;************************** MODULE DESCRIPTION ******************************
;
;    The output of this module is the updated value of ap(k), the
;    speed control parameter.
;    Updating depends on the detection of tone transition according 
;    to the equation :
;           ap(k) = app(k),  if tr(k) = 0
;                 = 1,       if tr(k) = 1
;           where
;                app(k) = Speed control parameter
;                ap(k)  = Updated Speed coontrol parameter
;                tr(k)  = Tone transition detect signal
;
;    SYMBOLS USED :
;                  app(k) <==>  APP
;                  ap(k)  <==>  APR
;                  tr(k)  <==>  TR
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;          Set APR = APP
;          If (TR = 1)
;            Set APR = 1
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 6
; NLOAC       : 6
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   c  c  c  -
;           1   -    -    -   c  c  -  -
;           2   c    -    -   c  c
;           3   -    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;******************************************************************************
TRIGA_T
        MOVE    X:TR_T,A          ;Get TR.
        TST     A      X:(R2)+,X0 ;Check TR, get `1' in 10SM format($2000).
        TMI     X0,B              ;If TR=1 set APR=1, else APR=APP.
        MOVE    B1,X:DMS_T+3      ;Save APR to AP.
END_TRIGA_T

;******************************************************************************
;
;  Module Name     :  FUNCTW
;  Module Number   :  VI.1
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   I  = | siii. 0000 | 0000 0000 | (ADPCM format)
; 
;   Note: R0 already points to correct value of WI and is used to fetch WI
;
; Output:
;   WI = | siii iiii. | ffff 0000 | (12TCb) in A1
;
;**************************** MODULE DESCRIPTION ******************************
;
;  Map the quantizer output into logarithmic version of scale factor multiplier
;  for getting the output WI(k).
;
;      -----------------------------------------------------------------------
;      | |I(k)|  |    0  |   1  |   2  |  3   |  4   |   5   |    6  |  7    |
;      -----------------------------------------------------------------------
;      | W[I(k)] | -0.75 | 1.13 | 2.56 | 4.00 | 7.00 | 12.38 | 22.19 | 70.13 |
;      -----------------------------------------------------------------------
; 
;  SYMBOLS USED :
;                              I(k)    <==> I
;                              W[I(k)] <==> WI
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;    Get IMAG from memory
;    Set pointer to base of table containing WI values
;    Get WI from table using IMAG as offset
;                          
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 1
; NLOAC       : 3
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   c    -    -   c  -  -  -
;           1   -    -    -   c  -  -  -
;           2   -    -    -   c  -
;           3   -    -    -
;
;         -  = not used   u = used & unchanged  c = used & changed    
;
;**************************************************************************
FUNCTW_T
        MOVE    X:(R0)+,A      ;WI value for the corresponding I value moved
                               ;  to the accumulator A.
END_FUNCTW_T

;******************************************************************************
;
;  Module Name     :  FILTD
;  Module Number   :  VI.2
;
;*************************** INPUT AND OUTPUT *******************************
;                                    
; Input:
;   WI  = | siii iiii. | ffff f000 |  (12TCb) in A1
;   Y   = | 0iii i.fff | ffff ff00 |  (13SM)  in N3
;
; Output:
;   YUT = | 0iii i.fff | ffff ff00 |   (13SM)  in A1
;
;***************************** MODULE DESCRIPTION ****************************
;
;   The purpose of this module is to update the fast quantizer scale 
;   factor. The output of this module is yuu(k).   
;
;            yuu(k) = (1 - 2**(-5))*y(k) + 2**(-5) * W[I(k)]
;                   = y(k) + (2**-5) * (W[I(k)] - y(k))   
;    where
;       yuu(k)  = Unlimited fast quantizer scale factor.
;       y(k)    = quantizer scale factor
;       W[I(k)] = Quantizer Multiplier
;
;     SYMBOLS USED :
;               W[I(k)]  <==>  WI
;               y(k)     <==>  Y
;               yuu(k)   <==>  YUT
; 
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
; 
;    Find DIFF = WI - Y 
;    YU = Y+ 2**-5 * DIFF
;                          
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 8
; NLOAC       : 10
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   -    -    -   c  c  c  c
;           1   -    -    -   c  c  -  c
;           2   c    -    -   c  c
;           3   -    -    u
;
;         -  = not used   u = used & unchanged  c = used & changed
;
;******************************************************************************
FILTD_T
        MOVE    N3,B                   ;Get Y in accum. B
        MOVE    X:(R2)+,X0             ;Get shift constant for >>3 ($1000)
        MPY     X0,B1,B     B,Y1       ;Shift Y right by 3 bits to align the
                                       ;  binary point of WI and Y.
                                       ;  Save Y in Y1.
        SUB     B,A       X:(R2)+,Y0   ;Find DIFF = WI - Y, get shift constant
                                       ;  for >>2 ($2000) in Y0.
        MPY     A1,Y0,A                ;Shift DIFF right by 2 bits to get
                                       ;  DIFF*2**-5 in 13 SM format. Actually
                                       ;  DIFF without any shift itself is
                                       ;  already  multiplied  by  2**-3  if
                                       ;  interpreted in 13 SM.
        MOVE    A1,A                   ;Zero fill A0.
        ADD     Y1,A      X:(R2)+,X0   ;Find YU = Y + 2**-5*DIFF, get trunc.
                                       ;  mask for 13SM format ($FFFC)
        AND     X0,A      X:(R2)+,X0   ;  YU in 13SM format.
                                       ;  get upper limit (10.0) in 13SM
                                       ;  ($5000) for LIMB
END_FILTD_T

;******************************************************************************
;
;  Module Name     :  LIMB
;  Module Number   :  VI.3
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   YUT = | 0iii i.fff | ffff ff00 |  (13SM) in A1
;
; Output:
;   YUP = | 0iii i.fff | ffff ff00 |  (13SM) in A1 & X:YU_T
;
;***************************** MODULE DESCRIPTION ****************************
;
; This module restricts yuu(k) within the limits given below to get yu(k).
;
;       1.06  <=  yu(k)   <=  10.00
;         i.e.
;               yu(k) = 1.06   , yuu(k) < 1.06
;                     = yuu(k) , 1.06 <= yuu(k) <= 10.0   
;                     = 10.0   , yuu(k) > 10.0
;
;   SYMBOLS USED :
;                     yuu(k) <==> YUT = Fast scale factor (unlimited)
;                     yu(k)  <==> YUP = Fast scale factor (limited)
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;    Set YUP = YUT
;    If (YUP > 10.0)
;       YUP = 10.0
;    If (YUP < 1.06)
;        YUP = 1.06
;
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 7
; NLOAC       : 8
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   c    -    -   c  -  u  c
;           1   -    -    -   c  -  -  -
;           2   c    -    -   c  -
;           3   -    -    -
;
;         -  = not used   u = used & unchanged c = used & changed
;
;******************************************************************************
LIMB_T
        MOVE    #YU_T,R0            ;Load pointer to YU and YL in R0
        CMP     X0,A  X:(R2)+,Y0    ;Check for YU > 10.0
                                    ;  get lower limit (1.06) in 13SM ($0880)
        TGT     X0,A                ;If YU>10 set YU = 10.0
        CMP     Y0,A                ;Check for YU < 1.06
        TLT     Y0,A                ;If YU < 1.06 set YU = 1.06
        MOVE    A1,X:(R0)+          ;Save YU in memory         
END_LIMB_T        

;******************************************************************************
;
;  Module Name     :  FILTE
;  Module Number   :  VI.5
;
;*************************** INPUT AND OUTPUT *******************************
;
; Input:
;   YUP = | 0iii i.fff | ffff ff00 (13SM) in A1
;   YL  = | 0iii i.fff | ffff ffff | ffff 0000 | 0000 0000 | (19SM) 
;         | <--------------------> | <-------------------> |
;                X:YU_T+1                 X:YU_T+2      in X:YU_T+1 & X:YU_T+2
; Output:
;   YLP = | 0iii i.fff | ffff ffff | ffff 0000 | 0000 0000 | (19SM) 
;         | <--------------------> | <-------------------> |
;                X:YU_T+1                 X:YU_T+2      in X:YU_T+1 & X:YU_T+2
;
;***************************** MODULE DESCRIPTION ****************************
;
;    This module updates the slow quantizer scale factor, yl(k)
;      yl(k) = (1 - 2**(-6)) * yl(k-1) + 2**(-6) * yu(k)
;      yl(k) =  yl(k-1) + 2**(-6) * (yu(k) -yl(k-1))     
;    where,
;      yl(k) = Slow scale factor.
;      yu(k) = Fast scale factor.
;
;    SYMBOLS USED:
;              yu(k)    <==>  YUP
;              yl(k-1)  <==>  YL
;              yl(k)    <==>  YLP
;
;******************* STRUCTURED ENGLISH OR PSEUDO CODE ************************
;
;    Get YU and YL.    
;    Find DIFF = YU - YL        
;    Find YLP = (2**-6 * DIFF) + YL
;                          
;******************************** RESOURCES ***********************************
;
; CYCLE COUNT : 10
; NLOAC       : 12
;
; REGISTERS USED:
;
;               R    M    N   A  B  X  Y
;           0   c    -    -   c  c  -  c
;           1   -    -    -   c  c  -  -
;           2   c    -    -   c  c
;           3   -    -    -
;
;         -  = not used   u = used & unchanged   c = used & changed
;
;******************************************************************************
FILTE_T
        MOVE    X:(R0)+,B1                ;Get 16 MSBs of YL
        MOVE    X:(R0)-,B0              ;Get last 4 bits of YL
        SUB     B,A                      ;DIFF = YU - YL in A
        ASR4    A                        ;Move A right by 4 bits.
        ASR     A                        ;Move A right by 1 bit.
        ASR     A                        ;Move A right by 1 bit.
;-------------------------------------------------------------------------
; Now A contains DIFF*(2**-6)
; 6 right shifts were used in place of multiplication to achieve the same,
; because 20 bit multiplication is not directly possible
;--------------------------------------------------------------------------
        ADD     A,B     X:(R2)+,A        ;Get YLP in B, get mask for last
                                         ;  4 bits of 19SM format ($F000)
        MOVE    B0,Y0                    ;Get B0 to Y0 which can be ANDed
                                         ;  with A to get last 4 bits of 19 SM.
;----------------------------------------------------------------------------
; This MOVE is needed because AND B0,A is not allowed.
;----------------------------------------------------------------------------
        AND     Y0,A   B,X:(R0)+         ;Get last 4 bits of YL(in 19SM) to A1
                                         ;  save 16 MSBs of YL in memory
        MOVE    A1,X:(R0)                ;Save last 4 bits of YL in memory.
END_FILTE_T

        MOVE    R1,X:DATAPTR_T           ;Save pointer to DQ1EXP of DATA_T 
                                         ;  circular buffer for initialization
                                         ;  at the next ENCODER loop.
        RTS  
;-----------------------------------------------------------------------------

;********** INITIALIZATION OF X MEMORY CONSTANTS & VARIABLES ******************

INIT
        CLR     A                   ;Clear Accumulator A
        MOVE    #0,R0
        MOVE    #$01FF,R1
        REP     R1
        MOVE    A,X:(R0)+           ;Clear internal X memory RAM

; X memory initialization

        MOVE    #$0880,X0
        MOVE    X0,X:YU_T           ;Initialize YU,
        MOVE    X0,X:YU_T+1         ; & YL

        MOVE    #$3E20,X0
        MOVE    X0,X:DMS_T+3        ;Initialize AP

        MOVE    #DATA_T,X0          ;Set transmit data pointer
        MOVE    X0,X:DATAPTR_T      ; to start of transmit data buffer

        MOVE    #RSHFT,R3           ;Transfer RSHFT tables to X memory
        CLR     A
        MOVE    #$8000,A1
        REP     #27
        ASR     A       A,X:(R3)+
        MOVE    #TABINIT,R0         ;Transfer constant tables from program
        REP     #126                ;  memory to data memory     
        MOVE    P:(R0)+,X:(R3)+     

        MOVE    #DATA_T+1,R3
        MOVE    #$4000,X0
        MOVE    #$8000,X1
        DO      #6,XDBUFFER
        MOVE    X0,X:(R3)+          ;Initialize DQnMANT and DQnS
        MOVE    X1,X:(R3)+          ; in the transmit data buffer
        MOVE    X:(R3)+,Y0          ;Dummy read to increment R3
XDBUFFER
        MOVE    X0,X:(R3)+          ;Initialize SR1MANT in transmit data buffer
        MOVE    X:(R3)+,Y0          ;Dummy read to increment R3
        MOVE    X:(R3)+,Y0          ;Dummy read to increment R3
        MOVE    X0,X:(R3)+          ;Initialize SR2MANT in transmit data buffer

;Mu-law/A-law selection

        MOVE    #<$10,Y1            ;Set LAW to 0 for Mu-law
        MOVE    Y1,X:LAW            ;(Anything else for A-law)
        TST     Y1                  ;Check LAW  
        BEQ     SKIP_A              ;If Mu-law, skip A-law initialization  

        MOVE    #$0002,Y0           ;Initialization for A-law CCITT sequences 
        MOVE    Y0,X:DATA_T+6
        MOVE    Y0,X:DATA_T+18
        MOVE    Y0,X:DATA_T+21
        MOVE    X1,X:DATA_T+20
        MOVE    X1,X:DATA_T+23

        MOVE    #$1064,X0
        MOVE    X0,X:COEF_T
        MOVE    #$104C,X0
        MOVE    X0,X:COEF_T+1
        MOVE    #$0FE9,X0
        MOVE    X0,X:COEF_T+2
        MOVE    #$0FDF,X0
        MOVE    X0,X:COEF_T+3
        MOVE    #$1040,X0
        MOVE    X0,X:COEF_T+4
        MOVE    #$104A,X0
        MOVE    X0,X:COEF_T+5
        MOVE    #$1C95,X0
        MOVE    X0,X:COEF_T+6
        MOVE    #$FAFE,X0
        MOVE    X0,X:COEF_T+7

        MOVE    X1,X:PK_T
        MOVE    X1,X:PK_T+1
SKIP_A
        MOVE    #$FFFF,M0           ;Set R0 for linear addressing
        MOVE    #$FFFF,M2           ;Set R2 for linear addressing
        MOVE    #$FFFF,M3           ;Set R3 for linear addressing
        MOVE    #-8,N2              ;Set N2 to -8 for use after FMULT
        MOVE    #23,M1              ;Set R1 for modulo 24 addressing
        MOVE    #3,N1               ;Set N1 for increment of 3 for XOR/UPB
        RTS

;****************** CONSTANTS USED IN ENCODER *********************************

TABINIT

;FM_CONS
        DC      $7E00
        DC      $0180
        DC      $7F80
        DC      $001A
        DC      $7FFF
        DC      $0000
        DC      $7FFC
        DC      $0009
;QUANTAB
        DC      $f840   ;-0.98
        DC      $0500   ;0.62
        DC      $0b20   ;1.38
        DC      $0f60   ;1.91
        DC      $12c0   ;2.34
        DC      $15d0   ;2.72
        DC      $1900   ;3.12
        DC      $7fff   ;15.99
;IQUANTAB
        DC      $8000   ;-15.99
        DC      $0040   ;0.031
        DC      $0870   ;1.05
        DC      $0D50   ;1.66
        DC      $1110   ;2.13
        DC      $1430   ;2.52
        DC      $1750   ;2.91
        DC      $1A90   ;3.32
;FIBASE 
        DC      $0000   ;0
        DC      $0000   ;0
        DC      $0000   ;0
        DC      $1000   ;1
        DC      $1000   ;1
        DC      $1000   ;1
        DC      $3000   ;3
        DC      $7000   ;7
;WIBASE
        DC      $FF40   ;-0.75
        DC      $0120   ;1.13
        DC      $0290   ;2.56
        DC      $0400   ;4.0
        DC      $0700   ;7.0
        DC      $0C60   ;12.38
        DC      $1630   ;22.19
        DC      $4620   ;70.13
;SEGTAB
        DC      $E000
        DC      $0000
        DC      $2000
        DC      $4000
        DC      $6000
        DC      $8000
        DC      $A000
        DC      $C000
;CONST
        DC      $2000       ;lima
        DC      $7f00

        DC      $7FFC       ;mix
        DC      $FFFC
        DC      $7FFE

        DC      $5500       ;expand        
        DC      $7000
        DC      $0F00
        DC      $0008
        DC      $0108
        DC      $0400
        DC      $0084

        DC      $0006       ;log
        DC      $3F80
        DC      $0800
        DC      $1000

        DC      $7FF0       ;subtb

        DC      $1000       ;quan
        DC      $F000

        DC      $FFF0       ;adda

        DC      $0000       ;antilog
        DC      $0010
        DC      $4000
        DC      $8000

        DC      $0080       ;xor
        DC      $FF80
        DC      $FF00

        DC      $0006       ;floata
        DC      $7e00

        DC      $0010       ;trans
        DC      $0008
        DC      $5D00
        DC      $07C0
        DC      $0003
        DC      $3000
;CONST_TRANS
        DC      $8000       ;used in the middle of trans
;CONST_ADDC
        DC      $0000       ;addc
        DC      $8000

        DC      $1000       ;upa2
        DC      $F000
        DC      $1FFF
        DC      $E001
        DC      $0400
        DC      $0000
        DC      $0000
        DC      $FF80
        DC      $0100

        DC      $D000       ;limc
        DC      $3000

        DC      $FF40       ;upa1
        DC      $00C0       
        DC      $0000
        DC      $000f       ;for FLOATB
        DC      $0080
        DC      $0000

        DC      $3C00       ;limd

        DC      $0007       ;floatb
        DC      $7e00       
        DC      $0003


        DC      $D200       ;tone
        DC      $8000

        DC      $0400       ;filta
        DC      $FFF8

        DC      $0100       ;filtb
        DC      $fffe

        DC      $1000       ;subtc
        DC      $7ffe
        DC      $4000
        DC      $1800

        DC      $ffe0       ;filtc

        DC      $2000       ;triga
        
        DC      $1000       ;filtd
        DC      $2000
        DC      $FFFC

        DC      $5000       ;limb
        DC      $0880
        
        DC      $f000       ;filte        
;DUMMY
        DC      $BABA       ;This is used to check that all constants have
                            ;  been moved properly to X memory. At the end of
                            ;  INIT, X:DUMMY should contain $BABA.
