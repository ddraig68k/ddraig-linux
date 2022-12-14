#if 0
;------------------------------------------------------------------------------
;
;   File    :   MPEGSUBB.a
;
;   Author  :   St?phane TAVENARD
;
;   (C) Copyright 1997-1998 St?phane TAVENARD
;       All Rights Reserved
;
;   #Rev|   Date   |                      Comment
;   ----|----------|--------------------------------------------------------
;   0   |10/04/1997| Initial revision                                     ST
;   1   |18/05/1997| Optimized windowing for 060                         ST
;   2   |01/11/1997| Use link instead of static vars                      ST
;   3   |08/04/1998| Use overflow optimization from ... (IRC man)         ST
;   4   |21/06/1998| Use external dewindow                                ST
;
;   ------------------------------------------------------------------------
;
;   MPEG SUBroutines optimized !
;
;------------------------------------------------------------------------------
#endif
#if 0
               XDEF     @MPEGSUBB_antialias
               XDEF     _MPEGSUBB_antialias

               XDEF     @MPEGSUBB_filter_band
               XDEF     _MPEGSUBB_filter_band

               XDEF     @MPEGSUBB_window_band
               XDEF     _MPEGSUBB_window_band

;               XDEF     @MPEGSUBB_config ; #2
;               XDEF     _MPEGSUBB_config ; #2

               section  ASMCODE,code
#endif

#if 0
INT_FACTOR     equ      14

ALIAS_BITS     equ      15
#else
#define	INT_FACTOR	14
#define	ALIAS_BITS	15
#endif

#if 0
;
; ALIAS_U bu,csi,bd,cai,<tmp reg>,<reg>
; performs: (bu * csi) - (bd * cai) -> <reg>
; <tmp reg> is used for calculation
; d6 must be set to ALIAS_BITS
;
ALIAS_U        MACRO
               move.w   \1,\6
               muls.w   \2,\6
               move.w   \3,\5
               muls.w   \4,\5
               sub.l    \5,\6
               asr.l    d6,\6
               ENDM
#else
#define	ALIAS_U	move.w   $$1,$$6 \
               muls.w   $$2,$$6 \
               move.w   $$3,$$5 \
               muls.w   $$4,$$5 \
               sub.l    $$5,$$6 \
               asr.l    %d6,$$6
#endif
#endif
#if 0
;
; ALIAS_D bd,csi,bu,cai,<tmp reg>,<reg>
; performs: (bd * csi) + (bu * cai) -> <reg>
; <tmp reg> is used for calculation
; d6 must be set to ALIAS_BITS
;
ALIAS_D        MACRO
               move.w   \1,\6
               muls.w   \2,\6
               move.w   \3,\5
               muls.w   \4,\5
               add.l    \5,\6
               asr.l    d6,\6
               ENDM

bd             equr             a1
bu             equr             a2

#else
#define ALIAS_D	move.w   $$1,$$6 \
               muls.w   $$2,$$6 \
               move.w   $$3,$$5 \
               muls.w   $$4,$$5 \
               add.l    $$5,$$6 \
               asr.l    %d6,$$6

#define	bd	%a1
#define	bu	%a2

#endif
	
#if 0
;
; ALIAS_B csi,cai
; perform 1 alias butterfly
; uses d0..d3
;
ALIAS_B            MACRO
               move.w   -(bu),d2
               move.w   (bd),d3
               ALIAS_U  d2,\1,d3,\2,d0,d1
               move.w   d1,(bu)
               ALIAS_D  d3,\1,d2,\2,d0,d1
               move.w   d1,(bd)+
               ENDM

#else
#endif
#if 0
; Layer III antialiasing filter
;
; a0 = sample buffer xr (INT16)
; d0 = sblimit
;
@MPEGSUBB_antialias
_MPEGSUBB_antialias

               movem.l  d2-d7/a2,-(sp)

               move.w   d0,d7
               subq.w   #1,d7
               bmi      antialias2
               clr.l    d5              ; d5 =i = 0
               moveq.l  #ALIAS_BITS,d6  ; for alias mult
antialias1
               add.l    #18,d5          ; i += SSLIMIT
               lea      (a0,d5.w*2),bu  ; bu = xr[ i ]
               move.l   bu,bd           ; bd = bu
               ALIAS_B  #28098,#-16858
               ALIAS_B  #28892,#-15457
               ALIAS_B  #31117,#-10268
               ALIAS_B  #32221,#-5960
               ALIAS_B  #32621,#-3099
               ALIAS_B  #32740,#-1342
               ALIAS_B  #32764,#-465
               ALIAS_B  #32767,#-121
               dbra     d7,antialias1
antialias2
               movem.l  (sp)+,d2-d7/a2
               rts



ps1            equr     a3
ps2            equr     a4
pd1            equr     a5
;pd2            equr     a6 ; #2

#else
#endif
#if 0
;
;              Filter COS values for fast cosine transform
;
SH             equ      1      ; In order to preserve bits in multiply
MUL_SHIFT      equ      16-SH

COS1_64        equ      $8028>>SH
COS3_64        equ      $8167>>SH
COS5_64        equ      $83F4>>SH
COS7_64        equ      $87F2>>SH
COS9_64        equ      $8D98>>SH
COS11_64       equ      $953B>>SH
COS13_64       equ      $9F5C>>SH
COS15_64       equ      $ACC0>>SH
COS17_64       equ      $BE9A>>SH
COS19_64       equ      $D6E0>>SH
COS21_64       equ      $F8FA>>SH
COS23_64       equ      $12B60>>SH
COS25_64       equ      $17BF2>>SH
COS27_64       equ      $20ECB>>SH
COS29_64       equ      $36859>>SH
COS31_64       equ      $A30A4>>SH
COS1_32        equ      $809F>>SH
COS3_32        equ      $85C2>>SH
COS5_32        equ      $9123>>SH
COS7_32        equ      $A596>>SH
COS9_32        equ      $C9C5>>SH
COS11_32       equ      $10F89>>SH
COS13_32       equ      $1B8F2>>SH
COS15_32       equ      $519E5>>SH
COS1_16        equ      $8282>>SH
COS3_16        equ      $99F2>>SH
COS5_16        equ      $E665>>SH
COS7_16        equ      $2901B>>SH
COS1_8         equ      $8A8C>>SH
COS3_8         equ      $14E7B>>SH
COS1_4         equ      $B505>>SH

MUL32          MACRO
               muls.l   \1,\2
               asr.l    d6,\2
               ENDM

FFF_MUL        MACRO
               move.l   (ps1)+,d0
               move.l   d0,d1
               move.l   -(ps2),d2
               add.l    d2,d1
               move.l   d1,(pd1)+
               sub.l    d2,d0
               MUL32    #\1,d0
               move.l   d0,\2*4-4(pd1)
               ENDM

#else
#endif
#if 0
;
;              FF4_MUL s1,s2,s3,s4
;              s1..s4 must be registers
;              d0, s1, s2 modified
;
;              s1 + s4 -> (pd1)+
;              s2 + s3 -> (pd1)+
;              (s1 - s4) * COS1_8 -> (pd1)+
;              (s2 - s3) * COS3_8 -> (pd1)+
;
FF4_MUL        MACRO
               move.l   \1,d0
               add.l    \4,d0
               move.l   d0,(pd1)+
               move.l   \2,d0
               add.l    \3,d0
               move.l   d0,(pd1)+
               sub.l    \4,\1
               MUL32    #COS1_8,\1
               move.l   \1,(pd1)+
               sub.l    \3,\2
               MUL32    #COS3_8,\2
               move.l   \2,(pd1)+
               ENDM

#else
#endif
#if 0
;
;              FF2_MUL s1,s2,COSx
;              s1..s2, COSx must be registers
;              d0, s1 modified
;
;              s1 + s2 -> (pd1)+
;              (s1 - s2) * COSx -> (pd1)+
;
FF2_MUL        MACRO
               move.l   \1,d0
               add.l    \2,d0
               move.l   d0,(pd1)+
               sub.l    \2,\1
               MUL32    \3,\1
               move.l   \1,(pd1)+
               ENDM

fast_filter_sub
;               lea      filter_p,ps1   ; ps1=@p(0)
               lea      -32*4(a6),ps1   ; ps1=@p(0)
               lea      16*4(ps1),ps2  ; ps2=@p(16)
;               lea      filter_pp,pd1  ; pd1=@pp(0)
               lea      -16*4(a6),pd1  ; pd1=@pp(0)
               FFF_MUL  COS1_32,8      ; pp(i=0..7)  = p(i) + p(15-i)
               FFF_MUL  COS3_32,8      ; pp(i=8..15) = COSx*[p(i) - p(15-i)]
               FFF_MUL  COS5_32,8
               FFF_MUL  COS7_32,8
               FFF_MUL  COS9_32,8
               FFF_MUL  COS11_32,8
               FFF_MUL  COS13_32,8
               FFF_MUL  COS15_32,8

;               lea      filter_pp,ps1  ; ps1=@pp(0)
               lea      -16*4(a6),ps1  ; ps1=@pp(0)
               lea      8*4(ps1),ps2   ; ps2=@pp(8)
;               lea      filter_p,pd1   ; pd1=@p(0)
               lea      -32*4(a6),pd1   ; pd1=@p(0)
               FFF_MUL  COS1_16,4      ; p(i=0..3) = pp(i) + pp(7-i)
               FFF_MUL  COS3_16,4      ; p(i=4..7) = COSx*[pp(i) - pp(7-i)]
               FFF_MUL  COS5_16,4
               FFF_MUL  COS7_16,4
               lea      4*4(ps1),ps1   ; ps1=@pp(8)
               lea      8*4(ps1),ps2   ; ps2=@pp(16)
               lea      4*4(pd1),pd1   ; pd1=@p(8)
               FFF_MUL  COS1_16,4
               FFF_MUL  COS3_16,4
               FFF_MUL  COS5_16,4
               FFF_MUL  COS7_16,4

;               lea      filter_p,ps1   ; ps1=@p(0)
               lea      -32*4(a6),ps1   ; ps1=@p(0)
;               lea      filter_pp,pd1  ; pd1=@pp(0)
               lea      -16*4(a6),pd1  ; pd1=@pp(0)
               movem.l  (ps1)+,d1-d4   ; get p0..p3
               FF4_MUL  d1,d2,d3,d4
               movem.l  (ps1)+,d1-d4   ; get p4..p7
               FF4_MUL  d1,d2,d3,d4
               movem.l  (ps1)+,d1-d4   ; get p8..p11
               FF4_MUL  d1,d2,d3,d4
               movem.l  (ps1)+,d1-d4   ; get p12..p15
               FF4_MUL  d1,d2,d3,d4

;               lea      filter_pp,ps1  ; ps1=@pp(0)
               lea      -16*4(a6),ps1  ; ps1=@pp(0)
;               lea      filter_p,pd1   ; pd1=@p(0)
               lea      -32*4(a6),pd1   ; pd1=@p(0)
               move.l   #COS1_4,d3
               moveq.w  #8-1,d4
fast_filter_s2
               movem.l  (ps1)+,d1-d2
               FF2_MUL  d1,d2,d3
               dbra     d4,fast_filter_s2

               rts

#else
#endif
#if 0
;
; GET_P <index> <dest ea>
; copy p[ index ] into a destination <ea>
; p is ps1
;
GET_P          MACRO
               move.l   \1*4(ps1),\2
               ENDM
#else
#endif
#if 0
;
; ADD_P <index> <dest ea>
; add p[ index ] to a destination <ea>
; p is ps1
;
ADD_P          MACRO
               add.l    \1*4(ps1),\2
               ENDM

#else
#endif
#if 0
;
; SET_S0 <index> <src ea>
; copy a source <ea> into s0[ index ]
; s0 is a1
;
SET_S0         MACRO
               move.w   \2,\1*2*16(a1)
               ENDM
#else
#endif
#if 0
;
; SET_S1 <index> <src ea>
; copy a source <ea> into s1[ index ]
; s1 is a2
;
SET_S1         MACRO
               move.w   \2,\1*2*16(a2)
               ENDM

#else
#endif
#if 0
;
; SET_S0_P <index> <src reg>
; copy <reg>  into s0[ index ]
; copy -<reg> into s0[ 32-index ]
; s0 is a1
; <reg> is modified
;
SET_S0_P       MACRO
               move.w   \2,\1*2*16(a1)
               neg.l    \2
               move.w   \2,-\1*2*16+1024(a1)
               ENDM
#else
#endif
#if 0
;
; SET_S1_P <index> <src reg>
; copy -<reg> into s1[ index ]
; copy -<reg> into s1[ 32-index ]
; s1 is a2
; <reg> is modified
;
SET_S1_P       MACRO
               neg.l    \2
               move.w   \2,\1*2*16(a2)
               move.w   \2,-\1*2*16+1024(a2)
               ENDM

#else
#endif
#if 0
;              Apply the FAST synthesis filter to a sub band
;              Generate full frequency sample
;
;              a0: bandPtr (=fraction)
;              a1: out_filter_buffer 0
;              a2: out_filter_buffer 1
;              d0.w: freq_div ; #2
;
; registers allocation: d6=MUL_SHIFT
;
@MPEGSUBB_filter_band
_MPEGSUBB_filter_band

               movem.l  d2-d7/a2-a6,-(sp)
               move.w   d0,d7 ; #2

               link     a6,#-32*4      ; #2 need of 2*16 longs

               moveq.l  #MUL_SHIFT,d6  ; For MUL32 normalize shift

               move.l   a0,ps1         ; ps1=fraction(0)
               lea      32*2(ps1),ps2  ; ps2=fraction(32)
;               lea      filter_p,pd1   ; pd1=@p(0)
               lea      -32*4(a6),pd1  ; pd1=@p(0)

;               move.w   freq_div,d0
;               cmp.w    #4,d0
               cmp.w    #4,d7 ; #2
               bne      filter_band1
               ; Filter only 1/4 freq
               moveq.w  #8-1,d2
filter_band_q1                                             ; p(i=0..7) = f(i)
               move.w   (ps1)+,d0
               ext.l    d0
               move.l   d0,(pd1)+
               dbra     d2,filter_band_q1

;               lea      filter_p,ps1   ; ps1=@p(0)
               lea      -32*4(a6),ps1   ; ps1=@p(0)
               lea      8*4(ps1),ps2   ; ps2=@p(8)
;               lea      filter_pp,pd1  ; pd1=@pp(0)
               lea      -16*4(a6),pd1  ; pd1=@pp(0)
               FFF_MUL  COS1_16,4      ; pp(i=0..3) = p(i) + p(7-i)
               FFF_MUL  COS3_16,4      ; pp(i=4..7) = COSx*[p(i) - p(7-i)]
               FFF_MUL  COS5_16,4
               FFF_MUL  COS7_16,4

;               lea      filter_pp,ps1  ; ps1=@pp(0)
               lea      -16*4(a6),ps1  ; ps1=@pp(0)
;               lea      filter_p,pd1   ; pd1=@p(0)
               lea      -32*4(a6),pd1   ; pd1=@p(0)
               movem.l  (ps1)+,d1-d4   ; get pp0..pp3
               FF4_MUL  d1,d2,d3,d4
               movem.l  (ps1)+,d1-d4   ; get pp4..pp7
               FF4_MUL  d1,d2,d3,d4

;               lea      filter_p,ps1   ; ps1=@p(0)
               lea      -32*4(a6),ps1   ; ps1=@p(0)
;               lea      filter_p,pd1   ; pd1=@p(0)
               lea      -32*4(a6),pd1   ; pd1=@p(0)
               move.l   #COS1_4,d3
               moveq.w  #4-1,d4
fast_filter_q2
               movem.l  (ps1)+,d1-d2
               FF2_MUL  d1,d2,d3
               dbra     d4,fast_filter_q2

;               lea      filter_p,ps1   ; ps1=@p(0)
               lea      -32*4(a6),ps1   ; ps1=@p(0)

               GET_P    1,d1
               SET_S0   0,d1
               neg.l    d1
               SET_S1   0,d1

               GET_P    5,d1
               ADD_P    7,d1
               SET_S0_P 4,d1

               GET_P    3,d1
               SET_S0_P 8,d1

               GET_P    7,d1
               SET_S0_P 12,d1

               SET_S0   16,#0

               GET_P    6,d3
               ADD_P    7,d3

               move.l   d3,d4
               ADD_P    5,d4
               SET_S1_P 4,d4

               GET_P    2,d4
               ADD_P    3,d4
               SET_S1_P 8,d4

               move.l   d3,d4
               ADD_P    4,d4
               SET_S1_P 12,d4

               GET_P    0,d4
               neg.l    d4
               SET_S1   16,d4

               bra      filter_band9

filter_band1
;               cmp.w    #2,d0
               cmp.w    #2,d7 ; #2
               bne      filter_band2
               ; Filter only 1/2 freq
               moveq.w  #16-1,d2
filter_band_h1                                             ; p(i=0..15) = f(i)
               move.w   (ps1)+,d0
               ext.l    d0
               move.l   d0,(pd1)+
               dbra     d2,filter_band_h1
               bra      filter_band3

filter_band2
               ; Filter full freq
               moveq.w  #16-1,d2
filter_band_f1                         ; p(i=0..15) = f(i) + f(31-i)
               move.w   (ps1)+,d0
               move.w   -(ps2),d1
               ext.l    d0
               ext.l    d1
               add.l    d0,d1
               move.l   d1,(pd1)+
               dbra     d2,filter_band_f1
filter_band3
               bsr      fast_filter_sub

;               lea      filter_p,ps1   ; ps1=@p(0)
               lea      -32*4(a6),ps1   ; ps1=@p(0)

               GET_P    13,d0
               ADD_P    15,d0

               GET_P    1,d1
               SET_S0   0,d1
               neg.l    d1
               SET_S1   0,d1

               move.l   d0,d1
               ADD_P    9,d1
               SET_S0_P 2,d1

               GET_P    5,d1
               ADD_P    7,d1
               SET_S0_P 4,d1

               move.l   d0,d1
               ADD_P    11,d1
               SET_S0_P 6,d1

               GET_P    3,d1
               SET_S0_P 8,d1

               GET_P    11,d1
               ADD_P    15,d1
               SET_S0_P 10,d1

               GET_P    7,d1
               SET_S0_P 12,d1

               GET_P    15,d1
               SET_S0_P 14,d1

               SET_S0   16,#0


               ADD_P    14,d0
               GET_P    12,d1
               ADD_P    14,d1
               ADD_P    15,d1
               GET_P    10,d2
               ADD_P    11,d2
               GET_P    6,d3
               ADD_P    7,d3

               move.l   d0,d4
               ADD_P    9,d4
               SET_S1_P 2,d4

               move.l   d3,d4
               ADD_P    5,d4
               SET_S1_P 4,d4

               move.l   d0,d4
               add.l    d2,d4
               SET_S1_P 6,d4

               GET_P    2,d4
               ADD_P    3,d4
               SET_S1_P 8,d4

               move.l   d1,d4
               add.l    d2,d4
               SET_S1_P 10,d4

               move.l   d3,d4
               ADD_P    4,d4
               SET_S1_P 12,d4

               move.l   d1,d4
               ADD_P    8,d4
               SET_S1_P 14,d4

               GET_P    0,d4
               neg.l    d4
               SET_S1   16,d4

;               move.w   freq_div,d0
;               cmp.w    #1,d0
               cmp.w    #1,d7 ; #2
               bne      filter_band9

               move.l   a0,ps1         ; ps1=fraction(0)
               lea      32*2(ps1),ps2  ; ps2=fraction(32)
;               lea      filter_p,pd1   ; pd1=@p(0)
               lea      -32*4(a6),pd1   ; pd1=@p(0)

               lea      filter_cos64,a0
               moveq.w  #16-1,d2
filter_band_f2                         ; p(i=0..15) = COSx*[f(i) - f(31-i)]
               move.w   (ps1)+,d0
               move.w   -(ps2),d1
               ext.l    d0
               ext.l    d1
               sub.l    d1,d0
               MUL32    (a0)+,d0
               move.l   d0,(pd1)+
               dbra     d2,filter_band_f2

               bsr      fast_filter_sub

;               lea      filter_p,ps1   ; ps1=@p(0)
               lea      -32*4(a6),ps1   ; ps1=@p(0)

               GET_P    13,d0
               ADD_P    15,d0
               GET_P    11,d1
               ADD_P    15,d1
               GET_P    5,d2
               ADD_P    7,d2

               move.l   d0,d3
               ADD_P    9,d3
               move.l   d3,d4
               ADD_P    1,d4
               SET_S0_P 1,d4

               move.l   d2,d4
               add.l    d3,d4
               SET_S0_P 3,d4

               move.l   d0,d3
               ADD_P    11,d3
               move.l   d3,d4
               add.l    d2,d4
               SET_S0_P 5,d4

               move.l   d3,d4
               ADD_P    3,d4
               SET_S0_P 7,d4

               move.l   d1,d4
               ADD_P    3,d4
               SET_S0_P 9,d4

               move.l   d1,d4
               ADD_P    7,d4
               SET_S0_P 11,d4

               GET_P    7,d4
               ADD_P    15,d4
               SET_S0_P 13,d4

               GET_P    15,d4
               SET_S0_P 15,d4


               ADD_P    14,d0
               GET_P    12,d1
               ADD_P    14,d1
               ADD_P    15,d1
               GET_P    10,d2
               ADD_P    11,d2
               GET_P    6,d3
               ADD_P    7,d3

               GET_P    1,d4
               ADD_P    9,d4
               add.l    d0,d4
               SET_S1_P 1,d4

               GET_P    5,d5
               add.l    d3,d5
               add.l    d0,d5
               GET_P    9,d4
               add.l    d5,d4
               SET_S1_P 3,d4

               move.l   d5,d4
               add.l    d2,d4
               SET_S1_P 5,d4

               GET_P    2,d5
               ADD_P    3,d5
               add.l    d2,d5
               move.l   d0,d4
               add.l    d5,d4
               SET_S1_P 7,d4

               move.l   d1,d4
               add.l    d5,d4
               SET_S1_P 9,d4

               GET_P    4,d5
               add.l    d3,d5
               add.l    d1,d5
               move.l   d2,d4
               add.l    d5,d4
               SET_S1_P 11,d4

               GET_P    8,d4
               add.l    d5,d4
               SET_S1_P 13,d4

               GET_P    0,d4
               ADD_P    8,d4
               add.l    d1,d4
               SET_S1_P 15,d4

filter_band9
               unlk     a6    ; #2
               movem.l  (sp)+,d2-d7/a2-a6
               rts

#else
#endif
#if 0

               section  ASMDATA,data
               CNOP     0,4

filter_cos64   dc.l     COS1_64, COS3_64, COS5_64, COS7_64
               dc.l     COS9_64, COS11_64, COS13_64, COS15_64
               dc.l     COS17_64, COS19_64, COS21_64, COS23_64
               dc.l     COS25_64, COS27_64, COS29_64, COS31_64

               ; #2 Begin
               ; pcm_loops[ freq_div ] = = (32 / freq_div) - 1
pcm_loops      dc.w     0, 31, 15, 9, 7, 0

               ; #2 End

;               section  ASMVARS,bss
;               CNOP     0,4

;filter_p       ds.l     16
;filter_pp      ds.l     16

               section  ASMCODE,code


WINDOW_CLIP    equ      1

#else
#endif
#if 0
;
;              Window a sub band filtered sample
;
;              a0: out_filter_buffer
;              a1: out_sample_buffer
;              a2: dewindow (##4)
;              d0: buffer offset
;              d1.w: w_begin  (#2)
;              d2.w: w_width  (#2)
;              d3.w: freq_div (#2)
;              d4.l: dew_shift (##4)
;              -> a1 = out_sample_buffer + out_sample_length
@MPEGSUBB_window_band
_MPEGSUBB_window_band

               movem.l  d2-d7/a2-a6,-(sp)

;               move.w   w_begin,d1 ; #2
               move.w   d2,-(sp) ; #2
               move.l   a2,a4    ; ##4
               lea.l    pcm_loops,a2 ; #2
               move.w   (a2,d3.w*2),d6 ; #2
               move.w   d3,d2 ; #2

;               lea.l    window_table,a2     ;  ##4 External now
               lea.l    (a4,d1.w*2),a2       ;  ##4 a2 = &dewindow[ w_begin ]
               moveq.l  #INT_FACTOR-15,d7    ;  ##4 External now
               add.l    d4,d7                ;  ##4 new scale
               add.w    d0,d1
               and.w    #15,d1               ; d1 = start
               move.l   a0,a4                ; buf0 = &buf_ptr[ 0 ]
               lea      (a0,d1.w*2),a3       ; buf1 = &buf_ptr[ start ]
;               move.w   w_width,d3 ; #2
               move.w   (sp),d3 ; #2
               add.w    d1,d3
               cmp.w    #16,d3
               ble      window_band1
               moveq.w  #16,d3               ; d3 = top
window_band1   sub.w    d1,d3                ; d3 = cnt1
;               move.w   w_width,d4  ; #2
               move.w   (sp),d4 ; #2
               sub.w    d3,d4                ; d4 = cnt0

;               move.w   freq_div,d2 ; #2
               lsl.w    #4,d2
               move.w   d2,d5
               sub.w    d3,d5                ; off1 = freq_div*16 - cnt1
               ext.l    d5
               add.l    d5,d5
               move.l   d5,a5                ; a5 = off1
               move.w   d2,d5
               sub.w    d4,d5                ; off0 = freq_div*16 - cnt0
               ext.l    d5
               add.l    d5,d5
               move.l   d5,a6                ; a6 = off0
;               sub.w    w_width,d2           ; offd = freq_div*16 - w_width (#2)
               sub.w    (sp),d2                ; offd = freq_div*16 - w_width (#2)
               ext.l    d2
               add.l    d2,d2

;               moveq.l  #WINDOW_FACTOR+INT_FACTOR-15,d7 ; ##4 External now
;               move.w   pcm_count,d6  ; #2
;               subq.w   #1,d6         ; for DBRA loop ; #2

               IFD      OLD_WIN  ; ##1

               subq.w   #1,d3                ; for DBRA loop
               subq.w   #1,d4                ; for DBRA loop
window_band2
               clr.l    d5                   ; d5 = sum

               move.w   d3,d0                ; d0 = cnt1
               bmi      window_band4
window_band3   move.w   (a3)+,d1
               muls.w   (a2)+,d1
               add.l    d1,d5
               dbra     d0,window_band3
window_band4
               add.l    a5,a3                ; buf1 += off1

               move.w   d4,d0                ; d0 = cnt0
               bmi      window_band6
window_band5   move.w   (a4)+,d1
               muls.w   (a2)+,d1
               add.l    d1,d5
               dbra     d0,window_band5
window_band6
               add.l    a6,a4                ; buf0 += off0
               add.l    d2,a2                ; dewindow += offd

               asr.l    d7,d5                ; sum >>= (WINDOW_FACTOR+INT_FACTOR-15)

               IFD      WINDOW_CLIP
               cmp.l    #-32768,d5
               blt      window_band4l
               cmp.l    #32767,d5
               bgt      window_band4h
window_band4n
               ENDC

               move.w   d5,(a1)+
               dbra     d6,window_band2

               IFD      WINDOW_CLIP
               bra      window_band9
window_band4l  move.l   #-32768,d5
               bra.s    window_band4n
window_band4h  move.l   #32767,d5
               bra.s    window_band4n
               ENDC

window_band9
               move.w   (sp)+,d2 ; #2
               movem.l  (sp)+,d2-d7/a2-a6
               rts

#else
#endif
#if 0
               ; ##1 Begin

               ELSE

;              MULADDWIN {a3|a4}
MULADDWIN      MACRO
               move.w   (\1)+,d1
               muls.w   (a2)+,d1
               add.l    d1,d5
               ENDM

#else
#endif
#if 0
               ; WARING GENAM bug in rept & macros
               ; rept part is moved to END of MACRO !
               ;
;              MULADDN <rept num>,{a3|a4}
MULADDN        MACRO
               IFNE     \2
               REPT     \2
               MULADDWIN \1
               ENDR
               ENDC
               ENDM

#else
#endif
#if 0
;              MULADD0 <rept num>
MULADD0        MACRO
               clr.l    d5
               MULADDN  a3,\1
               ENDM

#else
#endif
#if 0
;              MULADD1 <rept num>
MULADD1        MACRO
               add.l    a5,a3
               MULADDN  a4,\1
               ENDM

               IFD      OLD_OVERFLOW ; ##3 Begin

#else
#endif
#if 0
               ; CHECKBOUNDS <loop label>
CHECKBOUNDS    MACRO
               add.l    a6,a4
               add.l    d2,a2
               asr.l    d7,d5
               IFD      WINDOW_CLIP
               cmp.l    #-32768,d5
               blt      .window_bandl
               cmp.l    #32767,d5
               bgt      .window_bandh
.window_bandn
               ENDC
               move.w   d5,(a1)+
               dbra     d6,\1
               bra      window_band9
               IFD      WINDOW_CLIP
               bra      window_band9
.window_bandl  move.w   #-32768,d5
               bra.s    .window_bandn
.window_bandh  move.w   #32767,d5
               bra.s    .window_bandn
               ENDC
               ENDM

               ELSE ; ##3

#else
#endif
#if 0

               ; CHECKBOUNDS <loop label>
CHECKBOUNDS    MACRO
               add.l    a6,a4
               add.l    d2,a2
               asr.l    d7,d5
               IFD      WINDOW_CLIP
               move.w   d5,a0
               cmp.l    a0,d5
               bne.b    .window_bandov
.window_bandn
               ENDC
               move.w   d5,(a1)+
               dbra     d6,\1
               bra      window_band9
               IFD      WINDOW_CLIP
               bra      window_band9
.window_bandov not.l    d5
               add.l    d5,d5
               move.w   #$8000,d5
               negx.w   d5
               bra.s    .window_bandn
;               move.w   d5,(a1)+
;               dbra     d6,\1
;               bra      window_band9
               ENDC
               ENDM

               ENDC  ; ##3 End


;               move.w   w_width,d3 ; #2
               move.w   (sp),d3 ; #2
               lea      window_q0muls,a0
               cmp.w    #4,d3
               ble      window_qmul
               lea      window_q1muls,a0
               cmp.w    #8,d3
               ble      window_qmul
               lea      window_q2muls,a0
window_qmul
               move.l   (a0,d4.w*4),a0
               jmp      (a0)

window_q2mul0
               MULADD0  16
               MULADD1  0
               CHECKBOUNDS window_q2mul0
window_q2mul1
               MULADD0  15
               MULADD1  1
               CHECKBOUNDS window_q2mul1
window_q2mul2
               MULADD0  14
               MULADD1  2
               CHECKBOUNDS window_q2mul2
window_q2mul3
               MULADD0  13
               MULADD1  3
               CHECKBOUNDS window_q2mul3
window_q2mul4
               MULADD0  12
               MULADD1  4
               CHECKBOUNDS window_q2mul4
window_q2mul5
               MULADD0  11
               MULADD1  5
               CHECKBOUNDS window_q2mul5
window_q2mul6
               MULADD0  10
               MULADD1  6
               CHECKBOUNDS window_q2mul6
window_q2mul7
               MULADD0  9
               MULADD1  7
               CHECKBOUNDS window_q2mul7
window_q2mul8
               MULADD0  8
               MULADD1  8
               CHECKBOUNDS window_q2mul8
window_q2mul9
               MULADD0  7
               MULADD1  9
               CHECKBOUNDS window_q2mul9
window_q2mul10
               MULADD0  6
               MULADD1  10
               CHECKBOUNDS window_q2mul10
window_q2mul11
               MULADD0  5
               MULADD1  11
               CHECKBOUNDS window_q2mul11
window_q2mul12
               MULADD0  4
               MULADD1  12
               CHECKBOUNDS window_q2mul12
window_q2mul13
               MULADD0  3
               MULADD1  13
               CHECKBOUNDS window_q2mul13
window_q2mul14
               MULADD0  2
               MULADD1  14
               CHECKBOUNDS window_q2mul14
window_q2mul15
               MULADD0  1
               MULADD1  15
               CHECKBOUNDS window_q2mul15

window_q1mul0
               MULADD0  8
               MULADD1  0
               CHECKBOUNDS window_q1mul0
window_q1mul1
               MULADD0  7
               MULADD1  1
               CHECKBOUNDS window_q1mul1
window_q1mul2
               MULADD0  6
               MULADD1  2
               CHECKBOUNDS window_q1mul2
window_q1mul3
               MULADD0  5
               MULADD1  3
               CHECKBOUNDS window_q1mul3
window_q1mul4
               MULADD0  4
               MULADD1  4
               CHECKBOUNDS window_q1mul4
window_q1mul5
               MULADD0  3
               MULADD1  5
               CHECKBOUNDS window_q1mul5
window_q1mul6
               MULADD0  2
               MULADD1  6
               CHECKBOUNDS window_q1mul6
window_q1mul7
               MULADD0  1
               MULADD1  7
               CHECKBOUNDS window_q1mul7


window_q0mul0
               MULADD0  4
               MULADD1  0
               CHECKBOUNDS window_q0mul0
window_q0mul1
               MULADD0  3
               MULADD1  1
               CHECKBOUNDS window_q0mul1
window_q0mul2
               MULADD0  2
               MULADD1  2
               CHECKBOUNDS window_q0mul2
window_q0mul3
               MULADD0  1
               MULADD1  3
               CHECKBOUNDS window_q0mul3

window_band9
               move.w   (sp)+,d2
               movem.l  (sp)+,d2-d7/a2-a6
               rts

window_q2muls  dc.l     window_q2mul0, window_q2mul1, window_q2mul2, window_q2mul3
               dc.l     window_q2mul4, window_q2mul5, window_q2mul6, window_q2mul7
               dc.l     window_q2mul8, window_q2mul9, window_q2mul10, window_q2mul11
               dc.l     window_q2mul12, window_q2mul13, window_q2mul14, window_q2mul15

window_q1muls  dc.l     window_q1mul0, window_q1mul1, window_q1mul2, window_q1mul3
               dc.l     window_q1mul4, window_q1mul5, window_q1mul6, window_q1mul7

window_q0muls  dc.l     window_q0mul0, window_q0mul1, window_q0mul2, window_q0mul3

               ENDC

#else
#endif
#if 0
               ; ##1 End

               ; #2 Begin: MPEGSUBB_config suppressed
               IFNE     0

;              Configure subband routines
;
;              d0 = freq_div
;              d1 = quality
;              d2 = output_8bits
;
;@MPEGSUBB_config
;_MPEGSUBB_config
               movem.l  d2-d7/a2-a6,-(sp)
               move.w   d0,freq_div
               move.w   d1,quality
               move.w   d2,output_8bits

               move.w   #32,d3         ; pcm_count for full freq output
               cmp.w    #2,d0
               beq      init_decode_1
               cmp.w    #4,d0
               beq      init_decode_2
               bra      init_decode_5
init_decode_1  move.w   #16,d3         ; pcm_count for half freq output
               bra      init_decode_5
init_decode_2  moveq.l  #8,d3          ; pcm_count for quarter freq output
init_decode_5  move.w   d3,pcm_count
               moveq.l  #0,d3          ; start offset for windowing
               moveq.l  #16,d4         ; #coeff used for windowing
               move.w   quality,d2
               cmp.b    #2,d2          ; best quality
               beq.s    init_decode_6
               move.w   #4,d3
               moveq.l  #8,d4
               cmp.b    #1,d2
               beq.s    init_decode_6
               move.w   #6,d3
               moveq.l  #4,d4
init_decode_6  move.w   d3,w_begin
               move.w   d4,w_width
               movem.l  (sp)+,d2-d7/a2-a6
               rts
#else
#endif
#if 0

               ENDC
               ; #2 End

; ##4 Begin  (Table suppressed)
               IFD OLD_DEW_TABLE

               section  ASM_TABLES,data

WINDOW_FACTOR  equ      14

               even
window_table
               dc.w  +0,    -7,   +53,  -114,  +509, -1288, +1643, -9372,+18759, +9372, +1643, +1288,  +509,  +114,   +53,    +7
               dc.w  +0,    -7,   +54,  -129,  +499, -1379, +1489, -9833,+18747, +8909, +1783, +1196,  +515,  +100,   +51,    +6
               dc.w  +0,    -8,   +55,  -145,  +487, -1469, +1321,-10293,+18713, +8447, +1909, +1106,  +519,   +86,   +50,    +6
               dc.w  +0,    -9,   +56,  -161,  +473, -1559, +1140,-10751,+18657, +7986, +2022, +1015,  +521,   +73,   +49,    +5
               dc.w  +0,   -10,   +56,  -177,  +455, -1647,  +943,-11205,+18578, +7527, +2122,  +926,  +521,   +61,   +47,    +4
               dc.w  +0,   -11,   +57,  -194,  +434, -1733,  +733,-11654,+18476, +7072, +2209,  +837,  +518,   +49,   +45,    +4
               dc.w  +0,   -12,   +57,  -211,  +410, -1817,  +509,-12097,+18353, +6620, +2284,  +750,  +514,   +38,   +44,    +4
               dc.w  +0,   -13,   +56,  -229,  +383, -1899,  +270,-12534,+18208, +6173, +2347,  +665,  +507,   +27,   +42,    +3
               dc.w  +0,   -14,   +56,  -247,  +353, -1977,   +17,-12963,+18042, +5732, +2397,  +582,  +500,   +18,   +40,    +3
               dc.w  +0,   -15,   +55,  -266,  +320, -2052,  -249,-13383,+17854, +5297, +2437,  +501,  +490,    +8,   +38,    +2
               dc.w  +0,   -17,   +53,  -284,  +282, -2122,  -530,-13794,+17647, +4869, +2465,  +422,  +479,    +0,   +36,    +2
               dc.w  +0,   -18,   +51,  -302,  +242, -2188,  -824,-14194,+17419, +4449, +2483,  +346,  +467,    -7,   +34,    +2
               dc.w  +0,   -19,   +50,  -320,  +198, -2249, -1133,-14583,+17172, +4038, +2491,  +273,  +454,   -14,   +32,    +1
               dc.w  +0,   -21,   +47,  -338,  +151, -2304, -1454,-14959,+16907, +3636, +2489,  +203,  +439,   -20,   +31,    +1
               dc.w  +0,   -22,   +44,  -356,  +100, -2353, -1788,-15322,+16623, +3244, +2478,  +136,  +424,   -26,   +29,    +1
               dc.w  -1,   -24,   +40,  -374,   +46, -2396, -2134,-15670,+16322, +2863, +2459,   +71,  +408,   -31,   +27,    +1
               dc.w  -1,   -25,   +36,  -391,   -11, -2431, -2493,-16004,+16004, +2493, +2431,   +11,  +391,   -36,   +25,    +1
               dc.w  -1,   -27,   +31,  -408,   -71, -2459, -2863,-16322,+15670, +2134, +2396,   -46,  +374,   -40,   +24,    +1
               dc.w  -1,   -29,   +26,  -424,  -136, -2478, -3244,-16623,+15322, +1788, +2353,  -100,  +356,   -44,   +22,    +0
               dc.w  -1,   -31,   +20,  -439,  -203, -2489, -3636,-16907,+14959, +1454, +2304,  -151,  +339,   -47,   +21,    +0
               dc.w  -1,   -32,   +14,  -454,  -273, -2491, -4038,-17172,+14583, +1133, +2249,  -198,  +320,   -50,   +19,    +0
               dc.w  -2,   -34,    +7,  -467,  -346, -2483, -4449,-17419,+14194,  +825, +2188,  -242,  +302,   -51,   +18,    +0
               dc.w  -2,   -36,    +0,  -479,  -422, -2465, -4869,-17647,+13794,  +530, +2122,  -282,  +284,   -53,   +17,    +0
               dc.w  -2,   -38,    -8,  -490,  -501, -2437, -5297,-17854,+13383,  +249, +2052,  -319,  +266,   -55,   +15,    +0
               dc.w  -3,   -40,   -18,  -500,  -582, -2397, -5732,-18042,+12963,   -17, +1977,  -353,  +247,   -55,   +14,    +0
               dc.w  -3,   -42,   -27,  -507,  -665, -2347, -6173,-18208,+12534,  -270, +1899,  -383,  +229,   -56,   +13,    +0
               dc.w  -4,   -44,   -38,  -514,  -750, -2284, -6620,-18353,+12097,  -509, +1817,  -410,  +211,   -57,   +12,    +0
               dc.w  -4,   -45,   -49,  -518,  -837, -2209, -7072,-18476,+11654,  -733, +1733,  -434,  +194,   -57,   +11,    +0
               dc.w  -4,   -47,   -61,  -521,  -926, -2122, -7527,-18578,+11205,  -943, +1647,  -455,  +177,   -56,   +10,    +0
               dc.w  -5,   -49,   -73,  -521, -1015, -2022, -7986,-18657,+10751, -1140, +1559,  -473,  +161,   -56,    +9,    +0
               dc.w  -6,   -50,   -86,  -519, -1106, -1909, -8447,-18713,+10293, -1321, +1469,  -487,  +145,   -55,    +8,    +0
               dc.w  -6,   -51,  -100,  -515, -1196, -1783, -8909,-18747, +9833, -1489, +1379,  -499,  +129,   -54,    +7,    +0

               ENDC
; ##4 End

               ; #2: vars suppressed
;               section  ASMVARS,bss

;freq_div       ds.w     1  ; 1,2 or 4
;output_8bits   ds.w     1  ; 0 if 16 bits output
;quality        ds.w     1  ; 0..2 (best)

;w_begin        ds.w     1  ; start offset for windowing
;w_width        ds.w     1  ; #coeff used for windowing
;pcm_count      ds.w     1  ; # of samples per subband
#else
#endif
