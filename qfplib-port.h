inline float qfp_fpow(float b, float e)
{
    return qfp_fexp(qfp_fmul(e, qfp_fln(b)));
}

inline float qfp_flog10(float x)
{
    static const auto ln10 = qfp_fln(10.f);
    return qfp_fdiv(qfp_fln(x), ln10);
}

__attribute__((naked, section(".data")))
inline float qfp_fadd_asm(float, float)
{
asm(R"(
.syntax unified
 push {r4,r5,r6,r14}
 asrs r4,r0,#31
 lsls r2,r0,#1
 lsrs r2,#24     @ x exponent
 beq fa_xe0
 cmp r2,#255
 beq fa_xe255
fa_xe:
 asrs r5,r1,#31
 lsls r3,r1,#1
 lsrs r3,#24     @ y exponent
 beq fa_ye0
 cmp r3,#255
 beq fa_ye255
fa_ye:
 ldr r6,=#0x007fffff
 ands r0,r0,r6   @ extract mantissa bits
 ands r1,r1,r6
 adds r6,#1      @ r6=0x00800000
 orrs r0,r0,r6   @ set implied 1
 orrs r1,r1,r6
 eors r0,r0,r4   @ complement...
 eors r1,r1,r5
 subs r0,r0,r4   @ ... and add 1 if sign bit is set: 2's complement
 subs r1,r1,r5
 subs r5,r3,r2   @ ye-xe
 subs r4,r2,r3   @ xe-ye
 bmi fa_ygtx
@ here xe>=ye
 cmp r4,#30
 bge fa_xmgty    @ xe much greater than ye?
 adds r5,#32
 movs r3,r2      @ save exponent
@ here y in r1 must be shifted down r4 places to align with x in r0
 movs r2,r1
 lsls r2,r2,r5   @ keep the bits we will shift off the bottom of r1
 asrs r1,r1,r4
 b fa_0

.ltorg
 
fa_ymgtx:
 movs r2,#0      @ result is just y
 movs r0,r1
 b fa_1
fa_xmgty:
 movs r3,r2      @ result is just x
 movs r2,#0
 b fa_1

fa_ygtx:
@ here ye>xe
 cmp r5,#30
 bge fa_ymgtx    @ ye much greater than xe?
 adds r4,#32
@ here x in r0 must be shifted down r5 places to align with y in r1
 movs r2,r0
 lsls r2,r2,r4   @ keep the bits we will shift off the bottom of r0
 asrs r0,r0,r5
fa_0:
 adds r0,r1      @ result is now in r0:r2, possibly highly denormalised or zero; exponent in r3
 beq fa_9        @ if zero, inputs must have been of identical magnitude and opposite sign, so return +0
fa_1: 
 lsrs r1,r0,#31  @ sign bit
 beq fa_8
 mvns r0,r0
 rsbs r2,r2,#0
 bne fa_8
 adds r0,#1
fa_8:
 adds r6,r6
@ r6=0x01000000
 cmp r0,r6
 bhs fa_2
fa_3:
 adds r2,r2      @ normalisation loop
 adcs r0,r0
 subs r3,#1      @ adjust exponent
 cmp r0,r6
 blo fa_3
fa_2:
@ here r0:r2 is the result mantissa 0x01000000<=r0<0x02000000, r3 the exponent, and r1 the sign bit
 lsrs r0,#1
 bcc fa_4
@ rounding bits here are 1:r2
 adds r0,#1      @ round up
 cmp r2,#0
 beq fa_5        @ sticky bits all zero?
fa_4:
 cmp r3,#254
 bhs fa_6        @ exponent too large or negative?
 lsls r1,#31     @ pack everything
 add r0,r1
 lsls r3,#23
 add r0,r3
fa_end:
 pop {r4,r5,r6,r15}

fa_9:
 cmp r2,#0       @ result zero?
 beq fa_end      @ return +0
 b fa_1

fa_5:
 lsrs r0,#1
 lsls r0,#1      @ round to even
 b fa_4

fa_6:
 bge fa_7
@ underflow
@ can handle denormals here
 lsls r0,r1,#31  @ result is signed zero
 pop {r4,r5,r6,r15}
fa_7:
@ overflow
 lsls r0,r1,#8
 adds r0,#255
 lsls r0,#23     @ result is signed infinity
 pop {r4,r5,r6,r15}


fa_xe0:
@ can handle denormals here
 subs r2,#32
 adds r2,r4       @ exponent -32 for +Inf, -33 for -Inf
 b fa_xe

fa_xe255:
@ can handle NaNs here
 lsls r2,#8
 add r2,r2,r4 @ exponent ~64k for +Inf, ~64k-1 for -Inf
 b fa_xe

fa_ye0:
@ can handle denormals here
 subs r3,#32
 adds r3,r5       @ exponent -32 for +Inf, -33 for -Inf
 b fa_ye

fa_ye255:
@ can handle NaNs here
 lsls r3,#8
 add r3,r3,r5 @ exponent ~64k for +Inf, ~64k-1 for -Inf
 b fa_ye
 )");
}

__attribute__((naked, section(".data")))
inline float qfp_fmul_asm(float, float)
{
asm(R"(
.syntax unified
 push {r7,r14}
 mov r2,r0
 eors r2,r1       @ sign of result
 lsrs r2,#31
 lsls r2,#31
 mov r14,r2
 lsls r0,#1
 lsls r1,#1
 lsrs r2,r0,#24   @ xe
 beq fm_xe0
 cmp r2,#255
 beq fm_xe255
fm_xe:
 lsrs r3,r1,#24   @ ye
 beq fm_ye0
 cmp r3,#255
 beq fm_ye255
fm_ye:
 adds r7,r2,r3    @ exponent of result (will possibly be incremented)
 subs r7,#128     @ adjust bias for packing
 lsls r0,#8       @ x mantissa
 lsls r1,#8       @ y mantissa
 lsrs r0,#9
 lsrs r1,#9

 adds r2,r0,r1    @ for later
 mov r12,r2
 lsrs r2,r0,#7    @ x[22..7] Q16
 lsrs r3,r1,#7    @ y[22..7] Q16
 muls r2,r2,r3    @ result [45..14] Q32: never an overestimate and worst case error is 2*(2^7-1)*(2^23-2^7)+(2^7-1)^2 = 2130690049 < 2^31
 muls r0,r0,r1    @ result [31..0] Q46
 lsrs r2,#18      @ result [45..32] Q14
 bcc 1f
 cmp r0,#0
 bmi 1f
 adds r2,#1       @ fix error in r2
1:
 lsls r3,r0,#9    @ bits off bottom of result
 lsrs r0,#23      @ Q23
 lsls r2,#9
 adds r0,r2       @ cut'n'shut
 add r0,r12       @ implied 1*(x+y) to compensate for no insertion of implied 1s
@ result-1 in r3:r0 Q23+32, i.e., in range [0,3)

 lsrs r1,r0,#23
 bne fm_0         @ branch if we need to shift down one place
@ here 1<=result<2
 cmp r7,#254
 bhs fm_3a        @ catches both underflow and overflow
 lsls r3,#1       @ sticky bits at top of R3, rounding bit in carry
 bcc fm_1         @ no rounding
 beq fm_2         @ rounding tie?
 adds r0,#1       @ round up
fm_1:
 adds r7,#1       @ for implied 1
 lsls r7,#23      @ pack result
 add r0,r7
 add r0,r14
 pop {r7,r15}
fm_2:             @ rounding tie
 adds r0,#1
fm_3:
 lsrs r0,#1
 lsls r0,#1       @ clear bottom bit
 b fm_1

@ here 1<=result-1<3
fm_0:
 adds r7,#1       @ increment exponent
 cmp r7,#254
 bhs fm_3b        @ catches both underflow and overflow
 lsrs r0,#1       @ shift mantissa down
 bcc fm_1a        @ no rounding
 adds r0,#1       @ assume we will round up
 cmp r3,#0        @ sticky bits
 beq fm_3c        @ rounding tie?
fm_1a:
 adds r7,r7
 adds r7,#1       @ for implied 1
 lsls r7,#22      @ pack result
 add r0,r7
 add r0,r14
 pop {r7,r15}

fm_3c:
 lsrs r0,#1
 lsls r0,#1       @ clear bottom bit
 b fm_1a

fm_xe0:
 subs r2,#16
fm_xe255:
 lsls r2,#8
 b fm_xe
fm_ye0:
 subs r3,#16
fm_ye255:
 lsls r3,#8
 b fm_ye

@ here the result is under- or overflowing
fm_3b:
 bge fm_4        @ branch on overflow
@ trap case where result is denormal 0x007fffff + 0.5ulp or more
 adds r7,#1      @ exponent=-1?
 bne fm_5
@ corrected mantissa will be >= 3.FFFFFC (0x1fffffe Q23)
@ so r0 >= 2.FFFFFC (0x17ffffe Q23)
 adds r0,#2
 lsrs r0,#23
 cmp r0,#3
 bne fm_5
 b fm_6

fm_3a:
 bge fm_4        @ branch on overflow
@ trap case where result is denormal 0x007fffff + 0.5ulp or more
 adds r7,#1      @ exponent=-1?
 bne fm_5
 adds r0,#1      @ mantissa=0xffffff (i.e., r0=0x7fffff)?
 lsrs r0,#23
 beq fm_5
fm_6:
 movs r0,#1      @ return smallest normal
 lsls r0,#23
 add r0,r14
 pop {r7,r15}

fm_5:
 mov r0,r14
 pop {r7,r15}
fm_4:
 movs r0,#0xff
 lsls r0,#23
 add r0,r14
 pop {r7,r15}
)");
}

__attribute__((naked, section(".data")))
inline float qfp_int2float_asm(int)
{
asm(R"(
.syntax unified
 movs r1,#0      @ fall through
 push {r4,r5,r6,r14}
 movs r2,#29
 subs r2,r1      @ fix exponent
 movs r5,#0
 bl qfp_int2float_packx
 pop {r4,r5,r6,r15}
qfp_int2float_packx:
 lsrs r4,r0,#31 @ save sign bit
 lsls r4,r4,#31 @ sign now in b31
 bpl 2f         @ skip if positive
 cmp r5,#0
 beq 11f
 adds r0,#1     @ fiddle carry in to following rsb if sticky bits are non-zero
11:
 rsbs r0,#0     @ can now treat r0 as unsigned
 bmi 3f         @ catch r0=0x80000000 case
2:
 subs r2,#1     @ normalisation loop
 adds r0,r0
 beq 1f         @ zero? special case
 bpl 2b         @ normalise so leading "1" in bit 31
3:
 adds r2,#129   @ (mis-)offset exponent
 bne 12f        @ special case: highest denormal can round to lowest normal
 adds r0,#0x80  @ in special case, need to add 256 to r0 for rounding
 bcs 4f         @ tripped carry? then have leading 1 in C as required
12:
 adds r0,#0x80  @ rounding
 bcs 4f         @ tripped carry? then have leading 1 in C as required (and result is even so can ignore sticky bits)
 cmp r5,#0
 beq 7f         @ sticky bits zero?
8:
 lsls r0,#1     @ remove leading 1
9:
 subs r2,#1     @ compensate exponent on this path
4:
 cmp r2,#254
 bge 5f         @ overflow?
 adds r2,#1     @ correct exponent offset
 ble 10f        @ denormal/underflow?
 lsrs r0,#9     @ align mantissa
 lsls r2,#23    @ align exponent
 orrs r0,r2     @ assemble exponent and mantissa
6:
 orrs r0,r4     @ apply sign
1:
 bx r14

5:
 movs r0,#0xff  @ create infinity
 lsls r0,#23
 b 6b

10:
 movs r0,#0     @ create zero
 bx r14

7:              @ sticky bit rounding case
 lsls r5,r0,#24 @ check bottom 8 bits of r0
 bne 8b         @ in rounding-tie case?
 lsrs r0,#9     @ ensure even result
 lsls r0,#10
 b 9b
)");
}


