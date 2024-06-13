@ Copyright 2019-2024 Mark Owen
@ http://www.quinapalus.com
@ E-mail: qfp@quinapalus.com
@
@ This file is free software: you can redistribute it and/or modify
@ it under the terms of version 2 of the GNU General Public License
@ as published by the Free Software Foundation.
@
@ This file is distributed in the hope that it will be useful,
@ but WITHOUT ANY WARRANTY; without even the implied warranty of
@ MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
@ GNU General Public License for more details.
@
@ You should have received a copy of the GNU General Public License
@ along with this file.  If not, see <http://www.gnu.org/licenses/> or
@ write to the Free Software Foundation, Inc., 51 Franklin Street,
@ Fifth Floor, Boston, MA  02110-1301, USA.

.syntax unified
.cpu cortex-m0plus
.thumb

@ exported symbols

.global qfp_fadd
.global qfp_fsub
.global qfp_fmul
.global qfp_fdiv
.global qfp_fcmp
.global qfp_fsqrt
.global qfp_float2int
.global qfp_float2fix
.global qfp_float2uint
.global qfp_float2ufix
.global qfp_int2float
.global qfp_fix2float
.global qfp_uint2float
.global qfp_ufix2float
.global qfp_fexp
.global qfp_fln

qfp_lib_start:

@ exchange r0<->r1, r2<->r3
xchxy:
 push {r0,r2,r14}
 mov r0,r1
 mov r2,r3
 pop {r1,r3,r15}

@ IEEE single in r0-> signed (two's complemennt) mantissa in r0 9Q23 (24 significant bits), signed exponent (bias removed) in r2
@ trashes r4; zero, denormal -> mantissa=+/-1, exponent=-380; Inf, NaN -> mantissa=+/-1, exponent=+640
unpackx:
 lsrs r2,r0,#23 @ save exponent and sign
 lsls r0,#9     @ extract mantissa
 lsrs r0,#9
 movs r4,#1
 lsls r4,#23
 orrs r0,r4     @ reinstate implied leading 1
 cmp r2,#255    @ test sign bit
 uxtb r2,r2     @ clear it
 bls 1f         @ branch on positive
 rsbs r0,#0     @ negate mantissa
1:
 subs r2,#1
 cmp r2,#254    @ zero/denormal/Inf/NaN?
 bhs 2f
 subs r2,#126   @ remove exponent bias: can now be -126..+127
 bx r14

2:              @ here with special-case values
 cmp r0,#0
 mov r0,r4      @ set mantissa to +1
 bpl 3f
 rsbs r0,#0     @ zero/denormal/Inf/NaN: mantissa=+/-1
3:
 subs r2,#126   @ zero/denormal: exponent -> -127; Inf, NaN: exponent -> 128
 lsls r2,#2     @ zero/denormal: exponent -> -508; Inf, NaN: exponent -> 512
 adds r2,#128   @ zero/denormal: exponent -> -380; Inf, NaN: exponent -> 640
 bx r14

@ normalise and pack signed mantissa in r0 nominally 3Q29, signed exponent in r2-> IEEE single in r0
@ trashes r4, preserves r1,r3
@ r5: "sticky bits", must be zero iff all result bits below r0 are zero for correct rounding
packx:
 lsrs r4,r0,#31 @ save sign bit
 lsls r4,r4,#31 @ sign now in b31
 bpl 2f         @ skip if positive
 cmp r5,#0
 beq 11f
 adds r0,#1     @ fiddle carry in to following rsb if sticky bits are non-zero
11:
 rsbs r0,#0     @ can now treat r0 as unsigned
packx0:
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

.align 2
.ltorg

@ signed multiply r0 1Q23 by r1 4Q23, result in r0 7Q25, sticky bits in r5
@ trashes r3,r4
mul0:
 uxth r3,r0      @ Q23
 asrs r4,r1,#16  @ Q7
 muls r3,r4      @ L*H, Q30 signed
 asrs r4,r0,#16  @ Q7
 uxth r5,r1      @ Q23
 muls r4,r5      @ H*L, Q30 signed
 adds r3,r4      @ sum of middle partial products
 uxth r4,r0
 muls r4,r5      @ L*L, Q46 unsigned
 lsls r5,r4,#16  @ initialise sticky bits from low half of low partial product
 lsrs r4,#16     @ Q25
 adds r3,r4      @ add high half of low partial product to sum of middle partial products
                 @ (cannot generate carry by limits on input arguments)
 asrs r0,#16     @ Q7
 asrs r1,#16     @ Q7
 muls r0,r1      @ H*H, Q14 signed
 lsls r0,#11     @ high partial product Q25
 lsls r1,r3,#27  @ sticky
 orrs r5,r1      @ collect further sticky bits
 asrs r1,r3,#5   @ middle partial products Q25
 adds r0,r1      @ final result
 bx r14

.thumb_func
qfp_fcmp:
 lsls r2,r0,#1
 lsrs r2,#24
 beq 1f
 cmp r2,#0xff
 bne 2f
1:
 lsrs r0,#23     @ clear mantissa if NaN or denormal
 lsls r0,#23
2:
 lsls r2,r1,#1
 lsrs r2,#24
 beq 1f
 cmp r2,#0xff
 bne 2f
1:
 lsrs r1,#23     @ clear mantissa if NaN or denormal
 lsls r1,#23
2:
 movs r2,#1      @ initialise result
 eors r1,r0
 bmi 4f          @ opposite signs? then can proceed on basis of sign of x
 eors r1,r0      @ restore y
 bpl 1f
 rsbs r2,#0      @ both negative? flip comparison
1:
 cmp r0,r1
 bgt 2f
 blt 3f
5:
 movs r2,#0
3:
 rsbs r2,#0
2:
 subs r0,r2,#0
 bx r14
4:
 orrs r1,r0
 adds r1,r1
 beq 5b
 cmp r0,#0
 bge 2b
 b 3b

.ltorg

@ convert float to signed int, rounding towards -Inf, clamping
.thumb_func
qfp_float2int:
 movs r1,#0      @ fall through

@ convert float in r0 to signed fixed point in r0, clamping
.thumb_func
qfp_float2fix:
 push {r4,r14}
 bl unpackx
 movs r3,r2
 adds r3,#130
 bmi 6f          @ -0?
 add r2,r1       @ incorporate binary point position into exponent
 subs r2,#23     @ r2 is now amount of left shift required
 blt 1f          @ requires right shift?
 cmp r2,#7       @ overflow?
 ble 4f
3:               @ overflow
 asrs r1,r0,#31  @ +ve:0 -ve:0xffffffff
 mvns r1,r1      @ +ve:0xffffffff -ve:0
 movs r0,#1
 lsls r0,#31
5:
 eors r0,r1      @ +ve:0x7fffffff -ve:0x80000000 (unsigned path: 0xffffffff)
 pop {r4,r15}
1:
 rsbs r2,#0      @ right shift for r0, >0
 cmp r2,#32
 blt 2f          @ more than 32 bits of right shift?
 movs r2,#32
2:
 asrs r0,r0,r2
 pop {r4,r15}
6:
 movs r0,#0
 pop {r4,r15}

@ unsigned version
.thumb_func
qfp_float2uint:
 movs r1,#0      @ fall through
.thumb_func
qfp_float2ufix:
 push {r4,r14}
 bl unpackx
 add r2,r1       @ incorporate binary point position into exponent
 movs r1,r0
 bmi 5b          @ negative? return zero
 subs r2,#23     @ r2 is now amount of left shift required
 blt 1b          @ requires right shift?
 mvns r1,r0      @ ready to return 0xffffffff
 cmp r2,#8       @ overflow?
 bgt 5b
4:
 lsls r0,r0,r2   @ result fits, left shifted
 pop {r4,r15}

@ convert signed int to float, rounding
.thumb_func
qfp_int2float:
 movs r1,#0      @ fall through

@ convert signed fix to float, rounding; number of r0 bits after point in r1
.thumb_func
qfp_fix2float:
 push {r4,r5,r14}
1:
 movs r2,#29
 subs r2,r1      @ fix exponent
packretns:       @ pack and return, sticky bits=0
 movs r5,#0
packret:         @ common return point: "pack and return"
 bl packx
ret_pop45:
 pop {r4,r5,r15}


@ unsigned version
.thumb_func
qfp_uint2float:
 movs r1,#0      @ fall through
.thumb_func
qfp_ufix2float:
 push {r4,r5,r14}
 cmp r0,#0
 bge 1b          @ treat <2^31 as signed
 movs r2,#30
 subs r2,r1      @ fix exponent
 lsls r5,r0,#31  @ one sticky bit
 lsrs r0,#1
 b packret

.thumb_func
qfp_fexp:
 push {r4,r5,r14}
 movs r1,#24
 bl qfp_float2fix    @ Q24: covers entire valid input range
 asrs r1,r0,#16      @ Q8
 ldr r2,=#5909       @ log_2(e) Q12
 muls r2,r1          @ estimate exponent of result Q20 (always an underestimate)
 asrs r2,#20         @ Q0
 lsls r1,r0,#6       @ Q30
 ldr r0,=#0x2c5c85fe @ ln(2) Q30
 muls r0,r2          @ accurate contribution of estimated exponent
 subs r1,r0          @ residual to be exponentiated, guaranteed ≥0, < about 0.75 Q30

@ here
@ r1: mantissa to exponentiate, 0...~0.75 Q30
@ r2: first exponent estimate

 movs r5,#1          @ shift
 adr r3,ftab_exp     @ could use alternate words from dtab_exp to save space if required
 movs r0,#1
 lsls r0,#29         @ x=1 Q29

3:
 ldmia r3!,{r4}
 subs r4,r1,r4
 bmi 1f
 movs r1,r4          @ keep result of subtraction
 movs r4,r0
 lsrs r4,r5
 adcs r0,r4          @ x+=x>>i with rounding

1:
 adds r5,#1
 cmp r5,#15
 bne 3b

@ here
@ r0: exp a Q29 1..2+
@ r1: ε (residual x where x=a+ε), < 2^-14 Q30
@ r2: first exponent estimate
@ and we wish to calculate exp x=exp a exp ε~(exp a)(1+ε)

 lsrs r3,r0,#15      @ exp a Q14
 muls r3,r1          @ ε exp a Q44
 lsrs r3,#15         @ ε exp a Q29
 adcs r0,r3          @ (1+ε) exp a Q29 with rounding

 b packretns         @ pack result

.thumb_func
qfp_fln:
 push {r4,r5,r14}
 asrs r1,r0,#23
 bmi 3f              @ -ve argument?
 beq 3f              @ 0 argument?
 cmp r1,#0xff
 beq 4f              @ +Inf/NaN
 bl unpackx
 adds r2,#1
 ldr r3,=#0x2c5c85fe @ ln(2) Q30
 lsrs r1,r3,#14      @ ln(2) Q16
 muls r1,r2          @ result estimate Q16
 asrs r1,#16         @ integer contribution to result
 muls r3,r2
 lsls r4,r1,#30
 subs r3,r4          @ fractional contribution to result Q30, signed
 lsls r0,#8          @ Q31

@ here
@ r0: mantissa Q31
@ r1: integer contribution to result
@ r3: fractional contribution to result Q30, signed

 movs r5,#1          @ shift
 adr r4,ftab_exp     @ could use alternate words from dtab_exp to save space if required

2:
 movs r2,r0
 lsrs r2,r5
 adcs r2,r0          @ x+(x>>i) with rounding
 bcs 1f              @ >=2?
 movs r0,r2          @ keep result
 ldr r2,[r4]
 subs r3,r2
1:
 adds r4,#4
 adds r5,#1
 cmp r5,#15
 bne 2b

@ here
@ r0: residual x, nearly 2 Q31
@ r1: integer contribution to result
@ r3: fractional part of result Q30

 asrs r0,#2
 adds r0,r3,r0

 cmp r1,#0
 bne 2f

 asrs r0,#1
 lsls r1,#29
 adds r0,r1
 movs r2,#0
 b packretns

2:
 lsls r1,#24
 asrs r0,#6          @ Q24
 adcs r0,r1          @ with rounding
 movs r2,#5
 b packretns

3:
 ldr r0,=#0xff800000 @ -Inf
 pop {r4,r5,r15}
4:
 ldr r0,=#0x7f800000 @ +Inf
 pop {r4,r5,r15}

.align 2
ftab_exp:
.word 0x19f323ed   @ log 1+2^-1 Q30
.word 0x0e47fbe4   @ log 1+2^-2 Q30
.word 0x0789c1dc   @ log 1+2^-3 Q30
.word 0x03e14618   @ log 1+2^-4 Q30
.word 0x01f829b1   @ log 1+2^-5 Q30
.word 0x00fe0546   @ log 1+2^-6 Q30
.word 0x007f80aa   @ log 1+2^-7 Q30
.word 0x003fe015   @ log 1+2^-8 Q30
.word 0x001ff803   @ log 1+2^-9 Q30
.word 0x000ffe00   @ log 1+2^-10 Q30
.word 0x0007ff80   @ log 1+2^-11 Q30
.word 0x0003ffe0   @ log 1+2^-12 Q30
.word 0x0001fff8   @ log 1+2^-13 Q30
.word 0x0000fffe   @ log 1+2^-14 Q30

.align 2
.thumb_func
qfp_fsub:
 ldr r2,=#0x80000000
 eors r1,r2    @ flip sign on second argument
@ drop into fadd, on .align2:ed boundary

.thumb_func
qfp_fadd:
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


.align 2
.thumb_func
qfp_fmul:
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

@ This version of the division algorithm uses external divider hardware to estimate the
@ reciprocal of the divisor to about 14 bits; then a multiplication step to get a first
@ quotient estimate; then the remainder based on this estimate is used to calculate a
@ correction to the quotient. The result is good to about 27 bits and so we only need
@ to calculate the exact remainder when close to a rounding boundary.
.align 2
.thumb_func
qfp_fdiv:
 push {r4,r5,r6,r14}
fdiv_n:

 movs r4,#1
 lsls r4,#23   @ implied 1 position
 lsls r2,r1,#9 @ clear out sign and exponent
 lsrs r2,r2,#9
 orrs r2,r2,r4 @ divisor mantissa Q23 with implied 1

@ here
@ r0=packed dividend
@ r1=packed divisor
@ r2=divisor mantissa Q23
@ r4=1<<23

// see divtest.c
 lsrs r3,r2,#18 @ x2=x>>18; // Q5 32..63
 adr r5,rcpapp-32
 ldrb r3,[r5,r3] @ u=lut5[x2-32]; // Q8
 lsls r5,r2,#5
 muls r5,r5,r3
 asrs r5,#14 @ e=(i32)(u*(x<<5))>>14; // Q22
 asrs r6,r5,#11
 muls r6,r6,r6 @ e2=(e>>11)*(e>>11); // Q22
 subs r5,r6
 muls r5,r5,r3 @ c=(e-e2)*u; // Q30
 lsls r6,r3,#8
 asrs r5,#13
 adds r5,#1
 asrs r5,#1
 subs r5,r6,r5 @ u0=(u<<8)-((c+0x2000)>>14); // Q16

@ here
@ r0=packed dividend
@ r1=packed divisor
@ r2=divisor mantissa Q23
@ r4=1<<23
@ r5=reciprocal estimate Q16

 lsrs r6,r0,#23
 uxtb r3,r6        @ dividend exponent
 lsls r0,#9
 lsrs r0,#9
 orrs r0,r0,r4     @ dividend mantissa Q23

 lsrs r1,#23
 eors r6,r1        @ sign of result in bit 8
 lsrs r6,#8
 lsls r6,#31       @ sign of result in bit 31, other bits clear

@ here
@ r0=dividend mantissa Q23
@ r1=divisor sign+exponent
@ r2=divisor mantissa Q23
@ r3=dividend exponent
@ r5=reciprocal estimate Q16
@ r6b31=sign of result

 uxtb r1,r1        @ divisor exponent
 cmp r1,#0
 beq retinf
 cmp r1,#255
 beq 20f           @ divisor is infinite
 cmp r3,#0
 beq retzero
 cmp r3,#255
 beq retinf
 subs r3,r1        @ initial result exponent (no bias)
 adds r3,#125      @ add bias

 lsrs r1,r0,#8     @ dividend mantissa Q15

@ here
@ r0=dividend mantissa Q23
@ r1=dividend mantissa Q15
@ r2=divisor mantissa Q23
@ r3=initial result exponent
@ r5=reciprocal estimate Q16
@ r6b31=sign of result

 muls r1,r5

 lsrs r1,#16       @ Q15 qu0=(q15)(u*y0);
 lsls r0,r0,#15    @ dividend Q38
 movs r4,r2
 muls r4,r1        @ Q38 qu0*x
 subs r4,r0,r4     @ Q38 re0=(y<<15)-qu0*x; note this remainder is signed
 asrs r4,#10
 muls r4,r5        @ Q44 qu1=(re0>>10)*u; this quotient correction is also signed
 asrs r4,#16       @ Q28
 lsls r1,#13
 adds r1,r1,r4     @ Q28 qu=(qu0<<13)+(qu1>>16);

@ here
@ r0=dividend mantissa Q38
@ r1=quotient Q28
@ r2=divisor mantissa Q23
@ r3=initial result exponent
@ r6b31=sign of result

 lsrs r4,r1,#28
 bne 1f
@ here the quotient is less than 1<<28 (i.e., result mantissa <1.0)

 adds r1,#5
 lsrs r4,r1,#4     @ rounding + small reduction in systematic bias
 bcc 2f            @ skip if we are not near a rounding boundary
 lsrs r1,#3        @ quotient Q25
 lsls r0,#10       @ dividend mantissa Q48
 muls r1,r1,r2     @ quotient*divisor Q48
 subs r0,r0,r1     @ remainder Q48
 bmi 2f
 b 3f

1:
@ here the quotient is at least 1<<28 (i.e., result mantissa >=1.0)

 adds r3,#1        @ bump exponent (and shift mantissa down one more place)
 adds r1,#9
 lsrs r4,r1,#5     @ rounding + small reduction in systematic bias
 bcc 2f            @ skip if we are not near a rounding boundary

 lsrs r1,#4        @ quotient Q24
 lsls r0,#9        @ dividend mantissa Q47
 muls r1,r1,r2     @ quotient*divisor Q47
 subs r0,r0,r1     @ remainder Q47
 bmi 2f
3:
 adds r4,#1        @ increment quotient as we are above the rounding boundary

@ here
@ r3=result exponent
@ r4=correctly rounded quotient Q23 in range [1,2] *note closed interval*
@ r6b31=sign of result

2:
 cmp r3,#254
 bhs 10f           @ this catches both underflow and overflow
 lsls r1,r3,#23
 adds r0,r4,r1
 adds r0,r6
 pop {r4,r5,r6,r15}

@ here divisor is infinite; dividend exponent in r3
20:
 cmp r3,#255
 bne retzero

retinf:
 movs r0,#255
21:
 lsls r0,#23
 orrs r0,r6
 pop {r4,r5,r6,r15}

10:
 bge retinf       @ overflow?
 adds r1,r3,#1
 bne retzero      @ exponent <-1? return 0
@ here exponent is exactly -1
 lsrs r1,r4,#25
 bcc retzero      @ mantissa is not 01000000?
@ return minimum normal
 movs r0,#1
 lsls r0,#23
 orrs r0,r6
 pop {r4,r5,r6,r15}

retzero:
 movs r0,r6
 pop {r4,r5,r6,r15}

@ x2=[32:1:63]/32;
@ round(256 ./(x2+1/64))
.align 2
rcpapp:
.byte 252,245,237,231,224,218,213,207,202,197,193,188,184,180,176,172
.byte 169,165,162,159,156,153,150,148,145,142,140,138,135,133,131,129

@ The square root routine uses an initial approximation to the reciprocal of the square root of the argument based
@ on the top four bits of the mantissa (possibly shifted one place to make the exponent even). It then performs two
@ Newton-Raphson iterations, resulting in about 14 bits of accuracy. This reciprocal is then multiplied by
@ the original argument to produce an approximation to the result, again with about 14 bits of accuracy.
@ Then a remainder is calculated, and multiplied by the reciprocal estiamte to generate a correction term
@ giving a final answer to about 28 bits of accuracy. A final remainder calculation rounds to the correct
@ result if necessary.
@ Again, the fixed-point calculation is carefully implemented to preserve accuracy, and similar comments to those
@ made above on the fast division routine apply.
@ The reciprocal square root calculation has been tested for all possible (possibly shifted) input mantissa values.
.align 2
.thumb_func
qfp_fsqrt:
 push {r4}
 lsls r1,r0,#1
 bcs sq_0         @ negative?
 lsls r1,#8
 lsrs r1,#9       @ mantissa
 movs r2,#1
 lsls r2,#23
 adds r1,r2       @ insert implied 1
 lsrs r2,r0,#23   @ extract exponent
 beq sq_2         @ zero?
 cmp r2,#255      @ infinite?
 beq sq_1
 adds r2,#125     @ correction for packing
 asrs r2,#1       @ exponent/2, LSB into carry
 bcc 1f
 lsls r1,#1       @ was even: double mantissa; mantissa y now 1..4 Q23
1:
 adr r4,rsqrtapp-4@ first four table entries are never accessed because of the mantissa's leading 1
 lsrs r3,r1,#21   @ y Q2
 ldrb r4,[r4,r3]  @ initial approximation to reciprocal square root a0 Q8

 lsrs r0,r1,#7    @ y Q16: first Newton-Raphson iteration
 muls r0,r4       @ a0*y Q24
 muls r0,r4       @ r0=p0=a0*y*y Q32
 asrs r0,#12      @ r0 Q20
 muls r0,r4       @ dy0=a0*r0 Q28
 asrs r0,#13      @ dy0 Q15
 lsls r4,#8       @ a0 Q16
 subs r4,r0       @ a1=a0-dy0/2 Q16-Q15/2 -> Q16
 adds r4,#170     @ mostly remove systematic error in this approximation: gains approximately 1 bit

 movs r0,r4       @ second Newton-Raphson iteration
 muls r0,r0       @ a1*a1 Q32
 lsrs r0,#15      @ a1*a1 Q17
 lsrs r3,r1,#8    @ y Q15
 muls r0,r3       @ r1=p1=a1*a1*y Q32
 asrs r0,#12      @ r1 Q20
 muls r0,r4       @ dy1=a1*r1 Q36
 asrs r0,#21      @ dy1 Q15
 subs r4,r0       @ a2=a1-dy1/2 Q16-Q15/2 -> Q16

 muls r3,r4       @ a3=y*a2 Q31
 lsrs r3,#15      @ a3 Q16
@ here a2 is an approximation to the reciprocal square root
@ and a3 is an approximation to the square root
 movs r0,r3
 muls r0,r0       @ a3*a3 Q32
 lsls r1,#9       @ y Q32
 subs r0,r1,r0    @ r2=y-a3*a3 Q32 remainder
 asrs r0,#5       @ r2 Q27
 muls r4,r0       @ r2*a2 Q43
 lsls r3,#7       @ a3 Q23
 asrs r0,r4,#15   @ r2*a2 Q28
 adds r0,#16      @ rounding to Q24
 asrs r0,r0,#6    @ r2*a2 Q22
 add r3,r0        @ a4 Q23: candidate final result
 bcc sq_3         @ near rounding boundary? skip if no rounding needed
 mov r4,r3
 adcs r4,r4       @ a4+0.5ulp Q24
 muls r4,r4       @ Q48
 lsls r1,#16      @ y Q48
 subs r1,r4       @ remainder Q48
 bmi sq_3
 adds r3,#1       @ round up
sq_3:
 lsls r2,#23      @ pack exponent
 adds r0,r2,r3
sq_6:
 pop {r4}
 bx r14

sq_0:
 lsrs r1,#24
 beq sq_2         @ -0: return it
@ here negative and not -0: return -Inf
 asrs r0,#31
sq_5:
 lsls r0,#23
 b sq_6
sq_1:             @ +Inf
 lsrs r0,#23
 b sq_5
sq_2:
 lsrs r0,#31
 lsls r0,#31
 b sq_6

@ round(sqrt(2^22./[72:16:248]))
rsqrtapp:
.byte 0xf1,0xda,0xc9,0xbb, 0xb0,0xa6,0x9e,0x97, 0x91,0x8b,0x86,0x82

qfp_lib_end:
