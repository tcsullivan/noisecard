#ifndef __QFPLIB_M0_FULL_H__
#define __QFPLIB_M0_FULL_H__

/*
Copyright 2019-2024 Mark Owen
http://www.quinapalus.com
E-mail: qfp@quinapalus.com

This file is free software: you can redistribute it and/or modify
it under the terms of version 2 of the GNU General Public License
as published by the Free Software Foundation.

This file is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this file.  If not, see <http://www.gnu.org/licenses/> or
write to the Free Software Foundation, Inc., 51 Franklin Street,
Fifth Floor, Boston, MA  02110-1301, USA.
*/

typedef unsigned           int ui32;
typedef                    int i32;

extern float  qfp_fadd          (float x,float y);
extern float  qfp_fsub          (float x,float y);
extern float  qfp_fmul          (float x,float y);
extern float  qfp_fdiv          (float x,float y);
extern int    qfp_fcmp          (float x,float y);
extern float  qfp_fsqrt         (float x);
extern i32    qfp_float2int     (float x);
extern i32    qfp_float2fix     (float x,int f);
extern ui32   qfp_float2uint    (float x);
extern ui32   qfp_float2ufix    (float x,int f);
extern float  qfp_int2float     (i32 x);
extern float  qfp_fix2float     (i32 x,int f);
extern float  qfp_uint2float    (ui32 x);
extern float  qfp_ufix2float    (ui32 x,int f);
extern float  qfp_fexp          (float x);
extern float  qfp_fln           (float x);

#endif
