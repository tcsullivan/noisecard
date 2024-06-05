/*
 * ESP32 Second-Order Sections IIR Filter implementation
 *
 * (c)2019 Ivan Kostoski
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *    
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef SOS_IIR_FILTER_H
#define SOS_IIR_FILTER_H

#include <algorithm>
#include <array>
#include <cstdint>
#include <ranges>
#include <utility>

extern "C" {
#include <qfplib-m0-full.h>
}

float qfp_fpow(float b, float e)
{
    return qfp_fexp(qfp_fmul(e, qfp_fln(b)));
}

float qfp_flog10(float x)
{
    static const auto ln10 = qfp_fln(10.f);
    return qfp_fdiv(qfp_fln(x), ln10);
}

class sos_t
{
    float v;

public:
    constexpr sos_t(float v_ = 0.f): v(v_) {}

    sos_t operator+(auto x) const noexcept {
        return qfp_fadd(v, x);
    }

    sos_t operator-(const sos_t& o) const noexcept {
        return qfp_fsub(v, o.v);
    }

    sos_t operator*(auto x) const noexcept {
        return qfp_fmul(v, x);
    }

    sos_t operator/(auto x) const noexcept {
        return qfp_fdiv(v, x);
    }

    sos_t& operator+=(const sos_t& o) noexcept {
        return (*this = *this + o);
    }

    operator float() const noexcept {
        return v;
    }
};

struct SOS_Coefficients {
  sos_t b1;
  sos_t b2;
  sos_t a1;
  sos_t a2;
};

struct SOS_Delay_State {
  sos_t w0;
  sos_t w1;
};

/**
 * Envelops above asm functions into C++ class
 */
template<std::size_t N>
struct SOS_IIR_Filter {
  const sos_t gain;
  std::array<SOS_Coefficients, N> sos;
  std::array<SOS_Delay_State, N> w;

  // Template constructor for const filter declaration
  constexpr SOS_IIR_Filter(const sos_t gain, const SOS_Coefficients (&_sos)[N]):
    gain(gain)
  {
    std::copy(_sos, _sos + N, sos.begin());
  }

  void filter(auto samples, std::size_t n = N) {
    for (auto [coeffs, ww] : std::views::zip(sos, w) | std::views::take(n)) {
        // Assumes a0 and b0 coefficients are one (1.0)
        for (auto& s : samples) {
            auto f6 = s + coeffs.a1 * ww.w0 + coeffs.a2 * ww.w1;
            s = f6 + coeffs.b1 * ww.w0 + coeffs.b2 * ww.w1;
            ww.w1 = std::exchange(ww.w0, f6);
        }
    }
  }

  sos_t filter_sum_sqr(auto samples) {
    const auto& coeffs = sos.back();
    auto& ww = w.back();
    sos_t sum_sqr (0.f);

    filter(samples, N - 1);

    // Assumes a0 and b0 coefficients are one (1.0)
    for (auto& s : samples) {
      auto f6 = s + coeffs.a1 * ww.w0 + coeffs.a2 * ww.w1;
      s = f6 + coeffs.b1 * ww.w0 + coeffs.b2 * ww.w1;
      ww.w1 = std::exchange(ww.w0, f6);
      sum_sqr += s * gain * s * gain;
    }

    return sum_sqr;
  }
};

#endif  // SOS_IIR_FILTER_H

// Knowles SPH0645LM4H-B, rev. B
// https://cdn-shop.adafruit.com/product-files/3421/i2S+Datasheet.PDF
// B ~= [1.001234, -1.991352, 0.990149]
// A ~= [1.0, -1.993853, 0.993863]
// With additional DC blocking component
SOS_IIR_Filter SPH0645LM4H_B_RB = {
  /* gain: */ sos_t(1.00123377961525f),
  /* sos: */ { // Second-Order Sections {b1, b2, -a1, -a2}
         { sos_t(-1.0f), sos_t(0.0f),
           sos_t(+0.9992f), sos_t(0.0f) },  // DC blocker, a1 = -0.9992
         { sos_t(-1.988897663539382f), sos_t(+0.988928479008099f),
           sos_t(+1.993853376183491f), sos_t(-0.993862821429572f) } }
};

//
// A-weighting IIR Filter, Fs = 48KHz
// (By Dr. Matt L., Source: https://dsp.stackexchange.com/a/36122)
// B = [0.169994948147430, 0.280415310498794, -1.120574766348363, 0.131562559965936, 0.974153561246036, -0.282740857326553, -0.152810756202003]
// A = [1.0, -2.12979364760736134, 0.42996125885751674, 1.62132698199721426, -0.96669962900852902, 0.00121015844426781, 0.04400300696788968]
SOS_IIR_Filter A_weighting = {
  /* gain: */ sos_t(0.169994948147430f),
  /* sos: */ { // Second-Order Sections {b1, b2, -a1, -a2}
         { sos_t(-2.00026996133106f), sos_t(+1.00027056142719f),
           sos_t(-1.060868438509278f), sos_t(-0.163987445885926f) },
         { sos_t(+4.35912384203144f), sos_t(+3.09120265783884f),
           sos_t(+1.208419926363593f), sos_t(-0.273166998428332f) },
         { sos_t(-0.70930303489759f), sos_t(-0.29071868393580f),
           sos_t(+1.982242159753048f), sos_t(-0.982298594928989f) } }
};

////
//// C-weighting IIR Filter, Fs = 48KHz
//// Designed by invfreqz curve-fitting, see respective .m file
//// B = [-0.49164716933714026, 0.14844753846498662, 0.74117815661529129, -0.03281878334039314, -0.29709276192593875, -0.06442545322197900, -0.00364152725482682]
//// A = [1.0, -1.0325358998928318, -0.9524000181023488, 0.8936404694728326   0.2256286147169398  -0.1499917107550188, 0.0156718181681081]
//SOS_IIR_Filter C_weighting = {
//  gain: -0.491647169337140,
//  sos: {
//    { +1.4604385758204708, +0.5275070373815286, +1.9946144559930252, -0.9946217070140883 },
//    { +0.2376222404939509, +0.0140411206016894, -1.3396585608422749, -0.4421457807694559 },
//    { -2.0000000000000000, +1.0000000000000000, +0.3775800047420818, -0.0356365756680430 } }
//};
