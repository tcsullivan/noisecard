/**
 * Copyright (C) 2024  Clyne Sullivan <clyne@bitgloo.com>
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
#include "hal.h"
#include "sos-iir-filter.h"

#include <algorithm>
#include <atomic>
#include <array>
#include <cstring>
#include <ranges>

static constexpr auto& WEIGHTING       = A_weighting;
static constexpr auto& MIC_EQUALIZER   = SPH0645LM4H_B_RB;
static constexpr sos_t MIC_OFFSET_DB   (  0.f); // Linear offset
static constexpr sos_t MIC_SENSITIVITY (-26.f); // dBFS value expected at MIC_REF_DB
static constexpr sos_t MIC_REF_DB      ( 94.f); // dB where sensitivity is specified
static constexpr sos_t MIC_OVERLOAD_DB (120.f); // dB - Acoustic overload point
static constexpr sos_t MIC_NOISE_DB    ( 29.f); // dB - Noise floor
static constexpr auto  MIC_BITS        = 18u;
static constexpr auto  SAMPLE_RATE     = 48000u;

static constexpr unsigned I2S_BUFSIZ = 1024;
static constexpr unsigned I2S_USESIZ = 16;

// Calculate reference amplitude value at compile time
static const auto MIC_REF_AMPL = sos_t((1 << (MIC_BITS - 1)) - 1) *
    qfp_fpow(10.f, MIC_SENSITIVITY / 20.f);

static std::atomic_bool i2sReady;
static std::array<uint32_t, I2S_BUFSIZ> i2sBuffer;
static sos_t Leq_sum_sqr (0.f);
static unsigned Leq_samples = 0;

static void blinkDb(int db);
static void i2sCallback(I2SDriver *i2s);

static constexpr unsigned I2SPRval = 16'000'000 / SAMPLE_RATE / 32 / 2;
static constexpr I2SConfig i2sConfig = {
    /* TX buffer */ NULL,
    /* RX buffer */ i2sBuffer.data(),
    /* Size */      i2sBuffer.size(),
    /* Callback */  i2sCallback,
    /* I2SCFGR */   (3 << SPI_I2SCFGR_I2SCFG_Pos) | // Master receive
                    (0 << SPI_I2SCFGR_I2SSTD_Pos) | // Philips I2S
                    (1 << SPI_I2SCFGR_DATLEN_Pos) | // 24-bit
                    SPI_I2SCFGR_CHLEN,              // 32-bit frame
    /* I2SPR */     (I2SPRval / 2) | ((I2SPRval & 1) ? SPI_I2SPR_ODD : 0)
};

int main(void)
{
    halInit();
    osalSysEnable();
  
    i2sReady.store(true);
    i2sStart(&I2SD1, &i2sConfig);
    i2sStartExchange(&I2SD1);
    // Microphone warmup time
    osalThreadSleepMilliseconds(140);
    // Reach filter delay steady state
    i2sReady.store(false);
    osalThreadSleepMilliseconds(120);
    // Discard initial readings
    Leq_sum_sqr = 0.f;
    Leq_samples = 0;

    for (;;) {
        i2sReady.store(false);
        SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
        //palClearLine(LINE_TP1);
        __WFI();
        //palSetLine(LINE_TP1);

        const auto sum_sqr = std::exchange(Leq_sum_sqr, sos_t(0.f));
        const auto count = std::exchange(Leq_samples, 0);
        const sos_t Leq_RMS = qfp_fsqrt(sum_sqr / qfp_uint2float(count));
        const sos_t Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + sos_t(20.f) *
            qfp_flog10(Leq_RMS / MIC_REF_AMPL);
        const auto n = std::clamp(qfp_float2int(Leq_dB), 0, 999);
        blinkDb(n);
    }
}

void blinkDb(int db)
{
    auto line = LINE_LED0;

    if (db < 45)
        line = LINE_LED0;
    else if (db < 55)
        line = LINE_LED1;
    else if (db < 65)
        line = LINE_LED2;
    else if (db < 75)
        line = LINE_LED3;
    else if (db < 82)
        line = LINE_LED4;
    else if (db < 87)
        line = LINE_LED5;
    else if (db < 92)
        line = LINE_LED6;
    else if (db < 97)
        line = LINE_LED7;
    else if (db < 102)
        line = LINE_LED8;
    else
        line = LINE_LED9;

    palClearLine(line);
    osalThreadSleepMilliseconds(50);
    palSetLine(line);
}

__attribute__((section(".data")))
int32_t fixsample(uint32_t s) {
    return (int32_t)(((s & 0xFFFF) << 16) | (s >> 16)) >> (32 - MIC_BITS);
}

__attribute__((section(".data")))
void i2sCallback(I2SDriver *i2s)
{
    if (i2sReady.load())
        return;

    //palSetLine(LINE_TP1);

    const auto halfsize = i2sBuffer.size() / 2;
    const auto source = i2sBuffer.data() + (i2sIsBufferComplete(i2s) ? halfsize : 0);
    auto samples = reinterpret_cast<sos_t *>(source);
    std::ranges::copy(
        std::views::counted(source, I2S_USESIZ * 2)
            | std::ranges::views::stride(2)
            | std::views::transform([](uint32_t s) { return sos_t(qfp_int2float_asm(fixsample(s))); }),
        samples);
    auto samps = std::views::counted(samples, I2S_USESIZ);

    // Accumulate Leq sum
    MIC_EQUALIZER.filter(samps);
    Leq_sum_sqr += WEIGHTING.filter_sum_sqr(samps);
    Leq_samples += halfsize / 2;

    // Wakeup main thread for dB calculation every half second
    if (Leq_samples >= SAMPLE_RATE / 2) {
        i2sReady.store(true);
        SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
    }

    //palClearLine(LINE_TP1);
}

