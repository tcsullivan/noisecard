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
static constexpr unsigned I2S_STRIDE = 16;

// Calculate reference amplitude value at compile time
static const auto MIC_REF_AMPL = sos_t((1 << (MIC_BITS - 1)) - 1) *
    qfp_fpow(10.f, MIC_SENSITIVITY / 20.f);

static std::atomic_bool i2sReady;
static std::array<uint32_t, I2S_BUFSIZ> i2sBuffer;
static sos_t Leq_sum_sqr (0.f);
static unsigned Leq_samples = 0;

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

//static const halclkcfg_t halClockDefault = {
//  .pwr_cr1              = PWR_CR1_VOS_0,
//  .pwr_cr2              = STM32_PWR_CR2,
//  .rcc_cr               = RCC_CR_PLLON | RCC_CR_HSION,
//  .rcc_cfgr             = (6 << RCC_CFGR_PPRE_Pos) | (1 << RCC_CFGR_HPRE_Pos) | RCC_CFGR_SW_PLL,
//  .rcc_pllcfgr          = (STM32_PLLR_VALUE << RCC_PLLCFGR_PLLR_Pos) | RCC_PLLCFGR_PLLREN |
//                          (STM32_PLLN_VALUE << RCC_PLLCFGR_PLLN_Pos) |
//                          ((STM32_PLLM_VALUE - 1) << RCC_PLLCFGR_PLLM_Pos) |
//                          RCC_PLLCFGR_PLLSRC_HSI,
//  .flash_acr            = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | (2 << FLASH_ACR_LATENCY_Pos)
//};
//
//static const halclkcfg_t halClockSleep = {
//  .pwr_cr1              = PWR_CR1_VOS_0,
//  .pwr_cr2              = STM32_PWR_CR2,
//  .rcc_cr               = RCC_CR_PLLON | RCC_CR_HSION,
//  .rcc_cfgr             = (0 << RCC_CFGR_PPRE_Pos) | (10 << RCC_CFGR_HPRE_Pos) | RCC_CFGR_SW_PLL,
//  .rcc_pllcfgr          = (STM32_PLLR_VALUE << RCC_PLLCFGR_PLLR_Pos) | RCC_PLLCFGR_PLLREN |
//                          (STM32_PLLN_VALUE << RCC_PLLCFGR_PLLN_Pos) |
//                          ((STM32_PLLM_VALUE - 1) << RCC_PLLCFGR_PLLM_Pos) |
//                          RCC_PLLCFGR_PLLSRC_HSI,
//  .flash_acr            = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | (2 << FLASH_ACR_LATENCY_Pos)
//};

int main(void)
{
    halInit();
    osalSysEnable();
    osalThreadSleepMilliseconds(2000);
  
    palSetPadMode(GPIOB, 7, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOF, 2, PAL_MODE_UNCONNECTED);
    palSetLineMode(LINE_I2S_SD, PAL_MODE_ALTERNATE(0));
    palSetLineMode(LINE_I2S_WS, PAL_MODE_ALTERNATE(0));
    palSetLineMode(LINE_I2S_CK, PAL_MODE_ALTERNATE(0));
    palSetLineMode(LINE_USART2_TX, PAL_MODE_ALTERNATE(1));
  
    sdStart(&SD2, NULL);
    sdWrite(&SD2, (uint8_t *)"Noisemeter\n", 11);
    osalThreadSleepMilliseconds(2);
  
    i2sStart(&I2SD1, &i2sConfig);
    i2sStartExchange(&I2SD1);

    i2sReady.store(false);
  
    uint8_t strbuf[7] = { 0, 0, 0, 'd', 'B', '\n', '\0' };
    for (;;) {
        //if (halClockSwitchMode(&halClockSleep)) {
        //    sdWrite(&SD2, (uint8_t *)"sleepf\n", 7);
        //    osalThreadSleepMilliseconds(5000);
        //}
        while (!i2sReady.load())
            asm("wfi");
        i2sReady.store(false);
        //if (halClockSwitchMode(&halClockDefault)) {
        //    sdWrite(&SD2, (uint8_t *)"sleepf\n", 7);
        //    osalThreadSleepMilliseconds(5000);
        //}

        palSetPad(GPIOB, 7);
        const sos_t Leq_RMS = qfp_fsqrt(Leq_sum_sqr / qfp_uint2float(Leq_samples));
        const sos_t Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + sos_t(20.f) *
            qfp_flog10(Leq_RMS / MIC_REF_AMPL);
        Leq_sum_sqr = sos_t(0.f);
        Leq_samples = 0;

        auto n = std::clamp(qfp_float2int(Leq_dB), 0, 999);
        strbuf[2] = n % 10 + '0'; n /= 10;
        strbuf[1] = n % 10 + '0'; n /= 10;
        strbuf[0] = n ? n + '0' : ' ';
        sdWrite(&SD2, strbuf, sizeof(strbuf));
        osalThreadSleepMilliseconds(2);
        palClearPad(GPIOB, 7);
    }
}

__attribute__((section(".data")))
int32_t fixsample(uint32_t s) {
    return (int32_t)(((s & 0xFFFF) << 16) | (s >> 16)) >> (32 - MIC_BITS);
}

__attribute__((section(".data")))
void i2sCallback(I2SDriver *i2s)
{
    //halClockSwitchMode(&halClockDefault);

    palSetPad(GPIOB, 7);
    const auto halfsize = i2sBuffer.size() / 2;
    const auto source = i2sBuffer.data() + (i2sIsBufferComplete(i2s) ? halfsize : 0);
    auto samples = reinterpret_cast<sos_t *>(source);
    std::ranges::copy(
        std::views::counted(source, halfsize / I2S_STRIDE)
            | std::ranges::views::stride(2)
            | std::views::transform([](uint32_t s) { return sos_t(qfp_int2float_asm(fixsample(s))); }),
        samples);
    auto samps = std::views::counted(samples, halfsize / (2 * I2S_STRIDE));

    // Accumulate Leq sum
    MIC_EQUALIZER.filter(samps);
    Leq_sum_sqr += WEIGHTING.filter_sum_sqr(samps);
    Leq_samples += samps.size();

    // Wakeup main thread for dB calculation every second
    if (Leq_samples >= SAMPLE_RATE / I2S_STRIDE) {
        i2sReady.store(true);
    }
    palClearPad(GPIOB, 7);

    //halClockSwitchMode(&halClockSleep);
}

