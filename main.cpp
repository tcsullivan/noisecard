#include "hal.h"
#include "ch.h"
#include "sos-iir-filter.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <ranges>

static constexpr auto& WEIGHTING       = A_weighting;
static constexpr auto& MIC_EQUALIZER   = SPH0645LM4H_B_RB;
static constexpr float MIC_OFFSET_DB   (  0.f); // Linear offset
static constexpr float MIC_SENSITIVITY (-26.f); // dBFS value expected at MIC_REF_DB
static constexpr float MIC_REF_DB      ( 94.f); // dB where sensitivity is specified
static constexpr sos_t MIC_OVERLOAD_DB (120.f); // dB - Acoustic overload point
static constexpr sos_t MIC_NOISE_DB    ( 29.f); // dB - Noise floor
static constexpr auto  MIC_BITS        = 18u;
static constexpr auto  SAMPLE_RATE     = 48000u;

static constexpr unsigned I2S_BUFSIZ = 1024;
static constexpr unsigned I2S_STRIDE = 16;

// Calculate reference amplitude value at compile time
static const auto MIC_REF_AMPL = qfp_fpow(10.f, MIC_SENSITIVITY / 20) *
    ((1 << (MIC_BITS - 1)) - 1);

static SEMAPHORE_DECL(i2sReady, 0);
static THD_WORKING_AREA(waThread1, 128);
static std::array<uint32_t, I2S_BUFSIZ> i2sBuffer;
static sos_t Leq_sum_sqr (0.f);
static unsigned Leq_samples = 0;

static THD_FUNCTION(Thread1, arg);
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

THD_TABLE_BEGIN
  THD_TABLE_THREAD(0, "main",     waThread1,       Thread1,      NULL)
THD_TABLE_END

int main(void)
{
    halInit();
    chSysInit();
    for (;;)
        asm("wfi");
}

THD_FUNCTION(Thread1, arg)
{
    (void)arg;
  
    chThdSleepMilliseconds(2000);
  
    palSetPadMode(GPIOB, 7, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOF, 2, PAL_MODE_UNCONNECTED);
    palSetLineMode(LINE_I2S_SD, PAL_MODE_ALTERNATE(0));
    palSetLineMode(LINE_I2S_WS, PAL_MODE_ALTERNATE(0));
    palSetLineMode(LINE_I2S_CK, PAL_MODE_ALTERNATE(0));
    palSetLineMode(LINE_USART2_TX, PAL_MODE_ALTERNATE(1));
  
    sdStart(&SD2, NULL);
    sdWrite(&SD2, (uint8_t *)"Noisemeter\n", 11);
    chThdSleepMilliseconds(100);
  
    i2sStart(&I2SD1, &i2sConfig);
    i2sStartExchange(&I2SD1);
  
    uint8_t strbuf[7] = { 0, 0, 0, 'd', 'B', '\n', '\0' };
    for (;;) {
        palSetPad(GPIOB, 7);
        chSemWait(&i2sReady);

        const auto Leq_RMS = qfp_fsqrt((float)Leq_sum_sqr / Leq_samples);
        const auto Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 *
            qfp_flog10(Leq_RMS / MIC_REF_AMPL);
        Leq_sum_sqr = sos_t(0.f);
        Leq_samples = 0;

        auto n = std::clamp(static_cast<int32_t>(Leq_dB), 0l, 999l);
        strbuf[2] = n % 10 + '0'; n /= 10;
        strbuf[1] = n % 10 + '0'; n /= 10;
        strbuf[0] = n ? n + '0' : ' ';
        sdWrite(&SD2, strbuf, sizeof(strbuf));
        palClearPad(GPIOB, 7);
    }
}

int32_t fixsample(uint32_t s) {
    return (int32_t)(((s & 0xFFFF) << 16) | (s >> 16)) >> (32 - MIC_BITS);
}

void i2sCallback(I2SDriver *i2s)
{
    palSetPad(GPIOB, 7);
    const auto halfsize = i2sBuffer.size() / 2;
    const auto offset = i2sIsBufferComplete(i2s) ? halfsize : 0;
    auto samples = reinterpret_cast<sos_t *>(i2sBuffer.data() + offset);
    std::ranges::copy(
        std::views::counted(i2sBuffer.begin() + offset, halfsize / I2S_STRIDE)
            | std::ranges::views::stride(2)
            | std::views::transform([](uint32_t s) { return sos_t(fixsample(s)); }),
        samples);
    auto samps = std::views::counted(samples, halfsize / (2 * I2S_STRIDE));

    // Accumulate Leq sum
    MIC_EQUALIZER.filter(samps);
    Leq_sum_sqr += WEIGHTING.filter_sum_sqr(samps);
    Leq_samples += samps.size();

    // Wakeup main thread for dB calculation every second
    if (Leq_samples >= SAMPLE_RATE / I2S_STRIDE) {
        chSemSignalI(&i2sReady);
    }
    palClearPad(GPIOB, 7);
}

