/* High-resolution PWM with adjustable frequency for Arduino Uno R4 (Renesas RA4M1).
 *
 * Uses the core's PwmOut class on a GPT timer channel and writes the duty in
 * raw timer counts, so the resolution is the full timer period:
 *     counts per period = 48 MHz / prescaler / frequency
 * For example 2400 counts (11.2 bits) at 20 kHz, 48000 counts (15.6 bits) at
 * 1 kHz. analogWrite() instead goes through analogWriteResolution (8 bits by
 * default) and a fixed 490 Hz.
 *
 * Run-time frequency change keeps the prescaler chosen at setup for the
 * minimum frequency and only writes a new period (buffered by the timer, so
 * the change takes effect at the end of the current period, without a
 * glitch). The duty fraction is kept.
 *
 * AGT pins are refused: AGT timers are 16-bit at a lower clock, and the core
 * uses an AGT for millis()/micros().
 * Each GPT channel drives two pins (A and B) from one counter, so two pins of
 * the same channel can't have independent frequencies: the second is refused.
 */
#include "MW_R4PWM.h"

#if !( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER) )
#include "Arduino.h"
#include "pwm.h"

#if !defined(ARDUINO_ARCH_RENESAS_UNO) && !defined(ARDUINO_ARCH_RENESAS)
#error "MW_R4PWM supports only Arduino Uno R4 / Nano R4 (Renesas) boards."
#endif

typedef struct {
    PwmOut  *pwm;
    uint8_T  pin;
    uint8_T  channel;      /* GPT channel */
    real32_T countsPerSec; /* timer clock after the prescaler */
    uint32_T maxCounts;    /* period limit for the prescaler (min frequency) */
    uint32_T period;       /* current counts per period */
    real32_T duty;         /* current duty fraction */
} r4Pwm;

static r4Pwm r4PwmInst[MW_R4PWM_MAX_INSTANCES];
static uint8_T r4PwmCount = 0;

static const uint16_T r4PwmDividers[] = {1, 4, 16, 64, 256, 1024};

static inline uint32_T r4PwmDutyCounts(const r4Pwm *p)
{
    return (uint32_T)(p->duty * (real32_T)p->period + 0.5f);
}

extern "C" int MW_R4PWM_Setup(uint8_T pin, real32_T freqHz, real32_T minFreqHz, uint8_T *index)
{
    *index = 255;
    if (r4PwmCount >= MW_R4PWM_MAX_INSTANCES || pin >= PINS_COUNT) {
        return -1;
    }
    if (!(freqHz > 0.0f)) {
        return -1;
    }
    if (!(minFreqHz > 0.0f) || minFreqHz > freqHz) {
        minFreqHz = freqHz;
    }

    auto cfg = getPinCfgs(pin, PIN_CFG_REQ_PWM);
    if (cfg[0] == 0 || IS_PIN_AGT_PWM(cfg[0])) {
        return -1;
    }
    uint8_T channel = (uint8_T)GET_CHANNEL(cfg[0]);
    for (uint8_T i = 0; i < r4PwmCount; i++) {
        if (r4PwmInst[i].channel == channel) {
            return -1;
        }
    }

    /* Start at the minimum frequency so the core picks the prescaler for it */
    PwmOut *pwm = new PwmOut(pin);
    if (pwm == nullptr || !pwm->begin(minFreqHz, 0.0f)) {
        delete pwm;
        return -1;
    }
    FspTimer *t = pwm->get_timer();
    uint32_T minPeriod = t->get_period_raw();

    /* Timer clock after the prescaler: the divider whose count matches */
    uint32_T pclk = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKD);
    real32_T cps = (real32_T)minPeriod * minFreqHz;
    for (uint8_T k = 0; k < sizeof(r4PwmDividers) / sizeof(r4PwmDividers[0]); k++) {
        real32_T c = (real32_T)pclk / (real32_T)r4PwmDividers[k];
        real32_T expected = c / minFreqHz;
        if (expected - (real32_T)minPeriod < 2.0f && (real32_T)minPeriod - expected < 2.0f) {
            cps = c;
            break;
        }
    }

    r4Pwm *p = &r4PwmInst[r4PwmCount];
    p->pwm = pwm;
    p->pin = pin;
    p->channel = channel;
    p->countsPerSec = cps;
    p->maxCounts = minPeriod;
    p->period = minPeriod;
    p->duty = 0.0f;
    *index = r4PwmCount++;

    MW_R4PWM_SetFrequency(*index, freqHz);
    return 0;
}

extern "C" void MW_R4PWM_SetDuty(uint8_T index, real32_T duty)
{
    if (index >= r4PwmCount) {
        return;
    }
    r4Pwm *p = &r4PwmInst[index];
    if (!(duty > 0.0f)) {          /* also catches NaN */
        duty = 0.0f;
    } else if (duty > 1.0f) {
        duty = 1.0f;
    }
    p->duty = duty;
    p->pwm->pulseWidth_raw((int)r4PwmDutyCounts(p));
}

extern "C" void MW_R4PWM_SetFrequency(uint8_T index, real32_T freqHz)
{
    if (index >= r4PwmCount || !(freqHz > 0.0f)) {
        return;
    }
    r4Pwm *p = &r4PwmInst[index];
    real32_T counts = p->countsPerSec / freqHz + 0.5f;
    uint32_T period;
    if (counts >= (real32_T)p->maxCounts) {
        period = p->maxCounts;
    } else if (counts < 2.0f) {
        period = 2;
    } else {
        period = (uint32_T)counts;
    }
    if (period == p->period) {
        return;
    }
    p->period = period;
    p->pwm->get_timer()->set_period(period);
    p->pwm->pulseWidth_raw((int)r4PwmDutyCounts(p));
}

extern "C" uint32_T MW_R4PWM_PeriodCounts(uint8_T index)
{
    return (index < r4PwmCount) ? r4PwmInst[index].period : 0U;
}

extern "C" void MW_R4PWM_Release(void)
{
    for (uint8_T i = 0; i < r4PwmCount; i++) {
        r4PwmInst[i].pwm->pulseWidth_raw(0);
        r4PwmInst[i].pwm->end();
        delete r4PwmInst[i].pwm;
        r4PwmInst[i].pwm = nullptr;
    }
    r4PwmCount = 0;
}

#endif
