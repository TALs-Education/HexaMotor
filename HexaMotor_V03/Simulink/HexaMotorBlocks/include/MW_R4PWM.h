/* High-resolution PWM with adjustable frequency for Arduino Uno R4 (Renesas RA4M1). */
#ifndef MW_R4PWM_H
#define MW_R4PWM_H

#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#if !( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER) )

/* Starts PWM on a GPT pin at freqHz with 0 % duty.
 * minFreqHz <= freqHz sets the timer prescaler: the frequency can later be
 * changed at run time down to minFreqHz (same prescaler, no restart).
 * Returns 0 on success, -1 on failure (too many instances, pin without a
 * GPT output, GPT channel already used by another R4 PWM block, or a
 * frequency the timer can't produce). */
int MW_R4PWM_Setup(uint8_T pin, real32_T freqHz, real32_T minFreqHz, uint8_T *index);

/* duty: 0..1 (clamped). Resolution is one timer count. */
void MW_R4PWM_SetDuty(uint8_T index, real32_T duty);

/* Changes the frequency (clamped to the range set up), keeps the duty fraction. */
void MW_R4PWM_SetFrequency(uint8_T index, real32_T freqHz);

/* Timer counts per period (duty resolution) of an instance; 0 if not set up. */
uint32_T MW_R4PWM_PeriodCounts(uint8_T index);

void MW_R4PWM_Release(void);

#else
#define MW_R4PWM_Setup(p, f, m, i) (0)
#define MW_R4PWM_SetDuty(i, d) ((void)0)
#define MW_R4PWM_SetFrequency(i, f) ((void)0)
#define MW_R4PWM_PeriodCounts(i) (0U)
#define MW_R4PWM_Release() ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#define MW_R4PWM_MAX_INSTANCES 8

#endif /* MW_R4PWM_H */
