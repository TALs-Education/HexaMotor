/* Quadrature encoder counter for Arduino Uno R4 (Renesas RA4M1).
 * Counts every edge of A and B (4x) with interrupts. */
#ifndef MW_R4ENCODER_H
#define MW_R4ENCODER_H

#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#if !( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER) )

/* pullup: 1 = INPUT_PULLUP, 0 = INPUT
 * Returns 0 on success, -1 on failure (too many instances, pin without an
 * IRQ, or A and B on the same IRQ channel). */
int MW_R4Enc_Setup(uint8_T pinA, uint8_T pinB, uint8_T pullup, uint8_T *index);

/* Accumulated count (4 per encoder line); 0 for an index that failed setup. */
int32_T MW_R4Enc_Read(uint8_T index);

/* Sets the count of one encoder to 0. */
void MW_R4Enc_Reset(uint8_T index);

void MW_R4Enc_Release(void);

#else
#define MW_R4Enc_Setup(a, b, p, i) (0)
#define MW_R4Enc_Read(i) (0)
#define MW_R4Enc_Reset(i) ((void)0)
#define MW_R4Enc_Release() ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#define MW_R4ENC_MAX_INSTANCES 4

#endif /* MW_R4ENCODER_H */
