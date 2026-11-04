/* Analog input with selectable reference and resolution for Arduino Uno R4 (Renesas RA4M1). */
#ifndef MW_R4ANALOGIN_H
#define MW_R4ANALOGIN_H

#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#if !( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER) )

/* ref: 0 = default (AVCC, 5 V), 1 = external (AREF pin),
 *      2 = internal IVREF (1.43 V nominal).
 * The ADC reference is one setting for the whole board: the first block sets
 * it, and a block asking for a different reference returns -1 (and the
 * reference stays as the first block set it). Returns 0 on success. */
int MW_R4AnalogIn_Setup(uint8_T pin, uint8_T ref);

/* Raw reading at the requested resolution (8, 10, 12 or 14 bits). */
uint16_T MW_R4AnalogIn_Read(uint8_T pin, uint8_T bits);

#else
#define MW_R4AnalogIn_Setup(p, r) (0)
#define MW_R4AnalogIn_Read(p, b) (0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* MW_R4ANALOGIN_H */
