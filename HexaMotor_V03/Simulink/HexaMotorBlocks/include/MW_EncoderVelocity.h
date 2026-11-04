/* Encoder velocity from pulse timing (time between edges, moving average). */
#ifndef MW_ENCODERVELOCITY_H
#define MW_ENCODERVELOCITY_H

#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#if !( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER) )

/* edgeMode: 1 = A rising (1x), 2 = A both edges (2x), 4 = A and B both edges (4x)
 * window:   moving-average length in edge intervals (1..MW_ENCVEL_MAX_WINDOW)
 * timeoutUs: no edge for this long -> velocity 0 and the average restarts
 * decay:    1 = while waiting for the next edge, bound the rate by the elapsed
 *           time (velocity falls off smoothly instead of holding)
 * decayTol: decay starts when elapsed > decayTol * average interval, then uses
 *           elapsed / decayTol (>= 1; 1 = original behaviour)
 * maxWindowUs: 0 = fixed window of N intervals; > 0 = use the newest intervals
 *           spanning at most this time, at least one full cycle, whole cycles
 * anchorTau: 0 = off; > 0 = correct the output so its integral follows the edge
 *           count with this time constant [s] (removes low-speed bias)
 * filtTau:  0 = off; > 0 = first-order low-pass on the output [s]
 * Returns 0 on success, -1 on failure (too many instances, bad pins). */
int MW_EncVel_Setup(uint8_T pinA, uint8_T pinB, uint8_T edgeMode, uint8_T window,
                    uint32_T timeoutUs, uint8_T decay, real32_T decayTol,
                    uint32_T maxWindowUs, real32_T anchorTau, real32_T filtTau,
                    uint8_T *index);

/* Signed edge rate in edges/second; *count = accumulated edge count. */
real32_T MW_EncVel_Read(uint8_T index, int32_T *count);

void MW_EncVel_Release(void);

#else
/* Rapid accelerator / MEX: stubs */
#define MW_EncVel_Setup(a, b, m, w, t, d, dt, mw, at, ft, i) (0)
#define MW_EncVel_Read(i, c) (0.0f)
#define MW_EncVel_Release() ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#define MW_ENCVEL_MAX_WINDOW    48
#define MW_ENCVEL_MAX_INSTANCES 4

#endif /* MW_ENCODERVELOCITY_H */
