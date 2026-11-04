/* Quadrature encoder counter for Arduino Uno R4 (Renesas RA4M1).
 *
 * Same method as the Renesas driver in the R4 Minima patch
 * (MW_ArduinoEncoder.cpp), as a standalone block that doesn't replace any
 * MathWorks file: an interrupt on CHANGE of A and of B, and the direction
 * from the levels of A and B in the interrupt (4 counts per line).
 * The count is read and reset with interrupts briefly off.
 *
 * A and B need different IRQ channels: on the RA4M1 two pins on the same
 * channel share one interrupt, so setup fails for such a pair.
 */
#include "MW_R4Encoder.h"

#if !( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER) )
#include "Arduino.h"

#if !defined(ARDUINO_ARCH_RENESAS_UNO) && !defined(ARDUINO_ARCH_RENESAS)
#error "MW_R4Encoder supports only Arduino Uno R4 / Nano R4 (Renesas) boards."
#endif

typedef struct {
    volatile int32_T count;
    uint8_T pinA;
    uint8_T pinB;
} r4Encoder;

static r4Encoder r4Enc[MW_R4ENC_MAX_INSTANCES];
static uint8_T r4EncCount = 0;

static int r4EncIrqChannel(uint8_T pin)
{
    if (pin >= PINS_COUNT) {
        return -1;
    }
    auto cfg = getPinCfgs(pin, PIN_CFG_REQ_INTERRUPT);
    if (cfg[0] == 0) {
        return -1;
    }
    return GET_CHANNEL(cfg[0]);
}

/* Edge on A: A == B after the edge means one direction, A != B the other */
static inline void r4EncEdgeA(r4Encoder *e)
{
    int a = digitalRead(e->pinA);
    int b = digitalRead(e->pinB);
    e->count += (a != b) ? 1 : -1;
}

/* Edge on B: the opposite relation */
static inline void r4EncEdgeB(r4Encoder *e)
{
    int a = digitalRead(e->pinA);
    int b = digitalRead(e->pinB);
    e->count += (a == b) ? 1 : -1;
}

static void r4EncIsrA0() { r4EncEdgeA(&r4Enc[0]); }
static void r4EncIsrB0() { r4EncEdgeB(&r4Enc[0]); }
static void r4EncIsrA1() { r4EncEdgeA(&r4Enc[1]); }
static void r4EncIsrB1() { r4EncEdgeB(&r4Enc[1]); }
static void r4EncIsrA2() { r4EncEdgeA(&r4Enc[2]); }
static void r4EncIsrB2() { r4EncEdgeB(&r4Enc[2]); }
static void r4EncIsrA3() { r4EncEdgeA(&r4Enc[3]); }
static void r4EncIsrB3() { r4EncEdgeB(&r4Enc[3]); }

typedef void (*r4EncIsr_t)();
static const r4EncIsr_t r4EncIsrA[MW_R4ENC_MAX_INSTANCES] =
    {r4EncIsrA0, r4EncIsrA1, r4EncIsrA2, r4EncIsrA3};
static const r4EncIsr_t r4EncIsrB[MW_R4ENC_MAX_INSTANCES] =
    {r4EncIsrB0, r4EncIsrB1, r4EncIsrB2, r4EncIsrB3};

extern "C" int MW_R4Enc_Setup(uint8_T pinA, uint8_T pinB, uint8_T pullup, uint8_T *index)
{
    *index = 255;
    if (r4EncCount >= MW_R4ENC_MAX_INSTANCES) {
        return -1;
    }
    int chA = r4EncIrqChannel(pinA);
    int chB = r4EncIrqChannel(pinB);
    if (chA < 0 || chB < 0 || chA == chB) {
        return -1;
    }

    r4Encoder *e = &r4Enc[r4EncCount];
    e->count = 0;
    e->pinA = pinA;
    e->pinB = pinB;
    pinMode(pinA, pullup ? INPUT_PULLUP : INPUT);
    pinMode(pinB, pullup ? INPUT_PULLUP : INPUT);
    attachInterrupt(digitalPinToInterrupt(pinA), r4EncIsrA[r4EncCount], CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), r4EncIsrB[r4EncCount], CHANGE);
    *index = r4EncCount++;
    return 0;
}

extern "C" int32_T MW_R4Enc_Read(uint8_T index)
{
    if (index >= r4EncCount) {
        return 0;
    }
    noInterrupts();
    int32_T c = r4Enc[index].count;
    interrupts();
    return c;
}

extern "C" void MW_R4Enc_Reset(uint8_T index)
{
    if (index >= r4EncCount) {
        return;
    }
    noInterrupts();
    r4Enc[index].count = 0;
    interrupts();
}

extern "C" void MW_R4Enc_Release(void)
{
    for (uint8_T i = 0; i < r4EncCount; i++) {
        detachInterrupt(digitalPinToInterrupt(r4Enc[i].pinA));
        detachInterrupt(digitalPinToInterrupt(r4Enc[i].pinB));
    }
    r4EncCount = 0;
}

#endif
