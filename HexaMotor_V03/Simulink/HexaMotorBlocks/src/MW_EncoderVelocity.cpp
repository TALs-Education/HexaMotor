/* Encoder velocity from pulse timing.
 *
 * Every counted edge is timestamped with micros(). The interval to the
 * previous edge goes into a ring buffer, and the velocity is the inverse of
 * the moving average of the last N intervals:
 *
 *     edges/s = dir * 1e6 * n / sum(dt_i)
 *
 * The average restarts (buffer cleared) when the direction changes or when
 * an interval exceeds the stall timeout, so turnaround and stall intervals
 * never pollute the average. A reversal is accepted only after two
 * consecutive reversed edges; a single opposite edge is treated as a glitch
 * and the interval around it is skipped.
 *
 * Reversal plausibility: the motor has to slow down before it can reverse,
 * so a reversal is only possible when the edge intervals are long. While the
 * last interval and the time since the last edge are both shorter than
 * MW_ENCVEL_REV_MIN_US, reversed edges are misreads (noise, or A/B read after
 * the other channel already changed): they are counted in the confirmed
 * direction and the interval around them is skipped, so a pair of misreads
 * cannot flip the direction (which showed up as a short negative dip).
 * The decay and stall checks also restart at such a misread edge (it shows
 * the motor is moving); at low speed they behave exactly as before.
 *
 * Time-bounded window (maxWindowUs > 0): instead of always averaging the
 * last N intervals, the newest intervals are used while their span stays
 * within maxWindowUs, but at least one full cycle of edgeMode intervals, and
 * the count is rounded down to whole cycles (4 in 4x mode) so the A/B phase
 * error cancels. Long average at high speed, short (low lag) at low speed.
 * maxWindowUs = 0 keeps the fixed window of N intervals.
 *
 * Decay tolerance: with decay on, the rate is bounded only when the time
 * since the last edge exceeds decayTol x the average interval, and then by
 * elapsed / decayTol (continuous). decayTol > 1 keeps uneven edge spacing
 * from triggering the bound while running; decayTol = 1 is the original
 * behaviour.
 *
 * Count anchor (anchorTau > 0): edge timing reports the speed of the motion
 * between edges, so with uneven (stick-slip) motion at low speed its time
 * average is biased. The output is corrected so that its integral follows
 * the edge count:  out = rate + err / anchorTau,  err = count - integral(out),
 * kept as a small running error so float precision holds for any run length.
 * Fast response of edge timing, zero long-term bias of counting.
 *
 * Output filter (filtTau > 0): first-order low-pass on the output,
 * y += (out - y) * dt / (filtTau + dt), with dt measured between reads. Reset to 0
 * on stall. A few ms smooths the edge-timing ripple with little lag.
 */
#include "MW_EncoderVelocity.h"

#if !( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER) )
#include "Arduino.h"

/* Shortest edge interval at which a direction reversal is possible [us]
 * (2 ms = 500 edges/s, about 3 rad/s at the HexaMotor output shaft in 4x) */
#ifndef MW_ENCVEL_REV_MIN_US
#define MW_ENCVEL_REV_MIN_US 2000U
#endif

typedef struct {
    volatile uint32_T buf[MW_ENCVEL_MAX_WINDOW];
    volatile uint32_T sum;          /* sum of the n intervals in buf [us] */
    volatile uint8_T  n;            /* number of valid intervals */
    volatile uint8_T  head;         /* next write position */
    volatile uint32_T lastEdgeUs;   /* last edge used for timing */
    volatile uint32_T lastAnyUs;    /* last edge of any kind (decay, stall) */
    volatile uint32_T lastDtUs;     /* last interval pushed to the average */
    volatile uint8_T  hasLast;
    volatile int8_T   dir;          /* confirmed direction, +1 / -1 */
    volatile uint8_T  revPending;   /* 1 = last edge was opposite to dir */
    volatile int32_T  count;
    uint8_T  window;
    uint32_T timeoutUs;
    uint8_T  decay;
    real32_T decayTol;      /* decay starts at elapsed > decayTol * average interval */
    uint32_T maxWindowUs;   /* 0 = fixed window of N intervals */
    real32_T anchorTau;     /* count anchor time constant [s], 0 = off */
    real32_T anchorErr;     /* count - integral of the output [edges] (stays small) */
    real32_T filtTau;       /* output low-pass time constant [s], 0 = off */
    real32_T filtOut;       /* output low-pass state [edges/s] */
    int32_T  lastCount;
    uint32_T lastReadUs;
    uint8_T  readInit;
    uint8_T  pinA;
    uint8_T  pinB;
    uint8_T  edgeMode;
} encVel_t;

static encVel_t encVel[MW_ENCVEL_MAX_INSTANCES];
static uint8_T encVelCount = 0;

static inline void encVelResetAverage(encVel_t *e)
{
    e->n = 0;
    e->head = 0;
    e->sum = 0;
}

static void encVelEdge(encVel_t *e, int8_T dir)
{
    uint32_T now = micros();
    if (!e->hasLast) {
        e->count += dir;
        e->dir = dir;
        e->revPending = 0;
        e->lastEdgeUs = now;
        e->lastAnyUs = now;
        e->lastDtUs = 0xFFFFFFFFU;
        e->hasLast = 1;
        return;
    }
    if (dir != e->dir) {
        uint8_T plausible = (e->n == 0) || (e->lastDtUs >= MW_ENCVEL_REV_MIN_US) ||
                            ((now - e->lastEdgeUs) >= MW_ENCVEL_REV_MIN_US);
        if (!plausible) {
            /* Too fast to reverse: a misread edge. Count it in the confirmed
             * direction, skip the interval around it, and keep the decay
             * clock running from it (the motor is moving). */
            e->count += e->dir;
            e->revPending = 1;
            e->lastAnyUs = now;
            return;
        }
        e->count += dir;
        if (!e->revPending) {
            /* Possible glitch: wait for a second reversed edge before
             * accepting the reversal; keep the average and last edge time */
            e->revPending = 1;
            return;
        }
        /* Two reversed edges in a row: real reversal, restart the average */
        e->revPending = 0;
        e->dir = dir;
        encVelResetAverage(e);
        e->lastEdgeUs = now;
        e->lastAnyUs = now;
        e->lastDtUs = 0xFFFFFFFFU;       /* new direction: no interval yet */
        return;
    }
    e->count += dir;
    if (e->revPending) {
        /* The single reversed edge was a glitch. Skip this interval
         * (it is split or stretched by the glitch) and resume. */
        e->revPending = 0;
        e->lastEdgeUs = now;
        e->lastAnyUs = now;
        return;
    }
    uint32_T dt = now - e->lastEdgeUs;   /* wrap-safe */
    e->lastDtUs = dt;
    if (dt > e->timeoutUs) {
        encVelResetAverage(e);           /* stall: restart the average */
    } else {
        if (e->n >= e->window) {
            e->sum -= e->buf[e->head];   /* drop oldest */
        } else {
            e->n++;
        }
        e->buf[e->head] = dt;
        e->sum += dt;
        e->head = (uint8_T)((e->head + 1) % e->window);
    }
    e->lastEdgeUs = now;
    e->lastAnyUs = now;
}

/* Same quadrature convention as MW_ArduinoEncoder.cpp */
static inline void encVelProcessA(encVel_t *e)
{
    int a = digitalRead(e->pinA);
    int b = digitalRead(e->pinB);
    int8_T dir = (a == HIGH) ? ((b == LOW) ? 1 : -1) : ((b == HIGH) ? 1 : -1);
    encVelEdge(e, dir);
}

static inline void encVelProcessB(encVel_t *e)
{
    int a = digitalRead(e->pinA);
    int b = digitalRead(e->pinB);
    int8_T dir = (b == HIGH) ? ((a == HIGH) ? 1 : -1) : ((a == LOW) ? 1 : -1);
    encVelEdge(e, dir);
}

static inline void encVelProcessARising(encVel_t *e)
{
    /* A just rose: B low -> forward */
    encVelEdge(e, (digitalRead(e->pinB) == LOW) ? 1 : -1);
}

static void encVelIsrA(encVel_t *e)
{
    if (e->edgeMode == 1) {
        encVelProcessARising(e);
    } else {
        encVelProcessA(e);
    }
}

static void EncVel_ISR0_A() { encVelIsrA(&encVel[0]); }
static void EncVel_ISR0_B() { encVelProcessB(&encVel[0]); }
static void EncVel_ISR1_A() { encVelIsrA(&encVel[1]); }
static void EncVel_ISR1_B() { encVelProcessB(&encVel[1]); }
static void EncVel_ISR2_A() { encVelIsrA(&encVel[2]); }
static void EncVel_ISR2_B() { encVelProcessB(&encVel[2]); }
static void EncVel_ISR3_A() { encVelIsrA(&encVel[3]); }
static void EncVel_ISR3_B() { encVelProcessB(&encVel[3]); }

typedef void (*encVelIsr_t)();
static const encVelIsr_t encVelIsrA_tbl[MW_ENCVEL_MAX_INSTANCES] = {EncVel_ISR0_A, EncVel_ISR1_A, EncVel_ISR2_A, EncVel_ISR3_A};
static const encVelIsr_t encVelIsrB_tbl[MW_ENCVEL_MAX_INSTANCES] = {EncVel_ISR0_B, EncVel_ISR1_B, EncVel_ISR2_B, EncVel_ISR3_B};

#if defined(ARDUINO_ARCH_RENESAS_UNO)
static int encVelIrqChannel(uint8_T pin)
{
    if (pin >= PINS_COUNT) return -1;
    auto cfg = getPinCfgs(pin, PIN_CFG_REQ_INTERRUPT);
    if (cfg[0] == 0) return -1;
    return GET_CHANNEL(cfg[0]);
}
#endif

extern "C" int MW_EncVel_Setup(uint8_T pinA, uint8_T pinB, uint8_T edgeMode, uint8_T window,
                               uint32_T timeoutUs, uint8_T decay, real32_T decayTol,
                               uint32_T maxWindowUs, real32_T anchorTau, real32_T filtTau,
                               uint8_T *index)
{
    if (encVelCount >= MW_ENCVEL_MAX_INSTANCES) return -1;
    if (window < 1) window = 1;
    if (window > MW_ENCVEL_MAX_WINDOW) window = MW_ENCVEL_MAX_WINDOW;
    if ((edgeMode != 1) && (edgeMode != 2) && (edgeMode != 4)) edgeMode = 4;
    if (decayTol < 1.0f) decayTol = 1.0f;

#if defined(ARDUINO_ARCH_RENESAS_UNO)
    /* Pins that get an interrupt need an IRQ; A and B need different IRQs */
    int chA = encVelIrqChannel(pinA);
    if (chA < 0) return -1;
    if (edgeMode == 4) {
        int chB = encVelIrqChannel(pinB);
        if ((chB < 0) || (chA == chB)) return -1;
    }
#endif

    encVel_t *e = &encVel[encVelCount];
    e->sum = 0;
    e->n = 0;
    e->head = 0;
    e->hasLast = 0;
    e->lastAnyUs = 0;
    e->lastDtUs = 0xFFFFFFFFU;
    e->dir = 1;
    e->revPending = 0;
    e->count = 0;
    e->window = window;
    e->timeoutUs = timeoutUs;
    e->decay = decay;
    e->decayTol = decayTol;
    e->maxWindowUs = maxWindowUs;
    e->anchorTau = (anchorTau > 0.0f) ? anchorTau : 0.0f;
    e->anchorErr = 0.0f;
    e->filtTau = (filtTau > 0.0f) ? filtTau : 0.0f;
    e->filtOut = 0.0f;
    e->lastCount = 0;
    e->lastReadUs = 0;
    e->readInit = 0;
    e->pinA = pinA;
    e->pinB = pinB;
    e->edgeMode = edgeMode;

    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    attachInterrupt(digitalPinToInterrupt(pinA), encVelIsrA_tbl[encVelCount],
                    (edgeMode == 1) ? RISING : CHANGE);
    if (edgeMode == 4) {
        attachInterrupt(digitalPinToInterrupt(pinB), encVelIsrB_tbl[encVelCount], CHANGE);
    }
    *index = encVelCount++;
    return 0;
}

extern "C" real32_T MW_EncVel_Read(uint8_T index, int32_T *count)
{
    if (index >= encVelCount) {
        *count = 0;
        return 0.0f;
    }
    encVel_t *e = &encVel[index];

    noInterrupts();
    uint32_T sum = e->sum;
    uint8_T  n = e->n;
    uint32_T lastEdgeUs = e->lastAnyUs;   /* any edge: misreads still mean motion */
    uint8_T  hasLast = e->hasLast;
    int8_T   dir = e->dir;
    *count = e->count;
    if ((e->maxWindowUs > 0) && (n > 0)) {
        /* Time-bounded window: newest intervals first, at least one full
         * cycle, span <= maxWindowUs, whole cycles only (at most N = window
         * iterations, a few us at 48 MHz) */
        uint8_T  m = e->edgeMode;
        uint8_T  w = e->window;
        uint32_T span = 0, spanBest = 0;
        uint8_T  k = 0, kBest = 0;
        for (uint8_T i = 0; i < n; i++) {
            uint32_T dti = e->buf[(uint8_T)((e->head + w - 1 - i) % w)];
            if ((k >= m) && (span + dti > e->maxWindowUs)) break;
            span += dti;
            k++;
            if ((k % m) == 0) {
                kBest = k;
                spanBest = span;
            }
        }
        if (kBest == 0) {                       /* less than one cycle available */
            kBest = k;
            spanBest = span;
        }
        n = kBest;
        sum = spanBest;
    }
    interrupts();

    uint32_T now = micros();
    real32_T rate = 0.0f;
    uint8_T  stalled = !hasLast || ((now - lastEdgeUs) > e->timeoutUs);
    if (hasLast && (n > 0)) {
        uint32_T elapsed = now - lastEdgeUs;
        if (elapsed <= e->timeoutUs) {           /* else stalled: rate 0 */
            real32_T avgDt = (real32_T)sum / (real32_T)n;
            if (e->decay && ((real32_T)elapsed > e->decayTol * avgDt)) {
                avgDt = (real32_T)elapsed / e->decayTol; /* slowing down: bound the rate, continuous */
            }
            rate = (real32_T)dir * 1.0e6f / avgDt;
        }
    }

    if ((e->anchorTau > 0.0f) || (e->filtTau > 0.0f)) {
        /* time and edges since the previous read */
        if (!e->readInit) {
            e->anchorErr = 0.0f;
            e->filtOut = 0.0f;
            e->lastCount = *count;
            e->lastReadUs = now;
            e->readInit = 1;
        }
        real32_T dt = (real32_T)(now - e->lastReadUs) * 1.0e-6f;
        e->lastReadUs = now;
        int32_T newEdges = *count - e->lastCount;
        e->lastCount = *count;
        if (stalled) {
            /* no edge for the stall timeout: the motor is stopped, speed is exactly 0 */
            e->anchorErr = 0.0f;
            e->filtOut = 0.0f;
            return 0.0f;
        }
        if (e->anchorTau > 0.0f) {
            /* count anchor: integral of the output follows the edge count */
            e->anchorErr += (real32_T)newEdges;
            rate += e->anchorErr / e->anchorTau;
            e->anchorErr -= dt * rate;                       /* what the output has integrated */
        }
        if (e->filtTau > 0.0f) {
            /* first-order output low-pass */
            e->filtOut += (rate - e->filtOut) * dt / (e->filtTau + dt);
            rate = e->filtOut;
        }
    }
    return rate;
}

extern "C" void MW_EncVel_Release(void)
{
    for (uint8_T i = 0; i < encVelCount; i++) {
        detachInterrupt(digitalPinToInterrupt(encVel[i].pinA));
        if (encVel[i].edgeMode == 4) {
            detachInterrupt(digitalPinToInterrupt(encVel[i].pinB));
        }
    }
    encVelCount = 0;
}

#endif
