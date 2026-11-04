# HexaMotor custom Uno R4 blocks

Simulink blocks for the Arduino Uno R4 Minima (Renesas RA4M1) used by the HexaMotor
V03 models. They're MATLAB System objects with C++ drivers and run on a standard
MATLAB installation with the Simulink Support Package for Arduino Hardware; **no
patch of the support package is needed**.

| Block | Class | Driver |
|---|---|---|
| Encoder Velocity (*HexaMotor Encoder*) | `EncoderVelocity.m` | `src/MW_EncoderVelocity.cpp` |
| Encoder (Uno R4) | `R4Encoder.m` | `src/MW_R4Encoder.cpp` |
| PWM (Uno R4, high resolution) | `R4PWM.m` | `src/MW_R4PWM.cpp` |
| Analog Input (Uno R4) | `R4AnalogInput.m` | `src/MW_R4AnalogIn.cpp` |

A Simulink model stores only the class name and the parameters of these blocks, so
this folder must be on the MATLAB path: run `install_HexaMotor_V03` in the HexaMotor
folder once (the HexaMotor models also add this folder when they open). Library
Browser: *Arduino R4 Minima Extras* (`R4MinimaBlocks.slx`).

## Encoder Velocity block

The *HexaMotor Encoder* block in the HexaMotor models.

**Where:** Library Browser, *Arduino R4 Minima Extras*, then *Encoder Velocity*. You
can also open `R4MinimaBlocks.slx` directly.

**How it works:** every counted edge triggers an interrupt that records the time with
`micros()` and works out the direction from the A/B levels. The interval to the
previous edge goes into a ring buffer, and the output is

```
velocity = dir * 1e6 * n / sum(dt_i)      [edges/s]   (last n <= N intervals)
```

which is then converted to the selected units. The average restarts on a direction
change or when an interval is longer than the stall timeout. If no edge arrives
within the timeout, velocity is 0.

A reversal is accepted only after two consecutive edges in the new direction. A
single opposite edge is treated as a glitch (noise, or a misread A/B state): the
average is kept and the interval around it is skipped. Without this, one bad edge
cleared the average twice and the output read 0 for 1–2 ms at full speed. Real
reversals are reported one edge later.

### Low speed: edges slower than the sample rate

At 1 kHz with the HexaMotor (12 lines, 4x, 20.4:1) one edge per sample is
6.4 rad/s at the output, so most of the speed range has fewer edges than samples.
Three optional settings handle this. Their defaults reproduce the original block
exactly, so existing models (V04) are unchanged.

- **Maximum window time** (time-bounded window). Instead of always averaging the
  last N intervals, the block uses the newest intervals while they span at most this
  time. It always uses at least one full quadrature cycle and rounds down to whole
  cycles (4 edges in 4x mode), so the A/B phase error cancels. The result is a long
  average at high speed and low lag at low speed.
- **Decay tolerance**. With decay on, the rate is bounded by the time since the last
  edge only once that time exceeds *tolerance* × the average interval, and then by
  *elapsed / tolerance*, which is continuous. With uneven edge spacing, a tolerance of 1
  (the original) triggers the bound during normal running; 1.5 avoids that.
- **Count anchor time constant**. Edge timing reports the speed of the motion
  *between* edges. With stick-slip motion at low speed, its time average is biased:
  it holds the speed of the last burst while the motor pauses. The anchor corrects
  the output so that its integral follows the edge count, using
  `out = rate + err/tau` with `err = count - integral(out)`. The error is reset to 0
  when the stall timeout confirms a stop. The block keeps the fast response of edge
  timing and gains the zero long-term bias of counting.

**Recommended at 1 kHz** (as in `HexaMotor_Velocity_Control_V03`, block named
*HexaMotor Encoder*): window N = 48, maximum window time 10 ms, decay on, decay
tolerance 1.5, count anchor 0.1 s, output filter 2 ms, stall timeout 0.1 s, ticks
output on top. Speeds below one edge per stall timeout (about 0.06 rad/s at the
output) read 0.

**Rising from standstill:** the bump of about 1 rad/s during the first 20 ms of a
full-voltage start is real motor-shaft motion. The motor spins through the gearbox
backlash, then picks up the load, and the truth from the encoder angle shows the
same peak. The single-sample spikes on top of it (±0.4 rad/s) come from edge timing
and are removed by the 2 ms output filter.


**Hardware results** (Uno R4 Minima, 1 kHz, same PI Kp 0.6 / Ki 10, low-speed
staircase; truth = slope of the encoder angle). V03 = tick-count velocity, V04 and V05 =
this block with the original and the low-speed settings.

| True speed for command (rad/s) | 0.5 | 1 | 2 | 5 | −1 | Stop detected after the true stop |
|---|---|---|---|---|---|---|
| V03 counts + 0.05 s filter | 0.501 | 1.001 | 2.000 | 4.996 | −1.000 | (lags; motor overshoots to 6.5 rad/s and −0.8 at stop) |
| V04 Encoder Velocity N = 4 | **0.000** | 0.572 | 2.008 | 5.082 | −0.550 | holds the last burst speed |
| V05 low-speed settings | 0.502 | 1.009 | 2.016 | 5.048 | −1.008 | 13 ms |

With V04 the motor doesn't move at a 0.5 rad/s command, because the held reading
tells the PI it already does. At 5 rad/s, V05 measurement noise is 0.10 rad/s,
against 0.34 rad/s for V04.

| Parameter | Description |
|---|---|
| Encoder pin A / B | Interrupt-capable pins. On R4 in 4x mode, A and B must be on different IRQ channels (for example 2 and 3). |
| Edges used for timing | *A rising (1x)*: only A needs an interrupt, and intervals are unaffected by uneven A/B phase. *A both edges (2x)*. *A and B both edges (4x)*: best resolution; use a window that is a multiple of 4. |
| Moving average window | 1 to 48 intervals. 48 = one motor revolution for a 12-line encoder in 4x mode, which also averages out uneven line spacing. |
| Pulses per revolution | Encoder lines per revolution, used for Rev/s, RPM and rad/s. |
| Output units | Edges/s, Rev/s, RPM or rad/s. |
| Stall timeout | 0.1 ms to 10 s. Output is 0 after this long without an edge. |
| Maximum window time | 0 (default) = fixed window of N intervals. > 0 = time-bounded window, at least one full quadrature cycle, whole cycles only (see *Low speed*). |
| Decay velocity while waiting | If on and the time since the last edge exceeds *decay tolerance* × the average interval, uses *elapsed / tolerance* instead, so velocity falls smoothly as the motor slows rather than holding until the timeout. Off by default. |
| Decay tolerance | 1 (default, original behaviour) to 10. 1.5 is recommended with 4x decoding. |
| Count anchor time constant | 0 (default) = off. > 0 = output integral follows the edge count (removes the low-speed bias). 0.1–0.2 s is recommended. |
| Output filter time constant | 0 (default) = off. > 0 = first-order low-pass on the velocity output. 2 ms halves the edge-timing ripple on the HexaMotor and adds about 2.5 ms delay. |
| Output edge count (ticks) | Adds an int32 `ticks` output. |
| Ticks output on top | Off (default): outputs are velocity, then ticks (original order). On: ticks, then velocity, matching the stock Rotary Encoder block. |
| Sample time | Block sample time. |

**Outputs**

| Port label | Type | Meaning |
|---|---|---|
| `Motor_Rad/s` (or `Motor_RPM`, `Motor_Rev/s`, `Edges/s`) | single | Velocity of the shaft the encoder is mounted on, in the selected units. On the HexaMotor that's the motor shaft: divide by 20.4 for the output shaft. |
| `ticks` | int32 | Accumulated quadrature edge count: 4 per encoder line in 4x mode. HexaMotor: 48 per motor turn, 979.2 per output turn. |

| File | Purpose |
|---|---|
| `EncoderVelocity.m` | System object: parameters, validation, unit scaling, code generation hooks. |
| `src/MW_EncoderVelocity.cpp`, `include/MW_EncoderVelocity.h` | Interrupt and moving-average driver (up to 4 instances). |
| `createR4BlocksLibrary.m` | Builds `R4MinimaBlocks.slx` with all four blocks (`install_HexaMotor_V03` runs it if the library is missing). |
| `slblocks.m` | Registers the library in the Library Browser. |

**Status:** code generation and compilation for the Uno R4 Minima pass in R2026b.
Tested on hardware (Uno R4 Minima, HexaMotor, External mode at 1 kHz) with the
original settings (V04) and the low-speed settings (V05).

**Limitations:**
- Outputs are 0 in normal simulation; the block measures only in generated code
  (deployed or External mode).
- Resolution is limited by `micros()` (a few µs). At very high edge rates,
  counting pulses per sample is more accurate.
- The block isn't registered with the support package's pin-conflict checker, so
  don't use the same pins in this block and the stock Encoder block.
- Pins are configured as plain `INPUT`. Open-collector encoders need external
  pull-up resistors.

## Custom Uno R4 blocks (no patch needed)

Three more blocks, in the same library as Encoder Velocity. They replace the stock
Arduino Encoder, PWM and Analog Input blocks on the Uno R4 (which need a patch of the
support package to work on the RA4M1) without changing any MathWorks file. They use only documented interfaces
(`matlab.System`, `coder.ExternalDependency`, `createSampleTime`) and the Arduino
Renesas core, so they don't depend on the support package release.

**Setup:** run `install_HexaMotor_V03` in the HexaMotor folder. The blocks then appear in the Library Browser under
*Arduino R4 Minima Extras*. In a new release, check that the Renesas core still
provides `PwmOut`, `analogReference`, `attachInterrupt` and the `AR_*` names
(core 1.5.1 in R2026b).

**Model rule:** for each function, use either these blocks or the stock blocks,
not both. The ADC reference, `analogReadResolution` and the timers are shared by
the whole board, and the custom blocks aren't registered with the support
package's pin-conflict checker.

All three blocks output 0 in normal simulation and work only in generated code
(deployed or External mode). Each has a *Sample time* parameter (-1 = inherited).

### Encoder (Uno R4)

Counts every edge of A and B with interrupts (4 counts per encoder line, up to 4
encoders).

| Parameter | Description |
|---|---|
| Encoder pin A / B | Interrupt-capable pins on **different IRQ channels** (for example 2 and 3). Otherwise setup fails and the count stays 0. |
| Enable internal pull-ups | Configures A and B as `INPUT_PULLUP` (for open-collector encoders). |
| Reverse count direction | Negates the count. |
| Reset input | Adds a `reset` input: the count is set to 0 while the input is nonzero. |

Output: `count` (int32). Don't use the same pins in Encoder Velocity or the stock
Encoder block.

### PWM (Uno R4, high resolution)

Drives a GPT timer output with the duty in raw timer counts, so the resolution is
the full timer period: counts = 48 MHz / prescaler / frequency.

| Frequency | Counts per period | Resolution |
|---|---|---|
| 20 kHz | 2400 | 11.2 bits |
| 5 kHz | 9600 | 13.2 bits |
| 1 kHz | 48000 | 15.6 bits |

The stock block uses 8 bits (255 steps).

| Parameter | Description |
|---|---|
| Pin | A pin with a GPT output (see the channel table below). |
| Duty cycle input range | `0 to 1` or `0 to 100 (%)`. The input is clamped. |
| Frequency source | `Parameter` (fixed) or `Input port`: adds a `freq` input (Hz) that changes the frequency at run time. The new period starts at the end of the current one, so there's no glitch, and the duty fraction is kept. |
| Frequency (Hz) | Fixed frequency, or the start frequency with an input port. 1 Hz to 1 MHz. |
| Minimum frequency (Hz) | Input port only. Sets the prescaler, so the run-time frequency is clamped to at least this. A lower minimum gives coarser duty steps at high frequencies on the 16-bit channels. |
| Output timer counts per period | Adds a `counts` output (uint32): the current duty resolution. |

Each GPT channel has one counter, so **only one pin per channel** can be used. A
second pin on the same channel fails setup and stays off. On the Uno R4 Minima:

| Channel | Pins | Counter |
|---|---|---|
| GPT0 | D6, D7 | 32-bit |
| GPT1 | D2, D3, D11, D12 | 32-bit |
| GPT2 | D4, D5 | 16-bit |
| GPT3 | D10, D13 | 16-bit |
| GPT4 | D0, D1 | 16-bit |
| GPT5 | A4, A5 | 16-bit |
| GPT7 | D8, D9 | 16-bit |

AGT pins are refused, because the core uses an AGT timer for `millis()` and
`micros()`. The Simulink scheduler uses SysTick, so it doesn't conflict with
these timers.

### Analog Input (Uno R4)

| Parameter | Description |
|---|---|
| Pin | A0 to A5. |
| Reference | `Default (AVCC, 5 V)`, `External (AREF pin)` or `Internal 1.43 V` (IVREF; measured 1.43–1.46 V on a Minima). The core's 1.5/2.0/2.5 V modes aren't offered: on the Minima they read 0. |
| Resolution | 10, 12 or 14 bits. The ADC always converts at 14 bits; the core scales the result. |
| Output | `Counts` (uint16, full scale 2^bits − 1) or `Voltage` (single). |
| Reference voltage (V) | For `Voltage` with Default or External: the AVCC or AREF voltage used for scaling. |

The reference is set once at startup, after the support package's hardware init,
so it overrides the board's *Analog input reference voltage* setting. It's one
setting for the whole board: all Analog Input blocks in a model must use the same
reference (the first block's setting is kept). The resolution is set before every
read, so blocks can use different resolutions.

### Files

| File | Purpose |
|---|---|
| `R4BlockBase.m` | Shared base class: sample time, icon, build info. |
| `R4Encoder.m`, `src/MW_R4Encoder.cpp`, `include/MW_R4Encoder.h` | Encoder block and driver. |
| `R4PWM.m`, `src/MW_R4PWM.cpp`, `include/MW_R4PWM.h` | PWM block and driver. |
| `R4AnalogInput.m`, `src/MW_R4AnalogIn.cpp`, `include/MW_R4AnalogIn.h` | Analog Input block and driver. |

**Status:** tested on hardware (Uno R4 Minima, HexaMotor, R2026b, 2026-11-04). The
HexaMotor V03 models use these blocks and match the earlier patch-based models
(`../Patched Matlab`, stock blocks with the patch) within measurement noise; see
`../HexaMotor_Environment_Setup_V03.pdf` and `../Results/HardwareTests_V03_vs_Patched`. Tested: PWM duty (same motor speed as the
stock block), run-time frequency change (2400 counts at 20 kHz, 9600 at 5 kHz),
Encoder count, Analog Input with the Default and the internal reference (IVREF
checked against the board's AVCC divider on pin 20). Not tested: External (AREF)
with a known voltage on the AREF pin (nothing is connected to AREF on the HexaMotor).
The drivers compile with no warnings under `-Wall -Wextra`.

