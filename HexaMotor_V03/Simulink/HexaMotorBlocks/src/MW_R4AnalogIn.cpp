/* Analog input with selectable reference and resolution for Arduino Uno R4 (Renesas RA4M1).
 *
 * The reference is set once at setup with analogReference() (it closes and
 * reopens the ADC, so it isn't repeated every step). The model's setup code
 * runs after the support package's hardware init, so this setting wins over
 * the board's "Analog input reference voltage" option.
 *
 * analogReadResolution() is a global setting of the core, so it is set
 * before every read: other blocks in the model may use another resolution.
 * The ADC itself always converts at 14 bits; the core scales the result.
 */
#include "MW_R4AnalogIn.h"

#if !( defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER) )
#include "Arduino.h"

#if !defined(ARDUINO_ARCH_RENESAS_UNO) && !defined(ARDUINO_ARCH_RENESAS)
#error "MW_R4AnalogIn supports only Arduino Uno R4 / Nano R4 (Renesas) boards."
#endif

static int8_T r4AinRef = -1;   /* reference already set, -1 = none */

static uint8_t r4AinMode(uint8_T ref)
{
    switch (ref) {
        case 1:  return AR_EXTERNAL;
        /* IVREF, 1.43 V nominal (AR_INTERNAL_VOLTAGE in the core). The core's
         * AR_INTERNAL_1_5V/2_0V/2_5V modes read 0 on the Uno R4 Minima (tested). */
        case 2:  return AR_INTERNAL;
        default: return AR_DEFAULT;
    }
}

extern "C" int MW_R4AnalogIn_Setup(uint8_T pin, uint8_T ref)
{
    if (pin >= PINS_COUNT) {
        return -1;
    }
    if (r4AinRef < 0) {
        analogReference(r4AinMode(ref));
        r4AinRef = (int8_T)ref;
    } else if (r4AinRef != (int8_T)ref) {
        return -1;
    }
    return 0;
}

extern "C" uint16_T MW_R4AnalogIn_Read(uint8_T pin, uint8_T bits)
{
    analogReadResolution(bits);
    return (uint16_T)analogRead(pin);
}

#endif
