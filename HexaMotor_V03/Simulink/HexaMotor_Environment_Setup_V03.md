# HexaMotor V03: setting up the environment

Tel Aviv University, Faculty of Engineering. HexaMotor lab platform (velocity and
position control of a DC motor with an Arduino Uno R4 Minima).

This document covers what to install on a lab or student PC, how to install the
HexaMotor models and custom blocks, and which models make up the platform.

**The V03 models don't need the R4 Minima patch.** They use custom Uno R4 blocks
(in `HexaMotorBlocks`) for the motor PWM, the motor current input and the encoder,
and run on an unmodified MATLAB installation.

## 1. Install the models custom blocks

1. Copy the **whole HexaMotor folder**, including `HexaMotorBlocks`: the models and
   the block files travel together.
2. Run `install_HexaMotor_V03` once (section 4). It puts the HexaMotor folder and
   `HexaMotorBlocks` on the MATLAB path permanently.

The models also add `HexaMotorBlocks` to the path when they open (in the model's
*PreLoadFcn* callback). So a copied folder works even without the install step, as
long as `HexaMotorBlocks` sits next to the models. If it can't be found, the model
shows a warning that says to run `install_HexaMotor_V03`.

The MathWorks products and support packages (section 2) must be installed on each
machine. They're not part of the folder.

## 2. Software requirements

| Product | Needed for |
|---|---|
| MATLAB **R2026b** | Tested release. The models and blocks use no release-specific internals, so they should work in later releases (see section 6). |
| Simulink | All models. |
| Simulink Support Package for Arduino Hardware | Building and running the models on the board (*Monitor & Tune*). It installs the Arduino Renesas core (`renesas_uno` 1.5.1 in R2026b). |
| Simscape Electrical | The *Filtered Derivative* and *Lead-Lag* blocks (library `eeGeneralControl`) used in the models. |
| Control System Toolbox | Lab analysis: `tf`, `step`, `lsim`, `bode`, `LeadLag.m`. |
| System Identification Toolbox | Lab analysis of the frequency response: `idfrd`, `tfest` (Velocity Control manual, section 3.4). |
| MATLAB Support Package for Arduino Hardware | Optional. Not used by the Simulink models. |

The dependency analysis also lists Embedded Coder, because the models build with
`ert.tlc`. The Arduino support package builds and runs the models with the products
above; if a build asks for an Embedded Coder license on a new machine, install it.

Install the support package from *Home > Add-Ons > Get Hardware Support Packages*,
then follow its hardware setup (it installs the Arduino Uno R4 core and the USB
driver). **No patch of the support package is needed.**

## 3. Hardware

| Part | Details |
|---|---|
| Controller | Arduino Uno R4 Minima (Renesas RA4M1, 48 MHz Arm Cortex-M4), USB to the PC. |
| Motor driver | Pololu Dual TB9051FTG Motor Driver Shield. Motor on channel M2. |
| Motor | Pololu 25D 12 V gearmotor, 20.4:1 gearbox, 12-line encoder on the motor shaft (48 counts per motor turn in 4x, 979.2 per output turn). |
| Power | 12 V supply for the motor driver. |

| Signal | Pin | Block in the models |
|---|---|---|
| Motor PWM (M2) | D10 | PWM (Uno R4), 20 kHz, GPT3 |
| Motor direction (M2) | D8 | Digital Output (stock) |
| Motor current (M2 current sense, 500 mV/A) | A1 | Analog Input (Uno R4), 14 bits, default 5 V reference |
| Encoder A / B | D2 / D3 | HexaMotor Encoder (Encoder Velocity) |
| Sample-rate toggle | D13 | Digital Output (stock), in *System SampleRate Test* |

## 4. Installing on a new machine

1. Install MATLAB and the products in section 2, including the Simulink Support
   Package for Arduino Hardware and its hardware setup.
2. Copy the HexaMotor `Simulink` folder (from the GitHub repository,
   `HexaMotor_V03/Simulink`) to the machine, for example to
   `Documents\MATLAB\HexaMotor`. Students: copy it into a folder with your name.
3. In MATLAB, `cd` to the folder and run:

   ```matlab
   install_HexaMotor_V03
   ```

   It:
   - checks for Simulink, Simscape Electrical and the Arduino support package;
   - removes old copies of the blocks from the path (for example
     `R4MinimaPatch\blocks` from an earlier patch install);
   - adds the HexaMotor folder and `HexaMotorBlocks` to the MATLAB path and saves
     it. Without administrator rights it writes an `addpath` line into `startup.m`
     in the user folder instead;
   - checks that all block classes are found in `HexaMotorBlocks` and lists the
     models.

   Running it again is safe. To undo it: `install_HexaMotor_V03('uninstall')`.
4. Connect the board by USB and the 12 V supply. Open a model and press
   *Hardware > Monitor & Tune*.

After a copy to another folder, run `install_HexaMotor_V03` again from the new
folder, so the path points to the new location.

### Models (open these)

| Model | Manual | Purpose |
|---|---|---|
| `HexaMotor_MotorTest_V03.slx` | Velocity Control 3.1, 3.2 | Open-loop voltage (constant / slider), motor speed, Coulomb friction. |
| `HexaMotor_System_Identification_V03.slx` | Velocity Control 3.3, 3.4 | Step and sine voltage inputs for system identification, with friction pre-amp. |
| `HexaMotor_Velocity_Control_V03.slx` | Velocity Control 3.5 | PI(D) velocity loop, square-wave set point; tick-timing or tick-count feedback. |
| `HexaMotor_Position_Control_V03.slx` | Position Control | PID position loop, square-wave or chirp set point. |
| `HexaMotor_LeadLag_Control_V03.slx` | Position Control | Position loop with a lead-lag compensator. |

### Custom Uno R4 blocks (HexaMotorBlocks)

Library Browser: *Arduino R4 Minima Extras* (`HexaMotorBlocks/R4MinimaBlocks.slx`).

| Block | Class | Driver |
|---|---|---|
| Encoder Velocity (HexaMotor Encoder) | `EncoderVelocity.m` | `src/MW_EncoderVelocity.cpp` |
| Encoder (Uno R4) | `R4Encoder.m` | `src/MW_R4Encoder.cpp` |
| PWM (Uno R4, high resolution) | `R4PWM.m` | `src/MW_R4PWM.cpp` |
| Analog Input (Uno R4) | `R4AnalogInput.m` | `src/MW_R4AnalogIn.cpp` |
| (shared base class) | `R4BlockBase.m` | - |

Headers are in `include/`. `slblocks.m` registers the library;
`createR4BlocksLibrary.m` rebuilds it. Block parameters, limits and test results are described
in `HexaMotorBlocks/README.md`. In short:

- **PWM (Uno R4):** any GPT pin, 1 Hz to 1 MHz, fixed or set by an input at run
  time; duty resolution = 48 MHz / prescaler / frequency. One pin per GPT channel.
- **Analog Input (Uno R4):** A0-A5, 10/12/14 bits, reference Default (AVCC 5 V),
  External (AREF pin) or Internal 1.43 V; output in counts or volts. One reference
  for the whole board.
- **Encoder (Uno R4):** quadrature count (4 per line), optional pull-ups and reset
  input. Pins A/B on different IRQ channels.
- **Encoder Velocity:** velocity from the time between encoder edges (the *HexaMotor
  Encoder* block in the models); see the manuals' appendix.

## 6. Future MATLAB releases

The models and the custom blocks change no MathWorks files and use only documented
interfaces (`matlab.System`, `coder.ExternalDependency`, `createSampleTime`). In a
new release:

- Install the Simulink Support Package for Arduino Hardware for that release.
- Run `install_HexaMotor_V03`, then `HardwareTests/hwTestV03({'V03'})` with the
  board connected (section 7).
- If a build fails in a block driver, check the Arduino Renesas core installed by the
  new support package: the drivers use `PwmOut`, `analogReference`,
  `analogReadResolution`, `attachInterrupt`, `micros()` and the `AR_*` reference
  names.

One block still uses an undocumented MathWorks class: Encoder Velocity derives from
`matlabshared.svd.BlockSampleTime`. If a new release removes it, change the block to
derive from `R4BlockBase` like the other blocks.


## 7. Troubleshooting

| Problem | Fix |
|---|---|
| *Unable to find System object 'R4PWM'* (or `EncoderVelocity`) | `HexaMotorBlocks` isn't on the path or isn't next to the models. Run `install_HexaMotor_V03` from the HexaMotor folder. |
| Build fails with *No rule to make target '...\MW_EncoderVelocity.cpp'* (or another `MW_*.cpp`) with an old folder path | The Simulink build cache still has the folder from before the HexaMotor folder was moved or copied. Close the models, delete the `slprj` folder (and the `*.slxc` files) in the current MATLAB folder, and build again. |
| Build fails in `MW_ArduinoHWInit.cpp` with *user-defined literal in preprocessor expression* (`_RTT_ANALOG_REF_`) | The model was saved with the R4 Minima patch installed, which stores the board's *Analog input reference voltage* as text. In *Hardware Settings > Analog input reference voltage*, select *Default* again and save (the V03 models are already fixed). |
| Two copies of a block class (`which -all EncoderVelocity` shows more than one) | Run `install_HexaMotor_V03`; it removes old copies from the path. |
| *A model with the same name is already loaded* | Close the other copy of the model (for example a model from another student folder) before opening this one. |
| Encoder count stays 0 | Encoder pins must be interrupt-capable and on different IRQ channels (D2/D3 are). Don't use the same pins in two encoder blocks. |
| A PWM pin stays off | Only one PWM pin per GPT timer channel (D10 is on GPT3 with D13; D13 as a digital output is fine). |
| Build fails with a COM port error | Check the USB cable and the port in *Hardware Settings > Host-board connection*. Close the Arduino IDE serial monitor. |
| The motor doesn't move | Check the 12 V supply and that the shield is on the board. Run `HexaMotor_MotorTest_V03` with a few volts. |
