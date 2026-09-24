HexaMotor - MathWorks Demo 2026 (MATLAB R2026a)
===============================================

Open this folder as the MATLAB current folder, then run the Live Scripts in this order:

1. HexaMotor_SysID_Demo.mlx       Motor equations, closed-loop 2nd-order identification (chirp sweep),
                                  velocity step Kp=1/Ki=0, PI design for 30 % overshoot at 200 Hz.
                                  Saves the identified model as HexaMotorModel (workspace + HexaMotorModel.mat).
2. HexaMotor_LeadLag_Demo.mlx     Advanced: PD + lead compensator (PM -> 60 deg) on the position loop,
                                  step and frequency responses, Bode shift. Uses HexaMotorModel.
3. HexaMotor_StepVsFreq_Demo.mlx  Sub-demo: open-loop step (bump test) vs. frequency-response identification,
                                  compared with the closed-loop model from demo 1.

Run mode (dropdown at the top of each script):
  Virtual motor / Recorded sweep - no hardware needed (built-in motor emulator; FrequencyResponse.mat is a real recording)
  Hardware                       - builds and runs the Simulink models on the HexaMotor (Monitor & Tune / external mode);
                                   the model and its Scope open and stream live during each run.

Simulink models:  HexaMotor_Position_Control_V02, HexaMotor_Velocity_Control_V02,
                  HexaMotor_LeadLag_Control_V02, HexaMotor_System_Identification_V02
Sample rate:      200 Hz (the scripts set the workspace variable sampleRate = 0.005)
Manuals:          Manuals/  (Velocity Control V02, Position Control V02)
Graphical lead/lag charts from the manual: LeadLag.m

Editing: change the plain-text sources in LiveScriptSources/ and re-save them as .mlx
(in the Live Editor: open the .m file and use Save As > .mlx). Keep only the .mlx files in this
folder: when a .m and a .mlx have the same name in one folder, MATLAB runs the .mlx.
