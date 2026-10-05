% Start-up script for dcMotorControlSim_HexaMotor.slx (PreLoadFcn).
% Motor, gear, load and controller parameters of the HexaMotor setup: see hexaMotorParams.m.
hexaMotor = hexaMotorParams();
sampleTime = hexaMotor.sampleTime;   % 200 Hz, same as the HexaMotor controller
