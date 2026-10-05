function p = hexaMotorParams(verbose)
%HEXAMOTORPARAMS Physical parameters of the HexaMotor gearmotor for dcMotorControlSim_HexaMotor.slx.
%   p = hexaMotorParams() combines the motor datasheet with the first-order model identified
%   from the frequency response (HexaMotorModel.mat from HexaMotor_SysID_Demo, or the recorded
%   chirp FrequencyResponse.mat) into the parameters of the Simscape DC Motor, gear and load.
%
%   Datasheet (Pololu 25D, 12 V, 20.4:1, 48 CPR encoder):
%     stall current 0.9 A @ 12 V, no-load speed 280 RPM (gearbox output), stall torque 2.4 kg*cm (output)
%
%   Model:  G(s) = w_out/V = km/(tau*s + 1),  km = 1/(N*K),  tau = Ra*J/K^2   (viscous damping ~ 0)

if nargin < 1, verbose = true; end

% ---- datasheet ----
p.Vsupply   = 12;                       % [V]
p.Istall    = 0.9;                      % [A]  @ 12 V
p.N         = 20.4;                     % gear ratio (motor turns per output turn)
p.rpmNoLoad = 280;                      % [RPM] at the gearbox output, 12 V
p.Tstall    = 2.4*9.80665e-2;           % 2.4 kg*cm -> [N*m] at the gearbox output
p.encCPR    = 48;                       % quadrature counts per MOTOR revolution (encoder before the gearbox)
                                        % -> 48*20.4 = 979.62 counts per output revolution
p.encPPR    = p.encCPR/4;               % pulses per motor revolution on one channel (example's decoder only)

% ---- identified first-order model (output shaft, rad/s per V) ----
here = fileparts(mfilename("fullpath"));                     % demo folder, whatever the current folder is
if isfile(fullfile(here, "HexaMotorModel.mat"))
    s = load(fullfile(here, "HexaMotorModel.mat"), "HexaMotorModel");
    p.km = s.HexaMotorModel.km; p.tau = s.HexaMotorModel.tau;
    p.modelSource = "HexaMotorModel.mat (" + s.HexaMotorModel.source + ")";
else
    d = load(fullfile(here, "FrequencyResponse.mat"), "ScopeData"); d = d.ScopeData;
    T = tfest(iddata(d.signals.values(:,2), d.signals.values(:,1), median(diff(d.time))), 2, 0);
    den = T.Denominator/T.Denominator(1);
    p.tau = 1/den(2); p.km = den(3)*p.tau/10;                % recording made with Kp = 10
    p.modelSource = "FrequencyResponse.mat (closed-loop chirp, Kp = 10)";
end

% ---- derived motor parameters ----
p.Ra   = p.Vsupply/p.Istall;                                 % armature resistance [Ohm]
p.La   = 1e-3;                                               % armature inductance [H] (not in datasheet; L/R = 75 us, negligible)
p.K    = 1/(p.N*p.km);                                       % back-EMF = torque constant [V*s/rad] = [N*m/A]
p.Kdatasheet = p.Vsupply/(p.N*p.rpmNoLoad*2*pi/60);          % same constant from the no-load speed (check)
p.gearEff = min(p.Tstall/(p.N*p.K*p.Istall), 1);             % gearbox efficiency from the stall torque
p.Jtotal  = p.tau*p.K^2/p.Ra;                                % total effective inertia at the motor shaft [kg*m^2]

% ---- load: stainless-steel disk on the gearbox output ----
p.diskMass = 0.450;                                          % [kg]
p.diskDiam = 0.075;                                          % [m]
p.Jload    = 0.5*p.diskMass*(p.diskDiam/2)^2;                % solid disk, 1/2 m r^2 [kg*m^2] (= 3164 g*cm^2)

% ---- motor-side inertia (rotor + gear train) = what the identified tau leaves after the disk ----
% Calibration: in Simscape the H-bridge switch/diode resistances and the gear-efficiency model slow the
% open-loop response by ~6 % vs. the ideal formula tau = Ra*J/K^2; JCal makes the Simscape open-loop tau
% equal the identified tau (fitted on a 7 V step with the ideal speed sensor, see validateHexaMotorSim.m).
p.JCal    = 0.937;
p.JdiskAtMotor = p.Jload/(p.gearEff*p.N^2);                  % disk as seen from the motor (through the gear losses)
p.Jmotor  = p.Jtotal*p.JCal - p.JdiskAtMotor;                % rotor + gearbox inertia at the motor shaft [kg*m^2]
if p.Jmotor <= 0
    warning("Disk inertia alone exceeds the identified total; check the disk data or the identification.");
    p.Jmotor = 1e-8;
end
p.damping = 0;                                               % viscous damping [N*m/(rad/s)] (km already matches the no-load speed)

% ---- PI from HexaMotor_SysID_Demo (V per rad/s) converted to this model (duty per RPM) ----
p.KpV = 0.7767; p.KiV = 11.35;                               % 30 % OS design, 200 Hz
p.Kp_duty = p.KpV*(2*pi/60)/p.Vsupply;
p.Ki_duty = p.KiV*(2*pi/60)/p.Vsupply;
p.sampleTime = 1/200;

if verbose
    fprintf("HexaMotor parameters  (model: %s)\n", p.modelSource);
    fprintf("  identified km   = %.3f rad/s/V   tau = %.4f s\n", p.km, p.tau);
    fprintf("  datasheet  km   = %.3f rad/s/V   (280 RPM / 12 V)\n", 1/(p.N*p.Kdatasheet));
    fprintf("  Ra              = %.2f Ohm        (12 V / 0.9 A)\n", p.Ra);
    fprintf("  K = Ke = Kt     = %.5f V*s/rad   (datasheet no-load speed: %.5f, %+.1f %%)\n", p.K, p.Kdatasheet, 100*(p.K/p.Kdatasheet - 1));
    fprintf("  gear efficiency = %.2f            (stall torque %.3f N*m at the output)\n", p.gearEff, p.Tstall);
    fprintf("  disk load       = %.0f g*cm^2     (%.0f g, %.0f mm steel disk; %.1f g*cm^2 seen at the motor)\n", ...
        p.Jload*1e7, p.diskMass*1e3, p.diskDiam*1e3, p.JdiskAtMotor*1e7);
    fprintf("  J total (motor) = %.2f g*cm^2     -> rotor + gear train %.1f g*cm^2 (from the identified tau)\n", ...
        p.Jtotal*1e7, p.Jmotor*1e7);
    fprintf("  predicted tau without the disk = %.3f s\n", p.tau*p.Jmotor/(p.Jmotor + p.JdiskAtMotor));
    fprintf("  PI (duty/RPM)   : Kp = %.5f, Ki = %.5f\n", p.Kp_duty, p.Ki_duty);
end
end
