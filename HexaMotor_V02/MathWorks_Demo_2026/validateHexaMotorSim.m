%% validateHexaMotorSim
% Compares the adapted Simscape model dcMotorControlSim_HexaMotor.slx with the first-order model
% identified from the HexaMotor frequency response, G(s) = km/(tau*s + 1):
%   A) open loop, constant 7 V (as the bump test in the Velocity Control manual)
%   B) closed loop with the PI from HexaMotor_SysID_Demo, 100 RPM step command

mdl = "dcMotorControlSim_HexaMotor";
load_system(mdl);
p = hexaMotorParams(false);
s = tf('s');
G = p.km/(p.tau*s + 1);                                   % rad/s per V, gearbox output
rpm = 60/(2*pi);

% log the Motor System output (RPM vector) and the controller output (duty)
ms = mdl + "/Motor System";
phM = get_param(ms, "PortHandles");
set_param(phM.Outport(1), DataLogging="on", DataLoggingNameMode="Custom", DataLoggingName="rpmOut");
phC = get_param(mdl + "/Manual Switch1", "PortHandles");
set_param(phC.Outport(1), DataLogging="on", DataLoggingNameMode="Custom", DataLoggingName="duty");
phD = get_param(mdl + "/desired CMD", "PortHandles");
set_param(phD.Outport(1), DataLogging="on", DataLoggingNameMode="Custom", DataLoggingName="cmd");
% which element of the RPM vector is the encoder and which is the ideal sensor
phX = get_param(ms + "/Mux", "PortHandles");
muxSrc = arrayfun(@(h) string(get_param(get_param(get_param(h, "Line"), "SrcBlockHandle"), "Name")), phX.Inport);
iEnc   = find(muxSrc == "rad/s to RPM");   % HexaMotor measurement: 48 CPR quadrature + 0.05 s filtered derivative
iIdeal = find(muxSrc == "Gear Ratio");     % ideal speed sensor
iDec   = find(muxSrc == "Sampling");       % original example: pulse-timing decoder on one channel
LPF = 1/(0.05*s + 1);

%% A) open loop, 7 V
Vstep = 7;
inA = Simulink.SimulationInput(mdl);
inA = inA.setBlockParameter(mdl + "/Manual Switch1", "sw", "0");                 % open-loop constant
inA = inA.setBlockParameter(mdl + "/Open Loop Constant", "Value", num2str(Vstep/p.Vsupply));
inA = inA.setModelParameter(StopTime="1.5", SignalLogging="on", SignalLoggingName="logsout");
outA = sim(inA);
rA = outA.logsout.get("rpmOut").Values;
tA = rA.Time; yIdealA = rA.Data(:, iIdeal); yEncA = rA.Data(:, iEnc);
yModelA = rpm*Vstep*p.km*(1 - exp(-tA/p.tau));          % step response of G (variable-step time vector)
iss = tA > 1;
fprintf("A) open loop 7 V:  Simscape %.1f RPM | identified model %.1f RPM | datasheet %.1f RPM\n", ...
    mean(yIdealA(iss)), yModelA(end), 280*Vstep/12);
t63 = fminsearch(@(q) sum((yIdealA - mean(yIdealA(iss))*(1 - exp(-tA/q))).^2), p.tau);   % least-squares first-order fit
fprintf("   time constant:  Simscape %.4f s | identified %.4f s\n", t63, p.tau);

%% B) closed loop, PI from the SysID demo
inB = Simulink.SimulationInput(mdl);
inB = inB.setModelParameter(StopTime="10", SignalLogging="on", SignalLoggingName="logsout");
outB = sim(inB);
rB = outB.logsout.get("rpmOut").Values; cB = outB.logsout.get("cmd").Values;
tB = rB.Time; yEncB = rB.Data(:, iEnc); yIdealB = rB.Data(:, iIdeal); yDecB = rB.Data(:, iDec);
cmdB = interp1(cB.Time, squeeze(cB.Data), tB, "previous", "extrap");
% prediction: same discrete PI (duty per RPM, +/-1 limit) around the identified G and the 0.05 s measurement filter
Ts = p.sampleTime; tk = (0:Ts:tB(end))';
Gd = c2d(ss(G*LPF), Ts, "zoh"); [A, B, C] = ssdata(Gd);
x = zeros(size(A, 1), 1); I = 0; yPred = zeros(size(tk));
rk = interp1(cB.Time, squeeze(cB.Data), tk, "previous", "extrap");
for k = 1:numel(tk)
    yPred(k) = rpm*(C*x);
    e = rk(k) - yPred(k);
    u = min(max(p.Kp_duty*e + I, -1), 1);
    I = I + p.Ki_duty*Ts*e;
    x = A*x + B*(p.Vsupply*u);
end
osSim  = 100*(max(yEncB(tB > 1 & tB < 4)) - 100)/100;
osPred = 100*(max(yPred(tk > 1 & tk < 4)) - 100)/100;
fprintf("B) closed loop PI: overshoot (measured signal) Simscape %.1f %% | identified-model prediction %.1f %%\n", osSim, osPred);
fprintf("   holding 0 RPM (t > 8 s): ideal-speed ripple %.2f RPM rms\n", rms(yIdealB(tB > 8)));

%% plots
figure
tiledlayout(1, 2, TileSpacing="compact")
nexttile
plot(tA, yIdealA, LineWidth=2); hold on
plot(tA, yEncA, ".", MarkerSize=4)
plot(tA, yModelA, "k--", LineWidth=1.5)
Tf = 0.05;                                                % step response of G x filter (two real poles)
plot(tA, rpm*Vstep*p.km*(1 - (p.tau*exp(-tA/p.tau) - Tf*exp(-tA/Tf))/(p.tau - Tf)), "k:", LineWidth=1.5); hold off
grid on; xlabel("Time [s]"); ylabel("Output speed [RPM]")
legend("Simscape, ideal sensor", "Simscape, 48 CPR quadrature + filter", "Identified G(s)", "Identified G(s) \times filter", Location="southeast")
title(sprintf("Open loop, %g V", Vstep))
nexttile
plot(tB, cmdB, "k:", LineWidth=1); hold on
plot(tB, yEncB, ".", MarkerSize=4)
plot(tB, yIdealB, LineWidth=1.2)
plot(tk, yPred, "--", LineWidth=1.5)
plot(tB, yDecB, "-", Color=[0.6 0.6 0.6]); hold off
grid on; xlabel("Time [s]"); ylabel("Output speed [RPM]")
legend("Command", "Simscape, 48 CPR quadrature + filter (feedback)", "Simscape, ideal sensor", ...
    "Identified G(s) \times filter + same PI", "Example pulse-timing decoder (not used)", Location="southeast")
title(sprintf("Closed loop, PI K_p = %.4f, K_i = %.4f (duty/RPM)", p.Kp_duty, p.Ki_duty))
