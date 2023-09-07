% Velocity control

% step response
s=tf('s')
G = 1/(0.1*s+1)
figure(1)
step(G)
grid on

% sin response
t=[0:0.01:2]
u = sin(5*t)

[y,t]=lsim(G,u,t)

figure(2)
plot(t,u,t,y,LineWidth=2)
grid on
legend input[v] output[rad/s]
xlabel time[s]
ylabel amplitude
axis ([0 2 -1 1.3])
title 'Sinusoidal Signal Response'

figure(3)
bode(G)

figure(4)
step(feedback(10*G,1))

%% second order systems
% System parameters
wn = 1; % Natural frequency (rad/s)

% Damping values
damping_ratios = [0.1, 0.5, 0.7, 1, 1.5];

% Time vector
t = 0:0.01:10;

% Create figure
figure(5)
hold on;
% Plot step response for different damping ratios
for i = 1:length(damping_ratios)
    zeta = damping_ratios(i); % Damping ratio
    sys = tf(wn^2, [1, 2*zeta*wn, wn^2]); % Transfer function
    [y, ~] = step(sys, t); % Step response
    plot(t, y, 'LineWidth', 2);
end

% Add legend
legend('ζ = 0.1', 'ζ = 0.5', 'ζ = 0.7', 'ζ = 1', 'ζ = 1.5', 'Location', 'Southeast');

% Add labels
title('Step Response for Different Damping Ratios of (1/(s^2+2xi+1)');
xlabel('Time (seconds)');
ylabel('Amplitude');
grid on;
hold off;


%% Position control
Gp = 1/(0.1*s+1)
T = Gp/(1+Gp)
figure(6)
step(T)


%% In Lab velocity Control
plot(ScopeData.signals.values(1),ScopeData.signals.values(2))

% Frequency response
frequencies = [0,1,5,10,11,12,20,30]
amplitudes_dB = [8.057504527,8.057504527,7.394916161,5.640135167,5.105450102,4.825235813,1.138097027,-3.818429963] % in dB
phases = [0,-11.17267701,-38.67465117,-68.75493542,-75.63042896,-79.06817573,-103.1324031,-128.9155039] % in Degrees
amplitudes = [2.528571429,2.528571429,2.342857143,1.914285714,1.8,1.742857143,1.14,0.644285714] % Rad/s/v
% Create a frequency response data object
freq_data = idfrd(amplitudes, frequencies, 0);

% Estimate transfer function (change '1' to the desired order of the system)
sys = tfest(freq_data, 1)

% plot bode like plot, with the added points
w = logspace(-1,2,100);
[mag,phase] = bode(sys*(1/(0.05*s+1)),w);

figure(1);
subplot(2,1,1)
semilogx(w,20*log10(squeeze(mag)))
hold on
semilogx(frequencies, 20*log10(amplitudes), '*');

subplot(2,1,2)
semilogx(w,squeeze(phase))
hold on
semilogx(frequencies,phases)

subplot(2,1,1);
ylabel('Magnitude (dB)');
title('Bode Plot with Measured Data Points');
legend('Transfer function', 'Measured Data');
grid on;

subplot(2,1,2);
xlabel('Frequency (Hz)');
ylabel('Phase (degrees)');
grid on;

