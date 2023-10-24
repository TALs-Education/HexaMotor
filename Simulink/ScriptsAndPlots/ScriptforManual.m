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


%% Ziegler-Nichols - Step response velocity control  -  test
% select visually the slope start and end points
time = ScopeData.time(210:230);
y=ScopeData.signals.values(210:230,2);

% run fit function
[curve, goodness] = fit( time, y, 'poly1' );

% plot response:
figure(1);
plot(ScopeData.time,ScopeData.signals.values(:,1))
hold on
plot(ScopeData.time,ScopeData.signals.values(:,2),'.')
plot(curve)
plot(ScopeData.time,17.5*ones(size(ScopeData.time)))
grid on
axis([0.95 1.75 -1 25])
title('Ziegler-Nichols Curve fit')
legend Input Output Curve
xlabel Time [sec]
ylabel Amplitude

%% unity feedback

s=tf('s');
G = 2.5/(0.1*s+1);
LPF = 1/(0.05*s+1);
figure(1)
plot(ScopeData.time,ScopeData.signals.values(:,1),'LineWidth',1)
hold on
plot(ScopeData.time,ScopeData.signals.values(:,2),'.','LineWidth',4);
[y,t] = lsim(G/(1+G),ScopeData.signals.values(:,1),ScopeData.time);
plot(t,y,'LineWidth',1);
[y,t] = lsim(G*LPF/(1+G*LPF),ScopeData.signals.values(:,1),ScopeData.time);
plot(t,y,'LineWidth',1);
grid on
title('Square wave response - Comparison')
legend Input ActualSystem Theoretical WithFilter
xlabel Time[sec]
ylabel [Rad/s]

%% Pi Controller

s=tf('s');
G = 2.5/(0.1*s+1);
LPF = 1/(0.05*s+1);
C = 0.5+5/s;
figure(1)
plot(ScopeData.time,ScopeData.signals.values(:,1),'LineWidth',1)
hold on
plot(ScopeData.time,ScopeData.signals.values(:,2),'.','LineWidth',4);
[y,t] = lsim(G*LPF*C/(1+G*LPF*C),ScopeData.signals.values(:,1),ScopeData.time);
plot(t,y,'LineWidth',1);
grid on
title('PI Controller, Actual vs Simulated')
legend Input ActualSystem TheoreticalWithFilter
xlabel Time[sec]
ylabel [Rad/s]

%% transient response
s=tf('s');
wn=1
xsi=0.7
G=0.9*wn^2/(s^2+xsi*2*s+wn^2)
[sys t] = step(G);
figure(1);
step(G)
hold on
plot(t,sys,'LineWidth',3)
t = [10/size(sys,1):10/size(sys,1):10];
plot(t,ones(size(t,2),1))
axis([0 10 0 1.1])
title('Transient Response Analysis')

%% positionControl
s=tf('s');
data = iddata(ScopeData.signals.values(:,2),ScopeData.signals.values(:,1),0.005)
init_sys = 25/(0.05*s^2+s+25)
T1=tfest(data,2,0)
T2=tfest(data,init_sys)


%% Position step response:
OS = 0.23
Tp = 0.16
Kp = 10

xi = sqrt(1/(1+(pi/log(OS))^2))
wn = pi/Tp/sqrt(1-xi)

tau = 1/(2*xi*wn)
k = wn^2*tau/Kp

% Generate plots from recordings and simulated transfer function
s=tf('s');
G = k/(tau*s+1)/s; % update the transfer function values
figure(1)
plot(ScopeData.time,ScopeData.signals.values(:,1))
hold on
plot(ScopeData.time,ScopeData.signals.values(:,2),'.','MarkerSize',5,'MarkerEdgeColor','r');
[y,t] = lsim(Kp*G/(1+Kp*G),ScopeData.signals.values(:,1),ScopeData.time);
plot(t,y,'k','LineWidth',2);
hold off
grid on
title('Square wave response')
legend Input ActualSystem Theoretical
xlabel Time[sec]
ylabel [Rad]
axis([0 5 -0.3 1.3])

%% frequency response:
% Estimate transfer function
s=tf('s'); 
data = iddata(ScopeData.signals.values(:,2),ScopeData.signals.values(:,1),0.005);
T=tfest(data,2,0)

Kp = 10

wn = sqrt(T.Denominator(3))
tau = 1/T.Denominator(2)
k = wn^2*tau/Kp

% Generate plots from recordings and simulated transfer function
s=tf('s');
G = k/(tau*s+1)/s % update the transfer function values
figure(1)
plot(ScopeData.time,ScopeData.signals.values(:,1))
hold on
plot(ScopeData.time,ScopeData.signals.values(:,2),'.','MarkerSize',7,'MarkerEdgeColor','r');
[y,t] = lsim(Kp*G/(1+Kp*G),ScopeData.signals.values(:,1),ScopeData.time);
plot(t,y,'k','LineWidth',1);
hold off
grid on
title('Frequency response')
legend Input ActualSystem Theoretical
xlabel Time[sec]
ylabel [Rad]

%% OS and Tp calculation
k=2.454
tau=0.0707

OS = 0.25
Tp = 0.15
xi = sqrt(1/((pi/log(OS))^2+1))
wn = pi/Tp/sqrt(1-xi^2)

Kp = wn^2*tau/k
Kd = (2*xi*wn*tau-1)/k

C = Kp + Kd*s
G = k/(tau*s+1)/s

% figure(1)
% step(G*C/(1+G*C))

figure(1)
plot(ScopeData.time,ScopeData.signals.values(:,1))
hold on
plot(ScopeData.time,ScopeData.signals.values(:,2),'.','MarkerSize',5,'MarkerEdgeColor','r');
[y,t] = lsim(C*G/(1+C*G),ScopeData.signals.values(:,1),ScopeData.time);
plot(t,y,'k','LineWidth',2);
hold off
grid on
title('Square wave response PD Controller')
legend Input ActualSystem Theoretical
xlabel Time[sec]
ylabel [Rad]
axis([0 5 -0.3 1.5])

%% lead lag
k=2.454
tau=0.0707

Kp = 15.1
Kd = 0.125

s= tf('s')
P = k/(tau*s+1)/s
C= Kp+Kd*s

a= 3;
tauL = 0.2/23;
figure(3)
bode(P*C)
hold on
Lead = (1+tauL*a*s)/(1+tauL*s)
bode(P*C*Lead)
hold off
grid on
legend PD PD_Lead

figure(4)
plot(ScopeData.time,ScopeData.signals.values(:,1))
hold on
plot(ScopeData.time,ScopeData.signals.values(:,2),'.','MarkerSize',5,'MarkerEdgeColor','r');
[y,t] = lsim(C*G*Lead/(1+C*G*Lead),ScopeData.signals.values(:,1),ScopeData.time);
plot(t,y,'k','LineWidth',2);
hold off
grid on
title('Square wave response PD , LeadLag Controller')
legend Input ActualSystem Theoretical
xlabel Time[sec]
ylabel [Rad]
axis([0 5 -0.3 1.5])

%% bonus plot
figure(5)
plot(rec1.time,rec1.signals.values(:,1),'k')
hold on
plot(rec1.time,rec1.signals.values(:,2),'r','LineWidth',2)
plot(rec2.time,rec2.signals.values(:,2),'c','LineWidth',2)
plot(rec3.time,rec3.signals.values(:,2),'b','LineWidth',2)
hold off
grid on
title('Motor Controller design process')
legend Input Kp PD PD-Lead
xlabel Time[sec]
ylabel [Rad]
axis([0 5 -0.3 1.5])