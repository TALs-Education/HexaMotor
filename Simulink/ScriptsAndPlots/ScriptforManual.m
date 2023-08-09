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
step(feedback(2*10*G,1))

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
