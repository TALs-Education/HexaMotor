%% Lead Lag Control Graphs

% Define transfer function variable
s = tf('s');

% Define the parameter vector
a = [1.4 2 3 4 5 7 10 15];

% One color per curve (the default color order repeats after 7 colors)
colors = [lines(7); 0 0 0];

% The x axis is u = tau*Wc: the plots use tau = 1 s, so the frequency axis
% (in rad/s) shows u directly
uLabel = 'u = \tau\omega_c, frequency with \tau = 1 s';

%% Lead Compensator Plot
% Lead controller: Gc = (1 + a*tau*s) / (1 + tau*s)
figure(1);
% Plot first lead compensator
Lead = (1 + a(1)*s) / (1 + s);
h = bodeplot(Lead);
grid on
hold on

% Plot the remaining lead compensators
for i = 2:numel(a)
    Lead = (1 + a(i)*s) / (1 + s);
    bodeplot(Lead);
end
h.ColorOrder = colors;

title('Lead Compensator Gc=(1+a*tau*s)/(1+tau*s)')
xlabel(uLabel)

% Construct legend labels as a cell array
Leg = cell(1, numel(a));
for i = 1:numel(a)
    Leg{i} = ['a= ', num2str(a(i))];
end
legend(Leg);

% Save the figure, so it can be reopened in MATLAB (openfig / double click)
savefig(figure(1), fullfile(fileparts(mfilename('fullpath')), 'LeadCompensator.fig'));

%% Lag Compensator Plot
% Lag controller: Gc = (1 + tau*s) / (1 + a*tau*s)
figure(2);
% Plot first lag compensator
Lag = (1 + s) / (1 + a(1)*s);
h = bodeplot(Lag);
grid on
hold on

% Plot the remaining lag compensators
for i = 2:numel(a)
    Lag = (1 + s) / (1 + a(i)*s);
    bodeplot(Lag);
end
h.ColorOrder = colors;

title('Lag Compensator Gc=(1+tau*s)/(1+a*tau*s)')
xlabel(uLabel)

% Construct legend labels as a cell array for lag plot
Leg = cell(1, numel(a));
for i = 1:numel(a)
    Leg{i} = ['a= ', num2str(a(i))];
end
legend(Leg);

% Save the figure, so it can be reopened in MATLAB (openfig / double click)
savefig(figure(2), fullfile(fileparts(mfilename('fullpath')), 'LagCompensator.fig'));
