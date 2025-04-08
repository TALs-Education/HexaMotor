%% Lead Lag Control Graphs

% Define transfer function variable
s = tf('s');

% Define the parameter vector
a = [1.4 2 3 4 5 7 10 15];

%% Lead Compensator Plot
% Lead controller: Gc = (1 + a*tau*s) / (1 + tau*s)
figure(1);
% Plot first lead compensator
Lead = (1 + a(1)*s) / (1 + s);
bode(Lead)
grid on
hold on

% Plot the remaining lead compensators
for i = 2:numel(a)
    Lead = (1 + a(i)*s) / (1 + s);
    bode(Lead)
end

title('Lead Compensator Gc=(1+a*tau*s)/(1+tau*s)')
xlabel('u = tau*Wc')

% Construct legend labels as a cell array
Leg = cell(1, numel(a));
for i = 1:numel(a)
    Leg{i} = ['a= ', num2str(a(i))];
end
legend(Leg);

%% Lag Compensator Plot
% Lag controller: Gc = (1 + tau*s) / (1 + a*tau*s)
figure(2);
% Plot first lag compensator
Lag = (1 + s) / (1 + a(1)*s);
bode(Lag)
grid on
hold on

% Plot the remaining lag compensators
for i = 2:numel(a)
    Lag = (1 + s) / (1 + a(i)*s);
    bode(Lag)
end

title('Lag Compensator Gc=(1+tau*s)/(1+a*tau*s)')
xlabel('u = tau*Wc')

% Construct legend labels as a cell array for lag plot
Leg = cell(1, numel(a));
for i = 1:numel(a)
    Leg{i} = ['a= ', num2str(a(i))];
end
legend(Leg);
