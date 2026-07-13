%% Lead Lag Control Graphs

%Lead controller Gc=(1+a*tau*s)/(1+tau*s)
s=tf('s');
a=[1.4 2 3 4 5 7 10 15];
Lead=(1+a(1)*s)/(1+s);
figure(1);
bode(Lead)
grid on
hold on
for i=2:size(a,2)
Lead=(1+a(i)*s)/(1+s);
bode(Lead)
end
title('Lead Compensator Gc=(1+a*tau*s)/(1+tau*s)')
xlabel('u=tau*Wc')
for i=1:size(a,2)
temp=['a= ',num2str(a(i)),'    '];
Leg(i,1:6)=temp(1:6);
end
legend(Leg);

Lag=(1+s)/(1+a(1)*s);
figure(2);
bode(Lag)
grid on
hold on
for i=2:size(a,2)
Lag=(1+s)/(1+a(i)*s);
bode(Lag)
end
title('Lag Compensatork =1 Gc=(1+tau*s)/(1+a*tau*s)')
xlabel('u=tau*Wc')
for i=1:size(a,2)
temp=['a= ',num2str(a(i)),'    '];
Leg(i,1:6)=temp(1:6);
end
legend(Leg);

