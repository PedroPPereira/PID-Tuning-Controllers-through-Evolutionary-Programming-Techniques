%% TP3- Decision Systems 2020/21
clear all, close all, clc, warning off

%optimized theta parameters
theta = [-0.0552   -0.5962    0.0906    0.1255    0.1176    0.0939]';
a1 = theta(1); a2 = theta(2); 
b0 = theta(3); b1 = theta(4); b2 = theta(5); b3 = theta(6);

%variables declaration
Ts = 0.08;
Nsamps = 150;
sumdU = 0;
ref = [2*ones(1,Nsamps) 4*ones(1,Nsamps) 3*ones(1,Nsamps) 4.5*ones(1,Nsamps) 3*ones(1,Nsamps)]'; % reference
N = length(ref); 
u = zeros(N,1);
y = zeros(N,1);
e = zeros(N,1);
t = (1:N)*Ts;
I = 0;

%----------------------------------------------------------------------------
% PID controller w/ GA
Kp = 2; Ki = 1; Kd = 0.17; 
K = [Kp Ki Kd];
%penalties
P = 20; Q = 1; W = 10;
%handle function
fun = @(K)fitnessPID(K, Ts, theta, P, Q, W, ref);
%restrictions
UB = 5*ones(1, length(K));   %upper bound
LB = [0.1 0.01 0.01]; %lower bound
%GA specification
options = gaoptimset(@ga);
options = gaoptimset('PopulationType','doubleVector','PopulationSize',250,'CrossoverFraction',...
    0.85,'Generations',40,'SelectionFcn',@selectionroulette,'PlotFcns',@gaplotbestf);
%run GA
[K,fval] = ga(fun,length(K),[],[],[],[],LB,UB,[],[],options);
%plot results
%PID specifications
Kp = K(1); Ki = K(2); Kd = K(3); 


for k = 4:N
    e(k) = ref(k) - y(k); % control error
    P = Kp*e(k); 
    I = I + e(k);
    D = Kd * (e(k)-e(k-1)) / Ts;
    u(k) = max(min(P + D + Ki*I,5),0);
    y(k+1) = - a1*y(k) - a2*y(k-1) + b0*u(k) + b1*u(k-1) + b2*u(k-2) + b3*u(k-3);
    sumdU = sumdU + (u(k) - u(k-1))^2;
end
y = y(1:N);


figure(4)
subplot(2,1,1), plot(t, y, '-', t, ref, '-. k'), grid on,
title('Resposta em anel fechado PID com AG'), legend('y(k)','ref(k)'), 
xlabel('Tempo [s]','Interpreter','latex'), ylabel({'y(k)'},'Interpreter','latex');
subplot(2,1,2), stairs(t(1:end),u,'r-'), grid on,
title('Ação de Controlo'), legend('u(k)'), axis([0 65 0 5]),
xlabel('Tempo [s]','Interpreter','latex'), ylabel({'u(k)'},'Interpreter','latex');
%error results
fprintf('sum do erro quadratico medio = %.5f\n', immse(y, ref) );
fprintf('media do sum da accao quadratica = %.5f\n', (1/N)*sum(u.^2) );
fprintf('media do sum do módulo do incr da accao= %.5f\n', (1/N)*sumdU );

fprintf('Kp=%.4f; Ki=%.4f; Kd=%.4f\n', Kp, Ki, Kd);

