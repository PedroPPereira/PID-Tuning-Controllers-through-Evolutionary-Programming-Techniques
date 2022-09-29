%% TP3- Decision Systems 2020/21
clear all, close all, clc, warning off

%optimized theta parameters
theta = [-0.0552   -0.5962    0.0906    0.1255    0.1176    0.0939]';
a1 = theta(1); a2 = theta(2); 
b0 = theta(3); b1 = theta(4); b2 = theta(5); b3 = theta(6);

%variables declaration
Ts = 0.08;
sumdU = 0;
Nsamps = 100;
ref = [2*ones(1,Nsamps) 4*ones(1,Nsamps) 3*ones(1,Nsamps) 4.5*ones(1,Nsamps) 3*ones(1,Nsamps)]'; % reference
N = length(ref); 
u = zeros(N,1);
y = zeros(N,1);
e = zeros(N,1);
t1 = (1:N)*Ts;
I = 0; 


%----------------------------------------------------------------------------
boolTun = 2;
%1) método da sensibilidade última
if boolTun == 1
    Ku = 5; 
    Tu = .5; 
end
%2) relay feedback tunning
if boolTun == 2
    h = 10/100*2;
    a = (5-1.5)/2;
    d = (5-0)/2;
    Tu = .08/Ts;
    Ku = 4*d/(pi*sqrt(a^2-h^2));
end
%3) ajuste manual
if boolTun == 3
    Kp = 1.6
    Ti = .6
    Td = 0.06
    Kd = Kp*Td
    Ki = Kd/Ti
    %with overshoot
%     Kp = 2    
%     Ki = 1
%     Kd = 0.17
%conversao para ganhos
else
    Kp = 0.6*Ku
    Ti = Tu/2;
    Td = Tu/8;
    Kd = Kp*Td
    Ki = Kd/Ti
end
 

for k = 4:N
    
    e(k) = ref(k) - y(k); % control error
    u(k) = u(k-1) + Kp*(e(k)-e(k-1)) + 0.5*Ki*Ts*(e(k)+e(k-1)) + (Kd/Ts)*(e(k)-2*e(k-1)+e(k-2));
    u(k) = max(min(u(k),5),0);
%     P = Kp*e(k); 
%     I = I + e(k);
%     D = Kd * (e(k)-e(k-1)) / Ts;
%     u(k) = max(min(P + D + Ki*I,5),0);
    y(k+1) = - a1*y(k) - a2*y(k-1) + b0*u(k) + b1*u(k-1) + b2*u(k-2) + b3*u(k-3);
    sumdU = sumdU + (u(k) - u(k-1))^2;
end
y = y(1:N);


figure(2)
subplot(2,1,1), 
plot(t1, y, '-', t1, ref, '-. k'), grid on,
title('Resposta do PID classico pelo método de realimentação do relé'),
legend('y(k)','ref(k)'), axis([0 40 0 5]),
xlabel('Tempo [s]','Interpreter','latex'), 
ylabel({'Amplitude [V]'},'Interpreter','latex');

subplot(2,1,2), 
stairs(t1(1:end),u,'r-'), grid on,
title('Ação de Controlo'), legend('u(k)'), axis([0 40 0 5]),
xlabel('Tempo [s]','Interpreter','latex'), 
ylabel({'u(k) [V]'},'Interpreter','latex');

%error results
fprintf('sum do erro quadratico medio = %.5f\n', immse(y, ref) );
fprintf('media do sum da accao quadratica = %.5f\n', (1/N)*sum(u.^2) );
fprintf('media do sum do módulo do incr da accao= %.5f\n', (1/N)*sumdU );



