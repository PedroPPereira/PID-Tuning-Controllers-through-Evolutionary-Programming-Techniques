% TP3- Decision Systems 2020/21
clear all, close, clc
%load cntrWindupMAN.mat u y -mat 
%load cntrWindupZN.mat u y -mat 
%load cntrWindupRELE.mat u y -mat 
%load cntrWindupGA.mat u y -mat 
%%----------------------------
load cntrNormalGA.mat u y -mat 



%init setup
Ts = 0.08;
sumdU = 0;
Nsamps = 100;
ref = [2*ones(1,Nsamps) 4*ones(1,Nsamps) 3*ones(1,Nsamps) 4.5*ones(1,Nsamps) 3*ones(1,Nsamps)]'; % reference
N = length(ref); 
t1 = (1:N)*Ts;

%get the control action error
for k = 2:N
    sumdU = sumdU + (u(k) - u(k-1))^2;
end


%plot results
figure(1)
subplot(2,1,1), 
plot(t1, y, '-', t1, ref, '-. k'), grid on,
title('Resposta do PID classico'),
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


