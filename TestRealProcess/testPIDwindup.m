% TP3- Decision Systems 2020/21
clear all, close, clc
%variables declaration
Ts = 0.08;
sumdU = 0;
Nsamps = 100;
ref = [2*ones(1,Nsamps) 4*ones(1,Nsamps) 3*ones(1,Nsamps) 4.5*ones(1,Nsamps) 3*ones(1,Nsamps)]'; % reference
N = length(ref); 
u = zeros(N,1);
y = zeros(N,1);
t1 = (1:N)*Ts;
I = 0; 
Npid = 1; 
%------------------ TUNNING METHODS ------------------
boolTun = 4;
%1) método da sensibilidade última
if boolTun == 1
    Ku = 5; 
    Tu = .5; 
end
%2) relay feedback tunning
if boolTun == 2
    h = 10/100*2;  a = (5-1.5)/2;
    d = (5-0)/2;   Tu = .08/Ts;
    Ku = 4*d/(pi*sqrt(a^2-h^2))
end
%3) !!!!! GENETIC ALGORITHM !!!!!
if boolTun == 3
    Kp = 1.0998    
    Ti = 0.3666    
    Td = 4.9909 
    Npid = 0.0635 
    Kd = Kp*Td
    Ki = Kd/Ti
end
%4) ajuste manual
if boolTun == 4
    Kp = 1.6
    Ti = .6
    Td = 0.06
%conversao para ganhos
elseif(boolTun ~= 3)
    Kp = 0.6*Ku
    Ti = Tu/2;  Td = Tu/8
    Kd = Kp*Td
    Ki = Kd/Ti
end
%PID anti-windup specifications
Tt = Ti;
bi = Kp * Ts / Ti; % integral gain
ao = Ts / Tt;      % integral windup gain
ad = (2*Td-Npid*Ts)/(2*Td+Npid*Ts);
bd = 2*Kp*Npid*Td/(2*Td+Npid*Ts); % derivative gain
I = 0; D = 0;
%------------------ RUN TEST ------------------
usbinit %init DAQ
for k = 1:N
    y(k) = usbread(0);
    tic; %init timer
    if k <= 4
        u(k) = ref(k);
    else
        [u, D, I] = controlPID(ref, y, Kp, D, ad, bd, I, u, k, bi, ao);
    end
    u(k) = max(min(u(k),5),0); % saturation limiter
    if k>1, sumdU = sumdU + (u(k) - u(k-1))^2; end
    usbwrite(u(k),0);
    Dt = toc; %stop timer
    pause(Ts-Dt); 
end

usbwrite(0,0)
if boolTun==1, save cntrWindupZN.mat u y ref -mat; end
if boolTun==2, save cntrWindupRELE.mat u y ref -mat; end
if boolTun==3, save cntrWindupGA.mat u y ref -mat; end
if boolTun==4, save cntrWindupMAN.mat u y ref -mat; end

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
fprintf('sum do erro quadratico medio = %.5f\n', mse(y, ref) );
fprintf('media do sum da accao quadratica = %.5f\n', (1/N)*sum(u.^2) );
fprintf('media do sum do módulo do incr da accao= %.5f\n', (1/N)*sumdU );










