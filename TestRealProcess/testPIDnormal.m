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
e = zeros(N,1);
t1 = (1:N)*Ts;
I = 0; 
%------------------ TUNNING METHODS ------------------
boolTun = 4;
%1) método da sensibilidade última
if boolTun == 1
    Ku = 5; Tu = .5; 
end
%2) relay feedback tunning
if boolTun == 2
    h = 10/100*2;  a = (5-1.5)/2;
    d = (5-0)/2;   Tu = .08/Ts;
    Ku = 4*d/(pi*sqrt(a^2-h^2));
end
%3) !!!!! GENETIC ALGORITHM !!!!!
if boolTun == 3
    Kp=0.9447 
    Ki=2.8230
    Kd=0.0100
end
%4) ajuste manual
if boolTun == 4
    Kp = 1.6
    Ti = .6
    Td = 0.06
    Kd = Kp*Td
    Ki = Kd/Ti
%conversao para ganhos
elseif(boolTun ~= 3)
    Kp = 0.6*Ku
    Ti = Tu/2;  Td = Tu/8;
    Kd = Kp*Td
    Ki = Kd/Ti
end
%------------------ RUN TEST ------------------
usbinit %init DAQ
for k = 1:N
    y(k) = usbread(0);
    tic; %init timer
    if k <= 4
        u(k) = ref(k);
    else
        e(k) = ref(k) - y(k); % control error
        u(k) = u(k-1) + Kp*(e(k)-e(k-1)) + 0.5*Ki*Ts*(e(k)+e(k-1)) + (Kd/Ts)*(e(k)-2*e(k-1)+e(k-2));
    end
    u(k) = max(min(u(k),5),0); % saturation limiter
    if k>1, sumdU = sumdU + (u(k) - u(k-1))^2; end
    usbwrite(u(k),0);
    Dt = toc; %stop timer
    pause(Ts-Dt); 
end

usbwrite(0,0)
if boolTun==1, save cntrNormalZN.mat u y ref -mat; end
if boolTun==2, save cntrNormalRELE.mat u y ref -mat; end
if boolTun==3, save cntrNormalGA.mat u y ref -mat; end
if boolTun==4, save cntrNormalMAN.mat u y ref -mat; end

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










