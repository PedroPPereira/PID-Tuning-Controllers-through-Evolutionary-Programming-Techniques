
function modelSimulationPID(K, Ts, theta, ref)
    clc
    
    %ARX model
    a1 = theta(1); a2 = theta(2); 
    b0 = theta(3); b1 = theta(4); b2 = theta(5); b3 = theta(6);
    %PID specifications
    Kp = K(1); Ti = K(2); Td = K(3); Npid = K(4); 
    Tt = Ti;
    bi = Kp * Ts / Ti; % integral gain
    ao = Ts / Tt;      % integral windup gain
    ad = (2*Td-Npid*Ts)/(2*Td+Npid*Ts);
    bd = 2*Kp*Npid*Td/(2*Td+Npid*Ts); % derivative gain
    I = 0; D = 0;
    %setup simulation
    sumdU = 0;
    N = length(ref);
    u = zeros(N,1);
    y = zeros(N,1);
    t = (1:N)*Ts;
    
    for k = 4:N
        [u, D, I] = controlPID(ref, y, Kp, D, ad, bd, I, u, k, bi, ao);
        y(k+1) = - a1*y(k) - a2*y(k-1) + b0*u(k) + b1*u(k-1) + b2*u(k-2) + b3*u(k-3);
        sumdU = sumdU + (u(k) - u(k-1))^2;
    end
    y = y(1:N);
    
    figure(3)
    subplot(2,1,1), plot(t, y, '-', t, ref, '-. k'), grid on,
    title('Resposta em anel fechado PID anti-windup com AG'), legend('y(k)','ref(k)'), 
    xlabel('Tempo [s]','Interpreter','latex'), axis([0 60 0 5]),
    ylabel({'Amplitude [V]'},'Interpreter','latex');
    subplot(2,1,2), stairs(t(1:end),u,'r-'), grid on,
    title('Ação de Controlo'), legend('u(k)'), axis([0 60 0 5]),
    xlabel('Tempo [s]','Interpreter','latex'),
    ylabel({'u(k) [V]'},'Interpreter','latex');
    %error results
    fprintf('sum do erro quadratico medio = %.5f\n', immse(y, ref) );
    fprintf('media do sum da accao quadratica = %.5f\n', (1/N)*sum(u.^2) );
    fprintf('media do sum do módulo do incr da accao= %.5f\n', (1/N)*sumdU );
end








