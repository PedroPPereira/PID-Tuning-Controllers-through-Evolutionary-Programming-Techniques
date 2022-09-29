%% TP3- Decision Systems 2020/21
clear all, close all, clc, warning off

%optimized theta parameters
theta = [-0.0552   -0.5962    0.0906    0.1255    0.1176    0.0939]';
a1 = theta(1); a2 = theta(2); 
b0 = theta(3); b1 = theta(4); b2 = theta(5); b3 = theta(6);

%variables declaration
Ts = 0.08;
Nsamps = 150;
ref = [2*ones(1,Nsamps) 4*ones(1,Nsamps) 3*ones(1,Nsamps) 4.5*ones(1,Nsamps) 3*ones(1,Nsamps)]'; % reference
N = length(ref); 
u = zeros(N,1);
y = zeros(N,1);
e = zeros(N,1);
t1 = 1:N;

%----------------------------------------------------------------------------
hyst = 10/100 * ref(1); % hysteresis
dir = 1;   % direction {+1,-1}	
umin = 0; umax = 5; 

for k = 4:N
    ymax = ref(k) + hyst; % threshold's
	ymin = ref(k) - hyst; 
    
    if (y(k) < ymin)
        u(k) = umax; 
		dir = 1;
	elseif (y(k) > ymax)
        u(k) = umin; 
		dir = -1;
	else
        if (dir  == -1), u(k) = umin;
        else, u(k) = umax;
        end
    end 

    y(k+1) = - a1*y(k) - a2*y(k-1) + b0*u(k) + b1*u(k-1) + b2*u(k-2) + b3*u(k-3);
end
y = y(1:N);

figure(2)
subplot(2,1,1), plot(t1, y, '-', t1, ref, '-. k'), grid on,
title('Resposta em anel fechado PID'), legend('y(k)','ref(k)'), 
xlabel('Amostra','Interpreter','latex'), ylabel({'y(k)'},'Interpreter','latex');
subplot(2,1,2), stairs(t1(1:end),u,'r-'), grid on,
title('Ação de Controlo'), legend('u(k)'),
xlabel('Amostra','Interpreter','latex'), ylabel({'u(k)'},'Interpreter','latex');
%error results
fprintf('mean-squared error = %.5f\n', immse(y, ref) );




