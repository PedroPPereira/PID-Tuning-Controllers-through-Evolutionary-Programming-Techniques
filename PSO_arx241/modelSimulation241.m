
function modelSimulation241(theta, Ue, Ye, Uv, Yv)
    clc
    %coeficientes do ARX
    a1 = theta(1); a2 = theta(2); 
    b0 = theta(3); b1 = theta(4); b2 = theta(5); b3 = theta(6);
    Ts = 0.08; %tempo de amostragem

    %% Dados de estimação
    Ne = length(Ue);
    y1 = zeros(Ne, 1);
    te = (1:Ne)*Ts;
    y1(1:4) = Ye(1:4);
    for k = 5:Ne
        y1(k) = - a1*y1(k-1) - a2*y1(k-2) + b0*Ue(k-1) + b1*Ue(k-2) + b2*Ue(k-3) + b3*Ue(k-4);
    end
    % comparação dos modelos
    figure(1)
    subplot(2,1,1); plot(te, Ye, 'r', te, y1, 'b'), grid on,
    title('Conjunto de Estimação: Modelo vs Dados reais'), legend('y real','y simulado'), 
    xlabel('Tempo [s]','Interpreter','latex'), ylabel({'Output [V]'},'Interpreter','latex');
    % erro MSE
    Ee = Ye-y1;
    fprintf('mean-squared error (estimation) = %.5f\n', Ee'*Ee/Ne ); %immse(y1, Ye)

    %% Dados de validação
    %cortar componente transitoria
    Uv = Uv(length(Uv)-Ne+1:end);
    Yv = Yv(length(Yv)-Ne+1:end);
    Nv = length(Uv);
    y2 = zeros(Nv, 1);
    tv = (1:Nv)*Ts;
    y2(1:4) = Yv(1:4);
    for k = 5:Nv
        y2(k) = - a1*y2(k-1) - a2*y2(k-2) + b0*Uv(k-1) + b1*Uv(k-2) + b2*Uv(k-3) + b3*Uv(k-4);
    end
    % comparação dos modelos
    subplot(2,1,2); plot(tv, Yv, 'r', tv, y2, 'b'), grid on,
    title('Conjunto de Validação: Modelo vs Dados reais'), legend('y real','y simulado'), 
    xlabel('Tempo [s]','Interpreter','latex'), ylabel({'Output [V]'},'Interpreter','latex');
    % erro MSE
    Ev = Yv-y2;
    fprintf('mean-squared error (validation) = %.5f\n', Ev'*Ev/Nv ); %immse(y2, Yv)
end





