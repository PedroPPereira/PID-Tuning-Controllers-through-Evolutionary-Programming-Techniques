function f = fitness(theta, Ye, Ue)
    a1 = theta(1); a2 = theta(2); a3 = theta(3);
    b0 = theta(4); b1 = theta(5);
    f = 0;
    N = length(Ye);
    y = zeros(N);
    for k = 4:N
        %resposta do sistema simulada
        y(k) = - a1*y(k-1) - a2*y(k-2) - a3*y(k-3) + b0*Ue(k-2) + b1*Ue(k-3);
        %funcao de custo MSE
        f = f + (( y(k) - Ye(k))^2)/(N-4);
    end
    %E = Ye-y;
    %f = E'*E/(N-4)
end