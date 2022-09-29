function f = fitnessPID(K, Ts, theta, P, Q, W, ref)
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
    N = length(ref);
    u = zeros(N,1);
    y = zeros(N,1);
    sumP = 0; sumQ = 0; sumW = 0;
    
    for k = 4:N
        [u, D, I] = controlPID(ref, y, Kp, D, ad, bd, I, u, k, bi, ao);
        y(k+1) = - a1*y(k) - a2*y(k-1) + b0*u(k) + b1*u(k-1) + b2*u(k-2) + b3*u(k-3);
        %cost function terms
        sumP = sumP + P*(ref(k) - y(k))^2; 
        sumQ = sumQ + Q*u(k)^2; 
        sumW = sumW + W*(u(k) - u(k-1))^2;   
    end
    
    f = sumP + sumQ + sumW;
end