function f = fitnessPID(K, Ts, theta, P, Q, W, ref)
    %ARX model
    a1 = theta(1); a2 = theta(2); 
    b0 = theta(3); b1 = theta(4); b2 = theta(5); b3 = theta(6);
    %PID specifications
    Kp = K(1); Ki = K(2); Kd = K(3);
    %setup simulation
    N = length(ref);
    u = zeros(N,1);
    y = zeros(N,1);
    e = zeros(N,1);
    sumP = 0; sumQ = 0; sumW = 0;
    
    for k = 4:N
        e(k) = ref(k) - y(k);
        u(k) = u(k-1) + Kp*(e(k)-e(k-1)) + 0.5*Ki*Ts*(e(k)+e(k-1)) + (Kd/Ts)*(e(k)-2*e(k-1)+e(k-2));
        u(k) = max(min(u(k),5),0);
        y(k+1) = - a1*y(k) - a2*y(k-1) + b0*u(k) + b1*u(k-1) + b2*u(k-2) + b3*u(k-3);
        %cost function terms
        sumP = sumP + P*(ref(k) - y(k))^2; 
        sumQ = sumQ + Q*(u(k)^2); 
        sumW = sumW + W*(u(k) - u(k-1))^2;   
    end 
    f = (sumP + sumQ + sumW)/(N-4);
end