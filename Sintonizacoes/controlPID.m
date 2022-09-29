function [u, D, I] = controlPID(ref, y, Kp, D, ad, bd, I, u, k, bi, ao)
    e = ref(k) - y(k);            % control error
    P = Kp * e;                   % proportional control action
    D = ad*D - bd*(y(k)-y(k-1));  % update derivative action
    v = P + I + D;                % temporary control action
    u(k) = max(min(v,5),0);       % actuator saturation
    I = I + bi*e + ao*(u(k) - v); % update integral action
end