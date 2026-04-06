function dxdt = System(t, x, K, M_g, I_total, I_w, b_eq, Vmax)
    % Estados
    theta     = x(1);
    theta_dot = x(2);
    phi_dot   = x(3);

    % Ley de control LQR con saturación
    u = -K * x;
    u = max(-Vmax, min(Vmax, u));   % limitar al voltaje máximo

    % Ecuaciones no lineales (con sinθ real)
    theta_ddot = (M_g * sin(theta) - u) / I_total;
    phi_ddot   = u/I_w - theta_ddot - (b_eq/I_w)*phi_dot;

    dxdt = [theta_dot; theta_ddot; phi_ddot];
end
