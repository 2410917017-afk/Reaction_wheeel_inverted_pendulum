%% MODELO LINEALIZADO + DISEÑO LQR

load('params.mat')

% ─────────────────────────────────────────────────────────
%% 1. MATRICES DEL ESPACIO DE ESTADOS
%    x = [theta; theta_dot; phi_dot]
%    u = tau (torque, o voltaje según modelo)
% ─────────────────────────────────────────────────────────

A = [0,           1,    0;
     M_g/I_total, 0,    0;
    -M_g/I_total, 0,   -b_eq/I_w];

B = [0;
    -1/I_total;
     1/I_w + 1/I_total];

C = eye(3);   % observamos todos los estados
D = zeros(3,1);

fprintf('=== Matriz A ===\n'); disp(A)
fprintf('=== Matriz B ===\n'); disp(B)

% ─────────────────────────────────────────────────────────
%% 2. ANÁLISIS DE ESTABILIDAD EN LAZO ABIERTO
% ─────────────────────────────────────────────────────────

polos_LA = eig(A);
fprintf('=== Polos en lazo abierto ===\n')
disp(polos_LA)

% Si algún polo tiene parte real positiva → inestable → necesita control
if any(real(polos_LA) > 0)
    fprintf('⚠ Sistema INESTABLE en lazo abierto (esperado)\n\n')
else
    fprintf('Sistema estable en lazo abierto\n\n')
end

% ─────────────────────────────────────────────────────────
%% 3. VERIFICAR CONTROLABILIDAD
% ─────────────────────────────────────────────────────────

Co = ctrb(A, B);
rango = rank(Co);
fprintf('=== Controlabilidad ===\n')
fprintf('Rango de la matriz de controlabilidad: %d / %d\n', rango, size(A,1))

if rango == size(A, 1)
    fprintf('✓ Sistema CONTROLABLE\n\n')
else
    fprintf('✗ Sistema NO controlable — revisa el modelo\n\n')
end

% ─────────────────────────────────────────────────────────
%% 4. DISEÑO LQR
% ─────────────────────────────────────────────────────────
% Ajusta q1, q2, q3 y r para cambiar el comportamiento:
%   q1 → penaliza ángulo θ        (más grande = recupera más rápido)
%   q2 → penaliza velocidad θ̇    (más grande = menos oscilaciones)
%   q3 → penaliza velocidad φ̇    (más grande = limita velocidad rueda)
%   r  → penaliza esfuerzo motor  (más grande = respuesta más suave)

q1 = 10;    % peso en θ
q2 = 8;     % peso en θ̇
q3 = 5;      % peso en φ̇
r  = 250;    % peso en u


Q = diag([q1, q2, q3]);
R = r;

[K, P, polos_LC] = lqr(A, B, Q, R);

fprintf('=== Ganancias LQR ===\n')
fprintf('K = [K_theta, K_thetadot, K_phidot]\n')
fprintf('K = [%.4f,  %.4f,  %.4f]\n\n', K(1), K(2), K(3))

fprintf('=== Polos en lazo cerrado ===\n')
disp(polos_LC)

if all(real(polos_LC) < 0)
    fprintf('✓ Sistema ESTABLE en lazo cerrado\n\n')
else
    fprintf('✗ Sistema inestable — ajusta Q y R\n\n')
end

% ─────────────────────────────────────────────────────────
%% 5. RESPUESTA LINEAL — verificación rápida
% ─────────────────────────────────────────────────────────

A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);

t    = 0:0.001:3;
x0   = [0.1; 0; 0];    % condición inicial: 0.1 rad ≈ 5.7°

[y, t_out] = initial(sys_cl, x0, t);

figure('Name','Respuesta lineal lazo cerrado','NumberTitle','off')

subplot(3,1,1)
plot(t_out, y(:,1)*180/pi, 'b', 'LineWidth', 1.5)
ylabel('\theta [°]')
title('Respuesta del sistema linealizado con LQR')
grid on; yline(0, '--k')

subplot(3,1,2)
plot(t_out, y(:,2)*180/pi, 'r', 'LineWidth', 1.5)
ylabel('thetadot [°/s]')
grid on; yline(0, '--k')

subplot(3,1,3)
plot(t_out, y(:,3)*180/pi, 'm', 'LineWidth', 1.5)
ylabel('phidot [°/s]')
xlabel('Tiempo [s]')
grid on; yline(0, '--k')

save('modelo.mat', 'A', 'B', 'C', 'D', 'K', 'Q', 'R', 'polos_LC')
