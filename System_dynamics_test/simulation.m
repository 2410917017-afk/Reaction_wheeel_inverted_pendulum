%% SIMULACIÓN NO LINEAL DEL SISTEMA REAL
close all;
 
load('params.mat')
load('modelo.mat')

% ─────────────────────────────────────────────────────────
%% 1. FUNCIÓN DE LA DINÁMICA NO LINEAL
% ─────────────────────────────────────────────────────────
% ẋ = f(x, u) — ecuaciones originales con sinθ

f = @(t, x) System(t, x, K, M_g, I_total, I_w, b_eq, Vmax);

% ─────────────────────────────────────────────────────────
%% 2. CONDICIONES DE SIMULACIÓN
% ─────────────────────────────────────────────────────────

t_span = [0, 1];
x0     = [0.15; 0; 0];    % 0.15 rad ≈ 8.6° de inclinación inicial

% ─────────────────────────────────────────────────────────
%% 3. INTEGRACIÓN NUMÉRICA
% ─────────────────────────────────────────────────────────

options = odeset('MaxStep', 0.005, 'RelTol', 1e-6);
[t, x]  = ode45(f, t_span, x0, options);

% Calcular la señal de control en cada instante
u_hist = zeros(length(t), 1);
for k = 1:length(t)
    u_k = -K * x(k,:)';
    u_hist(k) = max(-Vmax, min(Vmax, u_k));  % saturación
end

% ─────────────────────────────────────────────────────────
%% 4. GRÁFICAS
% ─────────────────────────────────────────────────────────

figure('Name','Simulación no lineal — LQR','NumberTitle','off',...
       'Position',[100 100 800 600])

subplot(3,1,1)
plot(t, x(:,1)*180/pi, 'b', 'LineWidth', 1.8)
ylabel('\theta [°]', 'FontSize', 11)
title('Péndulo con Reaction Wheel — Simulación No Lineal (LQR)', 'FontSize', 12)
grid on; yline(0,'--k','LineWidth',1.2)
legend('\theta (ángulo péndulo)')

subplot(3,1,2)
plot(t, x(:,2)*180/pi, 'r', 'LineWidth', 1.8)
hold on
plot(t, x(:,3)*180/pi, 'm', 'LineWidth', 1.8)
ylabel('Velocidades [°/s]', 'FontSize', 11)
grid on; yline(0,'--k','LineWidth',1.2)
legend('\dot{\theta}', '\dot{\phi} (rueda)')

subplot(3,1,3)
plot(t, u_hist, 'b', 'LineWidth', 1.8)
ylabel('Control u [V]', 'FontSize', 11)
xlabel('Tiempo [s]', 'FontSize', 11)

%yline(Vmax,'--r','V_{max}','LineWidth',1)
%yline(-Vmax,'--r','V_{min}','LineWidth',1)
grid on
legend('Señal de control')

% ─────────────────────────────────────────────────────────
%% 5. ANÁLISIS DE DESEMPEÑO
% ─────────────────────────────────────────────────────────

theta_deg = x(:,1) * 180/pi;

% Tiempo de establecimiento (±2% del valor inicial)
tol       = 0.02 * abs(theta_deg(1));
idx_est   = find(abs(theta_deg) < tol, 1, 'first');
t_est     = t(idx_est);

% Sobreimpulso
[theta_max, idx_max] = max(abs(theta_deg));
sobreimpulso = (theta_max - abs(theta_deg(1))) / abs(theta_deg(1)) * 100;
sobreimpulso = max(0, sobreimpulso);

fprintf('=== Métricas de desempeño ===\n')
fprintf('Condición inicial:     %.2f °\n', theta_deg(1))
fprintf('Tiempo establecimiento: %.3f s\n', t_est)
fprintf('Sobreimpulso:          %.1f %%\n', sobreimpulso)
fprintf('Control máximo:        %.2f V\n', max(abs(u_hist)))

% ─────────────────────────────────────────────────────────
%% 6. COMPARACIÓN LINEAL vs NO LINEAL
% ─────────────────────────────────────────────────────────

A_cl   = A - B*K;
sys_cl = ss(A_cl, B, C, D);
[y_lin, t_lin] = initial(sys_cl, x0, t_span(2));

figure('Name','Lineal vs No lineal','NumberTitle','off')
plot(t_lin, y_lin(:,1)*180/pi, 'b--', 'LineWidth', 1.5)
hold on
plot(t, x(:,1)*180/pi, 'r', 'LineWidth', 1.5)
grid on
ylabel('\theta [°]')
xlabel('Tiempo [s]')
title('Comparación: modelo lineal vs no lineal')
legend('Modelo lineal','Modelo no lineal','Location','best')
yline(0,'--k')
