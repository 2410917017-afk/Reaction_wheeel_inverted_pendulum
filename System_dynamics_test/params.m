%% PARÁMETROS FÍSICOS DEL SISTEMA
% Reaction Wheel Inverted Pendulum
% Todos en unidades SI

clear; clc;

% ── Péndulo ──────────────────────────────────────────────
m_p  = 0.39;      % [kg]  masa del brazo + motor
l    = 0.12;      % [m]   distancia pivote a CM del péndulo
L    = 0.11;      % [m]   distancia pivote al centro de la rueda
I_p  = 0.006763;     % [kg·m²] inercia del péndulo respecto al pivote
                  

% ── Reaction Wheel ───────────────────────────────────────
m_w  = 0.11;      % [kg]  masa de la rueda
r_w  = 0.065;      % [m]   radio de la rueda
I_w  = m_w * r_w^2;  % [kg·m²] anillo: I_w = m_w*r_w^2
                            

% ── Parámetros combinados ────────────────────────────────
g       = 9.81;
M_g     = (m_p*l + m_w*L) * g;   % término gravitacional
I_total = I_p + m_w * L^2;       % inercia total respecto al pivote

% ── Motor DC (simplificado) ──────────────────────────────
Kt   = 0.038;      % [N·m/A]  constante de torque
Ke   = 0.038;      % [V·s/rad] constante FCEM (= Kt en SI)
Ra   = 0.8;       % [Ω]      resistencia de armadura
Vmax = 15.0;      % [V]      voltaje máximo

% Amortiguamiento eléctrico equivalente
b_eq = (Kt * Ke) / Ra;

tau_max = Kt * (Vmax / Ra);
fprintf('Torque máximo motor: %.3f N·m\n', tau_max)
fprintf('Torque gravitacional: %.3f N·m\n', M_g)
if tau_max > M_g
    fprintf('✓ Motor suficiente\n')
else
    fprintf('✗ Motor insuficiente')
end

fprintf('=== Parámetros calculados ===\n')
fprintf('M_g     = %.4f N·m\n', M_g)
fprintf('I_total = %.6f kg·m²\n', I_total)
fprintf('I_w     = %.6f kg·m²\n', I_w)
fprintf('b_eq    = %.6f N·m·s\n', b_eq)

save('params.mat')
