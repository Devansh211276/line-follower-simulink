%% TRANSFER_FUNCTION_ANALYSIS.m
%  Derives the linearized transfer function of the Line Follower
%  with P-only (proportional) control and performs classical analysis.
%
%  Sections:
%    1. Parameters (from configuration.h / plant model)
%    2. Component transfer functions (sensor, P-gain, motor, plant)
%    3. Open-loop & closed-loop TF
%    4. Stability analysis (Bode, Root Locus, Nyquist, Margins)
%    5. Step response & transient specs
%    6. Kp gain sensitivity sweep
%    7. Import data & system identification (optional)
%
%  Usage:
%    >> transfer_function_analysis

clc; clear; close all;

fprintf('============================================================\n');
fprintf('   LINE FOLLOWER — P-ONLY TRANSFER FUNCTION ANALYSIS\n');
fprintf('============================================================\n\n');

%% ===== 1. SYSTEM PARAMETERS =====

% --- From configuration.h ---
Kp        = 5.0;          % Proportional gain (only gain used)
baseSpeed = 90;           % PWM (0–255)

% --- Plant physical parameters ---
maxSpeed      = 0.30;     % max wheel speed at PWM 255 (m/s)
wheelBase     = 0.15;     % distance between wheels (m)
sensorSpacing = 0.009;    % between adjacent QTR sensors (m)

% --- Derived ---
v_fwd = (baseSpeed / 255) * maxSpeed;        % forward velocity (m/s)

fprintf('Proportional gain: Kp = %.1f\n', Kp);
fprintf('Base speed:        %d PWM  →  v_fwd = %.4f m/s\n', baseSpeed, v_fwd);
fprintf('Wheel base:        L = %.3f m\n', wheelBase);
fprintf('Sensor spacing:    d = %.3f m\n\n', sensorSpacing);

%% ===== 2. COMPONENT TRANSFER FUNCTIONS =====

% ---- Sensor Gain K_s ----
% Linearised mapping: lateral offset (m)  →  ir_error (dimensionless)
%   position ≈ Δy / sensorSpacing     (integer sensor units)
%   ir_error  = position / 2
%   ⇒  K_s = 1 / (2 × sensorSpacing)
K_s = 1 / (2 * sensorSpacing);
fprintf('Sensor gain:  K_s = %.2f  (m⁻¹)\n', K_s);

% ---- Motor Gain K_m ----
% Maps P correction to wheel velocity difference:
%   left  = base − correction,   right = base + correction
%   Δv = (2 × correction / 255) × maxSpeed
K_m = 2 * maxSpeed / 255;
fprintf('Motor gain:   K_m = %.6f  (m/s per PWM)\n', K_m);

% ---- Plant Transfer Function G_p(s) ----
% Differential-drive, linearised about straight motion:
%   ω = Δv / L            → θ(s)  = Δv(s) / (L s)
%   ẏ = v_fwd × θ         → y(s)  = v_fwd × θ(s) / s
%   G_p(s) = v_fwd / (L s²)      — double integrator
G_plant = tf(v_fwd, [wheelBase 0 0]);
fprintf('Plant TF:     G_p(s) = %.4f / (%.2f s²)  =  %.4f / s²\n', ...
    v_fwd, wheelBase, v_fwd / wheelBase);

% ---- P-only Controller ----
% C(s) = Kp   (pure gain — no dynamics)
C_p = tf(Kp, 1);
fprintf('Controller:   C(s) = Kp = %.1f  (proportional gain only)\n\n', Kp);

%% ===== 3. OPEN-LOOP & CLOSED-LOOP TF =====

% Total static gain in the loop
K_loop = K_s * K_m;
fprintf('Sensor×Motor gain:  K_loop = K_s × K_m = %.4f\n', K_loop);

% Total DC gain constant
K_total = K_s * Kp * K_m * v_fwd / wheelBase;
fprintf('Total loop gain:    K = K_s × Kp × K_m × v/(L) = %.4f\n', K_total);

% Open-loop  L(s) = K_s × Kp × K_m × G_p(s) = K_total / s²
G_open = K_loop * C_p * G_plant;
G_open = minreal(G_open);

fprintf('\n--- Open-Loop Transfer Function  L(s) = K / s² ---\n');
disp(G_open);

% Closed-loop  T(s) = L(s) / [1 + L(s)]  =  K / (s² + K)
G_closed = feedback(G_open, 1);
G_closed = minreal(G_closed);

fprintf('--- Closed-Loop Transfer Function  T(s) = K / (s² + K) ---\n');
disp(G_closed);

% Poles & zeros
fprintf('Open-loop  poles : '); fprintf('%.4f%+.4fj  ', [real(pole(G_open))'; imag(pole(G_open))']); fprintf('\n');
cl_p = pole(G_closed);
fprintf('Closed-loop poles: '); fprintf('%.4f%+.4fj  ', [real(cl_p)'; imag(cl_p)']); fprintf('\n');

% Natural frequency & damping
[wn_cl, zeta_cl] = damp(G_closed);
fprintf('\nClosed-loop natural frequencies & damping:\n');
for k = 1:length(wn_cl)
    fprintf('  ωn = %8.4f rad/s,  ζ = %8.4f\n', wn_cl(k), zeta_cl(k));
end

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║  NOTE: P-only on a double integrator gives ζ = 0       ║\n');
fprintf('║  (marginally stable — sustained oscillations).         ║\n');
fprintf('║  The nonlinear sensor model adds effective damping     ║\n');
fprintf('║  that prevents divergence on the real robot.           ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n');

%% ===== 4. STABILITY ANALYSIS =====

% ---- Bode Plot ----
figure('Name','Bode Plot','NumberTitle','off','Position',[50 450 620 420]);
bode(G_open); grid on;
title('Open-Loop Bode Plot — L(s) = K/s²');

% ---- Gain & Phase Margins ----
figure('Name','Margins','NumberTitle','off','Position',[690 450 620 420]);
margin(G_open); grid on;

[Gm, Pm, Wgm, Wpm] = margin(G_open);
fprintf('\nGain  Margin : %.2f dB  (at ω = %.2f rad/s)\n', 20*log10(Gm), Wgm);
fprintf('Phase Margin : %.2f °   (at ω = %.2f rad/s)\n', Pm, Wpm);

% ---- Root Locus ----
figure('Name','Root Locus','NumberTitle','off','Position',[50 20 620 380]);
rlocus(G_open); grid on;
title('Root Locus — L(s) = K/s²');
% Annotate: poles start at origin, move to imaginary axis
fprintf('\nRoot Locus: two poles at origin → move along imaginary axis.\n');
fprintf('All closed-loop poles are purely imaginary → marginally stable.\n');

% ---- Nyquist ----
figure('Name','Nyquist','NumberTitle','off','Position',[690 20 620 380]);
nyquist(G_open); grid on;
title('Nyquist Diagram — L(s) = K/s²');

% ---- Pole-Zero Map ----
figure('Name','Pole-Zero Map','NumberTitle','off','Position',[370 240 400 320]);
pzmap(G_closed);
title('Closed-Loop Pole-Zero Map');
grid on;

%% ===== 5. STEP RESPONSE & TRANSIENT SPECIFICATIONS =====
% A step = sudden lateral shift of the line track.
% P-only on double integrator → sustained oscillation (no steady state).

figure('Name','Step Response','NumberTitle','off','Position',[370 20 500 400]);
t_step = linspace(0, 20, 5000);
[y_step, t_step] = step(G_closed, t_step);
plot(t_step, y_step, 'b-', 'LineWidth', 1.5);
hold on;
yline(0.01, 'r--', 'Reference = 0.01 m', 'LineWidth', 1);
hold off;
title('P-Only Step Response  (sustained oscillation expected)');
xlabel('Time (s)'); ylabel('Lateral Position (m)');
grid on;

% Step info (may show large/infinite settling for marginally stable)
try
    info = stepinfo(G_closed);
    fprintf('\n--- Step Response Characteristics ---\n');
    fprintf('  Rise time     : %.4f s\n', info.RiseTime);
    fprintf('  Overshoot     : %.2f %%\n', info.Overshoot);
    fprintf('  Peak time     : %.4f s\n', info.PeakTime);
    fprintf('  Peak value    : %.6f m\n', info.Peak);
    if isfinite(info.SettlingTime)
        fprintf('  Settling time : %.4f s\n', info.SettlingTime);
    else
        fprintf('  Settling time : ∞  (marginally stable — never settles)\n');
    end
catch
    fprintf('\n  Step response does not settle (marginally stable system).\n');
end

%% ===== 6. Kp GAIN SENSITIVITY (P-only sweep) =====

fprintf('\n===== Kp SENSITIVITY SWEEP =====\n');

figure('Name','Kp Sweep','NumberTitle','off','Position',[100 100 700 450]);
Kp_vals = [1 2 5 10 20 50];
t_sweep = linspace(0, 15, 3000);
hold on;
legs = {};
for i = 1:numel(Kp_vals)
    K_i = K_s * Kp_vals(i) * K_m;
    G_ol_i = tf(K_i * v_fwd, [wheelBase 0 0]);
    T_i = feedback(G_ol_i, 1);
    [y_i, ~] = step(T_i, t_sweep);
    plot(t_sweep, y_i, 'LineWidth', 1.5);
    legs{end+1} = sprintf('Kp = %g  (ωn = %.2f rad/s)', ...
        Kp_vals(i), sqrt(K_s*Kp_vals(i)*K_m*v_fwd/wheelBase));  %#ok<AGROW>

    fprintf('  Kp = %5.1f  →  ωn = %.3f rad/s,  T_osc = %.3f s\n', ...
        Kp_vals(i), ...
        sqrt(K_s*Kp_vals(i)*K_m*v_fwd/wheelBase), ...
        2*pi/sqrt(K_s*Kp_vals(i)*K_m*v_fwd/wheelBase));
end
yline(0.01, 'k--', 'LineWidth', 1);
legend(legs, 'Location','best');
title('Effect of K_p on Step Response  (P-only, ζ = 0 for all)');
xlabel('Time (s)'); ylabel('y (m)');
grid on; hold off;

fprintf('\n  Higher Kp → faster oscillation but SAME zero damping.\n');
fprintf('  To add damping, increase physical friction or add Kd.\n');

%% ===== 7. IMPORT DATA & SYSTEM IDENTIFICATION (optional) =====
%  Uncomment AFTER running the nonlinear Simulink model or after
%  importing Arduino serial-log data.
%
%  Requirements:  System Identification Toolbox (skip if unavailable)

% --- Option A: From nonlinear Simulink simulation ---
% if exist('error_log','var') && exist('correction_log','var')
%     dt = 0.001;
%     data = iddata(error_log, correction_log, dt);
%     data.InputName  = 'Correction';
%     data.OutputName = 'Error';
%
%     % Estimate 2nd-order TF from data
%     sys_est = tfest(data, 2, 0);   % 2 poles, 0 zeros (matches K/s²)
%     fprintf('\n--- Identified TF from Simulation Data ---\n');
%     disp(sys_est);
%
%     figure('Name','System ID Comparison');
%     compare(data, sys_est);
%     title('Identified Model vs Simulation Data');
% end

% --- Option B: From Arduino serial CSV ---
%     csvData = readmatrix('serial_log.csv');
%     t_raw   = csvData(:,1);          % time column (s)
%     err_raw = csvData(:,2);          % error column
%     cor_raw = csvData(:,3);          % correction column
%
%     dt_raw  = mean(diff(t_raw));
%     data    = iddata(err_raw, cor_raw, dt_raw);
%     sys_hw  = tfest(data, 2, 0);
%     fprintf('\n--- Identified TF from Hardware Data ---\n');
%     disp(sys_hw);

%% ===== SUMMARY =====
fprintf('\n============================================================\n');
fprintf('   ANALYSIS COMPLETE — %d figures generated\n', length(findall(0,'Type','figure')));
fprintf('============================================================\n');
fprintf('\nKey Transfer Functions (copy for report):\n');
fprintf('  Sensor gain:   K_s    = %.2f  m⁻¹\n', K_s);
fprintf('  Controller:    C(s)   = Kp = %.1f\n', Kp);
fprintf('  Motor gain:    K_m    = %.6f  (m/s)/PWM\n', K_m);
fprintf('  Plant:         G_p(s) = %.4f / s²\n', v_fwd/wheelBase);
fprintf('  Open-loop:     L(s)   = %.4f / s²\n', K_total);
fprintf('  Closed-loop:   T(s)   = %.4f / (s² + %.4f)\n', K_total, K_total);
fprintf('  Nat. frequency: ωn    = %.4f rad/s\n', sqrt(K_total));
fprintf('  Damping ratio:  ζ     = 0  (P-only on double integrator)\n\n');
