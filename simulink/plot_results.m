%PLOT_RESULTS  Visualise Line Follower simulation results.
%
%  Run this AFTER simulating the LineFollowerPID model.
%  It reads the To Workspace variables logged during simulation.
%
%  Usage:
%    >> sim('LineFollowerPID');   % or click Run in Simulink
%    >> plot_results

%% ===== LOAD LOGGED DATA =====
if ~exist('tout', 'var')
    error('No simulation data found. Run the LineFollowerPID model first.');
end

t = tout;                       % time vector

% Position (x, y, theta)
pos = position_log;             % N×3 matrix
x     = pos(:,1);
y     = pos(:,2);
theta = pos(:,3);

% PID error
err = error_log;                % N×1

% Correction
cor = correction_log;           % N×1

% Motor speeds
mot = motor_log;                % N×2  (left, right)
leftM  = mot(:,1);
rightM = mot(:,2);

% Sensor readings (8 channels)
sens = sensor_log;              % N×8

%% ===== GENERATE LINE TRACK FOR REFERENCE =====
A      = 0.05;      % Must match SensorModel parameters
lambda = 2.0;
xtrack = linspace(min(x)-0.1, max(x)+0.1, 1000);
ytrack = A * sin(2*pi*xtrack / lambda);

%% ===== FIGURE 1: ROBOT TRAJECTORY vs LINE TRACK =====
figure('Name', 'Robot Trajectory', 'NumberTitle', 'off');

plot(xtrack, ytrack, 'k--', 'LineWidth', 2, 'DisplayName', 'Line Track');
hold on;
plot(x, y, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Robot Path');
hold off;

xlabel('X (m)');
ylabel('Y (m)');
title('Closed-Loop Trajectory: Robot vs Line Track');
legend('Location', 'best');
axis equal; grid on;

%% ===== FIGURE 2: PID ERROR & CORRECTION =====
figure('Name', 'PID Signals', 'NumberTitle', 'off');

subplot(2,1,1);
plot(t, err, 'r-', 'LineWidth', 1);
ylabel('Error');
title('PID Error over Time');
grid on;

subplot(2,1,2);
plot(t, cor, 'm-', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Correction');
title('PID Correction over Time');
grid on;

%% ===== FIGURE 3: MOTOR SPEEDS =====
figure('Name', 'Motor Speeds', 'NumberTitle', 'off');

plot(t, leftM, 'b-', 'LineWidth', 1, 'DisplayName', 'Left Motor');
hold on;
plot(t, rightM, 'r-', 'LineWidth', 1, 'DisplayName', 'Right Motor');
yline(0, 'k--', 'LineWidth', 0.5);
hold off;

xlabel('Time (s)');
ylabel('PWM (-255 to 255)');
title('Motor Speed Commands');
legend('Location', 'best');
grid on;

%% ===== FIGURE 4: SENSOR BAR HEATMAP =====
figure('Name', 'Sensor Readings', 'NumberTitle', 'off');

% Downsample for readability (every 100th sample = 10 Hz view)
step = max(1, floor(size(sens,1)/3000));
sensDS = sens(1:step:end, :);
tDS    = t(1:step:end);

imagesc(tDS, 1:8, sensDS');
colorbar;
caxis([0 1100]);
colormap(flipud(gray));
xlabel('Time (s)');
ylabel('Sensor #');
title('Sensor Bar Readings (dark = on line)');
set(gca, 'YDir', 'normal');

%% ===== FIGURE 5: HEADING ANGLE =====
figure('Name', 'Heading', 'NumberTitle', 'off');

plot(t, rad2deg(theta), 'g-', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Heading \theta (deg)');
title('Robot Heading Angle');
grid on;

fprintf('\n✅  All plots generated.\n');
