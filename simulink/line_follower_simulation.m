function line_follower_simulation(numSensors)
% LINE_FOLLOWER_SIMULATION (MATLAB R2024b Compatible)
% Continuous closed-loop line follower model with P-control.
%
% Architecture:
%   [S1..S8] -> [W1..W8] -> [Add Block] -> [Error Gain] -> [Kp Gain] 
%   -> [Conversion Block (Speed +- P_out)] -> [Lookup Tables & Motors] 
%   -> [Robot Simulator] -> (Continuous Closed-Loop Feedback)
%
% Usage:
%   >> line_follower_simulation      % Default: 8 sensors (physical hardware)
%   >> line_follower_simulation(6)   % 6-sensor version (reference diagram)

    if nargin < 1
        numSensors = 8; % Matches physical 8-channel QTR array (A0..A7)
    end

    modelName = sprintf('LineFollower_%dSensors_P', numSensors);

    clc;
    fprintf('===============================================================\n');
    fprintf(' 1. CREATING MODEL: %s.slx (MATLAB R2024b)\n', modelName);
    fprintf('===============================================================\n');

    %% Step 1: Clean up old model if open or exists
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    if exist([modelName '.slx'], 'file')
        delete([modelName '.slx']);
    end

    %% Step 2: Create new model & configure continuous solver (ode45)
    new_system(modelName);
    open_system(modelName);

    set_param(modelName, ...
        'SolverType',   'Variable-step', ...
        'Solver',       'ode45', ...
        'StopTime',     '10', ...
        'MaxStep',      '0.005', ...
        'SaveTime',     'on', ...
        'TimeSaveName', 'tout');

    %% Step 3: Configure Model Workspace Parameters
    hws = get_param(modelName, 'ModelWorkspace');
    hws.assignin('Kp', 5.0);           % Proportional Gain
    hws.assignin('baseSpeed', 0.08);   % Constant speed (0.08 from diagram)
    hws.assignin('errorGain', 0.001);  % Error gain (0.001 from diagram)
    hws.assignin('wheelRadius', 0.033);% Wheel radius: 33mm
    hws.assignin('wheelBase', 0.150);  % Wheel base: 150mm

    %% Step 4: Symmetric Sensor Weights
    if numSensors == 8
        weights = [350, 250, 150, 50, -50, -150, -250, -350]; % 8 sensors
    elseif numSensors == 6
        weights = [300, 200, 100, -100, -200, -300];           % 6 sensors
    else
        weights = linspace(100*(numSensors/2), -100*(numSensors/2), numSensors);
    end

    %% Step 5: Add Sensors, Weights, Add Block, and Sensor Scope
    yStart = 50;
    ySpacing = 55;

    % Multi-port Sensor Scope
    add_block('simulink/Sinks/Scope', [modelName '/Sensor_Scope'], ...
        'Position', [220, yStart + numSensors*ySpacing + 20, 260, yStart + numSensors*ySpacing + 70], ...
        'NumInputPorts', num2str(numSensors));

    % Add Block (sums all weighted sensor outputs)
    addSigns = repmat('+', 1, numSensors);
    add_block('simulink/Math Operations/Sum', [modelName '/Add_Block'], ...
        'Position', [340, yStart + 20, 375, yStart + numSensors*ySpacing - 10], ...
        'Inputs', addSigns);

    for i = 1:numSensors
        yPos = yStart + (i-1)*ySpacing;
        
        % Continuous sensor subsystem (Gaussian optical reflectance)
        sensorPath = sprintf('%s/S%d_Sensor_Simulation', modelName, i);
        add_block('built-in/Subsystem', sensorPath, ...
            'Position', [40, yPos, 140, yPos + 35]);
        buildSensorSubsystem(sensorPath, i, numSensors);

        % Sensor Gain (W1 .. W8)
        gainPath = sprintf('%s/W%d', modelName, i);
        add_block('simulink/Math Operations/Gain', gainPath, ...
            'Position', [200, yPos, 255, yPos + 35], ...
            'Gain', num2str(weights(i)));
        
        % Connect Sensor -> Gain -> Add Block
        add_line(modelName, sprintf('S%d_Sensor_Simulation/1', i), sprintf('W%d/1', i), 'autorouting', 'on');
        add_line(modelName, sprintf('S%d_Sensor_Simulation/1', i), sprintf('Sensor_Scope/%d', i), 'autorouting', 'on');
        add_line(modelName, sprintf('W%d/1', i), sprintf('Add_Block/%d', i), 'autorouting', 'on');
    end

    %% Step 6: Error Gain, P-Controller, and Monitoring Scopes
    centerY = yStart + (numSensors*ySpacing)/2 - 15;

    % Error Gain (0.001)
    add_block('simulink/Math Operations/Gain', [modelName '/Error_Gain'], ...
        'Position', [420, centerY - 15, 470, centerY + 15], ...
        'Gain', 'errorGain');
    add_line(modelName, 'Add_Block/1', 'Error_Gain/1', 'autorouting', 'on');

    % Error Scope
    add_block('simulink/Sinks/Scope', [modelName '/Error_Scope'], ...
        'Position', [435, centerY - 80, 470, centerY - 45]);
    add_line(modelName, 'Add_Block/1', 'Error_Scope/1', 'autorouting', 'on');

    % P Controller Gain (Kp = 5.0)
    add_block('simulink/Math Operations/Gain', [modelName '/P_Controller'], ...
        'Position', [520, centerY - 20, 575, centerY + 20], ...
        'Gain', 'Kp');
    add_line(modelName, 'Error_Gain/1', 'P_Controller/1', 'autorouting', 'on');

    % P Output Scope
    add_block('simulink/Sinks/Scope', [modelName '/P_Output_Scope'], ...
        'Position', [590, centerY - 80, 625, centerY - 45]);
    add_line(modelName, 'P_Controller/1', 'P_Output_Scope/1', 'autorouting', 'on');

    %% Step 7: Speed Constant & Conversion Block (Differential Steering Mixer)
    add_block('simulink/Sources/Constant', [modelName '/Speed'], ...
        'Position', [670, centerY - 85, 715, centerY - 55], ...
        'Value', 'baseSpeed');

    convPath = [modelName '/Conversion_Block'];
    add_block('built-in/Subsystem', convPath, ...
        'Position', [740, centerY - 50, 785, centerY + 50]);
    buildConversionSubsystem(convPath);

    add_line(modelName, 'Speed/1', 'Conversion_Block/1', 'autorouting', 'on');
    add_line(modelName, 'P_Controller/1', 'Conversion_Block/2', 'autorouting', 'on');

    %% Step 8: Lookup Tables & Motors
    % Left Motor
    add_block('simulink/Lookup Tables/1-D Lookup Table', [modelName '/Left_Lookup_Table'], ...
        'Position', [830, centerY - 75, 875, centerY - 40], ...
        'Table', '[-1 0 1]', 'BreakpointsForDimension1', '[-1 0 1]');
    add_block('simulink/Continuous/Transfer Fcn', [modelName '/Left_Motor'], ...
        'Position', [905, centerY - 80, 965, centerY - 35], ...
        'Numerator', '[10]', 'Denominator', '[0.05 1]');

    add_line(modelName, 'Conversion_Block/1', 'Left_Lookup_Table/1', 'autorouting', 'on');
    add_line(modelName, 'Left_Lookup_Table/1', 'Left_Motor/1', 'autorouting', 'on');

    % Right Motor
    add_block('simulink/Lookup Tables/1-D Lookup Table', [modelName '/Right_Lookup_Table'], ...
        'Position', [830, centerY + 30, 875, centerY + 65], ...
        'Table', '[-1 0 1]', 'BreakpointsForDimension1', '[-1 0 1]');
    add_block('simulink/Continuous/Transfer Fcn', [modelName '/Right_Motor'], ...
        'Position', [905, centerY + 25, 965, centerY + 70], ...
        'Numerator', '[10]', 'Denominator', '[0.05 1]');

    add_line(modelName, 'Conversion_Block/2', 'Right_Lookup_Table/1', 'autorouting', 'on');
    add_line(modelName, 'Right_Lookup_Table/1', 'Right_Motor/1', 'autorouting', 'on');

    %% Step 9: Robot Simulator & Closed-Loop Feedback
    robotPath = [modelName '/Robot_Simulator'];
    add_block('built-in/Subsystem', robotPath, ...
        'Position', [1020, centerY - 85, 1110, centerY + 95]);
    buildRobotSimulator(robotPath);

    add_line(modelName, 'Left_Motor/1', 'Robot_Simulator/1', 'autorouting', 'on');
    add_line(modelName, 'Right_Motor/1', 'Robot_Simulator/2', 'autorouting', 'on');

    % Closed-Loop Feedback: Connect Robot_Simulator output back to all 8 sensors
    for i = 1:numSensors
        add_line(modelName, 'Robot_Simulator/1', sprintf('S%d_Sensor_Simulation/1', i), 'autorouting', 'on');
    end

    % To Workspace for Trajectory plotting
    add_block('simulink/Sinks/To Workspace', [modelName '/Pose_Log'], ...
        'Position', [1140, centerY - 15, 1200, centerY + 15], ...
        'VariableName', 'pose_log', 'SaveFormat', 'Array');
    add_line(modelName, 'Robot_Simulator/1', 'Pose_Log/1', 'autorouting', 'on');

    try
        Simulink.BlockDiagram.arrangeSystem(modelName);
    catch
    end

    save_system(modelName, [modelName '.slx']);
    fprintf(' Model "%s.slx" created successfully.\n\n', modelName);

    %% ===============================================================
    %% Step 10: RUN CONTINUOUS SIMULATION & PLOT RESULTS
    %% ===============================================================
    fprintf('===============================================================\n');
    fprintf(' 2. RUNNING CONTINUOUS SIMULATION (ode45)...\n');
    fprintf('===============================================================\n');

    simOut = sim(modelName, 'StopTime', '10');

    fprintf(' Simulation completed successfully!\n');
    fprintf(' Opening scopes...\n');

    try
        open_system([modelName '/Sensor_Scope']);
        open_system([modelName '/Error_Scope']);
        open_system([modelName '/P_Output_Scope']);
    catch
    end

    % Plot Trajectory
    if exist('pose_log', 'var') || isfield(simOut, 'pose_log')
        if isfield(simOut, 'pose_log')
            pos = simOut.pose_log;
        else
            pos = pose_log;
        end

        figure('Name', 'Robot Trajectory vs Track', 'NumberTitle', 'off');
        x_rob = pos(:, 1);
        y_rob = pos(:, 2);
        
        x_track = linspace(min(x_rob)-0.1, max(x_rob)+0.1, 500);
        y_track = 0.05 * sin(2*pi*x_track / 2.0);

        plot(x_track, y_track, 'k--', 'LineWidth', 2, 'DisplayName', 'Track Centerline');
        hold on;
        plot(x_rob, y_rob, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Robot Trajectory (P-Gain)');
        xlabel('X Position (m)');
        ylabel('Y Position (m)');
        title('Line Follower: Continuous Closed-Loop P Control (8 Sensors)');
        legend('Location', 'best');
        axis equal; grid on;
        fprintf(' Trajectory plot generated.\n');
    end

    fprintf('===============================================================\n');
    fprintf(' All done! Simulink model and plots ready.\n');
    fprintf('===============================================================\n');
end

%% ========================================================================
% HELPER SUBSYSTEMS
% ========================================================================

function buildSensorSubsystem(subsystemPath, sensorIdx, totalSensors)
    Simulink.SubSystem.deleteContents(subsystemPath);

    add_block('built-in/Inport', [subsystemPath '/Robot_Pose'], ...
        'Position', [20, 50, 50, 65]);
    add_block('simulink/User-Defined Functions/MATLAB Function', [subsystemPath '/Analog_Reflectance'], ...
        'Position', [100, 35, 230, 80]);
    add_block('built-in/Outport', [subsystemPath '/Line'], ...
        'Position', [280, 50, 310, 65]);

    add_line(subsystemPath, 'Robot_Pose/1', 'Analog_Reflectance/1');
    add_line(subsystemPath, 'Analog_Reflectance/1', 'Line/1');

    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [subsystemPath '/Analog_Reflectance']);
    offsetIndex = sensorIdx - (totalSensors + 1)/2;
    
    ch.Script = sprintf([
        'function intensity = Analog_Reflectance(pose)\n' ...
        '%% Continuous optical Gaussian reflectance\n' ...
        'x = pose(1);\n' ...
        'y = pose(2);\n' ...
        'theta = pose(3);\n' ...
        'pitch = 0.009; %% 9mm spacing\n' ...
        'd_offset = (%f) * pitch;\n' ...
        'forward_offset = 0.06;\n' ...
        'sx = x + forward_offset*cos(theta) - d_offset*sin(theta);\n' ...
        'sy = y + forward_offset*sin(theta) + d_offset*cos(theta);\n' ...
        'y_line = 0.05 * sin(2*pi*sx / 2.0);\n' ...
        'dist = abs(sy - y_line);\n' ...
        'sigma = 0.012;\n' ...
        'intensity = exp(- (dist^2) / (2 * sigma^2)); %% Continuous [0, 1]\n' ...
        'end\n'
    ], offsetIndex);
end

function buildConversionSubsystem(subsystemPath)
    Simulink.SubSystem.deleteContents(subsystemPath);

    add_block('built-in/Inport', [subsystemPath '/Speed_v'], 'Position', [20, 30, 50, 45]);
    add_block('built-in/Inport', [subsystemPath '/P_out_w'], 'Position', [20, 80, 50, 95]);

    add_block('simulink/Math Operations/Sum', [subsystemPath '/Sum_Left'], ...
        'Position', [120, 25, 140, 50], 'Inputs', '+-');
    add_block('simulink/Math Operations/Sum', [subsystemPath '/Sum_Right'], ...
        'Position', [120, 75, 140, 100], 'Inputs', '++');

    add_block('built-in/Outport', [subsystemPath '/wl'], 'Position', [190, 30, 220, 45]);
    add_block('built-in/Outport', [subsystemPath '/wr'], 'Position', [190, 80, 220, 95]);

    add_line(subsystemPath, 'Speed_v/1', 'Sum_Left/1');
    add_line(subsystemPath, 'P_out_w/1', 'Sum_Left/2');
    add_line(subsystemPath, 'Speed_v/1', 'Sum_Right/1');
    add_line(subsystemPath, 'P_out_w/1', 'Sum_Right/2');
    add_line(subsystemPath, 'Sum_Left/1', 'wl/1');
    add_line(subsystemPath, 'Sum_Right/1', 'wr/1');
end

function buildRobotSimulator(subsystemPath)
    Simulink.SubSystem.deleteContents(subsystemPath);

    % Inports
    add_block('built-in/Inport', [subsystemPath '/wl'], 'Position', [20, 35, 50, 50]);
    add_block('built-in/Inport', [subsystemPath '/wr'], 'Position', [20, 75, 50, 90]);

    % Mux to bundle (wl, wr, pose) into 1 vector input for Kinematics_ODE
    add_block('simulink/Signal Routing/Mux', [subsystemPath '/Mux_Inputs'], ...
        'Position', [100, 35, 105, 105], 'Inputs', '3');

    add_block('simulink/User-Defined Functions/MATLAB Function', [subsystemPath '/Kinematics_ODE'], ...
        'Position', [140, 40, 240, 100]);

    add_block('simulink/Continuous/Integrator', [subsystemPath '/Integrate_Pose'], ...
        'Position', [280, 50, 315, 90], 'InitialCondition', '[0; 0; 0]');

    add_block('built-in/Outport', [subsystemPath '/Pose_XYTheta'], ...
        'Position', [370, 60, 400, 75]);

    % Wire connections
    add_line(subsystemPath, 'wl/1', 'Mux_Inputs/1');
    add_line(subsystemPath, 'wr/1', 'Mux_Inputs/2');
    add_line(subsystemPath, 'Integrate_Pose/1', 'Mux_Inputs/3');
    
    add_line(subsystemPath, 'Mux_Inputs/1', 'Kinematics_ODE/1');
    add_line(subsystemPath, 'Kinematics_ODE/1', 'Integrate_Pose/1');
    add_line(subsystemPath, 'Integrate_Pose/1', 'Pose_XYTheta/1');

    % Set MATLAB Function script (Single vector input 'u')
    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [subsystemPath '/Kinematics_ODE']);
    ch.Script = sprintf([
        'function dPose = Kinematics_ODE(u)\n' ...
        '%% u = [wl; wr; x; y; theta]\n' ...
        'wl    = u(1);\n' ...
        'wr    = u(2);\n' ...
        'theta = u(5);\n' ...
        'R = 0.033;\n' ...
        'L = 0.150;\n' ...
        'v = (R/2) * (wr + wl);\n' ...
        'omega = (R/L) * (wr - wl);\n' ...
        'dPose = [v * cos(theta); v * sin(theta); omega];\n' ...
        'end\n'
    ]);
end