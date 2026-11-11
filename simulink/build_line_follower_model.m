function build_line_follower_model()
%BUILD_LINE_FOLLOWER_MODEL  Closed-loop Line Follower — P-only control.
%
%   Replicates sensor processing from Line_Follower.ino with P-only gain.
%   Closed-loop plant: differential-drive robot + 8-sensor bar + line track.
%
%   Usage:
%     >> build_line_follower_model
%     Click RUN in Simulink.  Then:  plot_results

    modelName = 'LineFollowerPID';

    if bdIsLoaded(modelName), close_system(modelName, 0); end
    if exist([modelName '.slx'], 'file'), delete([modelName '.slx']); end

    new_system(modelName);
    open_system(modelName);

    %% Solver
    set_param(modelName, ...
        'SolverType','Fixed-step', 'Solver','FixedStepDiscrete', ...
        'FixedStep','0.001', 'StopTime','30', ...
        'SaveTime','on', 'TimeSaveName','tout', 'SaveOutput','on');

    %% Model Workspace (from configuration.h)
    hws = get_param(modelName, 'ModelWorkspace');
    hws.assignin('Kp',        5.0);
    hws.assignin('baseSpeed', 90);

    %% Add blocks
    addAllBlocks(modelName);
    save_system(modelName);

    %% Set MATLAB Function scripts
    setAllScripts(modelName);
    save_system(modelName);
    pause(0.5);

    %% Connect
    connectAllBlocks(modelName);
    try Simulink.BlockDiagram.arrangeSystem(modelName); catch, end
    save_system(modelName);

    fprintf('\n========================================\n');
    fprintf('  Model created: %s.slx  (P-only)\n', modelName);
    fprintf('  Tunable: Kp, baseSpeed\n');
    fprintf('========================================\n\n');
end

% =====================================================================
function addAllBlocks(m)

    % Unit Delays (robot state)
    add_block('simulink/Discrete/Unit Delay',[m '/Delay_X'],     'Position',[80 170 120 210],'InitialCondition','0','SampleTime','-1');
    add_block('simulink/Discrete/Unit Delay',[m '/Delay_Y'],     'Position',[80 250 120 290],'InitialCondition','0','SampleTime','-1');
    add_block('simulink/Discrete/Unit Delay',[m '/Delay_Theta'], 'Position',[80 330 120 370],'InitialCondition','0','SampleTime','-1');

    % MATLAB Functions
    add_block('simulink/User-Defined Functions/MATLAB Function',[m '/SensorModel'],    'Position',[220 190 380 350]);
    add_block('simulink/User-Defined Functions/MATLAB Function',[m '/P_Controller'],   'Position',[470 160 640 310]);
    add_block('simulink/User-Defined Functions/MATLAB Function',[m '/PlantModel'],     'Position',[470 470 640 620]);

    % Constants
    add_block('simulink/Sources/Constant',[m '/Kp_Gain'],   'Position',[370 70 430 90],  'Value','Kp');
    add_block('simulink/Sources/Constant',[m '/Base_Speed'],'Position',[370 110 430 130],'Value','baseSpeed');

    % Mux / visualisation
    add_block('simulink/Signal Routing/Mux',[m '/Mux_Motors'],'Position',[730 180 735 260],'Inputs','2');
    add_block('simulink/Signal Routing/Mux',[m '/Mux_Pos'],  'Position',[730 480 735 580],'Inputs','3');

    add_block('simulink/Sinks/Scope', [m '/Scope_Error'],  'Position',[780 300 820 340]);
    add_block('simulink/Sinks/Scope', [m '/Scope_Motors'], 'Position',[800 200 840 240]);
    add_block('simulink/Sinks/XY Graph',[m '/XY_Trajectory'],'Position',[250 470 340 550],'xmin','-0.5','xmax','4','ymin','-0.3','ymax','0.3');

    add_block('simulink/Sinks/To Workspace',[m '/Log_Error'],   'Position',[780 360 850 390],'VariableName','error_log',     'SaveFormat','Array');
    add_block('simulink/Sinks/To Workspace',[m '/Log_Motors'],  'Position',[800 260 870 290],'VariableName','motor_log',     'SaveFormat','Array');
    add_block('simulink/Sinks/To Workspace',[m '/Log_Position'],'Position',[800 500 870 530],'VariableName','position_log',  'SaveFormat','Array');
    add_block('simulink/Sinks/To Workspace',[m '/Log_Sensors'], 'Position',[420 400 490 430],'VariableName','sensor_log',    'SaveFormat','Array');
    add_block('simulink/Sinks/To Workspace',[m '/Log_Correction'],'Position',[780 420 850 450],'VariableName','correction_log','SaveFormat','Array');
end

% =====================================================================
function setAllScripts(m)
    rt = sfroot;

    ch = rt.find('-isa','Stateflow.EMChart','Path',[m '/SensorModel']);
    ch.Script = getSensorModelScript();

    ch = rt.find('-isa','Stateflow.EMChart','Path',[m '/P_Controller']);
    ch.Script = getPControllerScript();

    ch = rt.find('-isa','Stateflow.EMChart','Path',[m '/PlantModel']);
    ch.Script = getPlantModelScript();
end

% =====================================================================
function connectAllBlocks(m)
    ar = {'autorouting','on'};

    % Feedback loop
    add_line(m,'PlantModel/1','Delay_X/1',ar{:});
    add_line(m,'PlantModel/2','Delay_Y/1',ar{:});
    add_line(m,'PlantModel/3','Delay_Theta/1',ar{:});

    add_line(m,'Delay_X/1',    'SensorModel/1',ar{:});
    add_line(m,'Delay_Y/1',    'SensorModel/2',ar{:});
    add_line(m,'Delay_Theta/1','SensorModel/3',ar{:});

    add_line(m,'Delay_X/1',    'PlantModel/1',ar{:});
    add_line(m,'Delay_Y/1',    'PlantModel/2',ar{:});
    add_line(m,'Delay_Theta/1','PlantModel/3',ar{:});

    % Sensor → P Controller
    add_line(m,'SensorModel/1','P_Controller/1',ar{:});

    % Constants → P Controller  (port 2 = Kp, port 3 = baseSpeed)
    add_line(m,'Kp_Gain/1',   'P_Controller/2',ar{:});
    add_line(m,'Base_Speed/1','P_Controller/3',ar{:});

    % P Controller → Plant
    add_line(m,'P_Controller/1','PlantModel/4',ar{:});
    add_line(m,'P_Controller/2','PlantModel/5',ar{:});

    % Visualisation
    add_line(m,'P_Controller/3','Scope_Error/1',ar{:});
    add_line(m,'P_Controller/1','Mux_Motors/1',ar{:});
    add_line(m,'P_Controller/2','Mux_Motors/2',ar{:});
    add_line(m,'Mux_Motors/1',  'Scope_Motors/1',ar{:});
    add_line(m,'Delay_X/1',    'XY_Trajectory/1',ar{:});
    add_line(m,'Delay_Y/1',    'XY_Trajectory/2',ar{:});

    % Logging
    add_line(m,'P_Controller/3','Log_Error/1',ar{:});
    add_line(m,'P_Controller/4','Log_Correction/1',ar{:});
    add_line(m,'Mux_Motors/1',  'Log_Motors/1',ar{:});
    add_line(m,'PlantModel/1',  'Mux_Pos/1',ar{:});
    add_line(m,'PlantModel/2',  'Mux_Pos/2',ar{:});
    add_line(m,'PlantModel/3',  'Mux_Pos/3',ar{:});
    add_line(m,'Mux_Pos/1',    'Log_Position/1',ar{:});
    add_line(m,'SensorModel/1','Log_Sensors/1',ar{:});
end

% =====================================================================
%  SENSOR MODEL  (unchanged — simulates 8-channel QTR bar)
% =====================================================================
function s = getSensorModelScript()
    s = strjoin({
    'function sensorValues = SensorModel(x, y, theta)'
    '%#codegen'
    '    A      = 0.05;'
    '    lambda = 2.0;'
    '    sensorSpacing  = 0.009;'
    '    lineHalfWidth  = 0.010;'
    '    forwardOffset  = 0.06;'
    '    offsets = [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5] * sensorSpacing;'
    '    sensorValues = zeros(8, 1);'
    '    for i = 1:8'
    '        sx = x + forwardOffset * cos(theta) - offsets(i) * sin(theta);'
    '        sy = y + forwardOffset * sin(theta) + offsets(i) * cos(theta);'
    '        y_track = A * sin(2 * pi * sx / lambda);'
    '        dist = abs(sy - y_track);'
    '        if dist < lineHalfWidth'
    '            sensorValues(i) = 100 + (dist / lineHalfWidth) * 600;'
    '        else'
    '            sensorValues(i) = 1000;'
    '        end'
    '    end'
    'end'
    }, newline);
end

% =====================================================================
%  P-ONLY CONTROLLER  (same sensor processing, only proportional gain)
% =====================================================================
function s = getPControllerScript()
    s = strjoin({
    'function [leftMotorSpeed, rightMotorSpeed, error_out, correction_out] = P_Controller(sensorValues, Kp, baseSpeed)'
    '%#codegen'
    '% ======================================================='
    '%  P-only controller — same sensor processing as Arduino'
    '%  correction = Kp * error   (no I, no D)'
    '% ======================================================='
    ''
    '    persistent prevError'
    '    if isempty(prevError)'
    '        prevError = 0;'
    '    end'
    ''
    '    % ===== SENSOR PROCESSING (from Arduino code) ====='
    '    weights = [-3, -2, -1, 0, 0, 1, 2, 3];'
    '    position      = 0;'
    '    activeSensors = 0;'
    ''
    '    for i = 1:8'
    '        if sensorValues(i) < 900'
    '            binary = 1;'
    '        else'
    '            binary = 0;'
    '        end'
    '        if binary > 0'
    '            activeSensors = activeSensors + 1;'
    '        end'
    '        position = position + binary * weights(i);'
    '    end'
    ''
    '    % Lost line protection (uses previous error direction)'
    '    if activeSensors == 0'
    '        if prevError > 0'
    '            position = 12;'
    '        else'
    '            position = -12;'
    '        end'
    '    end'
    ''
    '    ir_error  = position / 2.0;'
    '    error_val = ir_error;'
    ''
    '    % ===== P-ONLY CONTROL ====='
    '    correction = Kp * error_val;'
    ''
    '    prevError = error_val;'
    ''
    '    % ===== MOTOR SPEEDS ====='
    '    leftMotorSpeed  = floor(baseSpeed - correction);'
    '    rightMotorSpeed = floor(baseSpeed + correction);'
    '    leftMotorSpeed  = max(-255, min(255, leftMotorSpeed));'
    '    rightMotorSpeed = max(-255, min(255, rightMotorSpeed));'
    ''
    '    error_out      = error_val;'
    '    correction_out = correction;'
    'end'
    }, newline);
end

% =====================================================================
%  PLANT MODEL  (unchanged — differential-drive kinematics)
% =====================================================================
function s = getPlantModelScript()
    s = strjoin({
    'function [new_x, new_y, new_theta] = PlantModel(old_x, old_y, old_theta, leftPWM, rightPWM)'
    '%#codegen'
    '    dt        = 0.001;'
    '    maxSpeed  = 0.30;'
    '    wheelBase = 0.15;'
    '    v_left  = (leftPWM  / 255.0) * maxSpeed;'
    '    v_right = (rightPWM / 255.0) * maxSpeed;'
    '    v     = (v_left + v_right) / 2.0;'
    '    omega = (v_right - v_left) / wheelBase;'
    '    new_x     = old_x     + v * cos(old_theta) * dt;'
    '    new_y     = old_y     + v * sin(old_theta) * dt;'
    '    new_theta = old_theta + omega * dt;'
    'end'
    }, newline);
end
