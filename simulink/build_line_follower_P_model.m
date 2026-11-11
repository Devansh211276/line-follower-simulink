function build_line_follower_P_model()
% BUILD_LINE_FOLLOWER_P_MODEL
% Continuous closed-loop line follower model using P control.
%
% Model:
%
% Reference
%    |
%    v
%  (+) Sum (-)<-----------------------------+
%    |                                      |
%    v                                      |
% K_sensor                                  |
%    |                                      |
%    v                                      |
%   Kp                                       |
%    |                                       |
%    v                                       |
% K_motor                                    |
%    |                                       |
%    v                                       |
% Robot Plant                                |
% v/(L*s^2 + B*s)                            |
%    |                                       |
%    +-------------------- Position ---------+
%
% Continuous solver: ode45

    clc;

    modelName = 'LineFollower_P_Controller';

    %% ------------------------------------------------------------
    % Close/delete old model
    % -------------------------------------------------------------
    
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end

    if exist([modelName '.slx'], 'file')
        delete([modelName '.slx']);
    end

    %% ------------------------------------------------------------
    % Create model
    % -------------------------------------------------------------
    
    new_system(modelName);
    open_system(modelName);

    %% ------------------------------------------------------------
    % Solver configuration
    % -------------------------------------------------------------
    
    set_param(modelName, ...
        'SolverType', 'Variable-step', ...
        'Solver', 'ode45', ...
        'StopTime', '10', ...
        'MaxStep', '0.01', ...
        'SaveTime', 'on', ...
        'TimeSaveName', 'tout');

    %% ------------------------------------------------------------
    % Model parameters
    % -------------------------------------------------------------
    
    hws = get_param(modelName, 'ModelWorkspace');

    % P controller
    Kp = 5;

    % Sensor scaling
    K_sensor = 1/(2*0.009);

    % Motor conversion
    K_motor = 2*0.30/255;

    % Forward velocity
    v_fwd = (90/255)*0.30;

    % Robot wheelbase
    L = 0.15;

    % Small damping term
    B = 0.05;

    % Store parameters
    hws.assignin('Kp', Kp);
    hws.assignin('K_sensor', K_sensor);
    hws.assignin('K_motor', K_motor);
    hws.assignin('v_fwd', v_fwd);
    hws.assignin('L', L);
    hws.assignin('B', B);

    %% ------------------------------------------------------------
    % Reference step
    % -------------------------------------------------------------
    
    add_block( ...
        'simulink/Sources/Step', ...
        [modelName '/Reference'], ...
        'Position', [30 180 70 210], ...
        'Time', '0.5', ...
        'Before', '0', ...
        'After', '0.01', ...
        'SampleTime', '0');

    %% ------------------------------------------------------------
    % Sum block: e = reference - feedback
    % -------------------------------------------------------------
    
    add_block( ...
        'simulink/Math Operations/Sum', ...
        [modelName '/Error_Sum'], ...
        'Position', [130 177 160 213], ...
        'Inputs', '+-');

    %% ------------------------------------------------------------
    % Sensor gain
    % -------------------------------------------------------------
    
    add_block( ...
        'simulink/Math Operations/Gain', ...
        [modelName '/K_sensor'], ...
        'Position', [210 180 280 210], ...
        'Gain', 'K_sensor');

    %% ------------------------------------------------------------
    % P controller
    % -------------------------------------------------------------
    
    add_block( ...
        'simulink/Math Operations/Gain', ...
        [modelName '/Kp'], ...
        'Position', [330 180 400 210], ...
        'Gain', 'Kp');

    %% ------------------------------------------------------------
    % Motor gain
    % -------------------------------------------------------------
    
    add_block( ...
        'simulink/Math Operations/Gain', ...
        [modelName '/K_motor'], ...
        'Position', [450 180 520 210], ...
        'Gain', 'K_motor');

    %% ------------------------------------------------------------
    % Robot plant
    %
    % G(s) = v_fwd / (L*s^2 + B*s)
    %
    % denominator = [L B 0]
    % -------------------------------------------------------------
    
    add_block( ...
        'simulink/Continuous/Transfer Fcn', ...
        [modelName '/Robot_Plant'], ...
        'Position', [570 170 690 220], ...
        'Numerator', 'v_fwd', ...
        'Denominator', '[L B 0]');

    %% ------------------------------------------------------------
    % Scope
    % -------------------------------------------------------------
    
    add_block( ...
        'simulink/Sinks/Scope', ...
        [modelName '/Scope'], ...
        'Position', [780 170 850 220]);

    %% ------------------------------------------------------------
    % To Workspace
    % -------------------------------------------------------------
    
    add_block( ...
        'simulink/Sinks/To Workspace', ...
        [modelName '/Position_Output'], ...
        'Position', [780 260 900 300], ...
        'VariableName', 'position', ...
        'SaveFormat', 'Structure With Time');

    %% ------------------------------------------------------------
    % Display
    % -------------------------------------------------------------
    
    add_block( ...
        'simulink/Sinks/Display', ...
        [modelName '/Position_Display'], ...
        'Position', [780 100 850 130]);

    %% ------------------------------------------------------------
    % Connect forward path
    % -------------------------------------------------------------
    
    add_line(modelName, ...
        'Reference/1', ...
        'Error_Sum/1', ...
        'autorouting', 'on');

    add_line(modelName, ...
        'Error_Sum/1', ...
        'K_sensor/1', ...
        'autorouting', 'on');

    add_line(modelName, ...
        'K_sensor/1', ...
        'Kp/1', ...
        'autorouting', 'on');

    add_line(modelName, ...
        'Kp/1', ...
        'K_motor/1', ...
        'autorouting', 'on');

    add_line(modelName, ...
        'K_motor/1', ...
        'Robot_Plant/1', ...
        'autorouting', 'on');

    %% ------------------------------------------------------------
    % Plant output to Scope
    % -------------------------------------------------------------
    
    add_line(modelName, ...
        'Robot_Plant/1', ...
        'Scope/1', ...
        'autorouting', 'on');

    %% ------------------------------------------------------------
    % Plant output to Workspace
    % -------------------------------------------------------------
    
    add_line(modelName, ...
        'Robot_Plant/1', ...
        'Position_Output/1', ...
        'autorouting', 'on');

    %% ------------------------------------------------------------
    % Plant output to Display
    % -------------------------------------------------------------
    
    add_line(modelName, ...
        'Robot_Plant/1', ...
        'Position_Display/1', ...
        'autorouting', 'on');

    %% ------------------------------------------------------------
    % Feedback
    % -------------------------------------------------------------
    
    add_line(modelName, ...
        'Robot_Plant/1', ...
        'Error_Sum/2', ...
        'autorouting', 'on');

    %% ------------------------------------------------------------
    % Add annotations
    % -------------------------------------------------------------
    
    add_block( ...
        'built-in/Note', ...
        [modelName '/Reference_Note'], ...
        'Position', [20 130 150 150], ...
        'Text', 'Desired Line Position');

    add_block( ...
        'built-in/Note', ...
        [modelName '/Controller_Note'], ...
        'Position', [315 130 410 150], ...
        'Text', 'P Controller');

    add_block( ...
        'built-in/Note', ...
        [modelName '/Plant_Note'], ...
        'Position', [560 130 710 150], ...
        'Text', 'Continuous Robot Dynamics');

    %% ------------------------------------------------------------
    % Save model
    % -------------------------------------------------------------
    
    save_system(modelName, [modelName '.slx']);

    %% ------------------------------------------------------------
    % Display model
    % -------------------------------------------------------------
    
    open_system(modelName);

    fprintf('\n');
    fprintf('============================================\n');
    fprintf(' Line Follower P-Control Model Created\n');
    fprintf('============================================\n');
    fprintf('Model: %s.slx\n', modelName);
    fprintf('Kp       = %.4f\n', Kp);
    fprintf('K_sensor = %.4f\n', K_sensor);
    fprintf('K_motor  = %.6f\n', K_motor);
    fprintf('v_fwd    = %.4f m/s\n', v_fwd);
    fprintf('L        = %.4f m\n', L);
    fprintf('B        = %.4f\n', B);
    fprintf('============================================\n');
    fprintf('Click RUN in Simulink to simulate.\n');
    fprintf('============================================\n');

end
