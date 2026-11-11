function build_gain_driven_model()
%BUILD_GAIN_DRIVEN_MODEL  Linearised P-only Simulink model with Gain blocks.
%
%  Block diagram:
%    Step → [Sum +/-] → [K_sensor] → [Kp] → [K_motor] → [Plant v/(Ls²)] → y
%                 ↑                                                          │
%                 └───────────────────── (−1) ◄──────────────────────────────┘
%
%  Every gain is an explicit, tunable Gain block.
%  Continuous solver (ode45).
%
%  Usage:
%    >> build_gain_driven_model
%    Click RUN in Simulink.

    modelName = 'LineFollowerPID_Linear';

    if bdIsLoaded(modelName), close_system(modelName, 0); end
    if exist([modelName '.slx'], 'file'), delete([modelName '.slx']); end

    new_system(modelName);
    open_system(modelName);

    %% Solver (continuous)
    set_param(modelName, ...
        'SolverType','Variable-step', 'Solver','ode45', ...
        'StopTime','10', 'MaxStep','0.01', ...
        'SaveTime','on', 'TimeSaveName','tout');

    %% Model Workspace
    hws = get_param(modelName, 'ModelWorkspace');
    hws.assignin('Kp',       5.0);
    hws.assignin('K_sensor', 1/(2*0.009));        % ≈ 55.56
    hws.assignin('K_motor',  2*0.30/255);          % ≈ 0.00235
    hws.assignin('v_fwd',    (90/255)*0.30);       % ≈ 0.1059
    hws.assignin('L',        0.15);

    %% Add blocks
    % Reference: 1 cm step at t = 0.5 s
    add_block('simulink/Sources/Step', [modelName '/Reference'], ...
        'Position',[30 175 60 205], ...
        'Time','0.5', 'Before','0', 'After','0.01', 'SampleTime','0');

    % Summing junction  e = r − y
    add_block('simulink/Math Operations/Sum', [modelName '/Sum_Error'], ...
        'Position',[120 178 140 202], 'Inputs','+-', 'IconShape','round');

    % Sensor gain  K_s
    add_block('simulink/Math Operations/Gain', [modelName '/K_Sensor'], ...
        'Position',[190 175 250 205], 'Gain','K_sensor');

    % Proportional gain  Kp
    add_block('simulink/Math Operations/Gain', [modelName '/Kp_Gain'], ...
        'Position',[300 175 360 205], 'Gain','Kp', 'ForegroundColor','blue');

    % Motor gain  K_m
    add_block('simulink/Math Operations/Gain', [modelName '/K_Motor'], ...
        'Position',[410 175 470 205], 'Gain','K_motor');

    % Plant  G(s) = v_fwd / (L s²)
    add_block('simulink/Continuous/Transfer Fcn', [modelName '/Plant_Gp'], ...
        'Position',[520 168 620 212], ...
        'Numerator','[v_fwd]', 'Denominator','[L 0 0]');

    % Scopes
    add_block('simulink/Sinks/Scope', [modelName '/Scope_Output'], ...
        'Position',[700 175 740 215]);
    add_block('simulink/Sinks/Scope', [modelName '/Scope_Error'], ...
        'Position',[200 110 240 140]);

    % To Workspace
    add_block('simulink/Sinks/To Workspace', [modelName '/Log_Output'], ...
        'Position',[700 240 770 270], 'VariableName','y_linear', 'SaveFormat','Array');
    add_block('simulink/Sinks/To Workspace', [modelName '/Log_Error'], ...
        'Position',[200 60 270 90],   'VariableName','e_linear', 'SaveFormat','Array');

    save_system(modelName);
    pause(0.3);

    %% Connect
    ar = {'autorouting','on'};

    % Forward path
    add_line(modelName, 'Reference/1',  'Sum_Error/1', ar{:});
    add_line(modelName, 'Sum_Error/1',  'K_Sensor/1',  ar{:});
    add_line(modelName, 'K_Sensor/1',   'Kp_Gain/1',   ar{:});
    add_line(modelName, 'Kp_Gain/1',    'K_Motor/1',   ar{:});
    add_line(modelName, 'K_Motor/1',    'Plant_Gp/1',  ar{:});

    % Feedback  y → Sum_Error (−)
    add_line(modelName, 'Plant_Gp/1',  'Sum_Error/2', ar{:});

    % Outputs
    add_line(modelName, 'Plant_Gp/1',  'Scope_Output/1', ar{:});
    add_line(modelName, 'Plant_Gp/1',  'Log_Output/1',   ar{:});
    add_line(modelName, 'Sum_Error/1', 'Scope_Error/1',  ar{:});
    add_line(modelName, 'Sum_Error/1', 'Log_Error/1',    ar{:});

    try Simulink.BlockDiagram.arrangeSystem(modelName); catch, end
    save_system(modelName);

    fprintf('\n==============================================\n');
    fprintf('  P-only linear model:  %s.slx\n', modelName);
    fprintf('  Gain blocks: Kp, K_sensor, K_motor\n');
    fprintf('  Plant TF:  G(s) = v_fwd / (L s²)\n');
    fprintf('  Solver:    ode45 (continuous)\n');
    fprintf('==============================================\n\n');
end
