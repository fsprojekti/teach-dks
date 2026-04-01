function randomWalkGenerator(block)
    setup(block);
end

%% Setup: Configure ports, sample time, and register methods.
function setup(block)
    % No dialog parameters to keep usage simple.
    block.NumDialogPrms = 0;

    % No input ports; one scalar output port (PWM command).
    block.NumInputPorts = 0;
    block.NumOutputPorts = 1;

    block.SetPreCompOutPortInfoToDynamic;
    block.OutputPort(1).Dimensions = 1;
    block.OutputPort(1).SamplingMode = 'Sample';

    % Sample time for block execution.
    block.SampleTimes = [0.01 0];

    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);
end

%% Define Dwork vectors for generator state.
function DoPostPropSetup(block)
    block.NumDworks = 2;

    block.Dwork(1).Name = 'currentValue';
    block.Dwork(1).Dimensions = 1;
    block.Dwork(1).DatatypeID = 0;  % double
    block.Dwork(1).Complexity = 'Real';

    block.Dwork(2).Name = 'holdCounter';
    block.Dwork(2).Dimensions = 1;
    block.Dwork(2).DatatypeID = 0;  % double
    block.Dwork(2).Complexity = 'Real';
end

%% Start: Initialize random-walk state.
function Start(block)
    cfg = getConfig();

    if isempty(cfg.seed)
        rng('shuffle');
    else
        rng(cfg.seed);
    end

    block.Dwork(1).Data = 0;  % start around center
    block.Dwork(2).Data = 0;  % force update on first sample
end

%% Outputs: Update only every deltaT and hold value in between.
function Outputs(block)
    cfg = getConfig();

    holdSamples = max(1, round(cfg.deltaT / cfg.sampleTime));
    value = block.Dwork(1).Data;
    holdCounter = block.Dwork(2).Data;

    if holdCounter <= 0
        step = randi([-cfg.maxStep, cfg.maxStep], 1, 1);
        value = value + step;
        value = min(cfg.maxValue, max(cfg.minValue, value));
        holdCounter = holdSamples - 1;
    else
        holdCounter = holdCounter - 1;
    end

    block.Dwork(1).Data = value;
    block.Dwork(2).Data = holdCounter;
    block.OutputPort(1).Data = value;
end

%% Local configuration for simple usage.
function cfg = getConfig()
    cfg.sampleTime = 0.01;   % [s] block execution step
    cfg.deltaT = 1;       % [s] update interval (50 Hz)
    cfg.maxStep = 500;       % max up/down change per update
    cfg.minValue = -4000;    % PWM lower limit
    cfg.maxValue = 4000;     % PWM upper limit
    cfg.seed = [];           % [] = random each run, or set integer for repeatability
end