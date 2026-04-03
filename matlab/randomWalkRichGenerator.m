function randomWalkRichGenerator(block)
    setup(block);
end

function setup(block)
    block.NumDialogPrms = 0;
    block.NumInputPorts = 0;
    block.NumOutputPorts = 1;

    block.SetPreCompOutPortInfoToDynamic;
    block.OutputPort(1).Dimensions = 1;
    block.OutputPort(1).SamplingMode = 'Sample';

    block.SampleTimes = [0.01 0];
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);
end

function DoPostPropSetup(block)
    block.NumDworks = 8;

    block.Dwork(1).Name = 'currentValue';
    block.Dwork(1).Dimensions = 1;
    block.Dwork(1).DatatypeID = 0;
    block.Dwork(1).Complexity = 'Real';

    block.Dwork(2).Name = 'mode';
    block.Dwork(2).Dimensions = 1;
    block.Dwork(2).DatatypeID = 0;
    block.Dwork(2).Complexity = 'Real';

    block.Dwork(3).Name = 'samplesLeft';
    block.Dwork(3).Dimensions = 1;
    block.Dwork(3).DatatypeID = 0;
    block.Dwork(3).Complexity = 'Real';

    block.Dwork(4).Name = 'targetValue';
    block.Dwork(4).Dimensions = 1;
    block.Dwork(4).DatatypeID = 0;
    block.Dwork(4).Complexity = 'Real';

    block.Dwork(5).Name = 'rampStep';
    block.Dwork(5).Dimensions = 1;
    block.Dwork(5).DatatypeID = 0;
    block.Dwork(5).Complexity = 'Real';

    block.Dwork(6).Name = 'stairStep';
    block.Dwork(6).Dimensions = 1;
    block.Dwork(6).DatatypeID = 0;
    block.Dwork(6).Complexity = 'Real';

    block.Dwork(7).Name = 'stairHold';
    block.Dwork(7).Dimensions = 1;
    block.Dwork(7).DatatypeID = 0;
    block.Dwork(7).Complexity = 'Real';

    block.Dwork(8).Name = 'stairCounter';
    block.Dwork(8).Dimensions = 1;
    block.Dwork(8).DatatypeID = 0;
    block.Dwork(8).Complexity = 'Real';
end

function Start(block)
    cfg = getConfig();

    if isempty(cfg.seed)
        rng('shuffle');
    else
        rng(cfg.seed);
    end

    block.Dwork(1).Data = 0;
    block.Dwork(2).Data = 0;
    block.Dwork(3).Data = 0;
    block.Dwork(4).Data = 0;
    block.Dwork(5).Data = 0;
    block.Dwork(6).Data = 0;
    block.Dwork(7).Data = 1;
    block.Dwork(8).Data = 0;
end

function Outputs(block)
    cfg = getConfig();

    value = block.Dwork(1).Data;
    mode = block.Dwork(2).Data;
    samplesLeft = block.Dwork(3).Data;
    targetValue = block.Dwork(4).Data;
    rampStep = block.Dwork(5).Data;
    stairStep = block.Dwork(6).Data;
    stairHold = block.Dwork(7).Data;
    stairCounter = block.Dwork(8).Data;

    if samplesLeft <= 0
        [mode, samplesLeft, targetValue, rampStep, stairStep, stairHold, stairCounter] = newSegment(value, cfg);
    end

    switch mode
        case 1
            % Hold segment.

        case 2
            % Ramp / cooldown segment.
            value = value + rampStep;
            if (rampStep >= 0 && value >= targetValue) || (rampStep < 0 && value <= targetValue)
                value = targetValue;
            end

        case 3
            % Stair segment.
            if stairCounter <= 0
                value = value + stairStep;
                stairCounter = stairHold;
            else
                stairCounter = stairCounter - 1;
            end
    end

    % Rare impulsive jump for fast-dynamics excitation.
    if rand < cfg.impulseProb
        value = value + signedUniform(cfg.impulseMin, cfg.impulseMax);
    end

    value = max(cfg.minValue, min(cfg.maxValue, value));
    samplesLeft = samplesLeft - 1;

    block.Dwork(1).Data = value;
    block.Dwork(2).Data = mode;
    block.Dwork(3).Data = samplesLeft;
    block.Dwork(4).Data = targetValue;
    block.Dwork(5).Data = rampStep;
    block.Dwork(6).Data = stairStep;
    block.Dwork(7).Data = stairHold;
    block.Dwork(8).Data = stairCounter;
    block.OutputPort(1).Data = value;
end

function cfg = getConfig()
    cfg.sampleTime = 0.01;
    cfg.minValue = -4000;
    cfg.maxValue = 4000;
    cfg.seed = [];

    cfg.segMinSec = 0.25;
    cfg.segMaxSec = 2.40;
    cfg.rampMinAbs = 20;
    cfg.rampMaxAbs = 280;
    cfg.stairStepSmall = [40, 160];
    cfg.stairStepLarge = [220, 900];
    cfg.stairHoldSamples = [2, 14];
    cfg.impulseProb = 0.015;
    cfg.impulseMin = 300;
    cfg.impulseMax = 1400;

    cfg.pHold = 0.20;
    cfg.pRamp = 0.35;
    cfg.pStair = 0.35;
    cfg.pCooldown = 0.10;
end

function [mode, samplesLeft, targetValue, rampStep, stairStep, stairHold, stairCounter] = newSegment(currentValue, cfg)
    samplesLeft = randi([max(1, round(cfg.segMinSec / cfg.sampleTime)), round(cfg.segMaxSec / cfg.sampleTime)], 1, 1);
    rampStep = 0;
    stairStep = 0;
    stairHold = 1;
    stairCounter = 0;
    targetValue = currentValue;

    r = rand;
    p1 = cfg.pHold;
    p2 = p1 + cfg.pRamp;
    p3 = p2 + cfg.pStair;

    if r < p1
        mode = 1;
        return;
    end

    if r < p2
        mode = 2;
        targetValue = pickTarget(currentValue, cfg);
        mag = randi([cfg.rampMinAbs, cfg.rampMaxAbs], 1, 1);
        rampStep = signOrToward(currentValue, targetValue) * mag;
        return;
    end

    if r < p3
        mode = 3;
        targetValue = pickTarget(currentValue, cfg);
        if rand < 0.65
            mag = randi(cfg.stairStepSmall, 1, 1);
        else
            mag = randi(cfg.stairStepLarge, 1, 1);
        end
        stairStep = signOrToward(currentValue, targetValue) * mag;
        stairHold = randi(cfg.stairHoldSamples, 1, 1);
        stairCounter = 0;
        return;
    end

    mode = 2;
    targetValue = 0;
    mag = randi([cfg.rampMinAbs, max(cfg.rampMinAbs, round(cfg.rampMaxAbs * 0.6))], 1, 1);
    rampStep = signOrToward(currentValue, targetValue) * mag;
end

function target = pickTarget(currentValue, cfg)
    amp = max(abs(cfg.minValue), abs(cfg.maxValue));

    if rand < 0.35
        target = signedUniform(round(0.80 * amp), amp);
    elseif rand < 0.70
        target = signedUniform(round(0.35 * amp), round(0.85 * amp));
    else
        target = currentValue + signedUniform(120, 1200);
    end

    target = max(cfg.minValue, min(cfg.maxValue, target));
end

function v = signedUniform(a, b)
    mag = randi([a, b], 1, 1);
    if rand < 0.5
        v = -mag;
    else
        v = mag;
    end
end

function s = signOrToward(fromVal, toVal)
    if toVal >= fromVal
        s = 1;
    else
        s = -1;
    end
end
