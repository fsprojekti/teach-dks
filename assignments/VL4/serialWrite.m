function serialWrite(block)
    setup(block);
end

function setup(block)
    block.NumDialogPrms = 1;  % e.g., COM port parameter
    block.NumInputPorts = 1;
    block.NumOutputPorts = 0;
    
    block.InputPort(1).Dimensions = 1;  % Signed PWM command
    block.InputPort(1).DatatypeID = 0;
    block.InputPort(1).Complexity = 'Real';
    block.InputPort(1).DirectFeedthrough = true;
    
    block.SampleTimes = [0.01 0];  % match read block sample time
    block.SimStateCompliance = 'DefaultSimState';
    
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Terminate', @Terminate);
end

function Start(block)
    global serialPortObj;
    global lastSentSignedPwm;

    comPort = block.DialogPrm(1).Data;
    baudRate = 115200;

    if isempty(serialPortObj) || ~isvalid(serialPortObj)
        serialPortObj = serialport(comPort, baudRate);
        configureTerminator(serialPortObj, "LF");
        serialPortObj.Timeout = 0.05;
    end

    lastSentSignedPwm = NaN;
end

function Outputs(block)
    global serialPortObj;
    global lastSentSignedPwm;

    if isempty(serialPortObj) || ~isvalid(serialPortObj)
        return;
    end

    cmdData = block.InputPort(1).Data;
    pwmVal = cmdData(1);

    if isnan(pwmVal) || ~isfinite(pwmVal)
        pwmVal = 0;
    end

    % Accept signed scalar PWM command from Simulink in range [-4096, 4096].
    if pwmVal > 4096
        pwmVal = 4096;
    elseif pwmVal < -4096
        pwmVal = -4096;
    end

    % Quantize command so we only send changed integer values.
    pwmSigned = int32(round(pwmVal));
    if isequal(pwmSigned, lastSentSignedPwm)
        return;
    end

    if pwmSigned < 0
        dir = 1;
    else
        dir = 0;
    end

    pwmAbs = int32(abs(pwmSigned));

    writeline(serialPortObj, sprintf('pwm %d', pwmAbs));
    writeline(serialPortObj, sprintf('dir %d', dir));

    lastSentSignedPwm = pwmSigned;
end

function Terminate(block)
    % Do not close the port here; let the read block handle it.
end
