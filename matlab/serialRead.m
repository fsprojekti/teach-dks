function serialRead(block)
    setup(block);
end

%% Setup: Configure ports, sample time, and register methods.
function setup(block)
    % Dialog parameters: COM port.
    block.NumDialogPrms = 1;
    
    % No input ports; one output port with 4 values:
    % [angle1_deg, angle2_deg, vel1_rpm, vel2_rpm].
    block.NumInputPorts = 0;
    block.NumOutputPorts = 1;
    
    block.SetPreCompOutPortInfoToDynamic;
    block.OutputPort(1).Dimensions = 4;
    block.OutputPort(1).SamplingMode = 'Sample';
    
    % Sample time.
    block.SampleTimes = [0.01 0];
    
    % Default simulation state compliance.
    block.SimStateCompliance = 'DefaultSimState';
    
    % Register methods.
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);    
    block.RegBlockMethod('Terminate', @Terminate);
end

%% Define a Dwork vector to hold the latest valid 4-value measurement.
function DoPostPropSetup(block)
    block.NumDworks = 1;
    block.Dwork(1).Name = 'encoderData';
    block.Dwork(1).Dimensions = 4;
    block.Dwork(1).DatatypeID = 0;  % double
    block.Dwork(1).Complexity = 'Real';
    % Do not initialize block.Dwork(1).Data here.
end

%% Start: Open the serial port, wait for the handshake, and send the initial command.
function Start(block)
    global serialPortObj;
    comPort = block.DialogPrm(1).Data;
    baudRate = 115200;

    wasCreatedNow = false;
    if isempty(serialPortObj) || ~isvalid(serialPortObj)
        serialPortObj = serialport(comPort, baudRate);
        configureTerminator(serialPortObj, "LF");
        serialPortObj.Timeout = 0.05;
        wasCreatedNow = true;
    end

    % Clear stale data when reusing an existing connection.
    if ~wasCreatedNow
        flush(serialPortObj);
    end

    % Wait for READY only when we created/opened the port in this block start.
    if wasCreatedNow
        timeout = 5; % seconds
        tStart = tic;
        readyReceived = false;
        while toc(tStart) < timeout
            pause(0.05);
            if serialPortObj.NumBytesAvailable > 0
                line = readline(serialPortObj);
                if contains(line, 'READY', 'IgnoreCase', true)
                    readyReceived = true;
                    break;
                end
            end
        end
        if ~readyReceived
            warning('serialRead: Handshake failed: No READY message received.');
        else
            disp('serialRead: Handshake successful.');
        end

        % Drop any trailing boot text so parser starts on clean data frames.
        flush(serialPortObj);
    end
    
    % Initialize the stored encoder data.
    block.Dwork(1).Data = [0 0 0 0];
    
    % Put device in known stream state. Send twice to tolerate reset race.
    for k = 1:2
        writeline(serialPortObj, 'stop');
        writeline(serialPortObj, 'plot 0');
        writeline(serialPortObj, 'ts 10');
        writeline(serialPortObj, 'plot 1');
        writeline(serialPortObj, 'run');
        pause(0.03);
    end
end

%% Outputs: Read a line, parse for encoder data, and output the data.
function Outputs(block)
    global serialPortObj;

    encoderData = block.Dwork(1).Data;
    if isempty(serialPortObj) || ~isvalid(serialPortObj)
        block.OutputPort(1).Data = encoderData;
        return;
    end

    % Read all currently buffered lines and keep the newest valid frame.
    while serialPortObj.NumBytesAvailable > 0
        try
            rawData = readline(serialPortObj);
            parsed = checkAndConvert(rawData);
            if ~isempty(parsed)
                encoderData = parsed;
            end
        catch
            break;
        end
    end

    block.Dwork(1).Data = encoderData;
    
    block.OutputPort(1).Data = encoderData;
end

%% Terminate: Send shutdown command and close the port.
function Terminate(block)
    %#ok<INUSD>
    global serialPortObj;
    if ~isempty(serialPortObj)
        try
            writeline(serialPortObj, 'plot 0');
            writeline(serialPortObj, 'stop');
        catch
            % Ignore errors during termination.
        end
        delete(serialPortObj);
        clear serialPortObj;
    end
end

%% Helper: Parse a line from the serial port to extract encoder data.
function output = checkAndConvert(str)
    str = strtrim(str);

    % Ignore non-data lines such as READY or status text.
    if isempty(regexp(str, '^[\d\-\+\.]', 'once'))
        output = [];
        return;
    end

    % Split by whitespace.
    parts = strsplit(str);
    if numel(parts) ~= 4
        output = [];
        return;
    end

    nums = str2double(parts);
    if any(isnan(nums)) || any(~isfinite(nums))
        output = [];
    else
        output = nums;
    end
end
