% The MIT License (MIT)
% 
% VectorNav SDK (v0.22.0)
% Copyright (c) 2024 VectorNav Technologies, LLC
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.

disp('Starting Simple Logger example.')

vnsdkAssembly = NET.addAssembly([pwd '\..\..\..\net\VnSdk_Net.dll']); % change to file path
import VNSDK.* % Get rid of 'VNSDK' namespace qualifications

if exist('sensor','var')
    sensor.Disconnect();  % We don't want to replace an object that already exists, and may keep the serial port locked
else
    sensor = Sensor();
end

if ~exist('port_name', 'var')
    port_name = 'COM14'; % Pass in port name and path as positional arguments, or edit them here
end

if ~exist('path', 'var')
    path = fullfile(pwd, 'log.bin');
end

sensor.AutoConnect(port_name);
fprintf('Connected to %s at %s\n', sensor.ConnectedPortName(), sensor.ConnectedBaudRate())

modelReg = Registers.System.Model();
sensor.ReadRegister(modelReg);
fprintf('Sensor model number: %s\n', modelReg.model)

bufferToLog = ByteBuffer(1024*3);
logger = SimpleLogger(bufferToLog, path);

% Register the logger's input buffer to receive all bytes from the sensor
sensor.RegisterReceivedByteBuffer(bufferToLog);

% Create the log file and start logging to it
if logger.Start()
    error('Failed to write to file.');
end
fprintf('Logging to %s\n', path);

% pause(5);

secondsToRun = 5;
tic;
while toc < secondsToRun
    pause(0.001);
    % Check to make sure we didn't get any async errors. Any buffer overruns means we have dropped data.
    asyncError = sensor.GetAsynchronousError();
    if asyncError.HasValue
        fprintf('Received async error: %d\n', asyncError.Value.Error);
    end
end

logger.Stop();

sensor.DeregisterReceivedByteBuffer();
sensor.Disconnect();

fprintf('Logged %d bytes.\n', logger.NumBytesLogged());

disp('SimpleLogger example complete.')