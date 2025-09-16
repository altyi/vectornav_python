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

%% Interacting with a VectorNav Sensor
% This GettingStarted example will walk through using the MATLAB usage of
% the SDK to connect to and interact with a VectorNav sensor.

disp('Starting NonBlockPollingCommands.m example.')

%{
This example demonstrates how to send commands without blocking, validating each command was received correctly
with different types of commands.

This example will achieve the following:
1. Connect to the sensor
2. Configure the ADOR and ADOF to YPR at 100Hz
3. Send generic command (known magnetic disturbance)
4. Wait and check response
5. Enter a loop for 5 seconds where it outputs VNYPR at 100 Hz and sends velocity aiding commands at 10 Hz:
    5.1. Check if a valid response has been received from the velocity aiding command
    5.2. Print response and send new command
6. Disconnect from the sensor
%}

%% Make .NET assembly visible to MATLAB
vnsdkAssembly = NET.addAssembly([pwd '\..\..\..\net\VnSdk_Net.dll']); % change to file path
import VNSDK.* % Get rid of 'VNSDK' namespace qualifications

%% Create a Sensor Object
if exist('sensor','var')
    sensor.Disconnect();
else
    sensor = Sensor();
end

%% 1. Instantiate Sensor and Establish Connection
if ~exist('port_name', 'var')
    port_name = 'COM6'; % Change the sensor port name to the comm port of your local machine
end
sensor.AutoConnect(port_name);
fprintf('\nConnected to %s at %d\n', port_name, sensor.ConnectedBaudRate());

%% 2. Configure ADOR (Asynchronous Data Output Register) and ADOF (Asynchronous Data Output Frequency)
asyncDataOutputType = Registers.System.AsyncOutputType();
asyncDataOutputType.ador = Registers.System.('AsyncOutputType+Ador').YPR; % Configure for Yaw, Pitch, Roll data.
asyncDataOutputType.serialPort = Registers.System.('AsyncOutputType+SerialPort').Serial1;
sensor.WriteRegister(asyncDataOutputType);
disp("ADOR configured.");

asyncDataOutputFreq = Registers.System.AsyncOutputFreq();
asyncDataOutputFreq.adof = Registers.System.('AsyncOutputFreq+Adof').Rate100Hz; % Set output rate to 100Hz.
asyncDataOutputFreq.serialPort = Registers.System.('AsyncOutputFreq+SerialPort').Serial1;
sensor.WriteRegister(asyncDataOutputFreq);
disp("ADOF configured.")

%% 3. Send Known Magnetic Disturbance Command
availableEnums = vnsdkAssembly.Enums;  % Get available enums

kmd = KnownMagneticDisturbance(VNSDK.('KnownMagneticDisturbance+State').Present);
sensor.SendCommand(kmd, VNSDK.('Sensor+SendCommandBlockMode').None); % Non-blocking

%% 4. Wait and Check Response
pause(0.25);
% We could check kmd.awaitingResponse() but it will be removed from the command queue (setting kmd.awaitingResponse to false) after commandRemovalTimeoutLength
% (default 200ms), if any command has been sent or received since.
if kmd.HasValidResponse()
    fprintf('\nKMD Response: %s\n', kmd.GetResponse());
    error_maybe = kmd.GetError();
    if error_maybe.HasValue
        error('Error: %s\n', error_maybe.Value);
    end
else
    error('Error: KMD did not receive a valid response.');
end


%% 5. Enter Loop for 5 Seconds to Process Commands and Measurements

% Initialize velocity aiding register, command, counters, flags, and timers
velAidRegister = Registers.VelocityAiding.VelAidingMeas();
velAidWRGCommand = velAidRegister.ToWriteCommand();

asciiCount = 0;
velAidSentCount = 0;
validResponseReceived = false;

totalTimeLimit = 5;
resendInterval = 0.5;

timer = tic;
resendTimer = tic;

while toc(timer) < totalTimeLimit
    % Handle ASCII YPR measurement
    measurement = sensor.GetNextMeasurement();
    if measurement.HasValue && measurement.Value.MatchesMessage("VNYPR")
        ypr = measurement.Value.attitude.ypr;
        fprintf('YPR: %f, %f, %f\n', ypr.Value.yaw, ypr.Value.pitch, ypr.Value.roll);
        asciiCount = asciiCount + 1;
    end

    %% 5.1.Check for Valid Response
    if (~validResponseReceived && ~velAidWRGCommand.IsAwaitingResponse())
        error_maybe = velAidWRGCommand.GetError();
        if (velAidWRGCommand.HasValidResponse())
            fprintf('\nWRG Response: %s\n', velAidWRGCommand.GetResponse())
            validResponseReceived = true;
        elseif error_maybe.HasValue
            error('Error: %s\n', error_maybe.Value);
        end
    end


    %% 5.2. Resend Command If Timeout Occurs
    if toc(resendTimer) > resendInterval
        if (~validResponseReceived && velAidSentCount > 0)
            error('\nError: Response Timeout\n');
        end

        velAidRegister = Registers.VelocityAiding.VelAidingMeas();
        velAidRegister.velocityX = rand(); % rand() to simulate different velocities
        velAidRegister.velocityY = rand();
        velAidRegister.velocityZ = rand();
        velAidWRGCommand = velAidRegister.ToWriteCommand();
        sensor.SendCommand(velAidWRGCommand, VNSDK.('Sensor+SendCommandBlockMode').None); % Non-blocking

        validResponseReceived = false;
        velAidSentCount = velAidSentCount + 1;
        resendTimer = tic;  % Reset the resend timer
    end
end

fprintf('\nTotal ASCII YPR Packets Received: %d\n', asciiCount);
fprintf('Total VelAid Commands Sent: %d\n', velAidSentCount);
fprintf('\nNonBlockingCommands example complete\n');

%% 6. Disconnect from Sensor
sensor.Disconnect();
