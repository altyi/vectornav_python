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

disp('Starting GettingStarted.m example.')

%{
This example will achieve the following:
1. Connect to the sensor
2. Poll and print the model number using a read register command
3. Poll and print the current yaw, pitch, and roll using a read register
command
4. Configure the ADOR and ADOF to YPR at 2Hz
5. Configure the first binary output to output timeStartup, accel, and
angRate, all from common group, with a 200 rate divisor
6. Enter a loop for 5 seconds where it:
    Determines which measurement it received (VNYPR or the necessary binary
    header)
    Prints out the relevant measurement from the CD strut
7. Disconnect from sensor
%}

%% Make .NET assembly visible to MATLAB
vnsdkAssembly = NET.addAssembly([pwd '\..\..\..\net\VnSdk_Net.dll']); % change to file path
import VNSDK.* % Get rid of 'VNSDK' namespace qualifications


%% Create a Sensor Object
if exist('sensor','var')
    sensor.Disconnect();  % We don't want to replace an object that already exists, and may keep the serial port locked
else
    sensor = Sensor();
end

%% 1. Connect to the Sensor
if ~exist('port_name', 'var')
    port_name = 'COM10'; % Change the sensor port name to the COM port of your local machine
end
sensor.AutoConnect(port_name); 

if ~sensor.VerifySensorConnectivity()
    disp('Error: Failed to connect to sensor.');
    return;
end

%% 2. Print the Sensor Model Number
modelReg = Registers.System.Model();
sensor.ReadRegister(modelReg);
fprintf('Sensor model number: %s\n', modelReg.model)

%% 3. Print the Current YPR
ypr = Registers.Attitude.YawPitchRoll();
sensor.ReadRegister(ypr);
fprintf(['Current reading:\n Yaw: %f\t'...
    ' Pitch: %f\t Roll: %f\n'],ypr.yaw, ypr.pitch, ypr.roll);

%% 4. Configure the Asynchronous ASCII Output to YPR at 2Hz
configuredAsciiHeader = "VNYPR"; % Used when identifying measurements

availableEnums = vnsdkAssembly.Enums;  % Get available enums
getEnumNames('VNSDK.Registers.System.AsyncOutputType+Ador', vnsdkAssembly)  % display enum names

% Set Measurements to YPR
asyncDataOutputType = Registers.System.AsyncOutputType();
asyncDataOutputType.ador = Registers.System.('AsyncOutputType+Ador').YPR;
asyncDataOutputType.serialPort = Registers.System.('AsyncOutputType+SerialPort').Serial1;
sensor.WriteRegister(asyncDataOutputType);
disp("ADOR configured.");

% Set Frequency to 2 Hz
asyncDataOutputFreq = Registers.System.AsyncOutputFreq();
asyncDataOutputFreq.adof = Registers.System.('AsyncOutputFreq+Adof').Rate2Hz;
asyncDataOutputFreq.serialPort = Registers.System.('AsyncOutputFreq+SerialPort').Serial1;
sensor.WriteRegister(asyncDataOutputFreq);
disp("ADOF configured.")

%% 5. Configure a Binary Output
binaryOutput = Registers.System.BinaryOutput1();
binaryOutput.rateDivisor = 100; % 4 Hz on VN-300
binaryOutput.asyncMode.serial1 = true;

% May alternatively be set as binaryOutput.common.value_ =  0x0321;
binaryOutput.common.timeStartup = true;
binaryOutput.common.angularRate = true;
binaryOutput.common.accel = true;
binaryOutput.common.imu = true;

sensor.WriteRegister(binaryOutput);

disp("Binary Output 1 message configured.")

%% 6. Listen to Data From the Sensor and Print Recognized Packets
secondsToRun = 5;
asciiMessageCount = 0;
binaryMessageCount = 0;
tic;
while toc < secondsToRun
    measurement = sensor.GetNextMeasurement();
    if ~measurement.HasValue
        continue;
    end
    if measurement.Value.MatchesMessage(configuredAsciiHeader)
        asciiMessageCount = asciiMessageCount + 1;
        ypr = measurement.Value.attitude.ypr;
        fprintf(' Yaw: %f\t Pitch: %f\t Roll: %f\n',ypr.Value.yaw, ypr.Value.pitch, ypr.Value.roll);
    elseif measurement.Value.MatchesMessage(binaryOutput)
        binaryMessageCount = binaryMessageCount + 1;
        timeStartup = measurement.Value.time.timeStartup.Value;
        accel = measurement.Value.imu.accel.Value;
        fprintf(" Time: %f\t", timeStartup.seconds());
        fprintf(" Accel X: %f\t Accel Y: %f\t Accel Z: %f\n", accel.x, accel.y, accel.z);
    else
        fprintf("Unrecognized asynchronous message received.\n");
    end
end

fprintf('Ascii message count: %d\nBinary message count: %d\n', asciiMessageCount, binaryMessageCount);

%% 7. Disconnect From the Sensor, Freeing Resources
sensor.Disconnect();
fprintf('Sensor disconnected.\n')
fprintf('GettingStarted.m example complete\n')
