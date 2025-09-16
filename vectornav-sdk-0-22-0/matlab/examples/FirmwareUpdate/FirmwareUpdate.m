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

disp('Starting Firmware Update example.')

vnsdkAssembly = NET.addAssembly([pwd '\..\..\net\VnSdk_Net.dll']); % change to file path
import VNSDK.* % Get rid of 'VNSDK' namespace qualifications
import System.Collections.Generic.*;


if exist('sensor','var')
    sensor.Disconnect();  % We don't want to replace an object that already exists, and may keep the serial port locked
else
    sensor = Sensor();
end

if ~exist('port_name', 'var')
    port_name = 'COM10'; % Change the sensor port name to the COM port of your local machine
end
files = NET.createGeneric('System.Collections.Generic.List', {'VNSDK.FirmwareUpdater+FirmwareFile'}, 0);
if ~exist('Nav', 'var')
    Nav = fullfile(pwd, 'ReferenceModels_v3.vn100.vnx'); % Change this to your sensor
else
    file = VNSDK.('FirmwareUpdater+FirmwareFile')(VNSDK.('FirmwareUpdater+Processor').Nav, Nav);
    files.Add(file)
    fprintf('Loading Nav firmware from %s\n', Nav);
end
if ~exist('Gnss', 'var')
    Gnss = '';
else
    file = VNSDK.('FirmwareUpdater+FirmwareFile')(VNSDK.('FirmwareUpdater+Processor').Gnss, Gnss);
    files.Add(file);
    fprintf('Loading Gnss firmware from %s\n', Gnss);
end
if ~exist('Imu', 'var')
    Imu = '';
else
    file = VNSDK.('FirmwareUpdater+FirmwareFile')(VNSDK.('FirmwareUpdater+Processor').Imu, Imu);
    files.Add(file);
    fprintf('Loading Imu firmware from %s\n', Imu);
end
if ~exist('vnXml', 'var')
    vnXml = '';
else
    fprintf('Loading VnXml from %s\n', vnXml);
end
if ~exist('firmwareBaudRate', 'var')
    firmwareBaudRate = 115200;
end
if ~exist('bootloaderBaudRate', 'var')
    bootloaderBaudRate = 115200;
end

% Connect to sensor. We are not autoconnecting or verifying connectivity because we can not assume the sensor has a valid firmware
sensor.Connect(port_name, 115200);
fprintf('Connected to %s at %s\n', sensor.ConnectedPortName(), sensor.ConnectedBaudRate())

params = VNSDK.("FirmwareUpdater+Params")(firmwareBaudRate, bootloaderBaudRate);
firmwareUpdater = FirmwareUpdater();
if ~isempty(vnXml)
    firmwareUpdater.UpdateFirmware(sensor, vnXml, params);
else
    firmwareUpdater.UpdateFirmware(sensor, files, params);
end

sensor.Disconnect();
fprintf('Firmware update example complete.\n');