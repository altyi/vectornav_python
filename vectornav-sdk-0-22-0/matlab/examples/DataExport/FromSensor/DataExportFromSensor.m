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

% DataExporterFromSensor.m

disp('Starting ExportFromSensor example.');

vnsdkAssembly = NET.addAssembly([pwd '\..\..\..\..\net\VnSdk_Net.dll']); % change to file path
import VNSDK.*

if exist('sensor','var')
    sensor.Disconnect();
else
    sensor = Sensor();
end

if ~exist('port_name', 'var')
    port_name = 'COM14';
end

if ~exist('path', 'var')
    outputDirectory = fileparts(mfilename('fullpath'));
else
    outputDirectory = fileparts(path);
end

sensor.AutoConnect(port_name);

if ~sensor.VerifySensorConnectivity()
    disp('Error: Failed to connect to sensor.');
    return;
end
fprintf('Connected to %s at %s\n', sensor.ConnectedPortName(), sensor.ConnectedBaudRate())

csvExporter = ExporterCsv(outputDirectory);

sensor.RegisterDataExporter(csvExporter);

if csvExporter.Start()
    disp('Error: Failed to start logging to CSV file.');
    return;
end

fprintf('Logging to %s\n', outputDirectory);
pause(5);

csvExporter.Stop();

sensor.Disconnect();
disp('ExportFromSensor example complete.');


