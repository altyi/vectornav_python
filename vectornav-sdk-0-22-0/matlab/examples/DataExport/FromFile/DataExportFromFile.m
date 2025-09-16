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

% DataExportFromFile.m

disp('Starting ExportFromFile example.');

vnsdkAssembly = NET.addAssembly([pwd '\..\..\..\..\net\VnSdk_Net.dll']); % change to file path
import VNSDK.* % Get rid of 'VNSDK' namespace qualifications

outputDirectory = fileparts(mfilename('fullpath'));
path = fullfile(outputDirectory, 'DataExportFromFile.bin');

if ~exist('path', 'var')
    outputDirectory = fileparts(path);
end

fprintf('Exporting %s\n', path);
fprintf('Outputting to %s\n\n', outputDirectory);

asciiExporter = ExporterAscii(outputDirectory);
csvExporter = ExporterCsv(outputDirectory);
skippedByteExporter = ExporterSkippedByte(outputDirectory);

fileExporter = FileExporter();
fileExporter.AddExporter(asciiExporter);
fileExporter.AddExporter(csvExporter);
fileExporter.AddSkippedByteExporter(skippedByteExporter);

if fileExporter.ProcessFile(path)
    disp('Error: File processing failed.');
    return;
end

disp(fileExporter.GetParsingStats().ToString());
disp('ExportFromFile example complete.');