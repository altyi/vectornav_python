# The MIT License (MIT)
# 
# VectorNav SDK (v0.22.0)
# Copyright (c) 2024 VectorNav Technologies, LLC
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import sys
import vectornav
from vectornav import Sensor
from vectornav.Plugins import FirmwareUpdater

usage = "[--PortName={port_name}] [--{Processor}={file_path}... | --VnXml={file_path}] [--firmwareBaudRate={baudRate}] [--bootloaderBaudRate={baudRate}]";

class FirmwareUpdateArgs:
    portName: str
    filePaths: str
    params: FirmwareUpdater.Params = FirmwareUpdater.Params()

def getIndexOfArg(needle, haystack):
    for i, arg in enumerate(haystack):
        if needle in arg:
            return i+1
    return 0

def main(argv):

    # Parse command line arguments
    fwArgs = FirmwareUpdateArgs()

    # Get Port name
    if (idx := getIndexOfArg('--PortName=', argv)):
        fwArgs.portName = argv[idx-1].split('=')[-1]
    else:
        print(f"PortName Required: \n usage -> {usage}")
        return
    
    # Get path to vnXml file if using .vnXml to update firmware
    if (idx := getIndexOfArg("--vnXml=", argv)):
        fwArgs.filePaths = argv[idx-1].split('=')[-1]
    else:

        fwArgs.filePaths = vectornav.FilePaths()

    # Parse list of processors and paths to .vnx files to update different processors
    if (idx := getIndexOfArg('--Nav=', argv)):
        fwArgs.filePaths.push_back(
            FirmwareUpdater.File(
                argv[idx-1].split('=')[-1],
                FirmwareUpdater.Processor.Nav
            )
        )
    if (idx := getIndexOfArg('--Gnss=', argv)):
        fwArgs.filePaths.push_back(
            FirmwareUpdater.File(
                argv[idx-1].split('=')[-1],
                FirmwareUpdater.Processor.Gnss
            )
        )
        
    if (idx := getIndexOfArg('--Imu=', argv)):
        fwArgs.filePaths.push_back(
            FirmwareUpdater.File(
                argv[idx-1].split('=')[-1],
                FirmwareUpdater.Processor.Gnss
            )
        )

    # Parse optional arguments for firmware and bootloader baud rates
    if (idx := getIndexOfArg('--firmwareBaudRate=', argv)):
        fwArgs.params.firmwareBaudRate = int(argv[idx-1].split('=')[-1])
        
    if (idx := getIndexOfArg('--bootloaderBaudRate=', argv)):
        fwArgs.params.bootloaderBaudRate = int(argv[idx-1].split('=')[-1])

    print(f"{fwArgs.params.bootloaderBaudRate},{fwArgs.params.firmwareBaudRate}")

    # Connect to sensor
    sensor = Sensor()
    sensor.autoConnect(fwArgs.portName)
    print(f"Connected to {fwArgs.portName} at {sensor.connectedBaudRate()}")

    # Initialize firmware updater object
    firmwareUpdater = FirmwareUpdater()

    # Update firmware
    fwUpdateFailure = firmwareUpdater.updateFirmware(sensor, fwArgs.filePaths, fwArgs.params)

    # Disconnect and check for failure
    sensor.disconnect()
    if fwUpdateFailure:
        raise Exception("Failed to update the firmware")
    
    print("FirmwareUpdate example complete")


if __name__ == '__main__':
    main(sys.argv[1:])
