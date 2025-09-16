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
import os
import vectornav
from enum import Enum
from vectornav import Registers, Plugins

usage = "[port] [fileLocation] (or \"generic\")"

class WriteType(Enum):
    Xml = 1
    Ascii = 2
    Generic = 3

def main(argv):
    # [1] Handle input arguments
    if len(argv) > 2:
        print("Usage: python saveNonDefaultConfiguration" , usage)
        return
    if len(argv) > 0:
        portName = argv[0]
    else:
        portName = "COM10" # Pass in the portname as a positional argument or edit here
    if len(argv) > 1:
        filePath = argv[1]
    else:
        current_folder = os.getcwd()
        filePath = os.path.join(current_folder, "configSettings.txt")
    
    if filePath == "generic":
        writeType = WriteType.Generic 
    else:
        _, file_extension = os.path.splitext(filePath)
        if file_extension == '':
            print("Error: File path extension was not specified")
            print("Usage: python saveNonDefaultConfiguration" , usage)
            return
        elif file_extension == ".xml":
            writeType = WriteType.Xml
        else:
            writeType = WriteType.Ascii

   # [2] Connect to the sensor
    vnSensor = vectornav.Sensor()
    vnSensor.autoConnect(portName)
    print(f"Connected to {portName} at {vnSensor.connectedBaudRate()}")
    
    # [3] Write all non default values as specified
    if writeType == WriteType.Xml:
        configWriter = Plugins.XmlConfigWriter(vnSensor, filePath)
    elif writeType == WriteType.Ascii:
        configWriter = Plugins.AsciiConfigWriter(filePath)
    else:
        print("Generic saveNonDefaultConfiguration example unimplemented.")
        vnSensor.disconnect()
        sys.exit()

    Plugins.saveNonDefaultConfiguration(vnSensor, configWriter)

    vnSensor.disconnect()
    print("RegisterScan save non-default config example complete")
    
if __name__ == '__main__':
    main(sys.argv[1:])
