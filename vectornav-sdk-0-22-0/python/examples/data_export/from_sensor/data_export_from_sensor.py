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

import time, sys, os

import vectornav
from vectornav import Sensor
from vectornav.Plugins import ExporterCsv

def main(argv):

    port = argv[0] if argv else "COM30"
    outputDirectory = argv[1] if len(argv)==2 else os.path.dirname(os.path.abspath(__file__))
    
    vs = Sensor()

    csvExporter = ExporterCsv(outputDirectory, True)
    
    vs.autoConnect(port)

    # Add a subscriber to all VN FA and ASCII packets
    vs.subscribeToMessage(
        csvExporter.getQueuePtr(),
        vectornav.Registers.BinaryOutputMeasurements(),
        vectornav.FaPacketDispatcher.SubscriberFilterType.AnyMatch
    )
    
    vs.subscribeToMessage(
        csvExporter.getQueuePtr(),
        "VN",
        vectornav.AsciiPacketDispatcher.SubscriberFilterType.StartsWith
    )

    csvExporter.start()
    print("Logging to ", outputDirectory)
    
    time.sleep(5)

    csvExporter.stop()
    print("ExportFromSensor example complete.")

if __name__ == '__main__':
    main(sys.argv[1:])
