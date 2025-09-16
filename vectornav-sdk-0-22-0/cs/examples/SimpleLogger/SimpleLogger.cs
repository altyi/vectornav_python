// The MIT License (MIT)
// 
// VectorNav SDK (v0.22.0)
// Copyright (c) 2024 VectorNav Technologies, LLC
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

using System;
using VNSDK;

class Program
{
    static int Main(string[] args)
    {
        // Pass in port name and path as positional arguments, or edit them here
        string port = (args.Length > 0) ? args[0] : "COM18";
        string path = (args.Length > 1) ? args[1] : System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "log.bin");

        Sensor sensor = new Sensor();
        sensor.AutoConnect(port);

        Console.WriteLine($"Connected to {port} at {sensor.ConnectedBaudRate()}");

        VNSDK.Registers.System.Model modelReg = new VNSDK.Registers.System.Model();
        sensor.ReadRegister(modelReg);
        Console.WriteLine($"Sensor Model Number: {modelReg.model}");

        ByteBuffer bufferToLog = new ByteBuffer(1024 * 3);
        SimpleLogger logger = new SimpleLogger(bufferToLog, path);

        // Register the logger's input buffer to receive all bytes from the sensor
        sensor.RegisterReceivedByteBuffer(bufferToLog);

        // Create the log file and start logging to it
        if (logger.Start())
        {
            Console.WriteLine("Error: Failed to write to file.");
            return 1;
        }

        Console.WriteLine($"Logging to {path}");

        Nullable<AsyncError> asyncError = null;

        DateTime endTime = DateTime.Now.AddSeconds(5); // Run for 5 seconds

        while (DateTime.Now < endTime)
        {
            System.Threading.Thread.Sleep(1);
            // Check to make sure we didn't get any async errors. Any buffer overruns means we have dropped data.
            asyncError = sensor.GetAsynchronousError();
            if (asyncError.HasValue)
            {
                Console.WriteLine($"Received async error: {asyncError.Value.Error}");
            }
        }

        logger.Stop();

        sensor.DeregisterReceivedByteBuffer();
        sensor.Disconnect();

        Console.WriteLine($"Logged {logger.NumBytesLogged()} bytes.");

        Console.WriteLine($"SimpleLogger example complete.");

        return 0;
    }
}