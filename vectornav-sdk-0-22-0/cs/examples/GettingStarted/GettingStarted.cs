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

namespace GettingStarted
{
    class Program
    {
        static int Main(string[] args)
        {
            Console.WriteLine($"Starting Getting Started example.");

            // This getting started example will walk you throgh the C# usage of the SDK to connect to and interact with a VectorNav sensor.

            // This example will achieve the following:
            // 1. Connect to the sensor
            // 2. Poll and print the model number using a read register command
            // 3. Poll and print the current yaw, pitch, and roll using a read register command
            // 4. Configure the ADOR and ADOF to YPR at 2Hz
            // 5. Configure the first binary output to output timeStartup, accel, and angRate, all from common group, with a 200 rate divisor
            // 6. Enter a loop for 5 seconds where it:
            //    Determines which measurement it received (VNYPR or the necessary binary header)
            //    Prints out the relevant measurement from the CD struct
            // 7. Disconnect from sensor

            // Define the port connection parameters to be used later
            String port = "COM7";           // Change the sensor port name to the comm port of your local machine
            if (args.Length > 0)
            {
                port = args[0];
            }

            // Instantiate a Sensor object that we'll use to interact with the sensor
            Sensor sensor = new Sensor();

            // ------------------------
            // 1. Connect to the sensor
            sensor.AutoConnect(port);

            if (!sensor.VerifySensorConnectivity())
            {
                Console.WriteLine("Error: Failed to connect to sensor.");
                return 1;
            }

            // -------------------------------
            // 2. Print the sensor model number

            // The Register base class object is needed for the SDK API calls
            // You can access the data by casting the object as the derived object
            VNSDK.Registers.System.Model modelReg = new VNSDK.Registers.System.Model();
            sensor.ReadRegister(modelReg);
            Console.WriteLine($"Sensor Model Number: {modelReg.model}");

            // ------------------------
            // 3. Print the current YPR

            // Another approach to accessing register is to define the derived object to more easily access the data members
            VNSDK.Registers.Attitude.YawPitchRoll ypr = new VNSDK.Registers.Attitude.YawPitchRoll();
            // And then making an base class alias of the derived object to pass to the SDK API
            // This is because .NET does not all polymorphism at the method parameter level
            // And .NET does not allow casting the derived object as an argument in the function call
            sensor.ReadRegister(ypr);
            Console.WriteLine($"Current Reading: " +
                $"Yaw {ypr.yaw}, " +
                $"Pitch {ypr.pitch}, " +
                $"Roll {ypr.roll}");

            // -------------------------------------------------------
            // 4. Configure the asynchronous ASCII output to YPR at 2Hz

            String messageType = "VNYPR";       // Used when identifying measurements

            // Set ASCII messages to YPR
            VNSDK.Registers.System.AsyncOutputType asyncOutputType = new VNSDK.Registers.System.AsyncOutputType();
            asyncOutputType.ador = VNSDK.Registers.System.AsyncOutputType.Ador.YPR;                       // YPR
            asyncOutputType.serialPort = VNSDK.Registers.System.AsyncOutputType.SerialPort.ActiveSerial;       // Active Serial Port

            sensor.WriteRegister(asyncOutputType);
            Console.WriteLine("ADOR configured.");

            VNSDK.Registers.System.AsyncOutputFreq asyncOutputFreq = new VNSDK.Registers.System.AsyncOutputFreq();
            asyncOutputFreq.adof = VNSDK.Registers.System.AsyncOutputFreq.Adof.Rate1Hz;           // 1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200 Hz
            asyncOutputFreq.serialPort = VNSDK.Registers.System.AsyncOutputFreq.SerialPort.ActiveSerial;       // Active Serial Port

            sensor.WriteRegister(asyncOutputFreq);

            Console.WriteLine("ADOF configured.");

            // ----------------------------
            // 5. Configure a binary output

            VNSDK.Registers.System.BinaryOutput1 binaryOutput1 = new VNSDK.Registers.System.BinaryOutput1();
            binaryOutput1.rateDivisor = 400;         // 2 Hz on all but VN-300
            binaryOutput1.asyncMode.serial1 = true;
            binaryOutput1.asyncMode.serial2 = true;
            binaryOutput1.common.timeStartup = true;
            binaryOutput1.common.accel = true;
            binaryOutput1.common.angularRate = true;
            binaryOutput1.common.imu = true;

            // The Register base object is also used when identifying measurements
            sensor.WriteRegister(binaryOutput1);

            Console.WriteLine("Binary Output 1 message configured.");

            // ------------------------------------------------------------------
            // 6. Listen to data from the sensor and print recognized packets

            Int32 secondsToRun = 5;
            Int32 asciiMessageCount = 0;
            Int32 binaryMessageCount = 0;
            System.Diagnostics.Stopwatch s = new System.Diagnostics.Stopwatch();
            s.Start();
            while (s.Elapsed < TimeSpan.FromSeconds(secondsToRun))
            {
                Nullable<CompositeData> measurement = sensor.GetNextMeasurement();
                if (measurement.HasValue)
                {
                    if (measurement.Value.MatchesMessage(messageType))
                    {
                        Console.WriteLine($"Ascii");
                        asciiMessageCount++;
                        Console.WriteLine($"YPR = {measurement.Value.attitude.ypr.Value.yaw}, {measurement.Value.attitude.ypr.Value.pitch}, {measurement.Value.attitude.ypr.Value.roll}");
                    }

                    else if (measurement.Value.MatchesMessage(binaryOutput1))
                    {
                        Console.WriteLine($"Binary");
                        binaryMessageCount++;
                        if (measurement.Value.time.timeStartup.HasValue)
                        {
                            Console.WriteLine($"Time: {measurement.Value.time.timeStartup.Value}");
                        }
                        if (measurement.Value.imu.accel.HasValue)
                        {
                            Console.WriteLine($"Accel: X {measurement.Value.imu.accel.Value.x}, Y {measurement.Value.imu.accel.Value.y}, Z {measurement.Value.imu.accel.Value.z}");
                        }
                    }
                    // else
                    // {
                    //     Console.WriteLine($"none");
                    // }
                }
            }
            s.Stop();
            Console.WriteLine($"Ascii Messages: {asciiMessageCount}, Binary Messages: {binaryMessageCount}");

            // --------------------------------------------------------------------------------
            // 7. Disconnect from the sensor, freeing resources and closing the started thread.

            sensor.Disconnect();
            Console.WriteLine($"Sensor Disconnected.");
            Console.WriteLine($"Getting Started example complete.");

            return 0;
        }
    }
}

