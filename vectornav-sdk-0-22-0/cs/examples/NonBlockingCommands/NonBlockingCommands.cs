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


namespace Examples
{
    class Program
    {
        static int Main(string[] args)
        {
            /*
            This example demonstrates how to send commands without blocking, validating each command was received correctly
            with different types of commands.

            This example will achieve the following:
            
            1. Connect to the sensor
            2. Configure the ADOR and ADOF to YPR at 100Hz
            3. Send generic command (known magnetic disturbance)
            4. Wait and check response
            5. Output VNYPR at 100 Hz and send velocity aiding commands at 10 Hz for 5 seconds:
                5.1. Check if a valid response has been received from the velocity aiding command
                5.2. Print response and send new command
            6. Disconnect from the sensor
            */

            // [1] Instantiate Sensor and Establish Connection
            String port = "COM6"; // Default port COM33 or as passed by user.
            if (args.Length > 0)
            {
                port = args[0];
            }
            Sensor sensor = new Sensor();
            sensor.AutoConnect(port);
            Console.WriteLine($"\nConnected to {port} at {sensor.ConnectedBaudRate()}\n");

            // [2] Configure ADOR (Asynchronous Data Output Register) and ADOF (Asynchronous Data Output Frequency)
            VNSDK.Registers.System.AsyncOutputType asyncDataOutputType = new VNSDK.Registers.System.AsyncOutputType();
            asyncDataOutputType.ador = VNSDK.Registers.System.AsyncOutputType.Ador.YPR; // Configure for Yaw, Pitch, Roll data.
            asyncDataOutputType.serialPort = VNSDK.Registers.System.AsyncOutputType.SerialPort.Serial1;
            sensor.WriteRegister(asyncDataOutputType);
            Console.WriteLine("ADOR configured");

            VNSDK.Registers.System.AsyncOutputFreq asyncDataOutputFreq = new VNSDK.Registers.System.AsyncOutputFreq();
            asyncDataOutputFreq.adof = VNSDK.Registers.System.AsyncOutputFreq.Adof.Rate100Hz; // Set output rate to 100Hz
            asyncDataOutputFreq.serialPort = VNSDK.Registers.System.AsyncOutputFreq.SerialPort.Serial1;
            sensor.WriteRegister(asyncDataOutputFreq);
            Console.WriteLine("ADOF configured");

            // [3] Send Known Magnetic Disturbanc Command
            VNSDK.GenericCommand kmd = new VNSDK.KnownMagneticDisturbance(VNSDK.KnownMagneticDisturbance.State.Present);
            sensor.SendCommand(ref kmd, Sensor.SendCommandBlockMode.None); // Non-blocking

            // [4] Wait and Check Response
            System.Threading.Thread.Sleep(250);
            // We could check kmd.awaitingResponse() but it will be removed from the command queue (setting kmd.awaitingResponse to false) after commandRemovalTimeoutLength
            // (default 200ms), if any command has been sent or received since.
            if (kmd.HasValidResponse())
            {
                Console.WriteLine($"\nKMD Response: {kmd.GetResponse()}");
                int? error_maybe = kmd.GetError();
                if (error_maybe.HasValue)
                {
                    Console.WriteLine($"\nError: {error_maybe.Value}");
                    return error_maybe.Value;
                }
            }
            else { Console.WriteLine("\nError: KMD did not receive a valid response"); }


            // [5] Enter Loop for 5 Seconds to Process Commands and Measurements
            // Initialize velocity aiding register, command, counters, flags, and timers
            VNSDK.Registers.VelocityAiding.VelAidingMeas velAidRegister = new VNSDK.Registers.VelocityAiding.VelAidingMeas();
            VNSDK.GenericCommand velAidWRGCommand = velAidRegister.ToWriteCommand();

            int asciiCount = 0;
            int velAidSentCount = 0;
            bool validResponseReceived = false;

            Random random = new Random();
            TimeSpan timeout = TimeSpan.FromSeconds(5);
            TimeSpan resendTimeout = TimeSpan.FromMilliseconds(500);
            System.Diagnostics.Stopwatch timer = System.Diagnostics.Stopwatch.StartNew();
            System.Diagnostics.Stopwatch resendTimer = System.Diagnostics.Stopwatch.StartNew();

            while (timer.Elapsed < timeout)
            {
                // Handle ASCII YPR Measurement
                Nullable<CompositeData> compositeData = sensor.GetNextMeasurement();
                if (compositeData.HasValue && compositeData.Value.MatchesMessage("VNYPR"))
                {
                    Nullable<Ypr> ypr = compositeData.Value.attitude.ypr.Value;
                    Console.WriteLine($"YPR: {ypr.Value.yaw}, {ypr.Value.pitch}, {ypr.Value.roll}");
                    asciiCount++;
                }

                // [5.1] Check for Valid Response
                if (!validResponseReceived && !velAidWRGCommand.IsAwaitingResponse())
                {
                    int? error_maybe = velAidWRGCommand.GetError();
                    if (velAidWRGCommand.HasValidResponse())
                    {
                        Console.WriteLine($"\nWRG Response {velAidWRGCommand.GetResponse()}");
                        validResponseReceived = true;
                    }
                    else if (error_maybe.HasValue)
                    {
                        Console.WriteLine($"Error: {error_maybe.Value}");
                        return error_maybe.Value;
                    }
                }

                // [5.2] Send command and check response at 2Hz
                if (resendTimer.Elapsed > resendTimeout)
                {
                    if (!validResponseReceived && velAidSentCount > 0) { Console.WriteLine($"\nError: Response Timeout\n"); }

                    velAidRegister.velocityX = (float)(random.NextDouble()); // random to simulate different velocities
                    velAidRegister.velocityY = (float)(random.NextDouble());
                    velAidRegister.velocityZ = (float)(random.NextDouble());
                    velAidWRGCommand = velAidRegister.ToWriteCommand(); // The Command object was instantiated before the loop.
                    sensor.SendCommand(ref velAidWRGCommand, Sensor.SendCommandBlockMode.None); // Non-blocking

                    validResponseReceived = false;
                    velAidSentCount++;
                    resendTimer.Restart(); // Restart send timer
                }
            }

            Console.WriteLine($"\nTotal ASCII YPR Packets Received: {asciiCount}");
            Console.WriteLine($"\nTotal VelAid Commands Sent: {velAidSentCount}");
            Console.WriteLine($"\nNonBlockingCommands example complete");

            // [6] Disconnect Sensor
            sensor.Disconnect();
            return 0;
        }
    }
}

