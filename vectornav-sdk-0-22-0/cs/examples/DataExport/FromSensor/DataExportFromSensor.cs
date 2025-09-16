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
using System.IO;
using System.Reflection;
using VNSDK;

namespace DataExportFromSensor
{
    class Program
    {
        static int Main(string[] args)
        {
            String port = "COM33";
            String outputDirectory = Path.GetDirectoryName(Assembly.GetEntryAssembly().Location);

            if (args.Length > 0)
            {
                port = args[0];
            }

            if (args.Length > 1)
            {
                outputDirectory = args[1];
            }

            Sensor sensor = new Sensor();
            sensor.AutoConnect(port);

            Console.WriteLine($"Connected to {port} at {sensor.ConnectedBaudRate()}");

            ExporterCsv csvExporter = new ExporterCsv(outputDirectory);

            // Subscribe to messages
            sensor.RegisterDataExporter(csvExporter);

            if (csvExporter.Start())
            {
                Console.WriteLine("Error: Failed to start logging to CSV file.");
                return 1;
            }

            Console.WriteLine($"Logging to {outputDirectory}");
            System.Threading.Thread.Sleep(5000);

            csvExporter.Stop();

            sensor.Disconnect();
            Console.WriteLine($"ExportFromSensor example complete.");
            return 0;
        }
    }
}

