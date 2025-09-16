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
using System.Collections.Generic;
using VNSDK;

namespace Examples
{
    public class Parameters
    {
        public string usage { get; }
        public bool valid { get; }
        public string port { get; }
        public string vnxmlPath { get; }
        public string navPath { get; }
        public string gnssPath { get; }
        public string imuPath { get; }
        public uint firmwareBaudRate { get; }
        public uint bootloaderBaudRate { get; }

        public Parameters(string[] args)
        {
            usage = "[--PortName={port_name}] [--{Processor}={file_path}... | --vnXml={file_path}] --firmwareBaudRate={firmware_baudrate} --bootloaderBaudRate={bootloader_baudrate}";
            port = string.Empty;
            vnxmlPath = string.Empty;
            navPath = string.Empty;
            gnssPath = string.Empty;
            imuPath = string.Empty;
            firmwareBaudRate = 115200;
            bootloaderBaudRate = 115200;
            valid = true;

            if (args.Length == 0)
            {
                // Change these to your local machine to run executable without arguments
                port = "COM11";
                navPath = "ReferenceModels_v3.vn100.vnx";
            }

            int index = 0;

            uint tempBaudRate;
            foreach (string parameter in args)
            {
                index = parameter.IndexOf('=');
                if (index == -1)
                {
                    valid = false;
                    return;
                }
                string value = parameter.Substring(index + 1);

                // [--PortName={port_name}]
                if (parameter.StartsWith("--PortName"))
                {
                    port = value;
                }

                // [--{Processor}={file_path}...]
                if (parameter.StartsWith("--Nav"))
                {
                    navPath = value;
                }
                if (parameter.StartsWith("--Gnss"))
                {
                    gnssPath = value;
                }
                if (parameter.StartsWith("--Imu"))
                {
                    imuPath = value;
                }

                // [--vnXml={file_path}]
                if (parameter.StartsWith("--vnXml"))
                {
                    vnxmlPath = value;
                }

                // [--firmwareBaudRate={firmware_baudrate}]
                if (parameter.StartsWith("--firmwareBaudRate"))
                {
                    if (uint.TryParse(value, out tempBaudRate))
                    {
                        firmwareBaudRate = tempBaudRate;
                    }
                }

                // [--bootloaderBaudRate={bootloader_baudrate}]
                if (parameter.StartsWith("--bootloaderBaudRate"))
                {
                    if (uint.TryParse(value, out tempBaudRate))
                    {
                        bootloaderBaudRate = tempBaudRate;
                    }
                }

            }
        }
    }

    class Program
    {
        static int Main(string[] args)
        {
            Parameters parameters = new Parameters(args);
            if (!parameters.valid)
            {
                Console.WriteLine(("Usage: " + parameters.usage));
                return 1;
            }

            Sensor sensor = new Sensor();
            // Connect to sensor. We are not autoconnecting or verifying connectivity because we can not assume the sensor has a valid firmware
            sensor.Connect(parameters.port, parameters.firmwareBaudRate);

            List<VNSDK.FirmwareUpdater.FirmwareFile> files = new List<VNSDK.FirmwareUpdater.FirmwareFile>();
            if (!string.IsNullOrEmpty(parameters.gnssPath)) {
                VNSDK.FirmwareUpdater.FirmwareFile file = new VNSDK.FirmwareUpdater.FirmwareFile(VNSDK.FirmwareUpdater.Processor.Gnss, parameters.gnssPath);
                files.Add(file);
            }
            if (!string.IsNullOrEmpty(parameters.navPath)) {
                VNSDK.FirmwareUpdater.FirmwareFile file = new VNSDK.FirmwareUpdater.FirmwareFile(VNSDK.FirmwareUpdater.Processor.Nav, parameters.navPath);
                files.Add(file);
            }
            if (!string.IsNullOrEmpty(parameters.imuPath)) {
                VNSDK.FirmwareUpdater.FirmwareFile file = new VNSDK.FirmwareUpdater.FirmwareFile(VNSDK.FirmwareUpdater.Processor.Imu, parameters.imuPath);
                files.Add(file);
            }

            VNSDK.FirmwareUpdater firmwareUpdater = new VNSDK.FirmwareUpdater();

            VNSDK.FirmwareUpdater.Params fwUpdateParams = new VNSDK.FirmwareUpdater.Params(parameters.firmwareBaudRate, parameters.bootloaderBaudRate);
            Console.WriteLine($"Begin updating.");

            if (!string.IsNullOrEmpty(parameters.vnxmlPath))
            {
                Console.WriteLine($"Update firmware with vnxml: {parameters.vnxmlPath}");
                firmwareUpdater.UpdateFirmware(ref sensor, parameters.vnxmlPath, fwUpdateParams);
            }
            else
            {
                Console.WriteLine($"Update NAV Firmware : {parameters.navPath}");
                Console.WriteLine($"Update GNSS Firmware: {parameters.gnssPath}");
                Console.WriteLine($"Update IMU Firmware : {parameters.imuPath}");
                firmwareUpdater.UpdateFirmware(ref sensor, files, fwUpdateParams);
            }
            sensor.Disconnect();
            return 0;
        }
    }
}

