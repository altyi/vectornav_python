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
using VNSDK;

namespace DataExportFromFile
{
    class Program
    {
        static int Main(string[] args)
        {
            String outputDirectory = Directory.GetParent(Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location)).Parent.FullName;
            String filePath = Path.Combine(outputDirectory, "DataExportFromFile.bin");

            if (args.Length > 0)
            {
                filePath = args[0];
                outputDirectory = Path.GetDirectoryName(Path.GetFullPath(filePath));
            }

            Console.WriteLine($"Exporting {filePath}");
            Console.WriteLine($"Outputting to {outputDirectory}\n");

            ExporterAscii asciiExporter = new ExporterAscii(outputDirectory);
            ExporterCsv csvExporter = new ExporterCsv(outputDirectory);
            ExporterSkippedByte skippedByteExporter = new ExporterSkippedByte(outputDirectory);

            FileExporter fileExporter = new FileExporter();
            fileExporter.AddExporter(asciiExporter);
            fileExporter.AddExporter(csvExporter);
            fileExporter.AddSkippedByteExporter(skippedByteExporter);

            if (fileExporter.ProcessFile(filePath))
            {
                Console.WriteLine("Error: File processing failed.");
                return 1;
            }

            Console.WriteLine(fileExporter.GetParsingStats());

            Console.WriteLine("FileExport example complete.");
            return 0;
        }
    }
}
