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

import sys, os

from vectornav.Plugins import FileExporter

def main(argv):
    parent_dir = os.path.dirname(os.path.abspath(__file__))
    filePath = argv[0] if argv else os.path.join(parent_dir, "data_export_from_file.bin")
    outputPath = argv[1] if len(argv)==2 else parent_dir

    print(f"Exporting {filePath}")
    print(f"Outputting to {outputPath}\n")

    fileExporter = FileExporter()
    fileExporter.addCsvExporter(outputPath, False)
    fileExporter.addAsciiExporter(outputPath)
    fileExporter.addSkippedByteExporter(outputPath)

    if (fileExporter.processFile(filePath)):
        print("Error: File processing failed.")
        return
    
    print(fileExporter.getParsingStats())

    print("ExportFromFile example complete")

if __name__ == '__main__':
    main(sys.argv[1:])    
