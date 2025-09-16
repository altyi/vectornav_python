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

#ifndef VN_CLI_EXPORTER_HPP_
#define VN_CLI_EXPORTER_HPP_

#pragma managed(push, off)
#include "vectornav/Exporter.hpp"
#include "vectornav/ExporterAscii.hpp"
#include "vectornav/ExporterCsv.hpp"
// #include "vectornav/ExporterImr.hpp"
// #include "vectornav/ExporterRinex.hpp"
#include "vectornav/FileExporter.hpp"
#include "vectornav/SkippedByteExporter.hpp"
#pragma managed(pop)

using namespace System;

namespace VNSDK
{
public ref struct ParsingStats
{
public:
    uint64_t validFaPacketCount;
    uint64_t invalidFaPacketCount;
    uint64_t validAsciiPacketCount;
    uint64_t invalidAsciiPacketCount;
    uint64_t validFbPacketCount;
    uint64_t invalidFbPacketCount;
    uint64_t skippedByteCount;
    uint64_t receivedByteCount;
    
    virtual String^ ToString() override
    {
        uint64_t totalValidPackets = validFaPacketCount + validAsciiPacketCount + validFbPacketCount;
        double validFaPercent = (totalValidPackets == 0) ? 0 : 100.0 * validFaPacketCount / totalValidPackets;
        double validAsciiPercent = (totalValidPackets == 0) ? 0 : 100.0 * validAsciiPacketCount / totalValidPackets;
        double validFbPercent = (totalValidPackets == 0) ? 0 : 100.0 * validFbPacketCount / totalValidPackets;
        double skippedBytePercent = (receivedByteCount == 0) ? 0 : 100.0 * skippedByteCount / receivedByteCount;

        return String::Format(
            "Valid FA Packets: {0} ({1:F2}%)\n" +
            "Invalid FA Packets: {2}\n" +
            "Valid ASCII Packets: {3} ({4:F2}%)\n" +
            "Invalid ASCII Packets: {5}\n" +
            "Valid FB Packets: {6} ({7:F2}%)\n" +
            "Invalid FB Packets: {8}\n" +
            "Skipped Bytes: {9} ({10:F2}%)\n" +
            "Received Bytes: {11}\n" +
            "Total Valid Packet Count: {12}\n" +
            "Overall Packet Count: {13}\n",

            validFaPacketCount, validFaPercent,
            invalidFaPacketCount,
            validAsciiPacketCount, validAsciiPercent,
            invalidAsciiPacketCount,
            validFbPacketCount, validFbPercent,
            invalidFbPacketCount,
            skippedByteCount, skippedBytePercent,
            receivedByteCount,
            totalValidPackets,
            validFaPacketCount + invalidFaPacketCount + validAsciiPacketCount + invalidAsciiPacketCount + validFbPacketCount + invalidFbPacketCount
        );
    };
};

public ref class Exporter
{
protected:
    VN::Exporter *_exporter;
    VN::Filesystem::FilePath* _filePath;

public:
    Exporter(String^ filePath)
    {
        marshal_context^ context = gcnew marshal_context();
        _filePath = new VN::Filesystem::FilePath(context->marshal_as<const char*>(filePath));
        delete context;
    }

    virtual ~Exporter()
    {
        if (_exporter != nullptr)
        {
            delete _exporter;
        }

        delete _filePath;
    };

    VN::Exporter* GetRawExporter()
    {
        return _exporter;
    }

    VN::PacketQueue_Interface* GetQueuePointer()
    {
        return _exporter->getQueuePtr();
    }

    /// @brief start logging
    bool Start()
    {
        return _exporter->start();
    }

    /// @brief stop logging
    void Stop()
    {
        _exporter->stop();
    }

    /// @brief is logging
    bool IsLogging()
    {
        return _exporter->isLogging();
    }

    /// @brief export data to a file
    void ExportToFile()
    {
        _exporter->exportToFile();
    }

    /// @brief finalize the file before exporting
    virtual void FinalizeFile()
    {
    }

};

public ref class ExporterAscii : Exporter
{
    public:
    
    ExporterAscii(String ^filePath) : Exporter(filePath)
    {
        _exporter = new VN::ExporterAscii(*_filePath);
    }

}; // public ref class ExporterAscii

public ref class ExporterCsv : Exporter
{
public:
    
    ExporterCsv(String^ filePath) : Exporter(filePath)
    {
        _exporter = new VN::ExporterCsv(*_filePath, false);
    }

    ExporterCsv(String^ filePath, bool enableSystemTimeStamps) : Exporter(filePath)
    {
        _exporter = new VN::ExporterCsv(*_filePath, enableSystemTimeStamps);
    }

}; // public ref class ExporterAscii

//public ref class ExporterImr : Exporter
//{
//public:
//    
//    ExporterImr(String ^filePath) : Exporter(filePath)
//    {
//        _exporter = new VN::ExporterImr(*_filePath);
//    }

//    virtual void FinalizeFile() override
//    {
//        ((VN::ExporterImr *)_exporter)->finalize();
//    }

//}; // public ref class ExporterAscii

//public ref class ExporterRinex : Exporter
//{
//public:
//    
//    ExporterRinex(String ^filePath, UInt32 gnssGroup) : Exporter(filePath)
//    {
//        _exporter = new VN::ExporterRinex(*_filePath, gnssGroup);
//    }

//}; // public ref class ExporterAscii


public ref class ExporterSkippedByte : Exporter
{
private:
    VN::SkippedByteExporter* _skippedByteExporter;

public:
    ExporterSkippedByte(String^ filePath) : Exporter(filePath)
    {
        _skippedByteExporter = new VN::SkippedByteExporter(*_filePath);
    }

    ExporterSkippedByte(String^ filePath, size_t byteCapacity) : Exporter(filePath)
    {
        _skippedByteExporter = new VN::SkippedByteExporter(*_filePath, byteCapacity);
    }

    VN::SkippedByteExporter* GetRawExporter()
    {
        return _skippedByteExporter;
    }
};

public ref class FileExporter
{
private:
    VN::FileExporter *_fileExporter = nullptr;
    VN::Filesystem::FilePath* _filePath;

public:
    FileExporter()
    {
        _fileExporter = new VN::FileExporter();
    }

    ~FileExporter()
    {
        delete _fileExporter;
    }

    bool AddExporter(Exporter^ exporter)
    {
        return _fileExporter->addExporter(std::unique_ptr<VN::Exporter>(exporter->GetRawExporter()));
    }

    bool ProcessFile(String^ filePath)
    {
        marshal_context^ context = gcnew marshal_context();
        _filePath = new VN::Filesystem::FilePath(context->marshal_as<const char*>(filePath));
        delete context;

        return _fileExporter->processFile(*_filePath);
    }

    void AddSkippedByteExporter(ExporterSkippedByte^ exporter)
    {
        return _fileExporter->addSkippedByteExporter(std::unique_ptr<VN::SkippedByteExporter>(exporter->GetRawExporter()));
    }

    ParsingStats^ GetParsingStats()
    {
        VN::FileExporter::ParsingStats stats = _fileExporter->getParsingStats();

        ParsingStats^ cliStats = gcnew ParsingStats();
        cliStats->validFaPacketCount = stats.validFaPacketCount;
        cliStats->invalidFaPacketCount = stats.invalidFaPacketCount;
        cliStats->validAsciiPacketCount = stats.validAsciiPacketCount;
        cliStats->invalidAsciiPacketCount = stats.invalidAsciiPacketCount;
        cliStats->validFbPacketCount = stats.validFbPacketCount;
        cliStats->invalidFbPacketCount = stats.invalidFbPacketCount;
        cliStats->skippedByteCount = stats.skippedByteCount;
        cliStats->receivedByteCount = stats.receivedByteCount;

        return cliStats;
    }
};
} // namespace VNSDK
#endif  // VN_CLI_EXPORTER_HPP_