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

#ifndef VN_CLI_FIRMWAREUPDATE_HPP_
#define VN_CLI_FIRMWAREUPDATE_HPP_

#include "vectornav/CLI_Sensor.hpp"

#pragma managed(push, off)
#include "vectornav/FirmwareUpdater.hpp"
#include "vectornav/HAL/File.hpp"
#pragma managed(pop)

#include <msclr/marshal_cppstd.h>

using namespace System;
namespace VNSDK
{

public
ref class FirmwareUpdater
{
  private:
    VN::FirmwareUpdater &_firmwareUpdater = VN::FirmwareUpdater();

  public:
    enum class Processor {
        Nav = VN::SetBootLoader::Processor::Nav,
        Gnss = VN::SetBootLoader::Processor::Gnss,
        Imu = VN::SetBootLoader::Processor::Imu
    };

    ref class FirmwareFile {
      private:
        Processor _processorId;
        String^ _filePath;

      public:

        FirmwareFile() {
            _processorId = Processor::Nav;
            _filePath = "";
        }

        FirmwareFile(Processor ProcessorId, String^ FilePath) {
            _processorId = ProcessorId;
            _filePath = FilePath;
        }


        VN::FirmwareUpdater::File ConvertToVnFile() {
            VN::FirmwareUpdater::File file;
            file.processor = static_cast<VN::FirmwareUpdater::Processor>(_processorId);
            msclr::interop::marshal_context context;
            file.filePath = context.marshal_as<std::string>(_filePath);

            return file;
        }
    };

    value class Params
    {
      private:
        UInt32 FirmwareBaudRate_;
        UInt32 BootloaderBaudRate_;

      public:
        Params(UInt32 FirmwareBaudRate, UInt32 BootloaderBaudRate) {
            FirmwareBaudRate_ = FirmwareBaudRate;
            BootloaderBaudRate_ = BootloaderBaudRate;
        }

        VN::FirmwareUpdater::Params ConvertToVnParams() {
            VN::FirmwareUpdater::Params params;
            params.bootloaderBaudRate = static_cast<VN::Sensor::BaudRate>(BootloaderBaudRate_);
            params.firmwareBaudRate = static_cast<VN::Sensor::BaudRate>(FirmwareBaudRate_);

            return params;
        }
    };


    FirmwareUpdater() {}

    void UpdateFirmware(Sensor ^ % sensor, String^ vnxmlPath, Params params)
    {
        std::string unmanagedPath = msclr::interop::marshal_as<std::string>(vnxmlPath);
        const bool failed = _firmwareUpdater.updateFirmware(&sensor->GetNativeInstance(), VN::Filesystem::FilePath(unmanagedPath), params.ConvertToVnParams());
        if (failed) {
            throw gcnew System::Exception("Firmware update failed");
        }
    }

    void UpdateFirmware(Sensor ^ % sensor, String^ vnxPath, Processor processorId, Params params) {
        std::string unmanagedPath = msclr::interop::marshal_as<std::string>(vnxPath);
        VN::InputFile vnxFile{VN::Filesystem::FilePath(unmanagedPath)};
        const bool failed = _firmwareUpdater.updateFirmware(&sensor->GetNativeInstance(), vnxFile, static_cast<VN::FirmwareUpdater::Processor>(processorId), params.ConvertToVnParams());
        if (failed) {
            throw gcnew System::Exception("Firmware update failed");
        }
    }

    void UpdateFirmware(Sensor ^ % sensor, List<FirmwareFile^>^ files, Params params) {
        VN::FirmwareUpdater::FilePaths vnFiles{};
        for (int i{0}; i != files->Count; ++i) {
            vnFiles.push_back(files[i]->ConvertToVnFile());
        }

        const bool failed = _firmwareUpdater.updateFirmware(&sensor->GetNativeInstance(), vnFiles, params.ConvertToVnParams());
        if (failed) {
            throw gcnew System::Exception("Firmware update failed");
        }
    }
};
} // namespace VNSDK

#endif  // VN_CLI_FIRMWAREUPDATE_HPP_
