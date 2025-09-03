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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "FirmwareUpdate/include/vectornav/FirmwareUpdater.hpp"
#include "vectornav/TemplateLibrary/Vector.hpp"
#include "vectornav/TemplateLibrary/String.hpp"
#include "vectornav/Interface/Sensor.hpp"
#include "vectornav/HAL/File.hpp"

#include "PyTemplates.hpp"

namespace py = pybind11;

namespace VN {
  
void init_firmware_updater(py::module& m) {

  py::module Plugins = m.def_submodule("Plugins", "Plugins Module");

  py::class_<FirmwareUpdater> firmwareUpdater(Plugins, "FirmwareUpdater");

  firmwareUpdater.def(py::init<>())
    .def("updateFirmware", py::overload_cast<Sensor*, Filesystem::FilePath, FirmwareUpdater::Params>(&FirmwareUpdater::updateFirmware))
    .def("updateFirmware", py::overload_cast<Sensor*, FirmwareUpdater::FilePaths, FirmwareUpdater::Params>(&FirmwareUpdater::updateFirmware));

  py::class_<FirmwareUpdater::Params> params(firmwareUpdater, "Params");

  params.def(py::init<>())
    .def(py::init<Sensor::BaudRate, Sensor::BaudRate>())
    .def_readwrite("firmwareBaudRate",  &FirmwareUpdater::Params::firmwareBaudRate)
    .def_readwrite("bootloaderBaudRate", &FirmwareUpdater::Params::bootloaderBaudRate);

  py::enum_<FirmwareUpdater::Processor>(firmwareUpdater, "Processor", py::module_local())
    .value("Imu",  FirmwareUpdater::Processor::Imu)
    .value("Gnss", FirmwareUpdater::Processor::Gnss)
    .value("Nav",  FirmwareUpdater::Processor::Nav)
    .value("Poll",  FirmwareUpdater::Processor::Poll);
    
  py::class_<FirmwareUpdater::File>(firmwareUpdater, "File")
    .def(py::init<>())
    .def(py::init<Filesystem::FilePath&, FirmwareUpdater::Processor>())
    .def_readwrite("filePath",  &FirmwareUpdater::File::filePath)
    .def_readwrite("processor", &FirmwareUpdater::File::processor);

  declare_vector<FirmwareUpdater::File, 6>(m, "FilePaths");

  declare_string<Filesystem::FilePath>(m, "Path");
  py::implicitly_convertible<py::str, Filesystem::FilePath>();
    
}} // namespace VN
