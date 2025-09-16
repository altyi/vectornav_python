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
#include <pybind11/stl/filesystem.h>

#include <cstdint>
#include <memory>
#include <sstream>

#include "vectornav/Exporter.hpp"
#include "vectornav/ExporterAscii.hpp"
#include "vectornav/ExporterCsv.hpp"
#include "vectornav/ExporterRinex.hpp"
#include "vectornav/FileExporter.hpp"
#include "vectornav/HAL/File.hpp"
#include "vectornav/SkippedByteExporter.hpp"

namespace py = pybind11;

namespace VN
{

void init_data_export(py::module& m)
{
    py::module Plugins = m.def_submodule("Plugins", "Plugins Module");

    py::class_<Exporter>(Plugins, "Exporter")
        .def("getQueuePtr", &Exporter::getQueuePtr, py::return_value_policy::reference)
        .def("start", &Exporter::start)
        .def("stop", &Exporter::stop)
        .def("isLogging", &Exporter::isLogging);

    py::class_<ExporterCsv, Exporter>(Plugins, "ExporterCsv")
        .def(py::init<const Filesystem::FilePath&, bool>(), py::arg("path"), py::arg("enableSystemTimeStamps") = false)
        .def("exportToFile", &ExporterCsv::exportToFile);

    py::class_<ExporterRinex, Exporter>(Plugins, "ExporterRinex")
        .def(py::init<const Filesystem::FilePath&, const uint32_t>())
        .def("exportToFile", &ExporterRinex::exportToFile);

    py::class_<ExporterAscii, Exporter>(Plugins, "ExporterAscii")
        .def(py::init<const Filesystem::FilePath&>())
        .def("exportToFile", &ExporterAscii::exportToFile);

    py::class_<SkippedByteExporter>(Plugins, "SkippedByteExporter")
        .def(py::init<const Filesystem::FilePath&>(), py::arg("path"))
        .def("exportToFile", &SkippedByteExporter::exportToFile);

    py::class_<FileExporter>(Plugins, "FileExporter")
        .def(py::init<>())
        .def("processFile", &FileExporter::processFile)
        .def("addCsvExporter", [](FileExporter& f, const Filesystem::FilePath& path, bool enableSystemTimeStamps = false)
             { f.addExporter(std::make_unique<ExporterCsv>(path, enableSystemTimeStamps)); })
        .def("addAsciiExporter", [](FileExporter& f, const Filesystem::FilePath& path) { f.addExporter(std::make_unique<ExporterAscii>(path)); })
        .def("addRinexExporter", [](FileExporter& f, const Filesystem::FilePath& path, const uint32_t gnssGroup)
             { f.addExporter(std::make_unique<ExporterRinex>(path, gnssGroup)); })
        .def("addSkippedByteExporter",
             [](FileExporter& f, const Filesystem::FilePath& path) { f.addSkippedByteExporter(std::make_unique<SkippedByteExporter>(path)); })
        .def("getParsingStats", [](FileExporter& f) -> FileExporter::ParsingStats { return f.getParsingStats(); });

    py::class_<FileExporter::ParsingStats>(Plugins, "ParsingStats")
        .def_readwrite("validFaPacketCount", &FileExporter::ParsingStats::validFaPacketCount)
        .def_readwrite("invalidFaPacketCount", &FileExporter::ParsingStats::invalidFaPacketCount)
        .def_readwrite("validAsciiPacketCount", &FileExporter::ParsingStats::validAsciiPacketCount)
        .def_readwrite("invalidAsciiPacketCount", &FileExporter::ParsingStats::invalidAsciiPacketCount)
        .def_readwrite("validFbPacketCount", &FileExporter::ParsingStats::validFbPacketCount)
        .def_readwrite("invalidFbPacketCount", &FileExporter::ParsingStats::invalidFbPacketCount)
        .def_readwrite("skippedByteCount", &FileExporter::ParsingStats::skippedByteCount)
        .def_readwrite("receivedByteCount", &FileExporter::ParsingStats::receivedByteCount)
        .def("__str__",
             [](const FileExporter::ParsingStats& stats)
             {
                 std::stringstream ss;

                 uint64_t totalValidPackets = stats.validFaPacketCount + stats.validAsciiPacketCount + stats.validFbPacketCount;
                 double validFaPercent = (totalValidPackets == 0) ? 0 : 100.0 * stats.validFaPacketCount / totalValidPackets;
                 double validAsciiPercent = (totalValidPackets == 0) ? 0 : 100.0 * stats.validAsciiPacketCount / totalValidPackets;
                 double validFbPercent = (totalValidPackets == 0) ? 0 : 100.0 * stats.validFbPacketCount / totalValidPackets;
                 double skippedBytePercent = (stats.receivedByteCount == 0) ? 0 : 100.0 * stats.skippedByteCount / stats.receivedByteCount;

                 ss << "Valid FA Packets: " << stats.validFaPacketCount << " (" << std::fixed << std::setprecision(2) << validFaPercent << "%)\n";
                 ss << "Invalid FA Packets: " << stats.invalidFaPacketCount << "\n";
                 ss << "Valid ASCII Packets: " << stats.validAsciiPacketCount << " (" << std::fixed << std::setprecision(2) << validAsciiPercent << "%)\n";
                 ss << "Invalid ASCII Packets: " << stats.invalidAsciiPacketCount << "\n";
                 ss << "Valid FB Packets: " << stats.validFbPacketCount << " (" << std::fixed << std::setprecision(2) << validFbPercent << "%)\n";
                 ss << "Invalid FB Packets: " << stats.invalidFbPacketCount << "\n";
                 ss << "Skipped Bytes: " << stats.skippedByteCount << " (" << std::fixed << std::setprecision(2) << skippedBytePercent << "%)\n";
                 ss << "Received Bytes: " << stats.receivedByteCount << "\n";
                 ss << "Total Valid Packet Count: " << totalValidPackets << "\n";
                 ss << "Overall Packet Count: "
                    << stats.validFaPacketCount + stats.invalidFaPacketCount + stats.validAsciiPacketCount + stats.invalidAsciiPacketCount +
                           stats.validFbPacketCount + stats.invalidFbPacketCount
                    << "\n";

                 return ss.str();
             });

    py::implicitly_convertible<py::str, Filesystem::FilePath>();
}

}  // namespace VN
