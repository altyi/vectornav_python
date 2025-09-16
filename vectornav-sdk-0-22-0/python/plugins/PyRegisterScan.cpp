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
#include "RegisterScan/include/vectornav/RegisterScan.hpp"

#include "PyTemplates.hpp"
namespace py = pybind11;
namespace VN {
  
void init_register_scan(py::module& m) {

  py::module Plugins = m.def_submodule("Plugins", "Plugins Module");


  py::class_<RegisterScan::ConfigWriter<RegisterScan::AsciiConfigWriter>>(Plugins, "ConfigWriter_Ascii")
    .def("writeConfig", &RegisterScan::ConfigWriter<RegisterScan::AsciiConfigWriter>::writeConfig)
    .def("close", &RegisterScan::ConfigWriter<RegisterScan::AsciiConfigWriter>::close);

  py::class_<RegisterScan::AsciiConfigWriter, RegisterScan::ConfigWriter<RegisterScan::AsciiConfigWriter>>(Plugins, "AsciiConfigWriter")
    .def(py::init<Filesystem::FilePath&>())
    .def("write", &RegisterScan::AsciiConfigWriter::write)
    .def("finalize", &RegisterScan::AsciiConfigWriter::finalize);  

  py::class_<RegisterScan::ConfigWriter<RegisterScan::GenericConfigWriter>>(Plugins, "ConfigWriter_Generic")
    .def("writeConfig", &RegisterScan::ConfigWriter<RegisterScan::GenericConfigWriter>::writeConfig)
    .def("close", &RegisterScan::ConfigWriter<RegisterScan::GenericConfigWriter>::close);

  py::class_<RegisterScan::GenericConfigWriter, RegisterScan::ConfigWriter<RegisterScan::GenericConfigWriter>>(Plugins, "GenericConfigWriter")
    .def(py::init<std::function<VN::Error(const AsciiMessage&)>, std::function<void()>>(), 
      py::arg("writeConfig"), py::arg("finalizeConfig") = std::function<void()>([]() {}))  
    .def("write", &RegisterScan::GenericConfigWriter::write)  
    .def("finalize", &RegisterScan::GenericConfigWriter::finalize); 
          
  py::class_<RegisterScan::ConfigWriter<RegisterScan::XmlConfigWriter>>(Plugins, "ConfigWriter_Xml")
    .def("writeConfig", &RegisterScan::ConfigWriter<RegisterScan::XmlConfigWriter>::writeConfig)
    .def("close", &RegisterScan::ConfigWriter<RegisterScan::XmlConfigWriter>::close);

  py::class_<RegisterScan::XmlConfigWriter, RegisterScan::ConfigWriter<RegisterScan::XmlConfigWriter>>(Plugins, "XmlConfigWriter")
    .def(py::init<Sensor&, Filesystem::FilePath>())
    .def("write", &RegisterScan::XmlConfigWriter::write)
    .def("finalize", &RegisterScan::XmlConfigWriter::finalize);

  py::enum_<RegisterScan::SaveConfigurationFilter::Type>(Plugins, "SaveConfigFilterType", py::arithmetic())
    .value("Include", RegisterScan::SaveConfigurationFilter::Type::Include)
    .value("Exclude", RegisterScan::SaveConfigurationFilter::Type::Exclude);

  py::class_<RegisterScan::SaveConfigurationFilter>(Plugins, "SaveConfigurationFilter")
    .def(py::init<>())
    .def_readwrite("type", &RegisterScan::SaveConfigurationFilter::type)
    .def_readwrite("list", &RegisterScan::SaveConfigurationFilter::list);

    Plugins.def("saveConfiguration", py::overload_cast<Sensor&, RegisterScan::ConfigWriter<RegisterScan::AsciiConfigWriter>&, RegisterScan::SaveConfigurationFilter>(&RegisterScan::saveConfiguration<RegisterScan::AsciiConfigWriter>),
        py::arg("sensor"),
        py::arg("configWriter"),
        py::arg("filter") = RegisterScan::SaveConfigurationFilter{RegisterScan::SaveConfigurationFilter::Type::Include, RegisterScan::getDefaultConfigRegisters()},
        "Save config to an Ascii file");
    Plugins.def("saveConfiguration", py::overload_cast<Sensor&, RegisterScan::ConfigWriter<RegisterScan::XmlConfigWriter>&, RegisterScan::SaveConfigurationFilter>(&RegisterScan::saveConfiguration<RegisterScan::XmlConfigWriter>),
        py::arg("sensor"),
        py::arg("configWriter"),
        py::arg("filter") = RegisterScan::SaveConfigurationFilter{RegisterScan::SaveConfigurationFilter::Type::Include, RegisterScan::getDefaultConfigRegisters()},
        "Save config to an Xml file");
    Plugins.def("saveConfiguration", py::overload_cast<Sensor&, RegisterScan::ConfigWriter<RegisterScan::GenericConfigWriter>&, RegisterScan::SaveConfigurationFilter>(&RegisterScan::saveConfiguration<RegisterScan::GenericConfigWriter>),
        py::arg("sensor"),
        py::arg("configWriter"),
        py::arg("filter") = RegisterScan::SaveConfigurationFilter{RegisterScan::SaveConfigurationFilter::Type::Include, RegisterScan::getDefaultConfigRegisters()},
        "Save config generically");

    Plugins.def("saveNonDefaultConfiguration", py::overload_cast<Sensor&, RegisterScan::ConfigWriter<RegisterScan::AsciiConfigWriter>&>(&RegisterScan::saveNonDefaultConfiguration<RegisterScan::AsciiConfigWriter>));
    Plugins.def("saveNonDefaultConfiguration", py::overload_cast<Sensor&, RegisterScan::ConfigWriter<RegisterScan::XmlConfigWriter>&>(&RegisterScan::saveNonDefaultConfiguration<RegisterScan::XmlConfigWriter>));
    Plugins.def("saveNonDefaultConfiguration", py::overload_cast<Sensor&, RegisterScan::ConfigWriter<RegisterScan::GenericConfigWriter>&>(&RegisterScan::saveNonDefaultConfiguration<RegisterScan::GenericConfigWriter>));
    
    
    Plugins.def("getDefaultConfigRegisters", &RegisterScan::getDefaultConfigRegisters);

    py::class_<RegisterScan::ConfigReader<RegisterScan::AsciiConfigReader>>(Plugins, "ConfigReader_Ascii")
        .def("getNextConfig", &RegisterScan::ConfigReader<RegisterScan::AsciiConfigReader>::getNextConfig);

    py::class_<RegisterScan::AsciiConfigReader, RegisterScan::ConfigReader<RegisterScan::AsciiConfigReader>>(Plugins, "AsciiConfigReader")
        .def(py::init<const Filesystem::FilePath&>())
        .def("next", &RegisterScan::AsciiConfigReader::next);

    py::class_<RegisterScan::ConfigReader<RegisterScan::XmlConfigReader>>(Plugins, "ConfigReader_Xml")
        .def("getNextConfig", &RegisterScan::ConfigReader<RegisterScan::XmlConfigReader>::getNextConfig);

    py::class_<RegisterScan::XmlConfigReader, RegisterScan::ConfigReader<RegisterScan::XmlConfigReader>>(Plugins, "XmlConfigReader")
        .def(py::init<const Filesystem::FilePath&>())
        .def("next", &RegisterScan::XmlConfigReader::next);

    py::class_<RegisterScan::ConfigReader<RegisterScan::GenericConfigReader>>(Plugins, "ConfigReader_Generic")
        .def("getNextConfig", &RegisterScan::ConfigReader<RegisterScan::GenericConfigReader>::getNextConfig);

    py::class_<RegisterScan::GenericConfigReader, RegisterScan::ConfigReader<RegisterScan::GenericConfigReader>>(Plugins, "GenericConfigReader")
        .def(py::init<std::function<VN::Error(AsciiMessage&)>>())
        .def("next", &RegisterScan::GenericConfigReader::next);

    Plugins.def("loadConfiguration", py::overload_cast<Sensor&, RegisterScan::ConfigReader<RegisterScan::AsciiConfigReader>&>(&RegisterScan::loadConfiguration<RegisterScan::AsciiConfigReader>));
    Plugins.def("loadConfiguration", py::overload_cast<Sensor&, RegisterScan::ConfigReader<RegisterScan::XmlConfigReader>&>(&RegisterScan::loadConfiguration<RegisterScan::XmlConfigReader>));
    Plugins.def("loadConfiguration", py::overload_cast<Sensor&, RegisterScan::ConfigReader<RegisterScan::GenericConfigReader>&>(&RegisterScan::loadConfiguration<RegisterScan::GenericConfigReader>));      

  declare_vector<uint8_t, 256>(m, "RegisterList");
  py::implicitly_convertible<py::list, Vector<uint8_t, 256>>();
          
}} // namespace VN

