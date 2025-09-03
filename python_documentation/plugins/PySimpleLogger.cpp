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

#include "SimpleLogger/SimpleLogger.hpp"
#include "vectornav/HAL/File.hpp"
#include "vectornav/TemplateLibrary/String.hpp"
#include "vectornav/TemplateLibrary/ByteBuffer.hpp"

namespace py = pybind11;

namespace VN {

void init_simple_logger(py::module& m) {
  
  py::module Plugins = m.def_submodule("Plugins", "Plugins Module");
  
  py::class_<SimpleLogger> simpleLogger(Plugins, "SimpleLogger");
  
  simpleLogger.def(py::init<ByteBuffer&, Filesystem::FilePath&>())
    .def("start", &SimpleLogger::start)
    .def("stop", &SimpleLogger::stop)
    .def("isLogging", &SimpleLogger::isLogging)
    .def("numBytesLogged", &SimpleLogger::numBytesLogged);
}

} // namespace VN
