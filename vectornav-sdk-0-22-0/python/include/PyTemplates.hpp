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

// clang-format off
#ifndef VN_PYTEMPLATES_HPP_
#define VN_PYTEMPLATES_HPP_
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <iostream>
#include <string>

#include "vectornav/Implementation/QueueDefinitions.hpp"

namespace py = pybind11;

template<size_t M, size_t N, typename T>
void declare_matrix(py::module& m, const std::string& typestr) {
  using Class = VN::Matrix<M, N, T>;
  py::class_<Class>(m, typestr.c_str())
    .def(py::init<>())
    .def(py::init<T>())
    .def(py::init<std::array<T,M*N>>())
    .def("__repr__",
      [](const Class& mat) {
        std::string str = "[";
        for (int i = 0; i < M*N - 1; i++) {
          str += std::to_string(mat[i]) + ",";
        }
        str += std::to_string(mat[M*N - 1]) + "]";
        return str;
      }
    )
    .def("__setitem__", [](Class& self, size_t index, float val) { self[index] = val; })
    .def("__getitem__", [](const Class& self, size_t index) { return self[index]; })
    .def("__iter__",
      [](Class& self) {
        return py::make_iterator(&self[0], &self[0] + M*N);
      }, py::keep_alive<0,1>()
    );
}

template<typename T, size_t S>
void declare_vector(py::module &m, const std::string& typestr) {
  using Class = VN::Vector<T,S>;

  py::class_<Class>(m, typestr.c_str())
    .def(py::init<>())
    .def("push_back", py::overload_cast<const T&>(&Class::push_back))
    .def("pop_back", &Class::pop_back)
    .def("clear", &Class::clear)
    .def("full", &Class::full)
    .def("empty", &Class::capacity)
    .def("size", &Class::size)
    .def("insert", &Class::insert);

}

template<class T>
void declare_string(py::module &m, const std::string& typestr) {
  using Class = T;

  py::class_<Class>(m, typestr.c_str())
    .def(py::init<>())
    .def(py::init<const std::string&>())
    .def("__repr__",
      [](const Class& str) {
        return str.c_str();
      }
    );
}

template<typename T, size_t Capacity>
void declare_direct_access_queue(py::module& m, const std::string& typestr) {
  using Class = VN::DirectAccessQueue<T, Capacity>;
  py::class_<Class, VN::DirectAccessQueue_Interface<T>>(m, typestr.c_str());
}

template<typename T>
void declare_direct_access_queue_base(py::module& m, const std::string& typestr) {
  using Class = VN::DirectAccessQueue_Interface<T>;
  py::class_<Class>(m, typestr.c_str());
}

#endif  // VN_PYTEMPLATES_HPP_
// clang-format on
