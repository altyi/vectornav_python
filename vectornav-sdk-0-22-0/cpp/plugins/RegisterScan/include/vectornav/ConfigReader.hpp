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

#ifndef VN_CONFIGREADER_HPP_
#define VN_CONFIGREADER_HPP_

#include <cstdint>
#include <functional>

#include "pugixml/pugixml.hpp"
#include "vectornav/HAL/File.hpp"
#include "vectornav/Interface/Sensor.hpp"
#include "vectornav/TemplateLibrary/String.hpp"

namespace VN
{

namespace RegisterScan
{

template <typename T>
struct ConfigReader
{
    VN::Error getNextConfig(AsciiMessage& msg) { return static_cast<T*>(this)->next(msg); }
};

class GenericConfigReader : public ConfigReader<GenericConfigReader>
{
public:
    GenericConfigReader(std::function<VN::Error(AsciiMessage&)> getNextConfig) : _next(getNextConfig) {}

    VN::Error next(AsciiMessage& msg)
    {
        VN::Error error = _next(msg);
        return error;
    }

private:
    std::function<VN::Error(AsciiMessage&)> _next = nullptr;
};

class AsciiConfigReader : public ConfigReader<AsciiConfigReader>
{
public:
    AsciiConfigReader(const Filesystem::FilePath& path) : file(path) {}

    VN::Error next(AsciiMessage& msg);

private:
    InputFile file;
};

class XmlConfigReader : public ConfigReader<XmlConfigReader>
{
public:
    XmlConfigReader(const Filesystem::FilePath& path) : _path(path) {}

    VN::Error next(AsciiMessage& msg);

private:
    bool double_config = false;
    pugi::xml_node _curr_node;
    pugi::xml_node _next_node;
    Filesystem::FilePath _path;
    pugi::xml_document _doc;
};

}  // namespace RegisterScan
}  // namespace VN

#endif  // VN_CONFIGREADER_HPP_
