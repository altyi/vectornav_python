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

#ifndef VN_CONFIGWRITER_HPP_
#define VN_CONFIGWRITER_HPP_

#include <cstdint>
#include <functional>
#include <map>

#include "pugixml/pugixml.hpp"
#include "vectornav/HAL/File.hpp"
#include "vectornav/Interface/Sensor.hpp"
#include "vectornav/TemplateLibrary/String.hpp"

namespace VN
{
namespace RegisterScan
{

template <typename T>
struct ConfigWriter
{
    VN::Error writeConfig(const AsciiMessage& msg) { return static_cast<T*>(this)->write(msg); }
    void close() { static_cast<T*>(this)->finalize(); }
};

class AsciiConfigWriter : public ConfigWriter<AsciiConfigWriter>
{
public:
    AsciiConfigWriter(const Filesystem::FilePath& path) : file(path) {}

    VN::Error write(const AsciiMessage& msg)
    {
        if (file.write(msg.c_str())) { return VN::Error::FileWriteFailed; }
        else { return VN::Error::None; }
    }

    void finalize()
    {
        if (file.is_open()) { file.close(); }
    }

private:
    OutputFile file;
};

class GenericConfigWriter : public ConfigWriter<GenericConfigWriter>
{
public:
    GenericConfigWriter(
        std::function<VN::Error(const AsciiMessage& msg)> writeConfig, std::function<void()> finalizeConfig = []() {})
        : _writeConfig(writeConfig), _finalizeConfig(finalizeConfig)
    {
    }

    VN::Error write(const AsciiMessage& msg) { return _writeConfig(msg); }

    void finalize() { _finalizeConfig(); }

private:
    std::function<VN::Error(const AsciiMessage& msg)> _writeConfig;
    std::function<void()> _finalizeConfig;
};

class XmlConfigWriter : public ConfigWriter<XmlConfigWriter>
{
public:
    XmlConfigWriter(Sensor& sensor, const Filesystem::FilePath path) : sensor(sensor), _path(path) {}

    VN::Error write(const AsciiMessage& msg);

    void finalize();

private:
    Sensor& sensor;
    Filesystem::FilePath _path;
    bool _initialized = false;
    pugi::xml_document _doc;
    pugi::xml_node _root;

    AsciiMessage _reg5 = "";
    AsciiMessage _reg6 = "";
    AsciiMessage _reg7 = "";
    AsciiMessage _reg99 = "";

    VN::Error _initialize();
    VN::Error _write(const AsciiMessage& msg, const uint16_t regId);
};

}  // namespace RegisterScan
}  // namespace VN

#endif  // VN_CONFIGWRITER_HPP_
