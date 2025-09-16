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

#include <filesystem>
#include <string>

#include "vectornav/HAL/File.hpp"
#include "vectornav/Interface/Registers.hpp"
#include "vectornav/Interface/Sensor.hpp"
#include "vectornav/RegisterScan.hpp"

using namespace VN;

std::string usage = "[port] [fileLocation or \"generic\")";

enum class ReadType
{
    Xml,
    Ascii,
    Generic
};

int main(int argc, char* argv[])
{
    // This load configuration example will walk through how to load a configuration from an xml or ascii file.

    // [1] Handle input arguments
    if (argc > 3) { std::cerr << "Usage: LoadConfiguration " << usage << std::endl; };

    const std::string portName = (argc > 1) ? argv[1] : "COM10";  // Pass in port name as a positional argument, or edit here
    std::filesystem::path filePath = (argc > 2) ? argv[2] : (std::filesystem::path(__FILE__).parent_path() / "configSettings.xml");
    // std::filesystem::path filePath = "generic";

    ReadType readType{ReadType::Generic};
    if (filePath.string() == std::string("generic")) { readType = ReadType::Generic; }
    else
    {
        if (!std::filesystem::exists(filePath))
        {
            std::cerr << "Error: File does not exist: " << filePath.string() << std::endl;
            return 1;
        }
        std::filesystem::path extension = filePath.extension();
        if (extension.empty())
        {
            std::cerr << "Error: File path extension was not specified." << std::endl;
            return 1;
        }
        else if (extension == ".xml") { readType = ReadType::Xml; }
        else { readType = ReadType::Ascii; }
    }

    // [2] Instantiate and connect to the sensor
    Sensor sensor;
    Error latestError = sensor.autoConnect(portName);
    if (latestError != Error::None)
    {
        std::cerr << "Error " << latestError << " encountered when connecting to " + portName << ".\t" << std::endl;
        if (argc > 1) { std::cout << "Usage: ./LoadConfiguration " << usage << std::endl; };
        return static_cast<int>(latestError);
    }
    std::cout << "Connected to " << portName << " at " << sensor.connectedBaudRate().value() << std::endl;

    // [3] Load the configuration onto the sensor
    VN::Error err;
    if (readType == ReadType::Xml)
    {
        RegisterScan::XmlConfigReader configReader(filePath.string());
        err = RegisterScan::loadConfiguration(sensor, configReader);
    }
    else if (readType == ReadType::Ascii)
    {
        RegisterScan::AsciiConfigReader configReader(filePath.string());
        err = RegisterScan::loadConfiguration(sensor, configReader);
    }
    else  // readType == ReadType::Generic
    {
        uint16_t messageCount{0};
        std::vector<VN::AsciiMessage> messages{"$VNRRG,06,1,1*0865", "$VNRRG,07,1,1*A234", "$VNRRG,75,3,400,01,0321*3B31"};
        RegisterScan::GenericConfigReader configReader(
            [&messages, &messageCount](VN::AsciiMessage& msg)
            {
                if (messageCount < messages.size())
                {
                    msg = messages[messageCount];
                    messageCount++;
                    return VN::Error::None;
                }
                else { return VN::Error::FileReadFailed; }
            });
        err = RegisterScan::loadConfiguration(sensor, configReader);
    }
    if (err != VN::Error::None)
    {
        std::cerr << "Error " << err << " occured when loading configuration." << std::endl;
        return 1;
    }
    std::cout << "Configuration has been loaded to the sensor." << std::endl;

    sensor.disconnect();
    std::cout << "LoadConfiguration example complete." << std::endl;
}
