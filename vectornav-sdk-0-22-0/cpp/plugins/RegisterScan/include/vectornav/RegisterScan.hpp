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

#ifndef VN_REGISTERSCAN_HPP_
#define VN_REGISTERSCAN_HPP_

#include <utility>

#include "vectornav/ConfigReader.hpp"
#include "vectornav/ConfigWriter.hpp"
#include "vectornav/Interface/Registers.hpp"

namespace VN
{

namespace RegisterScan
{

constexpr uint16_t NUM_REG = 256;

VN::Error _setConfigurationRegister(Sensor& sensor, const AsciiMessage& msg);

template <typename T>
VN::Error setConfigurationRegisters(Sensor& sensor, ConfigReader<T>& configReader)
{
    AsciiMessage msg;
    VN::Error err{VN::Error::None};
    bool readFlag = false;
    while (configReader.getNextConfig(msg) == VN::Error::None)
    {
        readFlag = true;
        VN::Error err = _setConfigurationRegister(sensor, msg);
        if (err != VN::Error::None) { return err; };
    }
    if (readFlag == false) { err = VN::Error::FileReadFailed; }
    return err;
}

template <typename T>
VN::Error loadConfiguration(Sensor& sensor, ConfigReader<T>& configReader)
{
    VN::Error error = sensor.restoreFactorySettings();
    if (error != VN::Error::None) { return error; }
    VN::Error err = setConfigurationRegisters(sensor, configReader);
    if (err != VN::Error::None) return err;

    error = sensor.writeSettings();
    if (error != Error::None) { return error; }
    error = sensor.reset();
    if (error != Error::None) { return error; }

    return error;
}

struct SaveConfigurationFilter
{
    enum class Type
    {
        Include,
        Exclude
    } type;
    Vector<uint8_t, NUM_REG> list;
};

template <typename T>
VN::Error _saveConfigurationRegister(Sensor& sensor, ConfigWriter<T>& configWriter, uint8_t regId)
{
    std::cout << "Polling register " << std::to_string(regId) << "...\n";
    AsciiMessage msg;
    GenericCommand cmd;
    Error err{VN::Error::None};

    if (!((regId == 5) || (regId == 6) || (regId == 7) || (regId == 99)))
    {
        if (regId == 250) { return Error::None; }  // invalid register

        AsciiMessage msg;
        std::snprintf(msg.data(), msg.capacity(), "RRG,%d", regId);
        cmd = GenericCommand(msg);
        err = sensor.sendCommand(&cmd, Sensor::SendCommandBlockMode::BlockWithRetry);
        if (err == Error::InvalidRegister || err == Error::UnauthorizedAccess || err == Error::NotEnoughParameters) { return VN::Error::None; }
        else if (err != Error::None) { return err; }

        AsciiMessage tmp = cmd.getResponse();
        const auto start = tmp.find(',');
        const auto end = tmp.find('*');

        if (tmp.find(',', start + 1) == AsciiMessage::npos) { return VN::Error::None; }  // catches responses with no arguments

        std::snprintf(tmp.data(), tmp.capacity(), "WRG%.*s", int(end - start), &tmp[start]);
        auto wrg = GenericCommand(tmp);

        err = sensor.sendCommand(&wrg, Sensor::SendCommandBlockMode::BlockWithRetry);
        if (err == Error::InvalidRegister || err == Error::UnauthorizedAccess)
        {
            // Read-only reg. (Some registers incorrectly report InvalidRegister, i.e. VN-100 WRG 101)
            err = VN::Error::None;
            return err;
        }
        else if (err != Error::None) { return err; }
        err = configWriter.writeConfig(cmd.getResponse());
        if (err != VN::Error::None) { return err; }
    }
    else  // (regId == 5) || (regId == 6) || (regId == 7) || (regId == 99))
    {
        std::snprintf(msg.data(), msg.capacity(), "RRG,%02d,1", regId);
        cmd = GenericCommand(msg);
        err = sensor.sendCommand(&cmd, Sensor::SendCommandBlockMode::BlockWithRetry);
        if (regId == 99 && err == Error::None)
        {
            // Some firmwares do not correctly report TooManyParameters, so we have to manually check whether it echoed correctly to see if this firmware
            // supports the optional parameter. A standard "RRG,99" call has 9 commas, so if there are only 9 commas the ",1" was not postpended, and the
            // optional parameter was ignored.
            const auto rsp = cmd.getResponse();
            const uint16_t commaCount = std::count(rsp.begin(), rsp.end(), ',');
            if (commaCount == 9) { err = Error::TooManyParameters; }
        }
        if (err == Error::None)
        {
            err = configWriter.writeConfig(cmd.getResponse());
            if (err != VN::Error::None) { return err; }
        }
        else if (err == Error::InvalidRegister)  // Unsupported reg
        {
            err = VN::Error::None;
            return err;
        }
        else if (err == Error::TooManyParameters)
        {
            std::snprintf(msg.data(), msg.capacity(), "RRG,%02d", regId);
            cmd = GenericCommand(msg);
            err = sensor.sendCommand(&cmd, Sensor::SendCommandBlockMode::BlockWithRetry);
            if (err == Error::None) { err = configWriter.writeConfig(cmd.getResponse()); }
            return err;
        }
        else { return err; }
        for (uint16_t i = 2;; ++i)
        {
            std::snprintf(msg.data(), msg.capacity(), "RRG,%02d,%d", regId, int(i));
            cmd = GenericCommand(msg);
            err = sensor.sendCommand(&cmd, Sensor::SendCommandBlockMode::BlockWithRetry);
            if (err == Error::None)
            {
                err = configWriter.writeConfig(cmd.getResponse());
                if (err != VN::Error::None) { return err; }
            }
            else if (err == Error::InvalidParameter)
            {
                err = VN::Error::None;
                return err;
            }
            else { return err; }
        }
    }

    return err;
}

constexpr Vector<uint8_t, NUM_REG> getDefaultConfigRegisters()
{
    Vector<uint8_t, NUM_REG> retVal;
    for (auto i = 0; i < 255; ++i)
    {
        if (Registers::getConfigRegisterName(i)) { retVal.push_back(i); }
    }
    return retVal;
}

template <typename T>
VN::Error saveConfiguration(Sensor& sensor, ConfigWriter<T>& configWriter,
                            SaveConfigurationFilter filter = SaveConfigurationFilter{SaveConfigurationFilter::Type::Include, getDefaultConfigRegisters()})
{
    VN::Error error{VN::Error::None};
    Vector<uint8_t, NUM_REG> reg_to_poll;
    if (filter.type == SaveConfigurationFilter::Type::Include) { reg_to_poll = filter.list; }
    else
    {
        for (uint16_t i = 0; i <= 255; ++i)
        {
            if (std::find(filter.list.begin(), filter.list.end(), i) == filter.list.end())
            {
                if (reg_to_poll.push_back(i))
                {
                    std::cerr << "Filter vector overload.\n";
                    VN_ABORT();
                }
            }
        }
    }

    error = sensor.asyncOutputEnable(AsyncOutputEnable::State::Disable);
    if (error != Error::None) { return error; }

    for (uint8_t regId : reg_to_poll)
    {
        error = _saveConfigurationRegister(sensor, configWriter, regId);
        if (error != VN::Error::None) { return error; }
    }

    configWriter.close();
    error = sensor.asyncOutputEnable(AsyncOutputEnable::State::Enable);
    if (error != Error::None) { return error; }

    return error;
}

template <typename T>
VN::Error saveNonDefaultConfiguration(Sensor& sensor, ConfigWriter<T>& configWriter)
{
    const Filesystem::FilePath tmpFileName = "UserSettings.tmp.txt";
    AsciiConfigWriter tmpWriter(tmpFileName);
    std::cout << "Collecting user settings...\n";
    VN::Error error = saveConfiguration(sensor, tmpWriter);
    if (error != Error::None) { return error; }
    error = sensor.restoreFactorySettings();
    if (error != Error::None) { return error; }

    AsciiConfigReader tmpReader(tmpFileName);
    GenericConfigWriter writeNonDefault{[&tmpReader, &sensor, &configWriter](const AsciiMessage& defaultMsg)
                                        {
                                            AsciiMessage userMsg;
                                            VN::Error err = tmpReader.next(userMsg);
                                            if (err != VN::Error::None) { return err; }
                                            // Only compare until the asterisk (crc might be different formats, and line endings change when reading from file
                                            // due to getLine)
                                            const auto userAsteriskPos = std::find(userMsg.begin(), userMsg.end(), '*');
                                            const auto defaultAsteriskPos = std::find(defaultMsg.begin(), defaultMsg.end(), '*');
                                            const uint16_t userLength = std::distance(userMsg.begin(), userAsteriskPos);
                                            const uint16_t defaultLength = std::distance(defaultMsg.begin(), defaultAsteriskPos);
                                            if (userLength != defaultLength || !std::equal(userMsg.begin(), userAsteriskPos, defaultMsg.begin()))
                                            {
                                                err = _setConfigurationRegister(sensor, userMsg);
                                                if (err != VN::Error::None) { return err; }
                                                err = configWriter.writeConfig(userMsg);
                                                return err;
                                            }
                                            else { return err; }
                                        },
                                        [&configWriter]() { configWriter.close(); }};

    std::cout << "Collecting default settings and writing diffs...\n";
    error = saveConfiguration(sensor, writeNonDefault);
    if (error != VN::Error::None) { return error; }

    std::cout << "Issuing Write Settings and Reset...\n";
    error = sensor.writeSettings();
    if (error != Error::None) { return error; }
    error = sensor.reset();
    if (error != Error::None) { return error; }
    return error;
}

}  // namespace RegisterScan
}  // namespace VN

#endif  // VN_REGISTERSCAN_HPP_
