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



#include "vectornav/CLI_Sensor.hpp"
#include "vectornav/CLI_Commands.hpp"

#include <conio.h>
#include <msclr/marshal.h>

#include <cstdint>
#include <string>

using namespace msclr::interop;

namespace VNSDK
{
void Sensor::Connect(String ^ portName, UInt32 baudRate)
{
    marshal_context ^ context = gcnew marshal_context();
    VN::Error error = _sensor->connect(context->marshal_as<const char*>(portName), static_cast<VN::Registers::System::BaudRate::BaudRates>(baudRate));
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
    delete context;
}

void Sensor::AutoConnect(String ^ portName)
{
    marshal_context ^ context = gcnew marshal_context();
    VN::Error error = _sensor->autoConnect(context->marshal_as<const char*>(portName));
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
    delete context;
}

bool Sensor::VerifySensorConnectivity()
{
    return _sensor->verifySensorConnectivity();
}

String ^ Sensor::ConnectedPortName()
{
    auto connectedPortName = _sensor->connectedPortName();
    if (!connectedPortName.has_value())
    {
        throw gcnew VnException(VN::Error::SerialPortClosed);
    }
    String ^ portName = msclr::interop::marshal_as<String ^>(connectedPortName.value().c_str());
    return portName;
}

uint32_t Sensor::ConnectedBaudRate()
{
    auto connectedBaudRate = _sensor->connectedBaudRate();
    if (!connectedBaudRate.has_value())
    {
        throw gcnew VnException(VN::Error::SerialPortClosed);
    }
    return static_cast<uint32_t>(connectedBaudRate.value());
}

void Sensor::ChangeBaudRate(UInt32 baudRate)
{
    VN::Error error = _sensor->changeBaudRate(static_cast<VN::Registers::System::BaudRate::BaudRates>(baudRate));
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

void Sensor::ChangeHostBaudRate(UInt32 baudRate)
{
    VN::Error error = _sensor->changeHostBaudRate(static_cast<VN::Registers::System::BaudRate::BaudRates>(baudRate));
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

void Sensor::Disconnect()
{
    _sensor->disconnect();
}

// ----------------------
// Accessing Measurements
// ----------------------

bool Sensor::HasMeasurement()
{
    return _sensor->hasMeasurement();
}

Nullable<CompositeData> Sensor::GetNextMeasurement()
{
    auto measurement = _sensor->getNextMeasurement();
    if (measurement)
    {
        return CompositeData(*measurement);
    }
    return Nullable<CompositeData>();
}

Nullable<CompositeData> Sensor::GetMostRecentMeasurement()
{
    auto measurement = _sensor->getMostRecentMeasurement();
    if (measurement)
    {
        return CompositeData(*measurement);
    }
    return Nullable<CompositeData>();
}

// ----------------------
// Commands
// ----------------------

void Sensor::ReadRegister(VNSDK::Registers::Register ^ reg)
{
    VN::Error error = _sensor->readRegister(reg->GetReference());
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

void Sensor::WriteRegister(VNSDK::Registers::Register ^ reg)
{
    VN::Error error = _sensor->writeRegister((VN::ConfigurationRegister*)reg->GetReference());
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

void Sensor::WriteSettings()
{
    VN::Error error = _sensor->writeSettings();
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

void Sensor::Reset()
{
    VN::Error error = _sensor->reset();
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

void Sensor::RestoreFactorySettings()
{
    VN::Error error = _sensor->restoreFactorySettings();
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

void Sensor::KnownMagneticDisturbance(uint8_t state)
{
    VN::Error error = _sensor->knownMagneticDisturbance(static_cast<VN::KnownMagneticDisturbance::State>(state));
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

void Sensor::KnownAccelerationDisturbance(uint8_t state)
{
    VN::Error error = _sensor->knownAccelerationDisturbance(static_cast<VN::KnownAccelerationDisturbance::State>(state));
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

void Sensor::SetInitialHeading(float heading)
{
    VN::Error error = _sensor->setInitialHeading(heading);
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

void Sensor::SetInitialHeading(Ypr ypr)
{
    VN::Ypr nativeYpr{ypr.yaw, ypr.pitch, ypr.roll};
    VN::Error error = _sensor->setInitialHeading(nativeYpr);
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

void Sensor::SetInitialHeading(Quaternion quat)
{
    const VN::Quat nativeQuat{VN::Vec3f{quat.vector.x, quat.vector.y, quat.vector.z}, quat.scalar};
    VN::Error error = _sensor->setInitialHeading(nativeQuat);
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

void Sensor::AsyncOutputEnable(uint8_t state)
{
    VN::Error error = _sensor->asyncOutputEnable(static_cast<VN::AsyncOutputEnable::State>(state));
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

void Sensor::SetFilterBias()
{
    VN::Error error = _sensor->setFilterBias();
    if (error != VN::Error::None)
    {
        throw gcnew VnException(error);
    }
}

// Error sendCommand(Command* commandToSend, SendCommandBlockMode waitMode, const Microseconds waitLength = Config::Sensor::commandSendTimeoutLength,
//     const Microseconds timeoutThreshold = Config::CommandProcessor::commandRemovalTimeoutLength) noexcept;

void Sensor::SendCommand(GenericCommand^% commandToSend, Sensor::SendCommandBlockMode waitMode)
{
    VN::Sensor::SendCommandBlockMode waitModeC = static_cast<VN::Sensor::SendCommandBlockMode>(static_cast<int>(waitMode));
	VN::Error error = _sensor->sendCommand(commandToSend->GetReference(), waitModeC); 	
    if (error != VN::Error::None)
	{
		VnException^ pVnException = gcnew VnException(error);
		throw pVnException;
	}
}

void Sensor::SerialSend(String ^ msgToSend)
{
    marshal_context ^ context = gcnew marshal_context();
    auto msgChar = context->marshal_as<const char*>(msgToSend);
    _sensor->serialSend(msgChar, strlen(msgChar));
    delete context;
}

// ----------------------
// Error Handling
// ----------------------

Nullable<AsyncError> Sensor::GetAsynchronousError() {
    auto latestError = _sensor->getAsynchronousError();
    if (!latestError.has_value()) {
        return Nullable<AsyncError>();
    }
    AsyncError error;
    error.Error = static_cast<uint16_t>(latestError->error);
    error.Message = marshal_as<String^>(latestError->message.to_string());
    error.Timestamp = _clock.ConvertTime(latestError->timestamp);
    return error;
}


System::DateTime Sensor::Now() 
{
    return _clock.now();
}

// ----------------------
// Additional logging
// ----------------------

void Sensor::RegisterReceivedByteBuffer(ByteBuffer^ buffer)
{
    _sensor->registerReceivedByteBuffer(buffer->GetReference());
}

void Sensor::DeregisterReceivedByteBuffer()
{
    _sensor->deregisterReceivedByteBuffer();
}

void Sensor::RegisterSkippedByteBuffer(ByteBuffer^ buffer)
{
    _sensor->registerSkippedByteBuffer(buffer->GetReference());
}

void Sensor::DeregisterSkippedByteBuffer()
{
    _sensor->deregisterSkippedByteBuffer();
}


void Sensor::RegisterDataExporter(Exporter^ exporter)
{
    VN::Error error = _sensor->subscribeToMessage(exporter->GetQueuePointer(), "VN", VN::Sensor::AsciiSubscriberFilterType::StartsWith);
    error = _sensor->subscribeToMessage(exporter->GetQueuePointer(), VN::Sensor::BinaryOutputMeasurements{}, VN::Sensor::FaSubscriberFilterType::AnyMatch);
}

void Sensor::DeregisterDataExporter(Exporter^ exporter)
{
    _sensor->unsubscribeFromMessage(exporter->GetQueuePointer(), "VN");
    _sensor->unsubscribeFromMessage(exporter->GetQueuePointer(), VN::Sensor::BinaryOutputMeasurements{});
}

#if (PLUGIN_CALIBRATION)
void Sensor::RegisterHsiCalibration(Calibration::HsiCalibration_Base^ hsi)
{
    VN::Error error = _sensor->subscribeToMessage(hsi->GetQueuePointer(), "VNYMR", VN::Sensor::AsciiSubscriberFilterType::StartsWith);
}

void Sensor::DeregisterHsiCalibration(Calibration::HsiCalibration_Base^ hsi)
{
    _sensor->unsubscribeFromMessage(hsi->GetQueuePointer(), "VNYMR");
}


#endif
} // namespace VNSDK