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

#ifndef VN_CLI_SENSOR_HPP_
#define VN_CLI_SENSOR_HPP_

#include "vectornav/CLI_ByteBuffer.hpp"
#include "vectornav/CLI_CompositeData.hpp"
#include "vectornav/CLI_GenericCommand.hpp"
#include "vectornav/CLI_Registers.hpp"
#include "vectornav/CLI_SteadyClock.hpp"

#pragma managed(push, off)
#include "vectornav/TemplateLibrary/ByteBuffer.hpp"
#include "vectornav/Interface/Sensor.hpp"
#pragma managed(pop)

using namespace System;

namespace VNSDK
{

#if (PLUGIN_DATAEXPORT)
    ref class Exporter;
#endif
#if (PLUGIN_CALIBRATION)
namespace Calibration
{
    ref class HsiCalibration_Base;
}  // namespace Calibration
#endif

public
ref class VnException : public Exception
{
public:
    VnException(VN::Error error) : Exception(gcnew String(VN::errorCodeToString(error)))
    {
    }
};

public
value struct AsyncError
{
  public:

    UInt16 Error;
    String^ Message;
    System::DateTime Timestamp;
};

public
ref class Sensor
{
private:
    VN::Sensor * _sensor;
    SteadyClock _clock{};

public:
    using BaudRate = Registers::System::BaudRate::BaudRates;
    Sensor()
    {
        _sensor = new VN::Sensor();
    }
    
    ~Sensor()
    {
        delete _sensor;
    }


    VN::Sensor &GetNativeInstance()
    {
        return *_sensor;
    };

    // ------------------------------------------
    /*! \name Serial Connectivity */ //@{
    // ------------------------------------------

    /// @brief Opens the serial port at the specified baud rate. This does not verify any connectivity, and as such only validates that the serial port is
    /// available available for opening.
    void Connect(String ^ portName, UInt32 baudRate);

    /// @brief Opens the serial port, scanning all possible baud rates until the sensor is verified connected. This performs a verifySensorConnectivity at each
    /// possible baud rate.
    void AutoConnect(String ^ portName);

    /// @brief Sends a ReadRegister for the Model register. Returns true if a valid response is received, otherwise returns false.
    bool VerifySensorConnectivity();

    /// @brief Get the port name of the open serial port. If no port is open, will return std::nullopt.
    String ^ ConnectedPortName();

    /// @brief Get the baud rate at which the serial port is opened. If no port is open, will return std::nullopt.
    uint32_t ConnectedBaudRate();

    /// @brief Sends a Write Register for the new baud rate to the sensor and reopens to the serial port under the new baud rate. Will retry on failure.
    void ChangeBaudRate(UInt32 baudRate);

    /// @brief Changes the host (computer) baud rate without commanding a change to the VectorNav unit's baud rate. Use with caution.
    void ChangeHostBaudRate(UInt32 baudRate);

    /// @brief Disconnect from the sensor.
    void Disconnect();

    // ------------------------------------------
    /*! \name Accessing Measurements */ //@{
    // ------------------------------------------

    /// @brief Checks to see if there is a new measurement avialable on the MeasurementQueue.
    bool HasMeasurement();

    /// @brief Gets (and pops) the front of the MeasurementQueue.
    /// @param block If true, wait a maximum of getMeasurementTimeoutLength for a new measurement.
    Nullable<CompositeData> GetNextMeasurement();

    /// @brief Gets the back (most recent) of the MeasurementQueue, popping every measurement in the queue.
    /// @param block If true, wait a maximum of getMeasurementTimeoutLength for a new measurement.
    Nullable<CompositeData> GetMostRecentMeasurement();

        /// @brief Block mode for sending a command. @see sendCommand()
    enum class SendCommandBlockMode
    {
        None = (int)VN::Sensor::SendCommandBlockMode::None,            ///< Do not wait for a response from the unit before returning.
        Block = (int)VN::Sensor::SendCommandBlockMode::Block,           ///< Block upon a response from the unit until the specific timeout; will return ResponseTimeout if timeout is hit.
        BlockWithRetry = (int)VN::Sensor::SendCommandBlockMode::BlockWithRetry,  ///< Block upon a response from the unit until the specific timeout; retry sending the command and blocking commandSendRetriesAllowed
    };

    // ------------------------------------------
    /*! \name Sending Commands */ //@{
    // ------------------------------------------

    /// @brief Sends a Read Register command to the unit to poll register values. This is always a blocking call.
    void ReadRegister(Registers::Register ^ reg);

    /// @brief Sends a Write Register command to the unit to set register values. This is always a blocking call.
    void WriteRegister(Registers::Register ^ reg);

    /// @brief Sends the Write Settings command to the unit and blocks on the unit's confirmation.
    void WriteSettings();

    /// @brief Sends a Reset command to the unit and blocks on the unit's confirmation. After confirmation, it sleeps for resetSleepDuration and verifies sensor
    /// connectivity, calling autoConnect if failed.
    void Reset();

    /// @brief Sends a Restore Factory Settings command to the unit, blocks on the unit's confirmation, reopens serial at the unit's default baud rate, sleeps
    /// for resetSleepDuration, then verifies unit connectivity. If THREADING_ENABLE, resets the Listening Thread.
    void RestoreFactorySettings();

    /// @brief Sends a Known Magnetic Disturbance command to the sensor and blocks on the unit's message confirmation.
    void KnownMagneticDisturbance(uint8_t state);

    /// @brief Sends a Known Acceleration Disturbance command to the sensor and blocks on the unit's message confirmation.
    void KnownAccelerationDisturbance(uint8_t state);

    /// @brief Sends a Set Initial Heading command to the sensor and blocks on the unit's message confirmation.
    void SetInitialHeading(float heading);
    void SetInitialHeading(Ypr ypr);
    void SetInitialHeading(Quaternion quat);

    /// @brief Sends an Async Output Enable command to the sensor and blocks on the unit's message confirmation.
    void AsyncOutputEnable(uint8_t state);

    /// @brief Sends a Set Filter Bias command to the sensor and blocks on the unit's message confirmation.
    void SetFilterBias();

    /// @brief Sends a Set Bootloader command to the sensor and blocks on the unit's message confirmation.
    // SetBootloader(const SetBootLoader::Processor processorId);  // Unnecessary for CLI.

    /// @brief Sends an arbitrary command to the unit. The direct command API shoudl be preferred unless it necessary to send without blocking.
   
    void SendCommand(GenericCommand^% commandToSend, Sensor::SendCommandBlockMode waitMode);
    // Error Sensor::sendCommand(Command* commandToSend, SendCommandBlockMode waitMode, const Microseconds waitLengthMs, const Microseconds timeoutThreshold) noexcept

    /// @brief Sends an arbitary message to the unit without any message modification or response validation. Not recommended to use.
    void SerialSend(String ^ msgToSend);

    // ------------------------------------------
    // Error Handling
    // ------------------------------------------

    /// @brief Return any Asynchronous Errors (as a UInt16) reported by the sensor
    Nullable<AsyncError> Sensor::GetAsynchronousError();

    /// @brief Get the current timestamp. Used mostly for debugging purposes.
    System::DateTime Sensor::Now();

    // -----------------------------------------
    // Additional logging
    // ------------------------------------------

    void RegisterReceivedByteBuffer(ByteBuffer^ buffer);
    void DeregisterReceivedByteBuffer();
    void RegisterSkippedByteBuffer(ByteBuffer^ buffer);
    void DeregisterSkippedByteBuffer();

#if (PLUGIN_DATAEXPORT)
    /// @brief Registers a Data Exporter, subscribing it to all ASCII and Binary messages.
    void RegisterDataExporter(Exporter^ exporter);
    /// @brief Deregisters the Data Exporter.
    void DeregisterDataExporter(Exporter^ exporter);
#endif

#if (PLUGIN_CALIBRATION)
    /// @brief Registers an HSI Calibration, subscribing it to VNYMR ASCII Message
    void RegisterHsiCalibration(Calibration::HsiCalibration_Base^ hsi);
    /// @brief Deregisters the HSI Calibration
    void DeregisterHsiCalibration(Calibration::HsiCalibration_Base^ hsi);
#endif
    // ------------------------------------------
    // Unthreaded Packet Processing
    // ------------------------------------------

    /// Not implemented.

}; // public ref class Sensor
} // namespace VNSDK

#include "vectornav/CLI_Plugins.hpp"

#endif  // VN_CLI_SENSOR_HPP_
