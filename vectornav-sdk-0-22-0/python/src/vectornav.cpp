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
#include <exception>
#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>

#include "vectornav/Config.hpp"
#include "vectornav/TemplateLibrary/String.hpp"
#include "vectornav/TemplateLibrary/ByteBuffer.hpp"

#include "vectornav/Interface/Registers.hpp"
#include "vectornav/Interface/Sensor.hpp"
#include "vectornav/Interface/CompositeData.hpp"
#include "vectornav/Interface/GenericCommand.hpp"
#include "vectornav/Implementation/MeasurementDatatypes.hpp"
#include "vectornav/Interface/Errors.hpp"
#include "vectornav/Implementation/BinaryHeader.hpp"
#include "vectornav/Implementation/Packet.hpp"
#include "vectornav/HAL/Timer.hpp"

#include "PyTemplates.hpp"

namespace py = pybind11;

namespace VN {
  
void init_registers(py::module& m);
void init_composite_data(py::module& m);
void init_commands(py::module& m);
  
// PLUGIN INIT FUNCTIONS
void init_register_scan(py::module& m);
void init_firmware_updater(py::module& m);
void init_simple_logger(py::module& m);
void init_data_export(py::module& m);
void init_calibration(py::module& m);

std::string genErrorMessage(Error error) {
  return "Error[" + std::to_string(static_cast<uint16_t>(error)) + "]: " + errorCodeToString(error);   
}

PYBIND11_MODULE(vectornav, m) {
  m.doc() = "VectorNav Python SDK";

  init_registers(m);
  init_composite_data(m);
  init_commands(m);

#ifdef __REGSCAN__
  init_register_scan(m);
#endif
    
#ifdef __FIRMWARE_UPDATE__
  init_firmware_updater(m);
#endif

#ifdef __SIMPLE_LOGGER__
  init_simple_logger(m);
#endif

#ifdef __DATAEXPORT__
  init_data_export(m);
#endif

#ifdef __CALIBRATION__
  init_calibration(m);
#endif
  
  py::class_<Sensor> sensor(m, "Sensor");
  
  sensor.def(py::init<>())
    // Serial Connectivity
    .def("connect",
      [](Sensor& vs, Serial_Base::PortName portName, Sensor::BaudRate baudRate) {
        Error error = vs.connect(portName, baudRate);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("autoConnect",
      [](Sensor& vs, Serial_Base::PortName portName) {
        Error error = vs.autoConnect(portName);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("verifySensorConnectivity", &Sensor::verifySensorConnectivity)
    .def("connectedPortName", &Sensor::connectedPortName)
    .def("connectedBaudRate", &Sensor::connectedBaudRate)
    .def("changeBaudRate",
      [](Sensor& vs, Sensor::BaudRate newBaudRate) {
        Error error = vs.changeBaudRate(newBaudRate);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("changeHostBaudRate",
      [](Sensor& vs, Sensor::BaudRate newBaudRate) {
        Error error = vs.changeHostBaudRate(newBaudRate);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("disconnect", &Sensor::disconnect)
    // Measurement Accessor
    .def("hasMeasurement", &Sensor::hasMeasurement)
    .def("getNextMeasurement",
      [](Sensor& vs) -> std::optional<VN::CompositeData> {
        auto ownPtr = vs.getNextMeasurement();
        if (ownPtr) { return std::make_optional(*ownPtr); } else { return std::nullopt; }      
      }
    )
    .def("getNextMeasurement",
      [](Sensor& vs, const bool blocking) -> std::optional<VN::CompositeData> {
        auto ownPtr = vs.getNextMeasurement(blocking);
        if (ownPtr) { return std::make_optional(*ownPtr); } else { return std::nullopt; }      
      }
    )
    .def("getMostRecentMeasurement",
      [](Sensor& vs) -> std::optional<VN::CompositeData> {
        auto ownPtr = vs.getMostRecentMeasurement();
        if (ownPtr) { return std::make_optional(*ownPtr); } else { return std::nullopt; }      
      }
    )
    .def("getMostRecentMeasurement",
      [](Sensor& vs, const bool blocking) -> std::optional<VN::CompositeData> {
        auto ownPtr = vs.getMostRecentMeasurement(blocking);
        if (ownPtr) { return std::make_optional(*ownPtr); } else { return std::nullopt; }      
      }
    )
    // Command Sending
    .def("readRegister",
      [](Sensor& vs, Register* registerToRead, const bool retryOnFailure) {
        Error error = vs.readRegister(registerToRead, retryOnFailure);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("readRegister",
      [](Sensor& vs, Register* registerToRead) {
        Error error = vs.readRegister(registerToRead);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("writeRegister",
      [](Sensor& vs, ConfigurationRegister* registerToWrite, const bool retryOnFailure) {
        Error error = vs.writeRegister(registerToWrite, retryOnFailure);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("writeRegister",
      [](Sensor& vs, ConfigurationRegister* registerToWrite) {
        Error error = vs.writeRegister(registerToWrite);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("writeSettings",
      [](Sensor& vs) {
        Error error = vs.writeSettings();
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("reset",
      [](Sensor& vs) {
        Error error = vs.reset();
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("restoreFactorySettings",
      [](Sensor& vs) {
        Error error = vs.restoreFactorySettings();    
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("knownMagneticDisturbance",
      [](Sensor& vs, const KnownMagneticDisturbance::State state) {
        Error error = vs.knownMagneticDisturbance(state);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("knownAccelerationDisturbance",
      [](Sensor& vs, const KnownAccelerationDisturbance::State state) {
        Error error = vs.knownAccelerationDisturbance(state);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("setInitialHeading",
      [](Sensor& vs, const float heading) {
        Error error = vs.setInitialHeading(heading);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("setInitialHeading",
      [](Sensor& vs, Ypr ypr) {
        Error error = vs.setInitialHeading(ypr);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("setInitialHeading",
      [](Sensor& vs, Quat quat) {
        Error error = vs.setInitialHeading(quat);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("asyncOutputEnable",
      [](Sensor& vs, const AsyncOutputEnable::State state) {
        Error error = vs.asyncOutputEnable(state);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("setFilterBias",
      [](Sensor& vs) {
        Error error = vs.setFilterBias();
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("sendCommand", 
      [](Sensor& vs, GenericCommand* commandToSend, Sensor::SendCommandBlockMode waitMode, const Microseconds waitLengthMs) {
        Error error = vs.sendCommand(commandToSend, waitMode, waitLengthMs);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("sendCommand", 
      [](Sensor& vs, GenericCommand* commandToSend, Sensor::SendCommandBlockMode waitMode) {
        Error error = vs.sendCommand(commandToSend, waitMode, Config::Sensor::commandSendTimeoutLength);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("serialSend",
      [](Sensor& vs, const char* msgToSend, const size_t len) {
        Error error = vs.serialSend(msgToSend, len);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("now",
      [](Sensor& vs) {
        return VN::now();
      }
    )
    // Additional Logging
    .def("registerSkippedByteBuffer", &Sensor::registerSkippedByteBuffer)
    .def("deregisterSkippedByteBuffer", &Sensor::deregisterSkippedByteBuffer)
    .def("registerReceivedByteBuffer", &Sensor::registerReceivedByteBuffer)
    .def("deregisterReceivedByteBuffer", &Sensor::deregisterReceivedByteBuffer)
    .def("subscribeToMessage",
      [](Sensor& vs, PacketQueue_Interface* q, const Registers::System::BinaryOutputMeasurements& b, FaPacketDispatcher::SubscriberFilterType c) {
        Error error = vs.subscribeToMessage(q, b, c);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("subscribeToMessage",
      [](Sensor& vs, PacketQueue_Interface* q, const AsciiHeader& b, AsciiPacketDispatcher::SubscriberFilterType c) {
        Error error = vs.subscribeToMessage(q, b, c);
        if (error != Error::None) { throw std::runtime_error(genErrorMessage(error)); }
      }
    )
    .def("unsubscribeFromMessage",
      [](Sensor& vs, PacketQueue_Interface* q, const Registers::System::BinaryOutputMeasurements& b) {
        vs.unsubscribeFromMessage(q, b);
      }
    )
    .def("unsubscribeFromMessage",
      [](Sensor& vs, PacketQueue_Interface* q, const AsciiHeader& b) {
        vs.unsubscribeFromMessage(q, b);
      }
    )
    // Error Handling
    .def("getAsynchronousError", &Sensor::getAsynchronousError)
    .def("__enter__", [](Sensor& vs) {
      return &vs;
      }
    )
    .def("__exit__",
      [](Sensor& vs,
         const std::optional<pybind11::type> &exc_type,
         const std::optional<pybind11::object> &exc_value,
         const std::optional<pybind11::object> &traceback) {
      
        vs.disconnect();
      }
    )
    ;

  py::enum_<Sensor::BaudRate>(sensor, "BaudRate", py::module_local())
		.value("Baud9600", Sensor::BaudRate::Baud9600)
		.value("Baud19200", Sensor::BaudRate::Baud19200)
		.value("Baud38400", Sensor::BaudRate::Baud38400)
		.value("Baud57600", Sensor::BaudRate::Baud57600)
		.value("Baud115200", Sensor::BaudRate::Baud115200)
		.value("Baud128000", Sensor::BaudRate::Baud128000)
		.value("Baud230400", Sensor::BaudRate::Baud230400)
		.value("Baud460800", Sensor::BaudRate::Baud460800)
		.value("Baud921600", Sensor::BaudRate::Baud921600);

  py::enum_<Sensor::SendCommandBlockMode>(sensor, "SendCommandBlockMode")
    .value("none", Sensor::SendCommandBlockMode::None)
    .value("Block", Sensor::SendCommandBlockMode::Block)
    .value("BlockWithRetry", Sensor::SendCommandBlockMode::BlockWithRetry);

  py::class_<BinaryHeader>(m, "BinaryHeader")
    .def(py::init<>());

  py::module vntypes = m.def_submodule("VnTypes", "VnTypes submodule");

  py::class_<Ypr>(vntypes, "Ypr")
    .def(py::init<>())
    .def_readwrite("yaw", &Ypr::yaw)
    .def_readwrite("pitch", &Ypr::pitch)
    .def_readwrite("roll", &Ypr::roll)
    .def("__repr__",
      [](const Ypr& ypr) {
        return std::to_string(ypr.yaw) + "," + std::to_string(ypr.pitch) + "," + std::to_string(ypr.roll);
      }
    );

		py::class_<TimeStatus>(vntypes, "TimeStatus")
		.def(py::init<const uint8_t&>())
		.def_property("towValid",
      [](TimeStatus& self) -> uint8_t { return self.towValid; },
      [](TimeStatus& self, const uint8_t value) { self.towValid= value; }
    )
		.def_property("dateValid",
      [](TimeStatus& self) -> uint8_t { return self.dateValid; },
      [](TimeStatus& self, const uint8_t value) { self.dateValid = value; }
    )
		.def_property("utcValid",
      [](TimeStatus& self) -> uint8_t { return self.utcValid; },
      [](TimeStatus& self, const uint8_t value) { self.utcValid = value; }
    );

		py::class_<InsStatus>(vntypes, "InsStatus")
		.def(py::init<const uint16_t&>())
		.def_property("mode",
      [](InsStatus& self) -> uint16_t { return self.mode; },
      [](InsStatus& self, const uint16_t value) { self.mode = value; }
    )
		.def_property("gnssFix",
      [](InsStatus& self) -> uint16_t { return self.gnssFix; },
      [](InsStatus& self, const uint16_t value) { self.gnssFix = value; }
    )
		.def_property("imuErr",
      [](InsStatus& self) -> uint16_t { return self.imuErr; },
      [](InsStatus& self, const uint16_t value) { self.imuErr = value; }
    )
		.def_property("magPresErr",
      [](InsStatus& self) -> uint16_t { return self.magPresErr; },
      [](InsStatus& self, const uint16_t value) { self.magPresErr = value; }
    )
		.def_property("gnssErr",
      [](InsStatus& self) -> uint16_t { return self.gnssErr; },
      [](InsStatus& self, const uint16_t value) { self.gnssErr = value; }
    )
		.def_property("gnssCompassFix",
      [](InsStatus& self) -> uint16_t { return self.gnssCompassFix; },
      [](InsStatus& self, const uint16_t value) { self.gnssCompassFix = value; }
    );
	
		py::class_<AhrsStatus>(vntypes, "AhrsStatus")
		.def(py::init<const uint16_t&>())
		.def_property("attitudeQuality",
      [](AhrsStatus& self) -> uint16_t { return self.attitudeQuality; },
      [](AhrsStatus& self, const uint16_t value) { self.attitudeQuality = value; }
    )
		.def_property("gyroSaturation",
      [](AhrsStatus& self) -> uint16_t { return self.gyroSaturation; },
      [](AhrsStatus& self, const uint16_t value) { self.gyroSaturation = value; }
    )
		.def_property("gyroSaturationRecovery",
      [](AhrsStatus& self) -> uint16_t { return self.gyroSaturationRecovery; },
      [](AhrsStatus& self, const uint16_t value) { self.gyroSaturationRecovery = value; }
    )
		.def_property("magDistrubance",
      [](AhrsStatus& self) -> uint16_t { return self.magDistrubance; },
      [](AhrsStatus& self, const uint16_t value) { self.magDistrubance = value; }
    )
		.def_property("magSaturation",
      [](AhrsStatus& self) -> uint16_t { return self.magSaturation; },
      [](AhrsStatus& self, const uint16_t value) { self.magSaturation = value; }
    )
		.def_property("accDisturbance",
      [](AhrsStatus& self) -> uint16_t { return self.accDisturbance; },
      [](AhrsStatus& self, const uint16_t value) { self.accDisturbance = value; }
    )
		.def_property("accSaturation",
      [](AhrsStatus& self) -> uint16_t { return self.accSaturation; },
      [](AhrsStatus& self, const uint16_t value) { self.accSaturation = value; }
    )
		.def_property("knownMagDisturbance",
      [](AhrsStatus& self) -> uint16_t { return self.knownMagDisturbance; },
      [](AhrsStatus& self, const uint16_t value) { self.knownMagDisturbance = value; }
    )
		.def_property("knownAccDisturbance",
      [](AhrsStatus& self) -> uint16_t { return self.knownAccDisturbance; },
      [](AhrsStatus& self, const uint16_t value) { self.knownAccDisturbance = value; }
    );
    
		py::class_<ImuStatus>(vntypes, "ImuStatus")
		.def(py::init<const uint16_t&>())
		.def_property("gyroStatus",
      [](ImuStatus& self) -> uint16_t { return self.gyroStatus; },
      [](ImuStatus& self, const uint16_t value) { return self.gyroStatus; }
    )
		.def_property("accelStatus",
      [](ImuStatus& self) -> uint16_t { return self.accelStatus; },
      [](ImuStatus& self, const uint16_t value) { self.accelStatus = value; }
    )
		.def_property("magStatus",
      [](ImuStatus& self) -> uint16_t { return self.magStatus; },
      [](ImuStatus& self, const uint16_t value) { self.magStatus = value; }
    )
		.def_property("presTempStatus",
      [](ImuStatus& self) -> uint16_t { return self.presTempStatus; },
      [](ImuStatus& self, const uint16_t value) { self.presTempStatus = value; }
    );
    
		py::class_<GnssStatus>(vntypes, "GnssStatus")
		.def(py::init<const uint16_t&>())
		.def_property("enabled",
      [](GnssStatus& self) -> uint16_t { return self.enabled; },
      [](GnssStatus& self, const uint16_t value) { self.enabled = value; }
    )
		.def_property("operational",
      [](GnssStatus& self) -> uint16_t { return self.operational; },
      [](GnssStatus& self, const uint16_t value) { self.operational = value; }
    )
		.def_property("fix",
      [](GnssStatus& self) -> uint16_t { return self.fix; },
      [](GnssStatus& self, const uint16_t value) { self.fix = value; }
    )
		.def_property("antennaSignalError",
      [](GnssStatus& self) -> uint16_t { return self.antennaSignalError; },
      [](GnssStatus& self, const uint16_t value) { self.antennaSignalError = value; }
    )
		.def_property("usedForNav",
      [](GnssStatus& self) -> uint16_t { return self.usedForNav; },
      [](GnssStatus& self, const uint16_t value) { self.usedForNav = value; }
    )
		.def_property("dataSource",
      [](GnssStatus& self) -> uint16_t { return self.dataSource; },
      [](GnssStatus& self, const uint16_t value) { self.dataSource = value; }
    )
		.def_property("usedForNavCurr",
      [](GnssStatus& self) -> uint16_t { return self.usedForNavCurr; },
      [](GnssStatus& self, const uint16_t value) { self.usedForNavCurr = value; }
    )
		.def_property("ppsUsedForTime",
      [](GnssStatus& self) -> uint16_t { return self.ppsUsedForTime; },
      [](GnssStatus& self, const uint16_t value) { self.ppsUsedForTime = value; }
    );
    
  py::class_<Time>(vntypes, "Time")
    .def(py::init<>())
    .def(py::init<uint64_t>())
    .def(py::init<double>())
    .def("nanoseconds", &Time::nanoseconds)
    .def("microseconds", &Time::microseconds)
    .def("milliseconds", &Time::milliseconds)
    .def("seconds", &Time::seconds);

  py::class_<DeltaTheta>(vntypes, "DeltaTheta")
    .def(py::init<>())
    .def_readwrite("deltaTime",     &DeltaTheta::deltaTime)
    .def_readwrite("deltaTheta",     &DeltaTheta::deltaTheta);

  py::class_<Quat>(vntypes, "Quat")
    .def(py::init<>())
    .def_readwrite("vector", &Quat::vector)
    .def_readwrite("scalar", &Quat::scalar);
    
  py::class_<Lla>(vntypes, "Lla")
    .def(py::init<>())
    .def_readwrite("lat", &Lla::lat)
    .def_readwrite("lon", &Lla::lon)
    .def_readwrite("alt", &Lla::alt);

  py::class_<TimeUtc>(vntypes, "TimeUtc")
    .def(py::init<>())
    .def_readwrite("year", &TimeUtc::year)
    .def_readwrite("month", &TimeUtc::month)
    .def_readwrite("day", &TimeUtc::day)
    .def_readwrite("hour", &TimeUtc::hour)
    .def_readwrite("minute", &TimeUtc::minute)
    .def_readwrite("second", &TimeUtc::second)
    .def_readwrite("fracSec", &TimeUtc::fracSec);
    
  py::class_<GnssTimeInfo>(vntypes, "GnssTimeInfo")
    .def(py::init<>())
    .def_readwrite("gnssTimeStatus", &GnssTimeInfo::gnssTimeStatus)
    .def_readwrite("leapSeconds", &GnssTimeInfo::leapSeconds);
    
  py::class_<GnssDop>(vntypes, "GnssDop")
    .def(py::init<>())
    .def_readwrite("gDop", &GnssDop::gDop)
    .def_readwrite("pDop", &GnssDop::pDop)
    .def_readwrite("tDop", &GnssDop::tDop)
    .def_readwrite("vDop", &GnssDop::vDop)
    .def_readwrite("hDop", &GnssDop::hDop)
    .def_readwrite("nDop", &GnssDop::nDop)
    .def_readwrite("eDop", &GnssDop::eDop);
    
  py::class_<GnssSatInfo>(vntypes, "GnssSatInfo")
    .def(py::init<>())
    .def_readwrite("numSats", &GnssSatInfo::numSats)
    .def_readwrite("resv",  &GnssSatInfo::resv)
    .def_readwrite("sys",   &GnssSatInfo::sys)
    .def_readwrite("svId",  &GnssSatInfo::svId)
    .def_readwrite("flags", &GnssSatInfo::flags)
    .def_readwrite("cno",   &GnssSatInfo::cno)
    .def_readwrite("qi",    &GnssSatInfo::qi)
    .def_readwrite("el",    &GnssSatInfo::el)
    .def_readwrite("az",    &GnssSatInfo::az);

  py::class_<GnssRawMeas>(vntypes, "GnssRawMeas")
    .def(py::init<>())
    .def_readwrite("tow",     &GnssRawMeas::tow)
    .def_readwrite("week",    &GnssRawMeas::week)
    .def_readwrite("numMeas", &GnssRawMeas::numMeas)
    .def_readwrite("resv",    &GnssRawMeas::resv)
    .def_readwrite("sys",     &GnssRawMeas::sys)
    .def_readwrite("svId",    &GnssRawMeas::svId)
    .def_readwrite("band",    &GnssRawMeas::band)
    .def_readwrite("chan",    &GnssRawMeas::chan)
    .def_readwrite("freqNum",    &GnssRawMeas::freqNum)
    .def_readwrite("cno",     &GnssRawMeas::cno)
    .def_readwrite("flags",   &GnssRawMeas::flags)
    .def_readwrite("pr",      &GnssRawMeas::pr)
    .def_readwrite("cp",      &GnssRawMeas::cp)
    .def_readwrite("dp",      &GnssRawMeas::dp);


  declare_matrix<3, 1, float>(vntypes, "Vec3f");
  declare_matrix<3, 1, double>(vntypes, "Vec3d");
  declare_matrix<3, 3, float>(vntypes, "Mat3f");


  declare_string<AsciiMessage>(m, "AsciiMessage");
  declare_string<Serial_Base::PortName>(m, "PortName");
  declare_string<AsciiHeader>(m, "AsciiHeader");

  py::implicitly_convertible<py::str, AsciiMessage>();
  py::implicitly_convertible<py::str, AsciiHeader>();
  py::implicitly_convertible<py::str, Serial_Base::PortName>();
  py::implicitly_convertible<py::int_, Sensor::BaudRate>();

  py::class_<ByteBuffer>(m, "ByteBuffer")
    .def(py::init<size_t>())
    .def("size", &ByteBuffer::size)
    .def("isEmpty", &ByteBuffer::isEmpty)
    .def("capacity", &ByteBuffer::capacity);

  py::class_<AsyncError>(m, "AsyncError")
    .def(py::init<>())
    .def_readwrite("error", &AsyncError::error)
    .def_readwrite("message", &AsyncError::message)
    .def_readwrite("timestamp", &AsyncError::timestamp);

  py::enum_<Error>(m, "Error")
    .value("HardFault", Error::HardFault)
    .value("SerialBufferOverflow", Error::SerialBufferOverflow)
    .value("InvalidChecksum", Error::InvalidChecksum)
    .value("InvalidCommand", Error::InvalidCommand)
    .value("NotEnoughParameters", Error::NotEnoughParameters)
    .value("TooManyParameters", Error::TooManyParameters)
    .value("InvalidParameter", Error::InvalidParameter)
    .value("InvalidRegister", Error::InvalidRegister)
    .value("UnauthorizedAccess", Error::UnauthorizedAccess)
    .value("WatchdogReset", Error::WatchdogReset)
    .value("OutputBufferOverflow", Error::OutputBufferOverflow)
    .value("InsufficientBaudRate", Error::InsufficientBaudRate)
    .value("ErrorBufferOverflow", Error::ErrorBufferOverflow)
    .value("CommandResent", Error::CommandResent)
    .value("CommandQueueFull", Error::CommandQueueFull)
    .value("ResponseTimeout", Error::ResponseTimeout)
    .value("ReceivedUnexpectedMessage", Error::ReceivedUnexpectedMessage)
    .value("InvalidPortName", Error::InvalidPortName)
    .value("AccessDenied", Error::AccessDenied)
    .value("SerialPortClosed", Error::SerialPortClosed)
    .value("UnsupportedBaudRate", Error::UnsupportedBaudRate)
    .value("SerialReadFailed", Error::SerialReadFailed)
    .value("SerialWriteFailed", Error::SerialWriteFailed)
    .value("UnexpectedSerialError", Error::UnexpectedSerialError)
    .value("MeasurementQueueFull", Error::MeasurementQueueFull)
    .value("PrimaryBufferFull", Error::PrimaryBufferFull)
    .value("MessageSubscriberCapacityReached", Error::MessageSubscriberCapacityReached)
    .value("ReceivedInvalidResponse", Error::ReceivedInvalidResponse);


  py::class_<FaPacketDispatcher> faPacketDispatcher(m, "FaPacketDispatcher");
  py::class_<AsciiPacketDispatcher> asciiPacketDispatcher(m, "AsciiPacketDispatcher");
  
  py::enum_<FaPacketDispatcher::SubscriberFilterType>(faPacketDispatcher, "SubscriberFilterType")
    .value("ExactMatch", FaPacketDispatcher::SubscriberFilterType::ExactMatch)
    .value("AnyMatch", FaPacketDispatcher::SubscriberFilterType::AnyMatch)
    .value("NotExactMatch", FaPacketDispatcher::SubscriberFilterType::NotExactMatch);
  
  py::enum_<AsciiPacketDispatcher::SubscriberFilterType>(asciiPacketDispatcher, "SubscriberFilterType")
    .value("StartsWith", AsciiPacketDispatcher::SubscriberFilterType::StartsWith)
    .value("DoesNotStartsWith", AsciiPacketDispatcher::SubscriberFilterType::DoesNotStartWith);


  declare_direct_access_queue_base<VN::Packet>(m, "PacketQueue_Interface");
  declare_direct_access_queue<VN::Packet, 1000>(m, "PacketQueue");
 
}}
// clang-format on
