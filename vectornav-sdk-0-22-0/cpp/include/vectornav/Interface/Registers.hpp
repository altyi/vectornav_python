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

#ifndef VN_REGISTERS_HPP_
#define VN_REGISTERS_HPP_

#include "vectornav/Implementation/BinaryHeader.hpp"
#include "vectornav/Implementation/BinaryMeasurementDefinitions.hpp"
#include "vectornav/Interface/Commands.hpp"
#include "vectornav/TemplateLibrary/Vector.hpp"

namespace VN
{
namespace Registers
{
namespace Attitude
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 8 - Yaw Pitch Roll </summary>
<remarks>

Attitude solution as yaw, pitch, and roll in degrees. The yaw, pitch, and roll is given as a 3,2,1
Euler angle rotation sequence describing the orientation of the sensor with respect to the inertial
North East Down (NED) frame.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class YawPitchRoll : public MeasurementRegister
{
public:
    float yaw = 0;    ///< Yaw angle.
    float pitch = 0;  ///< Pitch angle.
    float roll = 0;   ///< Roll angle.

    YawPitchRoll() : MeasurementRegister(8) {}
    static constexpr const char* name() { return "YawPitchRoll"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const YawPitchRoll& lhs, const YawPitchRoll& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 9 - Quaternion </summary>
<remarks>

Attitude solution as a quaternion.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class Quaternion : public MeasurementRegister
{
public:
    float quatX = 0;  ///< First vector component of quaternion.
    float quatY = 0;  ///< Second vector component of quaternion.
    float quatZ = 0;  ///< Third vector component of quaternion.
    float quatS = 0;  ///< Scalar component of quaternion.

    Quaternion() : MeasurementRegister(9) {}
    static constexpr const char* name() { return "Quaternion"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const Quaternion& lhs, const Quaternion& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 15 - Quaternion & Compensated IMU </summary>
<remarks>

Quaternion attitude solution, and compensated (Magnetic, Acceleration, Angular Rate) values.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class QuatMagAccelRate : public MeasurementRegister
{
public:
    float quatX = 0;   ///< First vector component of quaternion.
    float quatY = 0;   ///< Second vector component of quaternion.
    float quatZ = 0;   ///< Third vector component of quaternion.
    float quatS = 0;   ///< Scalar component of quaternion.
    float magX = 0;    ///< Compensated magnetometer measurement in the body-frame x-axis.
    float magY = 0;    ///< Compensated magnetometer measurement in the body-frame y-axis.
    float magZ = 0;    ///< Compensated magnetometer measurement in the body-frame z-axis.
    float accelX = 0;  ///< Compensated accelerometer measurement in the body-frame x-axis.
    float accelY = 0;  ///< Compensated accelerometer measurement in the body-frame y-axis.
    float accelZ = 0;  ///< Compensated accelerometer measurement in the body-frame z-axis.
    float gyroX = 0;   ///< Compensated angular rate measurement in the body-frame x-axis.
    float gyroY = 0;   ///< Compensated angular rate measurement in the body-frame y-axis.
    float gyroZ = 0;   ///< Compensated angular rate measurement in the body-frame z-axis.

    QuatMagAccelRate() : MeasurementRegister(15) {}
    static constexpr const char* name() { return "QuatMagAccelRate"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const QuatMagAccelRate& lhs, const QuatMagAccelRate& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 21 - Magnetic and Gravity Reference Vectors </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class MagGravRefVec : public ConfigurationRegister
{
public:
    float magRefN = 0;
    float magRefE = 0;
    float magRefD = 0;
    float gravRefN = 0;
    float gravRefE = 0;
    float gravRefD = 0;

    MagGravRefVec() : ConfigurationRegister(21) {}
    static constexpr const char* name() { return "MagGravRefVec"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const MagGravRefVec& lhs, const MagGravRefVec& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 27 - Yaw-Pitch-Roll & Compensated IMU </summary>
<remarks>

Yaw, Pitch, Roll, Accel, and Angular Rates

</remarks>
*-----------------------------------------------------------------------------------------------**/
class YprMagAccelAngularRates : public MeasurementRegister
{
public:
    float yaw = 0;     ///< Yaw angle.
    float pitch = 0;   ///< Pitch angle.
    float roll = 0;    ///< Roll angle.
    float magX = 0;    ///< Compensated magnetometer measurement in the body-frame x-axis.
    float magY = 0;    ///< Compensated magnetometer measurement in the body-frame y-axis.
    float magZ = 0;    ///< Compensated magnetometer measurement in the body-frame z-axis.
    float accelX = 0;  ///< Compensated accelerometer measurement in the body-frame x-axis.
    float accelY = 0;  ///< Compensated accelerometer measurement in the body-frame y-axis.
    float accelZ = 0;  ///< Compensated accelerometer measurement in the body-frame z-axis.
    float gyroX = 0;   ///< Compensated angular rate measurement in the body-frame x-axis.
    float gyroY = 0;   ///< Compensated angular rate measurement in the body-frame y-axis.
    float gyroZ = 0;   ///< Compensated angular rate measurement in the body-frame z-axis.

    YprMagAccelAngularRates() : MeasurementRegister(27) {}
    static constexpr const char* name() { return "YprMagAccelAngularRates"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const YprMagAccelAngularRates& lhs, const YprMagAccelAngularRates& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 35 - VPE Basic Control </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class VpeBasicControl : public ConfigurationRegister
{
public:
    enum class HeadingMode : uint8_t
    {
        Absolute = 0,
        Relative = 1,
        Indoor = 2,
    };

    enum class FilteringMode : uint8_t
    {
        Unfiltered = 0,
        AdaptivelyFiltered = 1,
    };

    enum class TuningMode : uint8_t
    {
        Static = 0,
        Adaptive = 1,
    };

    uint8_t resv = 0;
    HeadingMode headingMode = static_cast<HeadingMode>(0);
    FilteringMode filteringMode = static_cast<FilteringMode>(0);
    TuningMode tuningMode = static_cast<TuningMode>(0);

    VpeBasicControl() : ConfigurationRegister(35) {}
    static constexpr const char* name() { return "VpeBasicControl"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const VpeBasicControl& lhs, const VpeBasicControl& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 36 - VPE Magnetometer Basic Tuning </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class VpeMagBasicTuning : public ConfigurationRegister
{
public:
    float baseTuningX = 0;
    float baseTuningY = 0;
    float baseTuningZ = 0;
    float adaptiveTuningX = 0;
    float adaptiveTuningY = 0;
    float adaptiveTuningZ = 0;
    float adaptiveFilteringX = 0;
    float adaptiveFilteringY = 0;
    float adaptiveFilteringZ = 0;

    VpeMagBasicTuning() : ConfigurationRegister(36) {}
    static constexpr const char* name() { return "VpeMagBasicTuning"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const VpeMagBasicTuning& lhs, const VpeMagBasicTuning& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 38 - VPE Accelerometer Basic Tuning </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class VpeAccelBasicTuning : public ConfigurationRegister
{
public:
    float baseTuningX = 0;
    float baseTuningY = 0;
    float baseTuningZ = 0;
    float adaptiveTuningX = 0;
    float adaptiveTuningY = 0;
    float adaptiveTuningZ = 0;
    float adaptiveFilteringX = 0;
    float adaptiveFilteringY = 0;
    float adaptiveFilteringZ = 0;

    VpeAccelBasicTuning() : ConfigurationRegister(38) {}
    static constexpr const char* name() { return "VpeAccelBasicTuning"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const VpeAccelBasicTuning& lhs, const VpeAccelBasicTuning& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 239 - Yaw-Pitch-Roll, Linear Acceleration & Gyro </summary>
<remarks>

Yaw, Pitch, Roll, Linear Body Accel, and Angular Rates.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class YprLinearBodyAccelAngularRates : public MeasurementRegister
{
public:
    float yaw = 0;        ///< Yaw angle.
    float pitch = 0;      ///< Pitch angle.
    float roll = 0;       ///< Roll angle.
    float linAccelX = 0;  ///< Linear acceleration in body-frame x-axis.
    float linAccelY = 0;  ///< Linear acceleration in body-frame y-axis.
    float linAccelZ = 0;  ///< Linear acceleration in body-frame z-axis.
    float gyroX = 0;      ///< Compensated angular rate measurement in the body-frame x-axis.
    float gyroY = 0;      ///< Compensated angular rate measurement in the body-frame y-axis.
    float gyroZ = 0;      ///< Compensated angular rate measurement in the body-frame z-axis.

    YprLinearBodyAccelAngularRates() : MeasurementRegister(239) {}
    static constexpr const char* name() { return "YprLinearBodyAccelAngularRates"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const YprLinearBodyAccelAngularRates& lhs, const YprLinearBodyAccelAngularRates& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 240 - Yaw-Pitch-Roll, Inertial Linear Acceleration & Gyro </summary>
<remarks>

Yaw, Pitch, Roll, Linear Inertial Accel, and Angular Rates.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class YprLinearInertialAccelAngularRates : public MeasurementRegister
{
public:
    float yaw = 0;        ///< Yaw angle.
    float pitch = 0;      ///< Pitch angle.
    float roll = 0;       ///< Roll angle.
    float linAccelN = 0;  ///< Linear acceleration in North direction.
    float linAccelE = 0;  ///< Linear acceleration in East direction.
    float linAccelD = 0;  ///< Linear acceleration in Down direction.
    float gyroX = 0;      ///< Compensated angular rate measurement in the body-frame x-axis.
    float gyroY = 0;      ///< Compensated angular rate measurement in the body-frame y-axis.
    float gyroZ = 0;      ///< Compensated angular rate measurement in the body-frame z-axis.

    YprLinearInertialAccelAngularRates() : MeasurementRegister(240) {}
    static constexpr const char* name() { return "YprLinearInertialAccelAngularRates"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const YprLinearInertialAccelAngularRates& lhs, const YprLinearInertialAccelAngularRates& rhs);
}  // namespace Attitude

namespace GNSS
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 55 - GNSS Basic Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class GnssBasicConfig : public ConfigurationRegister
{
public:
    enum class ReceiverEnable : uint8_t
    {
        Internal = 0,
        VnWrite = 1,
        VnAdor = 2,
        GnssA = 100,
        VnWriteAndGnssA = 101,
        VnAdorAndGnssA = 102,
        VnWriteAndGnssAB = 201,
        VnAdorAndGnssAB = 202,
    };

    enum class PpsSource : uint8_t
    {
        PpsPinRising = 0,
        PpsPinFalling = 1,
        SyncInRising = 2,
        SyncInFalling = 3,
        None = 6,
    };

    enum class Rate : uint8_t
    {
        Rate1Hz = 1,
        Rate5Hz = 5,
    };

    enum class AntPower : uint8_t
    {
        Off = 0,
        Internal = 1,
        External = 2,
    };

    ReceiverEnable receiverEnable = static_cast<ReceiverEnable>(0);
    PpsSource ppsSource = static_cast<PpsSource>(0);
    Rate rate = static_cast<Rate>(0);
    uint8_t resv4 = 0;
    AntPower antPower = static_cast<AntPower>(0);

    GnssBasicConfig() : ConfigurationRegister(55) {}
    static constexpr const char* name() { return "GnssBasicConfig"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const GnssBasicConfig& lhs, const GnssBasicConfig& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 57 - GNSS Internal A Antenna Offset </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class GnssAOffset : public ConfigurationRegister
{
public:
    float positionX = 0;
    float positionY = 0;
    float positionZ = 0;

    GnssAOffset() : ConfigurationRegister(57) {}
    static constexpr const char* name() { return "GnssAOffset"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const GnssAOffset& lhs, const GnssAOffset& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 58 - GNSS Solution - LLA </summary>
<remarks>

Primary GNSS receiver measurement with lat/lon/alt position and velocity in NED frame.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class GnssSolLla : public MeasurementRegister
{
public:
    enum class Gnss1Fix : uint8_t
    {
        NoFix = 0,     ///< Not tracking.
        TimeFix = 1,   ///< Time-only fix.
        Fix2D = 2,     ///< Time plus 2D position fix.
        Fix3D = 3,     ///< Time plus full 3D position fix.
        SBAS = 4,      ///< 3D Fix using SBAS corrections.
        RtkFloat = 7,  ///< 3D Fix using an approximate RTK solution.
        RtkFix = 8,    ///< 3D Fix using a fixed RTK solution.
    };

    double gps1Tow = 0;     ///< GPS time of week.
    uint16_t gps1Week = 0;  ///< The current GPS week.
    Gnss1Fix gnss1Fix = static_cast<Gnss1Fix>(0);
    uint8_t gnss1NumSats = 0;        ///< Number of satellites tracked by GNSS receiver.
    double lat = 0;                  ///< GNSS geodetic latitude.
    double lon = 0;                  ///< GNSS longitude.
    double alt = 0;                  ///< GNSS altitude above WGS84 ellipsoid.
    float velN = 0;                  ///< GNSS velocity in North direction.
    float velE = 0;                  ///< GNSS velocity in East direction.
    float velD = 0;                  ///< GNSS velocity in Down direction.
    float posUncertaintyN = 0;       ///< GNSS position uncertainty, North direction.
    float posUncertaintyE = 0;       ///< GNSS position uncertainty, East direction.
    float posUncertaintyD = 0;       ///< GNSS position uncertainty, Down direction.
    float gnss1VelUncertainty = 0;   ///< GNSS velocity uncertainty (scalar).
    float gnss1TimeUncertainty = 0;  ///< GNSS time uncertainty.

    GnssSolLla() : MeasurementRegister(58) {}
    static constexpr const char* name() { return "GnssSolLla"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const GnssSolLla& lhs, const GnssSolLla& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 59 - GNSS Solution - ECEF </summary>
<remarks>

Primary GNSS receiver measurement in ECEF frame.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class GnssSolEcef : public MeasurementRegister
{
public:
    enum class Gnss1Fix : uint8_t
    {
        NoFix = 0,     ///< Not tracking.
        TimeFix = 1,   ///< Time-only fix.
        Fix2D = 2,     ///< Time plus 2D position fix.
        Fix3D = 3,     ///< Time plus full 3D position fix.
        SBAS = 4,      ///< 3D Fix using SBAS corrections.
        RtkFloat = 7,  ///< 3D Fix using an approximate RTK solution.
        RtkFix = 8,    ///< 3D Fix using a fixed RTK solution.
    };

    double gps1Tow = 0;     ///< GPS time of week.
    uint16_t gps1Week = 0;  ///< The current GPS week.
    Gnss1Fix gnss1Fix = static_cast<Gnss1Fix>(0);
    uint8_t gnss1NumSats = 0;        ///< Number of satellites tracked by GNSS receiver.
    double posX = 0;                 ///< GNSS position in ECEF-frame x-axis.
    double posY = 0;                 ///< GNSS position in ECEF-frame y-axis.
    double posZ = 0;                 ///< GNSS position in ECEF-frame z-axis.
    float velX = 0;                  ///< GNSS velocity in ECEF-frame x-axis.
    float velY = 0;                  ///< GNSS velocity in ECEF-frame y-axis.
    float velZ = 0;                  ///< GNSS velocity in ECEF-frame z-axis.
    float posUncertaintyX = 0;       ///< GNSS position uncertainty ECEF X.
    float posUncertaintyY = 0;       ///< GNSS position uncertainty ECEF Y.
    float posUncertaintyZ = 0;       ///< GNSS position uncertainty ECEF Z.
    float gnss1VelUncertainty = 0;   ///< GNSS velocity uncertainty (scalar).
    float gnss1TimeUncertainty = 0;  ///< GNSS time uncertainty.

    GnssSolEcef() : MeasurementRegister(59) {}
    static constexpr const char* name() { return "GnssSolEcef"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const GnssSolEcef& lhs, const GnssSolEcef& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 99 - GNSS System Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class GnssSystemConfig : public ConfigurationRegister
{
public:
    struct Systems
    {
        uint16_t gps : 1;
        uint16_t sbas : 1;
        uint16_t glonass : 1;
        uint16_t beidou : 1;
        uint16_t galileo : 1;
        uint16_t imes : 1;
        uint16_t qzssL1Ca : 1;
        uint16_t qzssL1Saif : 1;

        Systems() noexcept = default;
        Systems(uint16_t in) noexcept { std::memcpy(this, &in, sizeof(Systems)); }

        explicit operator uint16_t() const
        {
            uint16_t result;
            std::memcpy(&result, this, sizeof(Systems));
            return result;
        }

        Systems& operator=(const uint16_t& other)
        {
            std::memcpy(this, &other, sizeof(Systems));
            return *this;
        }
    };
    static_assert(sizeof(Systems) == 2);

    struct SbasMode
    {
        uint8_t ranging : 1;
        uint8_t diffCorr : 1;
        uint8_t integrity : 1;
        uint8_t testMode : 1;

        SbasMode() noexcept = default;
        SbasMode(uint8_t in) noexcept { std::memcpy(this, &in, sizeof(SbasMode)); }

        explicit operator uint8_t() const
        {
            uint8_t result;
            std::memcpy(&result, this, sizeof(SbasMode));
            return result;
        }

        SbasMode& operator=(const uint8_t& other)
        {
            std::memcpy(this, &other, sizeof(SbasMode));
            return *this;
        }
    };
    static_assert(sizeof(SbasMode) == 1);

    struct SbasSelect1
    {
        uint16_t sbas120 : 1;
        uint16_t sbas121 : 1;
        uint16_t sbas122 : 1;
        uint16_t sbas123 : 1;
        uint16_t sbas124 : 1;
        uint16_t sbas125 : 1;
        uint16_t sbas126 : 1;
        uint16_t sbas127 : 1;
        uint16_t sbas128 : 1;
        uint16_t sbas129 : 1;
        uint16_t sbas130 : 1;
        uint16_t sbas131 : 1;
        uint16_t sbas132 : 1;
        uint16_t sbas133 : 1;
        uint16_t sbas134 : 1;
        uint16_t sbas135 : 1;

        SbasSelect1() noexcept = default;
        SbasSelect1(uint16_t in) noexcept { std::memcpy(this, &in, sizeof(SbasSelect1)); }

        explicit operator uint16_t() const
        {
            uint16_t result;
            std::memcpy(&result, this, sizeof(SbasSelect1));
            return result;
        }

        SbasSelect1& operator=(const uint16_t& other)
        {
            std::memcpy(this, &other, sizeof(SbasSelect1));
            return *this;
        }
    };
    static_assert(sizeof(SbasSelect1) == 2);

    struct SbasSelect2
    {
        uint16_t sbas136 : 1;
        uint16_t sbas137 : 1;
        uint16_t sbas138 : 1;
        uint16_t sbas139 : 1;
        uint16_t sbas140 : 1;
        uint16_t sbas141 : 1;
        uint16_t sbas142 : 1;
        uint16_t sbas143 : 1;
        uint16_t sbas144 : 1;
        uint16_t sbas145 : 1;
        uint16_t sbas146 : 1;
        uint16_t sbas147 : 1;
        uint16_t sbas148 : 1;
        uint16_t sbas149 : 1;
        uint16_t sbas150 : 1;
        uint16_t sbas151 : 1;

        SbasSelect2() noexcept = default;
        SbasSelect2(uint16_t in) noexcept { std::memcpy(this, &in, sizeof(SbasSelect2)); }

        explicit operator uint16_t() const
        {
            uint16_t result;
            std::memcpy(&result, this, sizeof(SbasSelect2));
            return result;
        }

        SbasSelect2& operator=(const uint16_t& other)
        {
            std::memcpy(this, &other, sizeof(SbasSelect2));
            return *this;
        }
    };
    static_assert(sizeof(SbasSelect2) == 2);

    struct SbasSelect3
    {
        uint16_t sbas152 : 1;
        uint16_t sbas153 : 1;
        uint16_t sbas154 : 1;
        uint16_t sbas155 : 1;
        uint16_t sbas156 : 1;
        uint16_t sbas157 : 1;
        uint16_t sbas158 : 1;

        SbasSelect3() noexcept = default;
        SbasSelect3(uint16_t in) noexcept { std::memcpy(this, &in, sizeof(SbasSelect3)); }

        explicit operator uint16_t() const
        {
            uint16_t result;
            std::memcpy(&result, this, sizeof(SbasSelect3));
            return result;
        }

        SbasSelect3& operator=(const uint16_t& other)
        {
            std::memcpy(this, &other, sizeof(SbasSelect3));
            return *this;
        }
    };
    static_assert(sizeof(SbasSelect3) == 2);

    enum class ReceiverSelect : uint8_t
    {
        GnssAB = 0,
        GnssA = 1,
        GnssB = 2,
    };

    Systems systems = static_cast<Systems>(0);
    uint8_t minCno = 0;
    uint8_t minElev = 0;
    uint8_t maxSats = 0;
    SbasMode sbasMode = static_cast<SbasMode>(0);
    SbasSelect1 sbasSelect1 = static_cast<SbasSelect1>(0);
    SbasSelect2 sbasSelect2 = static_cast<SbasSelect2>(0);
    SbasSelect3 sbasSelect3 = static_cast<SbasSelect3>(0);
    ReceiverSelect receiverSelect = static_cast<ReceiverSelect>(0);

    GnssSystemConfig() : ConfigurationRegister(99) {}
    static constexpr const char* name() { return "GnssSystemConfig"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
    GenericCommand toReadCommand() override;
};

bool operator==(const GnssSystemConfig& lhs, const GnssSystemConfig& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 100 - GNSS Sync Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class GnssSyncConfig : public ConfigurationRegister
{
public:
    enum class GnssSyncEnable : uint8_t
    {
        Off = 0,
        AlwaysOn = 1,
        OnWhenLocked = 2,
    };

    enum class Polarity : uint8_t
    {
        FallingEdge = 0,
        RisingEdge = 1,
    };

    enum class SpecType : uint8_t
    {
        PeriodPulseWidth = 0,
        FreqDutyCycle = 1,
    };

    GnssSyncEnable gnssSyncEnable = static_cast<GnssSyncEnable>(0);
    Polarity polarity = static_cast<Polarity>(0);
    SpecType specType = static_cast<SpecType>(0);
    uint8_t resv = 0;
    uint32_t period = 0;
    uint32_t pulseWidth = 0;
    int32_t offset = 0;

    GnssSyncConfig() : ConfigurationRegister(100) {}
    static constexpr const char* name() { return "GnssSyncConfig"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const GnssSyncConfig& lhs, const GnssSyncConfig& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 103 - GNSS 2 Solution - LLA </summary>
<remarks>

Estimated GNSS 2 Solution with lat/lon/alt position. This register is deprecated and will be
removed in future firmware versions.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class Gnss2SolLla : public MeasurementRegister
{
public:
    enum class Gnss2Fix : uint8_t
    {
        NoFix = 0,     ///< Not tracking.
        TimeFix = 1,   ///< Time-only fix.
        Fix2D = 2,     ///< Time plus 2D position fix.
        Fix3D = 3,     ///< Time plus full 3D position fix.
        SBAS = 4,      ///< 3D Fix using SBAS corrections.
        RtkFloat = 7,  ///< 3D Fix using an approximate RTK solution.
        RtkFix = 8,    ///< 3D Fix using a fixed RTK solution.
    };

    double gps2Tow = 0;     ///< GPS time of week.
    uint16_t gps2Week = 0;  ///< The current GPS week.
    Gnss2Fix gnss2Fix = static_cast<Gnss2Fix>(0);
    uint8_t gnss2NumSats = 0;        ///< Number of satellites tracked by GNSS receiver.
    double lat = 0;                  ///< GNSS geodetic latitude.
    double lon = 0;                  ///< GNSS longitude.
    double alt = 0;                  ///< GNSS altitude above WGS84 ellipsoid.
    float velN = 0;                  ///< GNSS velocity in North direction.
    float velE = 0;                  ///< GNSS velocity in East direction.
    float velD = 0;                  ///< GNSS velocity in Down direction.
    float posUncertaintyN = 0;       ///< GNSS position uncertainty, North direction.
    float posUncertaintyE = 0;       ///< GNSS position uncertainty, East direction.
    float posUncertaintyD = 0;       ///< GNSS position uncertainty, Down direction.
    float gnss2VelUncertainty = 0;   ///< GNSS velocity uncertainty (scalar).
    float gnss2TimeUncertainty = 0;  ///< GNSS time uncertainty.

    Gnss2SolLla() : MeasurementRegister(103) {}
    static constexpr const char* name() { return "Gnss2SolLla"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const Gnss2SolLla& lhs, const Gnss2SolLla& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 104 - GNSS 2 Solution - ECEF </summary>
<remarks>

Estimated GNSS 2 Solution with ECEF position. This register is deprecated and will be removed in
future firmware versions.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class Gnss2SolEcef : public MeasurementRegister
{
public:
    enum class Gnss2Fix : uint8_t
    {
        NoFix = 0,     ///< Not tracking.
        TimeFix = 1,   ///< Time-only fix.
        Fix2D = 2,     ///< Time plus 2D position fix.
        Fix3D = 3,     ///< Time plus full 3D position fix.
        SBAS = 4,      ///< 3D Fix using SBAS corrections.
        RtkFloat = 7,  ///< 3D Fix using an approximate RTK solution.
        RtkFix = 8,    ///< 3D Fix using a fixed RTK solution.
    };

    double gps2Tow = 0;     ///< GPS time of week.
    uint16_t gps2Week = 0;  ///< The current GPS week.
    Gnss2Fix gnss2Fix = static_cast<Gnss2Fix>(0);
    uint8_t gnss2NumSats = 0;        ///< Number of satellites tracked by GNSS receiver.
    double posX = 0;                 ///< GNSS position in ECEF-frame x-axis.
    double posY = 0;                 ///< GNSS position in ECEF-frame y-axis.
    double posZ = 0;                 ///< GNSS position in ECEF-frame z-axis.
    float velX = 0;                  ///< GNSS velocity in ECEF-frame x-axis.
    float velY = 0;                  ///< GNSS velocity in ECEF-frame y-axis.
    float velZ = 0;                  ///< GNSS velocity in ECEF-frame z-axis.
    float posUncertaintyX = 0;       ///< GNSS position uncertainty ECEF X.
    float posUncertaintyY = 0;       ///< GNSS position uncertainty ECEF Y.
    float posUncertaintyZ = 0;       ///< GNSS position uncertainty ECEF Z.
    float gnss2VelUncertainty = 0;   ///< GNSS velocity uncertainty (scalar).
    float gnss2TimeUncertainty = 0;  ///< GNSS time uncertainty.

    Gnss2SolEcef() : MeasurementRegister(104) {}
    static constexpr const char* name() { return "Gnss2SolEcef"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const Gnss2SolEcef& lhs, const Gnss2SolEcef& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 157 - External GNSS Antenna Offset </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class ExtGnssOffset : public ConfigurationRegister
{
public:
    float positionX = 0;
    float positionY = 0;
    float positionZ = 0;

    ExtGnssOffset() : ConfigurationRegister(157) {}
    static constexpr const char* name() { return "ExtGnssOffset"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const ExtGnssOffset& lhs, const ExtGnssOffset& rhs);
}  // namespace GNSS

namespace GNSSCompass
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 86 - GNSS Compass Signal Health Status </summary>
<remarks>

Provides several indicators that serve as an overall health status of the GNSS compass subsystem.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class GnssCompassSignalHealthStatus : public MeasurementRegister
{
public:
    float numSatsPvtA = 0;    ///< Number of satellites available for PVT solution for receiver A.
    float numSatsRtkA = 0;    ///< Number of satellites available for RTK solution for receiver A.
    float highestCn0A = 0;    ///< Highest CN0 reported on receiver A.
    float numSatsPvtB = 0;    ///< Number of satellites available for the PVT solution of receiver B.
    float numSatsRtkB = 0;    ///< Number of satellites available for the RTK solution of receiver B.
    float highestCn0B = 0;    ///< Highest CN0 reported on receiver B.
    float numComSatsPvt = 0;  ///< The number of common satellites that are used in the PVT solutions of both receiver A and receiver B.
    float numComSatsRtk = 0;  ///< The number of common satellites that are used in the RTK solutions of both receiver A and receiver B.

    GnssCompassSignalHealthStatus() : MeasurementRegister(86) {}
    static constexpr const char* name() { return "GnssCompassSignalHealthStatus"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const GnssCompassSignalHealthStatus& lhs, const GnssCompassSignalHealthStatus& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 93 - GNSS Compass Antenna Baseline </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class GnssCompassBaseline : public ConfigurationRegister
{
public:
    float positionX = 0;
    float positionY = 0;
    float positionZ = 0;
    float uncertaintyX = 0;
    float uncertaintyY = 0;
    float uncertaintyZ = 0;

    GnssCompassBaseline() : ConfigurationRegister(93) {}
    static constexpr const char* name() { return "GnssCompassBaseline"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const GnssCompassBaseline& lhs, const GnssCompassBaseline& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 97 - GNSS Compass Estimated Baseline </summary>
<remarks>

Provides the estimated GNSS compass baseline measurement. The estimated position offset and
measurement uncertainty is for antenna B relative to antenna A in the body reference frame.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class GnssCompassEstBaseline : public MeasurementRegister
{
public:
    uint8_t estBaselineComplete = 0;  ///< Set to 1 when baseline estimation has complete.
    uint8_t resv = 0;                 ///< Reserved.
    uint16_t numMeas = 0;             ///< Number of measurements used by the estimated solution.
    float positionX = 0;              ///< Position of GNSS antenna B with respect to antenna A in the body-frame x-axis.
    float positionY = 0;              ///< Position of GNSS antenna B with respect to antenna A in the body-frame y-axis.
    float positionZ = 0;              ///< Position of GNSS antenna B with respect to antenna A in the body-frame z-axis.
    float uncertaintyX = 0;           ///< Uncertainty in the x-axis position measurement.
    float uncertaintyY = 0;           ///< Uncertainty in the y-axis position measurement.
    float uncertaintyZ = 0;           ///< Uncertainty in the z-axis position measurement.

    GnssCompassEstBaseline() : MeasurementRegister(97) {}
    static constexpr const char* name() { return "GnssCompassEstBaseline"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const GnssCompassEstBaseline& lhs, const GnssCompassEstBaseline& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 98 - GNSS Compass Startup Status </summary>
<remarks>

Provides status information on the GNSS compass startup process.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class GnssCompassStartupStatus : public MeasurementRegister
{
public:
    uint8_t percentComplete = 0;  ///< The estimated startup process completion percent.
    float currentHeading = 0;     ///< The current GNSS compass heading estimate.

    GnssCompassStartupStatus() : MeasurementRegister(98) {}
    static constexpr const char* name() { return "GnssCompassStartupStatus"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const GnssCompassStartupStatus& lhs, const GnssCompassStartupStatus& rhs);
}  // namespace GNSSCompass

namespace HardSoftIronEstimator
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 44 - Real-Time HSI Control </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class RealTimeHsiControl : public ConfigurationRegister
{
public:
    enum class Mode : uint8_t
    {
        Off = 0,
        Run = 1,
        Reset = 2,
    };

    enum class ApplyCompensation : uint8_t
    {
        Disable = 1,
        Enable = 3,
    };

    Mode mode = static_cast<Mode>(0);
    ApplyCompensation applyCompensation = static_cast<ApplyCompensation>(0);
    uint8_t convergeRate = 0;

    RealTimeHsiControl() : ConfigurationRegister(44) {}
    static constexpr const char* name() { return "RealTimeHsiControl"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const RealTimeHsiControl& lhs, const RealTimeHsiControl& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 47 - Real-Time HSI Results </summary>
<remarks>

Magnetometer calibration values calculated by the real-time HSI calibration filter.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class EstMagCal : public MeasurementRegister
{
public:
    float magGain00 = 0;  ///< Magnetometer calibration gain matrix, row 0, colum 0.
    float magGain01 = 0;  ///< Magnetometer calibration gain matrix, row 0, colum 1.
    float magGain02 = 0;  ///< Magnetometer calibration gain matrix, row 0, colum 2.
    float magGain10 = 0;  ///< Magnetometer calibration gain matrix, row 1, colum 0.
    float magGain11 = 0;  ///< Magnetometer calibration gain matrix, row 1, colum 1.
    float magGain12 = 0;  ///< Magnetometer calibration gain matrix, row 1, colum 2.
    float magGain20 = 0;  ///< Magnetometer calibration gain matrix, row 2, colum 0.
    float magGain21 = 0;  ///< Magnetometer calibration gain matrix, row 2, colum 1.
    float magGain22 = 0;  ///< Magnetometer calibration gain matrix, row 2, colum 2.
    float magBiasX = 0;   ///< Magnetometer bias calibration, sensor-frame x-axis.
    float magBiasY = 0;   ///< Magnetometer bias calibration, sensor-frame y-axis.
    float magBiasZ = 0;   ///< Magnetometer bias calibration, sensor-frame z-axis.

    EstMagCal() : MeasurementRegister(47) {}
    static constexpr const char* name() { return "EstMagCal"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const EstMagCal& lhs, const EstMagCal& rhs);
}  // namespace HardSoftIronEstimator

namespace Heave
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 115 - Heave and Heave Rate </summary>
<remarks>

Real-time heave and heave-rate estimates, plus a delayed-heave estimate.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class HeaveOutputs : public MeasurementRegister
{
public:
    float heave = 0;         ///< Real-time heave estimate.
    float heaveRate = 0;     ///< Real-time heave rate estimate.
    float delayedHeave = 0;  ///< Delayed heave. Higher accuracy than real-time heave, but 50 seconds old.

    HeaveOutputs() : MeasurementRegister(115) {}
    static constexpr const char* name() { return "HeaveOutputs"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const HeaveOutputs& lhs, const HeaveOutputs& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 116 - Heave Basic Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class HeaveBasicConfig : public ConfigurationRegister
{
public:
    float initialWavePeriod = 0;
    float initialWaveAmplitude = 0;
    float maxWavePeriod = 0;
    float minWaveAmplitude = 0;
    float delayedHeaveCutoffFreq = 0;
    float heaveCutoffFreq = 0;
    float heaveRateCutoffFreq = 0;

    HeaveBasicConfig() : ConfigurationRegister(116) {}
    static constexpr const char* name() { return "HeaveBasicConfig"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const HeaveBasicConfig& lhs, const HeaveBasicConfig& rhs);
}  // namespace Heave

namespace IMU
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 17 - Compensated Magnetometer </summary>
<remarks>

Compensated magnetometer measurements.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class Mag : public MeasurementRegister
{
public:
    float magX = 0;  ///< Compensated magnetometer measurement in the body-frame x-axis.
    float magY = 0;  ///< Compensated magnetometer measurement in the body-frame y-axis.
    float magZ = 0;  ///< Compensated magnetometer measurement in the body-frame z-axis.

    Mag() : MeasurementRegister(17) {}
    static constexpr const char* name() { return "Mag"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const Mag& lhs, const Mag& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 18 - Compensated Accelerometer </summary>
<remarks>

Compensated acceleration measurements

</remarks>
*-----------------------------------------------------------------------------------------------**/
class Accel : public MeasurementRegister
{
public:
    float accelX = 0;  ///< Compensated accelerometer measurement in the body-frame x-axis.
    float accelY = 0;  ///< Compensated accelerometer measurement in the body-frame y-axis.
    float accelZ = 0;  ///< Compensated accelerometer measurement in the body-frame z-axis.

    Accel() : MeasurementRegister(18) {}
    static constexpr const char* name() { return "Accel"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const Accel& lhs, const Accel& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 19 - Compensated Gyro </summary>
<remarks>

Compensated angular rate measurements.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class Gyro : public MeasurementRegister
{
public:
    float gyroX = 0;  ///< Compensated angular rate measurement in the body-frame x-axis.
    float gyroY = 0;  ///< Compensated angular rate measurement in the body-frame y-axis.
    float gyroZ = 0;  ///< Compensated angular rate measurement in the body-frame z-axis.

    Gyro() : MeasurementRegister(19) {}
    static constexpr const char* name() { return "Gyro"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const Gyro& lhs, const Gyro& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 20 - Compensated IMU </summary>
<remarks>

Compensated magnetic, acceleration, and angular rate measurements.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class MagAccelGyro : public MeasurementRegister
{
public:
    float magX = 0;    ///< Compensated magnetometer measurement in the body-frame x-axis.
    float magY = 0;    ///< Compensated magnetometer measurement in the body-frame y-axis.
    float magZ = 0;    ///< Compensated magnetometer measurement in the body-frame z-axis.
    float accelX = 0;  ///< Compensated accelerometer measurement in the body-frame x-axis.
    float accelY = 0;  ///< Compensated accelerometer measurement in the body-frame y-axis.
    float accelZ = 0;  ///< Compensated accelerometer measurement in the body-frame z-axis.
    float gyroX = 0;   ///< Compensated angular rate measurement in the body-frame x-axis.
    float gyroY = 0;   ///< Compensated angular rate measurement in the body-frame y-axis.
    float gyroZ = 0;   ///< Compensated angular rate measurement in the body-frame z-axis.

    MagAccelGyro() : MeasurementRegister(20) {}
    static constexpr const char* name() { return "MagAccelGyro"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const MagAccelGyro& lhs, const MagAccelGyro& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 23 - Magnetometer Calibration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class MagCal : public ConfigurationRegister
{
public:
    float magGain00 = 0;
    float magGain01 = 0;
    float magGain02 = 0;
    float magGain10 = 0;
    float magGain11 = 0;
    float magGain12 = 0;
    float magGain20 = 0;
    float magGain21 = 0;
    float magGain22 = 0;
    float magBiasX = 0;
    float magBiasY = 0;
    float magBiasZ = 0;

    MagCal() : ConfigurationRegister(23) {}
    static constexpr const char* name() { return "MagCal"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const MagCal& lhs, const MagCal& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 25 - Accel Calibration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class AccelCal : public ConfigurationRegister
{
public:
    float accelGain00 = 0;
    float accelGain01 = 0;
    float accelGain02 = 0;
    float accelGain10 = 0;
    float accelGain11 = 0;
    float accelGain12 = 0;
    float accelGain20 = 0;
    float accelGain21 = 0;
    float accelGain22 = 0;
    float accelBiasX = 0;
    float accelBiasY = 0;
    float accelBiasZ = 0;

    AccelCal() : ConfigurationRegister(25) {}
    static constexpr const char* name() { return "AccelCal"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const AccelCal& lhs, const AccelCal& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 26 - Reference Frame Rotation </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class RefFrameRot : public ConfigurationRegister
{
public:
    float rfr00 = 0;
    float rfr01 = 0;
    float rfr02 = 0;
    float rfr10 = 0;
    float rfr11 = 0;
    float rfr12 = 0;
    float rfr20 = 0;
    float rfr21 = 0;
    float rfr22 = 0;

    RefFrameRot() : ConfigurationRegister(26) {}
    static constexpr const char* name() { return "RefFrameRot"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const RefFrameRot& lhs, const RefFrameRot& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 54 - IMU Measurements </summary>
<remarks>

Provides the calibrated IMU measurements including barometric pressure.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class ImuMeas : public MeasurementRegister
{
public:
    float uncompMagX = 0;   ///< Magnetometer body-frame x-axis.
    float uncompMagY = 0;   ///< Magnetometer body-frame y-axis.
    float uncompMagZ = 0;   ///< Magnetometer body-frame z-axis.
    float uncompAccX = 0;   ///< Accelerometer body-frame x-axis.
    float uncompAccY = 0;   ///< Accelerometer body-frame y-axis.
    float uncompAccZ = 0;   ///< Accelerometer body-frame z-axis.
    float uncompGyroX = 0;  ///< Angular rate body-frame x-axis.
    float uncompGyroY = 0;  ///< Angular rate body-frame y-axis.
    float uncompGyroZ = 0;  ///< Angular rate body-frame z-axis.
    float temperature = 0;  ///< Sensor temperature.
    float pressure = 0;     ///< Barometric pressure.

    ImuMeas() : MeasurementRegister(54) {}
    static constexpr const char* name() { return "ImuMeas"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const ImuMeas& lhs, const ImuMeas& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 80 - Delta Theta and Delta Velocity </summary>
<remarks>

This register contains the output values of the onboard coning and sculling algorithm.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class DeltaThetaVelocity : public MeasurementRegister
{
public:
    float deltaTime = 0;    ///< Duration of integration interval.
    float deltaThetaX = 0;  ///< Integrated rotation vector x-axis.
    float deltaThetaY = 0;  ///< Integrated rotation vector y-axis.
    float deltaThetaZ = 0;  ///< Integrated rotation vector z-axis.
    float deltaVelX = 0;    ///< Integrated velocity x-axis.
    float deltaVelY = 0;    ///< Integrated velocity y-axis.
    float deltaVelZ = 0;    ///< Integrated velocity z-axis.

    DeltaThetaVelocity() : MeasurementRegister(80) {}
    static constexpr const char* name() { return "DeltaThetaVelocity"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const DeltaThetaVelocity& lhs, const DeltaThetaVelocity& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 82 - Delta Theta and Delta Velocity Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class DeltaThetaVelConfig : public ConfigurationRegister
{
public:
    enum class IntegrationFrame : uint8_t
    {
        Body = 0,
        NED = 1,
    };

    enum class GyroCompensation : uint8_t
    {
        None = 0,
        Bias = 1,
    };

    enum class AccelCompensation : uint8_t
    {
        None = 0,
        Gravity = 1,
        Bias = 2,
        BiasAndGravity = 3,
    };

    enum class EarthRateCompensation : uint8_t
    {
        None = 0,
        GyroRate = 1,
        CoriolisAccel = 2,
        RateAndCoriolis = 3,
    };

    IntegrationFrame integrationFrame = static_cast<IntegrationFrame>(0);
    GyroCompensation gyroCompensation = static_cast<GyroCompensation>(0);
    AccelCompensation accelCompensation = static_cast<AccelCompensation>(0);
    EarthRateCompensation earthRateCompensation = static_cast<EarthRateCompensation>(0);
    uint16_t resv = 0;

    DeltaThetaVelConfig() : ConfigurationRegister(82) {}
    static constexpr const char* name() { return "DeltaThetaVelConfig"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const DeltaThetaVelConfig& lhs, const DeltaThetaVelConfig& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 84 - Gyro Calibration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class GyroCal : public ConfigurationRegister
{
public:
    float gyroGain00 = 0;
    float gyroGain01 = 0;
    float gyroGain02 = 0;
    float gyroGain10 = 0;
    float gyroGain11 = 0;
    float gyroGain12 = 0;
    float gyroGain20 = 0;
    float gyroGain21 = 0;
    float gyroGain22 = 0;
    float gyroBiasX = 0;
    float gyroBiasY = 0;
    float gyroBiasZ = 0;

    GyroCal() : ConfigurationRegister(84) {}
    static constexpr const char* name() { return "GyroCal"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const GyroCal& lhs, const GyroCal& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 85 - IMU Filtering Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class ImuFilterControl : public ConfigurationRegister
{
public:
    struct MagFilterMode
    {
        uint8_t uncomp : 1;
        uint8_t comp : 1;

        MagFilterMode() noexcept = default;
        MagFilterMode(uint8_t in) noexcept { std::memcpy(this, &in, sizeof(MagFilterMode)); }

        explicit operator uint8_t() const
        {
            uint8_t result;
            std::memcpy(&result, this, sizeof(MagFilterMode));
            return result;
        }

        MagFilterMode& operator=(const uint8_t& other)
        {
            std::memcpy(this, &other, sizeof(MagFilterMode));
            return *this;
        }
    };
    static_assert(sizeof(MagFilterMode) == 1);

    struct AccelFilterMode
    {
        uint8_t uncomp : 1;
        uint8_t comp : 1;

        AccelFilterMode() noexcept = default;
        AccelFilterMode(uint8_t in) noexcept { std::memcpy(this, &in, sizeof(AccelFilterMode)); }

        explicit operator uint8_t() const
        {
            uint8_t result;
            std::memcpy(&result, this, sizeof(AccelFilterMode));
            return result;
        }

        AccelFilterMode& operator=(const uint8_t& other)
        {
            std::memcpy(this, &other, sizeof(AccelFilterMode));
            return *this;
        }
    };
    static_assert(sizeof(AccelFilterMode) == 1);

    struct GyroFilterMode
    {
        uint8_t uncomp : 1;
        uint8_t comp : 1;

        GyroFilterMode() noexcept = default;
        GyroFilterMode(uint8_t in) noexcept { std::memcpy(this, &in, sizeof(GyroFilterMode)); }

        explicit operator uint8_t() const
        {
            uint8_t result;
            std::memcpy(&result, this, sizeof(GyroFilterMode));
            return result;
        }

        GyroFilterMode& operator=(const uint8_t& other)
        {
            std::memcpy(this, &other, sizeof(GyroFilterMode));
            return *this;
        }
    };
    static_assert(sizeof(GyroFilterMode) == 1);

    struct TempFilterMode
    {
        uint8_t uncomp : 1;
        uint8_t comp : 1;

        TempFilterMode() noexcept = default;
        TempFilterMode(uint8_t in) noexcept { std::memcpy(this, &in, sizeof(TempFilterMode)); }

        explicit operator uint8_t() const
        {
            uint8_t result;
            std::memcpy(&result, this, sizeof(TempFilterMode));
            return result;
        }

        TempFilterMode& operator=(const uint8_t& other)
        {
            std::memcpy(this, &other, sizeof(TempFilterMode));
            return *this;
        }
    };
    static_assert(sizeof(TempFilterMode) == 1);

    struct PresFilterMode
    {
        uint8_t uncomp : 1;
        uint8_t comp : 1;

        PresFilterMode() noexcept = default;
        PresFilterMode(uint8_t in) noexcept { std::memcpy(this, &in, sizeof(PresFilterMode)); }

        explicit operator uint8_t() const
        {
            uint8_t result;
            std::memcpy(&result, this, sizeof(PresFilterMode));
            return result;
        }

        PresFilterMode& operator=(const uint8_t& other)
        {
            std::memcpy(this, &other, sizeof(PresFilterMode));
            return *this;
        }
    };
    static_assert(sizeof(PresFilterMode) == 1);

    uint16_t magWindowSize = 0;
    uint16_t accelWindowSize = 0;
    uint16_t gyroWindowSize = 0;
    uint16_t tempWindowSize = 0;
    uint16_t presWindowSize = 0;
    MagFilterMode magFilterMode = static_cast<MagFilterMode>(0);
    AccelFilterMode accelFilterMode = static_cast<AccelFilterMode>(0);
    GyroFilterMode gyroFilterMode = static_cast<GyroFilterMode>(0);
    TempFilterMode tempFilterMode = static_cast<TempFilterMode>(0);
    PresFilterMode presFilterMode = static_cast<PresFilterMode>(0);

    ImuFilterControl() : ConfigurationRegister(85) {}
    static constexpr const char* name() { return "ImuFilterControl"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const ImuFilterControl& lhs, const ImuFilterControl& rhs);
}  // namespace IMU

namespace INS
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 63 - INS Solution - LLA </summary>
<remarks>

Estimated INS solution with lat/lon/alt position.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class InsSolLla : public MeasurementRegister
{
public:
    double timeGpsTow = 0;     ///< GPS time of week.
    uint16_t timeGpsWeek = 0;  ///< The current GPS week.
    InsStatus insStatus = 0;   ///< Ins status bitfield.
    float yaw = 0;             ///< Yaw angle.
    float pitch = 0;           ///< Pitch angle.
    float roll = 0;            ///< Roll angle.
    double posLat = 0;         ///< Geodetic latitude.
    double posLon = 0;         ///< Longitude.
    double posAlt = 0;         ///< Altitude above WGS84 ellipsoid.
    float velN = 0;            ///< Velocity in North direction.
    float velE = 0;            ///< Velocity in East direction.
    float velD = 0;            ///< Velocity in Down direction.
    float attUncertainty = 0;  ///< Filter estimated attitude uncertainty.
    float posUncertainty = 0;  ///< Filter estimated position uncertainty.
    float velUncertainty = 0;  ///< Filter estimated velocity uncertainty.

    InsSolLla() : MeasurementRegister(63) {}
    static constexpr const char* name() { return "InsSolLla"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const InsSolLla& lhs, const InsSolLla& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 64 - INS Solution - ECEF </summary>
<remarks>

Estimated INS Solution with ECEF position

</remarks>
*-----------------------------------------------------------------------------------------------**/
class InsSolEcef : public MeasurementRegister
{
public:
    double timeGpsTow = 0;     ///< GPS time of week.
    uint16_t timeGpsWeek = 0;  ///< The current GPS week.
    InsStatus insStatus = 0;   ///< Ins status bitfield.
    float yaw = 0;             ///< Yaw angle.
    float pitch = 0;           ///< Pitch angle.
    float roll = 0;            ///< Roll angle.
    double posEx = 0;          ///< Position in ECEF-frame x-axis.
    double posEy = 0;          ///< Position in ECEF-frame y-axis.
    double posEz = 0;          ///< Position in ECEF-frame z-axis.
    float velEx = 0;           ///< Velocity in ECEF-frame x-axis.
    float velEy = 0;           ///< Velocity in ECEF-frame y-axis.
    float velEz = 0;           ///< Velocity in ECEF-frame z-axis.
    float attUncertainty = 0;  ///< Filter estimated attitude uncertainty.
    float posUncertainty = 0;  ///< Filter estimated position uncertainty.
    float velUncertainty = 0;  ///< Filter estimated velocity uncertainty.

    InsSolEcef() : MeasurementRegister(64) {}
    static constexpr const char* name() { return "InsSolEcef"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const InsSolEcef& lhs, const InsSolEcef& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 67 - INS Basic Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class InsBasicConfig : public ConfigurationRegister
{
public:
    enum class Scenario : uint8_t
    {
        Ahrs = 0,
        GnssInsWithPressure = 1,
        GnssInsNoPressure = 2,
        DualGnssNoPressure = 3,
        DualGnssWithPressure = 5,
    };

    enum class AhrsAiding : uint8_t
    {
        Disable = 0,
        Enable = 1,
    };

    enum class EstBaseline : uint8_t
    {
        Disable = 0,
        Enable = 1,
    };

    Scenario scenario = static_cast<Scenario>(0);
    AhrsAiding ahrsAiding = static_cast<AhrsAiding>(0);
    EstBaseline estBaseline = static_cast<EstBaseline>(0);
    uint8_t resv = 0;

    InsBasicConfig() : ConfigurationRegister(67) {}
    static constexpr const char* name() { return "InsBasicConfig"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const InsBasicConfig& lhs, const InsBasicConfig& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 72 - INS State - LLA </summary>
<remarks>

Estimated INS state with lat/lon/alt position.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class InsStateLla : public MeasurementRegister
{
public:
    float yaw = 0;      ///< Yaw angle.
    float pitch = 0;    ///< Pitch angle.
    float roll = 0;     ///< Roll angle.
    double posLat = 0;  ///< Geodetic latitude.
    double posLon = 0;  ///< Longitude.
    double posAlt = 0;  ///< Altitude above WGS84 ellipsoid.
    float velN = 0;     ///< Velocity in North direction.
    float velE = 0;     ///< Velocity in East direction.
    float velD = 0;     ///< Velocity in Down direction.
    float accelX = 0;   ///< Compensated accelerometer measurement in the body-frame x-axis.
    float accelY = 0;   ///< Compensated accelerometer measurement in the body-frame y-axis.
    float accelZ = 0;   ///< Compensated accelerometer measurement in the body-frame z-axis.
    float gyroX = 0;    ///< Compensated angular rate measurement in the body-frame x-axis.
    float gyroY = 0;    ///< Compensated angular rate measurement in the body-frame y-axis.
    float gyroZ = 0;    ///< Compensated angular rate measurement in the body-frame z-axis.

    InsStateLla() : MeasurementRegister(72) {}
    static constexpr const char* name() { return "InsStateLla"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const InsStateLla& lhs, const InsStateLla& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 73 - INS State - ECEF </summary>
<remarks>

Estimated INS state with ECEF position.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class InsStateEcef : public MeasurementRegister
{
public:
    float yaw = 0;     ///< Yaw angle.
    float pitch = 0;   ///< Pitch angle.
    float roll = 0;    ///< Roll angle.
    double posEx = 0;  ///< Position in ECEF-frame x-axis.
    double posEy = 0;  ///< Position in ECEF-frame y-axis.
    double posEz = 0;  ///< Position in ECEF-frame z-axis.
    float velEx = 0;   ///< Velocity in ECEF-frame x-axis.
    float velEy = 0;   ///< Velocity in ECEF-frame y-axis.
    float velEz = 0;   ///< Velocity in ECEF-frame z-axis.
    float accelX = 0;  ///< Compensated accelerometer measurement in the body-frame x-axis.
    float accelY = 0;  ///< Compensated accelerometer measurement in the body-frame y-axis.
    float accelZ = 0;  ///< Compensated accelerometer measurement in the body-frame z-axis.
    float gyroX = 0;   ///< Compensated angular rate measurement in the body-frame x-axis.
    float gyroY = 0;   ///< Compensated angular rate measurement in the body-frame y-axis.
    float gyroZ = 0;   ///< Compensated angular rate measurement in the body-frame z-axis.

    InsStateEcef() : MeasurementRegister(73) {}
    static constexpr const char* name() { return "InsStateEcef"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const InsStateEcef& lhs, const InsStateEcef& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 74 - Filter Startup Bias </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class FilterStartupBias : public ConfigurationRegister
{
public:
    float gyroBiasX = 0;
    float gyroBiasY = 0;
    float gyroBiasZ = 0;
    float accelBiasX = 0;
    float accelBiasY = 0;
    float accelBiasZ = 0;
    float presBias = 0;

    FilterStartupBias() : ConfigurationRegister(74) {}
    static constexpr const char* name() { return "FilterStartupBias"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const FilterStartupBias& lhs, const FilterStartupBias& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 105 - INS Reference Point Offset </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class InsRefOffset : public ConfigurationRegister
{
public:
    float refOffsetX = 0;
    float refOffsetY = 0;
    float refOffsetZ = 0;
    float refUncertX = 0;
    float refUncertY = 0;
    float refUncertZ = 0;

    InsRefOffset() : ConfigurationRegister(105) {}
    static constexpr const char* name() { return "InsRefOffset"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const InsRefOffset& lhs, const InsRefOffset& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 144 - INS GNSS Select </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class InsGnssSelect : public ConfigurationRegister
{
public:
    enum class ActiveReceiverSelect : uint8_t
    {
        None = 0,
        PrimaryReceiver = 1,
        SecondaryReceiver = 2,
        TertiaryReceiver = 3,
        FallbackOnFailure = 5,
    };

    enum class UseGnssCompass : uint8_t
    {
        Off = 0,
        On = 1,
    };

    ActiveReceiverSelect activeReceiverSelect = static_cast<ActiveReceiverSelect>(0);
    uint8_t usedForNavTime = 0;
    uint8_t hysteresisTime = 0;
    UseGnssCompass useGnssCompass = static_cast<UseGnssCompass>(0);
    uint8_t resv1 = 0;
    uint8_t resv2 = 0;

    InsGnssSelect() : ConfigurationRegister(144) {}
    static constexpr const char* name() { return "InsGnssSelect"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const InsGnssSelect& lhs, const InsGnssSelect& rhs);
}  // namespace INS

namespace System
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 0 - User Tag </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class UserTag : public ConfigurationRegister
{
public:
    AsciiMessage tag = "";

    UserTag() : ConfigurationRegister(0) {}
    static constexpr const char* name() { return "UserTag"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const UserTag& lhs, const UserTag& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 1 - Model </summary>
<remarks>

Product model string.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class Model : public MeasurementRegister
{
public:
    AsciiMessage model = "";  ///< Product model number, maximum length 24 characters.

    Model() : MeasurementRegister(1) {}
    static constexpr const char* name() { return "Model"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const Model& lhs, const Model& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 2 - Hardware Version </summary>
<remarks>

Hardware version number.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class HwVer : public MeasurementRegister
{
public:
    uint32_t hwVer = 0;     ///< Hardware version number.
    uint32_t hwMinVer = 0;  ///< Hardware minor version number.

    HwVer() : MeasurementRegister(2) {}
    static constexpr const char* name() { return "HwVer"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const HwVer& lhs, const HwVer& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 3 - Serial Number </summary>
<remarks>

Device serial number.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class Serial : public MeasurementRegister
{
public:
    uint32_t serialNum = 0;  ///< The unit's serial number.

    Serial() : MeasurementRegister(3) {}
    static constexpr const char* name() { return "Serial"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const Serial& lhs, const Serial& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 4 - Firmware Version </summary>
<remarks>

Firmware version number.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class FwVer : public MeasurementRegister
{
public:
    AsciiMessage fwVer = "";  ///< Firmware version.

    FwVer() : MeasurementRegister(4) {}
    static constexpr const char* name() { return "FwVer"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const FwVer& lhs, const FwVer& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 5 - Baud Rate </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class BaudRate : public ConfigurationRegister
{
public:
    enum class BaudRates : uint32_t
    {
        Baud9600 = 9600,
        Baud19200 = 19200,
        Baud38400 = 38400,
        Baud57600 = 57600,
        Baud115200 = 115200,
        Baud128000 = 128000,
        Baud230400 = 230400,
        Baud460800 = 460800,
        Baud921600 = 921600,
    };

    enum class SerialPort : uint8_t
    {
        ActiveSerial = 0,  ///< Set value on active serial port.
        Serial1 = 1,
        Serial2 = 2,
        Poll = static_cast<uint8_t>('?')
    };

    BaudRates baudRate = static_cast<BaudRates>(0);
    SerialPort serialPort = static_cast<SerialPort>(0);

    BaudRate() : ConfigurationRegister(5) {}
    static constexpr const char* name() { return "BaudRate"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
    GenericCommand toReadCommand() override;
};

bool operator==(const BaudRate& lhs, const BaudRate& rhs);

inline std::ostream& operator<<(std::ostream& outStream, const BaudRate::BaudRates& baudrate) noexcept { return outStream << static_cast<uint32_t>(baudrate); }
/**--------------------------------------------------------------------------------------------------
<summary> Register 6 - Async Data Output Type </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class AsyncOutputType : public ConfigurationRegister
{
public:
    enum class Ador : uint32_t
    {
        OFF = 0,
        YPR = 1,
        QTN = 2,
        QMR = 8,
        MAG = 10,
        ACC = 11,
        GYR = 12,
        MAR = 13,
        YMR = 14,
        YBA = 16,
        YIA = 17,
        IMU = 19,
        GPS = 20,
        GPE = 21,
        INS = 22,
        INE = 23,
        ISL = 28,
        ISE = 29,
        DTV = 30,
        G2S = 32,
        G2E = 33,
        HVE = 34,
    };

    enum class SerialPort : uint8_t
    {
        ActiveSerial = 0,  ///< Set value on active serial port.
        Serial1 = 1,
        Serial2 = 2,
        Poll = static_cast<uint8_t>('?')
    };

    Ador ador = static_cast<Ador>(0);
    SerialPort serialPort = static_cast<SerialPort>(0);

    AsyncOutputType() : ConfigurationRegister(6) {}
    static constexpr const char* name() { return "AsyncOutputType"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
    GenericCommand toReadCommand() override;
};

bool operator==(const AsyncOutputType& lhs, const AsyncOutputType& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 7 - Async Data Output Freq </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class AsyncOutputFreq : public ConfigurationRegister
{
public:
    enum class Adof : uint32_t
    {
        Rate0Hz = 0,
        Rate1Hz = 1,
        Rate2Hz = 2,
        Rate4Hz = 4,
        Rate5Hz = 5,
        Rate10Hz = 10,
        Rate20Hz = 20,
        Rate25Hz = 25,
        Rate40Hz = 40,
        Rate50Hz = 50,
        Rate100Hz = 100,
        Rate200Hz = 200,
    };

    enum class SerialPort : uint8_t
    {
        ActiveSerial = 0,  ///< Set value on active serial port.
        Serial1 = 1,
        Serial2 = 2,
        Poll = static_cast<uint8_t>('?')
    };

    Adof adof = static_cast<Adof>(0);
    SerialPort serialPort = static_cast<SerialPort>(0);

    AsyncOutputFreq() : ConfigurationRegister(7) {}
    static constexpr const char* name() { return "AsyncOutputFreq"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
    GenericCommand toReadCommand() override;
};

bool operator==(const AsyncOutputFreq& lhs, const AsyncOutputFreq& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 30 - Communication Protocol Control </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class ProtocolControl : public ConfigurationRegister
{
public:
    enum class AsciiAppendCount : uint8_t
    {
        None = 0,
        SyncInCount = 1,
        SyncInTime = 2,
        SyncOutCount = 3,
        GpsPps = 4,
        GpsTow = 5,
    };

    enum class AsciiAppendStatus : uint8_t
    {
        None = 0,
        Ahrs = 1,
        Ins = 2,
        Imu = 3,
        Gnss1 = 4,
        Gnss2 = 5,
        Gnss3 = 6,
    };

    enum class SpiAppendCount : uint8_t
    {
        None = 0,
        SyncInCount = 1,
        SyncInTime = 2,
        SyncOutCount = 3,
        GpsPps = 4,
        GpsTow = 5,
    };

    enum class SpiAppendStatus : uint8_t
    {
        None = 0,
        Ahrs = 1,
        Ins = 2,
        Imu = 3,
        Gnss1 = 4,
        Gnss2 = 5,
        Gnss3 = 6,
    };

    enum class AsciiChecksum : uint8_t
    {
        Checksum8bit = 1,
        Crc16bit = 3,
    };

    enum class SpiChecksum : uint8_t
    {
        Off = 0,
        Checksum8bit = 1,
        Crc16bit = 3,
    };

    enum class ErrorMode : uint8_t
    {
        Ignore = 0,
        SendError = 1,
        AdorOff = 2,
    };

    AsciiAppendCount asciiAppendCount = static_cast<AsciiAppendCount>(0);
    AsciiAppendStatus asciiAppendStatus = static_cast<AsciiAppendStatus>(0);
    SpiAppendCount spiAppendCount = static_cast<SpiAppendCount>(0);
    SpiAppendStatus spiAppendStatus = static_cast<SpiAppendStatus>(0);
    AsciiChecksum asciiChecksum = static_cast<AsciiChecksum>(0);
    SpiChecksum spiChecksum = static_cast<SpiChecksum>(0);
    ErrorMode errorMode = static_cast<ErrorMode>(0);

    ProtocolControl() : ConfigurationRegister(30) {}
    static constexpr const char* name() { return "ProtocolControl"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const ProtocolControl& lhs, const ProtocolControl& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 32 - Synchronization Control </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class SyncControl : public ConfigurationRegister
{
public:
    enum class SyncInMode : uint8_t
    {
        Disable = 0,
        Count = 3,
        ImuSample = 4,
        AsyncAll = 5,
        Async0 = 6,
    };

    enum class SyncInEdge : uint8_t
    {
        RisingEdge = 0,
        FallingEdge = 1,
    };

    enum class SyncOutMode : uint8_t
    {
        None = 0,
        ImuStart = 1,
        ImuReady = 2,
        NavReady = 3,
        GpsPps = 6,
    };

    enum class SyncOutPolarity : uint8_t
    {
        NegativePulse = 0,
        PositivePulse = 1,
    };

    SyncInMode syncInMode = static_cast<SyncInMode>(0);
    SyncInEdge syncInEdge = static_cast<SyncInEdge>(0);
    uint16_t syncInSkipFactor = 0;
    uint32_t resv1 = 0;
    SyncOutMode syncOutMode = static_cast<SyncOutMode>(0);
    SyncOutPolarity syncOutPolarity = static_cast<SyncOutPolarity>(0);
    uint16_t syncOutSkipFactor = 0;
    uint32_t syncOutPulseWidth = 0;
    uint32_t resv2 = 0;

    SyncControl() : ConfigurationRegister(32) {}
    static constexpr const char* name() { return "SyncControl"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const SyncControl& lhs, const SyncControl& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 33 - Synchronization Status </summary>
<remarks>

Contains counters based on the SyncIn and SyncOut events.

</remarks>
*-----------------------------------------------------------------------------------------------**/
class SyncStatus : public MeasurementRegister
{
public:
    uint32_t syncInCount = 0;   ///< Amount of SyncIn Events that have occurred.
    uint32_t syncInTime = 0;    ///< The amount of time that has elapsed since the last SyncIn Event.
    uint32_t syncOutCount = 0;  ///< Keeps track of the number of times that a SyncOut pulse has occurred.

    SyncStatus() : MeasurementRegister(33) {}
    static constexpr const char* name() { return "SyncStatus"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    using MeasurementRegister::toString;
};

bool operator==(const SyncStatus& lhs, const SyncStatus& rhs);
struct BinaryOutputMeasurements
{
    struct Common
    {
        uint32_t timeStartup : 1;
        uint32_t timeGps : 1;
        uint32_t timeSyncIn : 1;
        uint32_t ypr : 1;
        uint32_t quaternion : 1;
        uint32_t angularRate : 1;
        uint32_t posLla : 1;
        uint32_t velNed : 1;
        uint32_t accel : 1;
        uint32_t imu : 1;
        uint32_t magPres : 1;
        uint32_t deltas : 1;
        uint32_t insStatus : 1;
        uint32_t syncInCnt : 1;
        uint32_t timeGpsPps : 1;

        Common() noexcept = default;
        Common(uint32_t in) noexcept { std::memcpy(this, &in, sizeof(Common)); }
        explicit operator uint32_t() const
        {
            uint32_t result;
            std::memcpy(&result, this, sizeof(Common));
            return result;
        }
        Common& operator=(const uint32_t& other)
        {
            std::memcpy(this, &other, sizeof(Common));
            return *this;
        }

        uint32_t : 17;  ///< padding
    };
    static_assert(sizeof(Common) == 4);

    struct Time
    {
        uint32_t timeStartup : 1;
        uint32_t timeGps : 1;
        uint32_t timeGpsTow : 1;
        uint32_t timeGpsWeek : 1;
        uint32_t timeSyncIn : 1;
        uint32_t timeGpsPps : 1;
        uint32_t timeUtc : 1;
        uint32_t syncInCnt : 1;
        uint32_t syncOutCnt : 1;
        uint32_t timeStatus : 1;

        Time() noexcept = default;
        Time(uint32_t in) noexcept { std::memcpy(this, &in, sizeof(Time)); }
        explicit operator uint32_t() const
        {
            uint32_t result;
            std::memcpy(&result, this, sizeof(Time));
            return result;
        }
        Time& operator=(const uint32_t& other)
        {
            std::memcpy(this, &other, sizeof(Time));
            return *this;
        }

        uint32_t : 22;  ///< padding
    };
    static_assert(sizeof(Time) == 4);

    struct Imu
    {
        uint32_t imuStatus : 1;
        uint32_t uncompMag : 1;
        uint32_t uncompAccel : 1;
        uint32_t uncompGyro : 1;
        uint32_t temperature : 1;
        uint32_t pressure : 1;
        uint32_t deltaTheta : 1;
        uint32_t deltaVel : 1;
        uint32_t mag : 1;
        uint32_t accel : 1;
        uint32_t angularRate : 1;
        uint32_t sensSat : 1;

        Imu() noexcept = default;
        Imu(uint32_t in) noexcept { std::memcpy(this, &in, sizeof(Imu)); }
        explicit operator uint32_t() const
        {
            uint32_t result;
            std::memcpy(&result, this, sizeof(Imu));
            return result;
        }
        Imu& operator=(const uint32_t& other)
        {
            std::memcpy(this, &other, sizeof(Imu));
            return *this;
        }

        uint32_t : 20;  ///< padding
    };
    static_assert(sizeof(Imu) == 4);

    struct Gnss
    {
        uint32_t gnss1TimeUtc : 1;
        uint32_t gps1Tow : 1;
        uint32_t gps1Week : 1;
        uint32_t gnss1NumSats : 1;
        uint32_t gnss1Fix : 1;
        uint32_t gnss1PosLla : 1;
        uint32_t gnss1PosEcef : 1;
        uint32_t gnss1VelNed : 1;
        uint32_t gnss1VelEcef : 1;
        uint32_t gnss1PosUncertainty : 1;
        uint32_t gnss1VelUncertainty : 1;
        uint32_t gnss1TimeUncertainty : 1;
        uint32_t gnss1TimeInfo : 1;
        uint32_t gnss1Dop : 1;
        uint32_t gnss1SatInfo : 1;
        uint32_t : 1;  ///< 15: Extension Bit
        uint32_t gnss1RawMeas : 1;
        uint32_t gnss1Status : 1;
        uint32_t gnss1AltMsl : 1;

        Gnss() noexcept = default;
        Gnss(uint32_t in) noexcept { std::memcpy(this, &in, sizeof(Gnss)); }
        explicit operator uint32_t() const
        {
            uint32_t result;
            std::memcpy(&result, this, sizeof(Gnss));
            return result;
        }
        Gnss& operator=(const uint32_t& other)
        {
            std::memcpy(this, &other, sizeof(Gnss));
            return *this;
        }

        uint32_t : 13;  ///< padding
    };
    static_assert(sizeof(Gnss) == 4);

    struct Attitude
    {
        uint32_t : 1;  ///< 0: padding
        uint32_t ypr : 1;
        uint32_t quaternion : 1;
        uint32_t dcm : 1;
        uint32_t magNed : 1;
        uint32_t accelNed : 1;
        uint32_t linBodyAcc : 1;
        uint32_t linAccelNed : 1;
        uint32_t yprU : 1;
        uint32_t : 1;  ///< 9: padding
        uint32_t : 1;  ///< 10: padding
        uint32_t : 1;  ///< 11: padding
        uint32_t heave : 1;
        uint32_t attU : 1;

        Attitude() noexcept = default;
        Attitude(uint32_t in) noexcept { std::memcpy(this, &in, sizeof(Attitude)); }
        explicit operator uint32_t() const
        {
            uint32_t result;
            std::memcpy(&result, this, sizeof(Attitude));
            return result;
        }
        Attitude& operator=(const uint32_t& other)
        {
            std::memcpy(this, &other, sizeof(Attitude));
            return *this;
        }

        uint32_t : 18;  ///< padding
    };
    static_assert(sizeof(Attitude) == 4);

    struct Ins
    {
        uint32_t insStatus : 1;
        uint32_t posLla : 1;
        uint32_t posEcef : 1;
        uint32_t velBody : 1;
        uint32_t velNed : 1;
        uint32_t velEcef : 1;
        uint32_t magEcef : 1;
        uint32_t accelEcef : 1;
        uint32_t linAccelEcef : 1;
        uint32_t posU : 1;
        uint32_t velU : 1;

        Ins() noexcept = default;
        Ins(uint32_t in) noexcept { std::memcpy(this, &in, sizeof(Ins)); }
        explicit operator uint32_t() const
        {
            uint32_t result;
            std::memcpy(&result, this, sizeof(Ins));
            return result;
        }
        Ins& operator=(const uint32_t& other)
        {
            std::memcpy(this, &other, sizeof(Ins));
            return *this;
        }

        uint32_t : 21;  ///< padding
    };
    static_assert(sizeof(Ins) == 4);

    struct Gnss2
    {
        uint32_t gnss2TimeUtc : 1;
        uint32_t gps2Tow : 1;
        uint32_t gps2Week : 1;
        uint32_t gnss2NumSats : 1;
        uint32_t gnss2Fix : 1;
        uint32_t gnss2PosLla : 1;
        uint32_t gnss2PosEcef : 1;
        uint32_t gnss2VelNed : 1;
        uint32_t gnss2VelEcef : 1;
        uint32_t gnss2PosUncertainty : 1;
        uint32_t gnss2VelUncertainty : 1;
        uint32_t gnss2TimeUncertainty : 1;
        uint32_t gnss2TimeInfo : 1;
        uint32_t gnss2Dop : 1;
        uint32_t gnss2SatInfo : 1;
        uint32_t : 1;  ///< 15: Extension Bit
        uint32_t gnss2RawMeas : 1;
        uint32_t gnss2Status : 1;
        uint32_t gnss2AltMsl : 1;

        Gnss2() noexcept = default;
        Gnss2(uint32_t in) noexcept { std::memcpy(this, &in, sizeof(Gnss2)); }
        explicit operator uint32_t() const
        {
            uint32_t result;
            std::memcpy(&result, this, sizeof(Gnss2));
            return result;
        }
        Gnss2& operator=(const uint32_t& other)
        {
            std::memcpy(this, &other, sizeof(Gnss2));
            return *this;
        }

        uint32_t : 13;  ///< padding
    };
    static_assert(sizeof(Gnss2) == 4);

    struct Gnss3
    {
        uint32_t gnss3TimeUtc : 1;
        uint32_t gps3Tow : 1;
        uint32_t gps3Week : 1;
        uint32_t gnss3NumSats : 1;
        uint32_t gnss3Fix : 1;
        uint32_t gnss3PosLla : 1;
        uint32_t gnss3PosEcef : 1;
        uint32_t gnss3VelNed : 1;
        uint32_t gnss3VelEcef : 1;
        uint32_t gnss3PosUncertainty : 1;
        uint32_t gnss3VelUncertainty : 1;
        uint32_t gnss3TimeUncertainty : 1;
        uint32_t gnss3TimeInfo : 1;
        uint32_t gnss3Dop : 1;
        uint32_t gnss3SatInfo : 1;
        uint32_t : 1;  ///< 15: Extension Bit
        uint32_t gnss3RawMeas : 1;
        uint32_t gnss3Status : 1;
        uint32_t gnss3AltMsl : 1;

        Gnss3() noexcept = default;
        Gnss3(uint32_t in) noexcept { std::memcpy(this, &in, sizeof(Gnss3)); }
        explicit operator uint32_t() const
        {
            uint32_t result;
            std::memcpy(&result, this, sizeof(Gnss3));
            return result;
        }
        Gnss3& operator=(const uint32_t& other)
        {
            std::memcpy(this, &other, sizeof(Gnss3));
            return *this;
        }

        uint32_t : 13;  ///< padding
    };
    static_assert(sizeof(Gnss3) == 4);

    Common common{0};
    Time time{0};
    Imu imu{0};
    Gnss gnss{0};
    Attitude attitude{0};
    Ins ins{0};
    Gnss2 gnss2{0};
    Gnss3 gnss3{0};

    BinaryHeader toBinaryHeader() const noexcept;
};
class BinaryOutput : public ConfigurationRegister, public BinaryOutputMeasurements
{
public:
    using ConfigurationRegister::ConfigurationRegister;

    struct AsyncMode
    {
        uint16_t serial1 : 1;
        uint16_t serial2 : 1;
        uint16_t spi : 1;
        uint16_t : 13;  // padding

        AsyncMode() noexcept = default;
        AsyncMode(uint16_t in) noexcept { std::memcpy(this, &in, sizeof(AsyncMode)); }

        explicit operator uint16_t() const
        {
            uint16_t result;
            std::memcpy(&result, this, sizeof(AsyncMode));
            return result;
        }
        AsyncMode& operator=(const uint16_t& other)
        {
            std::memcpy(this, &other, sizeof(AsyncMode));
            return *this;
        }
    };
    static_assert(sizeof(AsyncMode) == 2);

    AsyncMode asyncMode{0};
    uint16_t rateDivisor = 0;

    bool fromString(const AsciiMessage& sensorResponsePayload) override;
    AsciiMessage toString() const override;

protected:
    using Register::_id;
};
inline bool operator==(const BinaryOutput& lhs, const BinaryOutput& rhs)
{
    return ((uint16_t(lhs.asyncMode) == uint16_t(rhs.asyncMode)) && (lhs.rateDivisor == rhs.rateDivisor) && (uint32_t(lhs.common) == uint32_t(rhs.common)) &&
            (uint32_t(lhs.time) == uint32_t(rhs.time)) && (uint32_t(lhs.imu) == uint32_t(rhs.imu)) && (uint32_t(lhs.gnss) == uint32_t(rhs.gnss)) &&
            (uint32_t(lhs.attitude) == uint32_t(rhs.attitude)) && (uint32_t(lhs.ins) == uint32_t(rhs.ins)) && (uint32_t(lhs.gnss2) == uint32_t(rhs.gnss2)) &&
            (uint32_t(lhs.gnss3) == uint32_t(rhs.gnss3)));
}

class BinaryOutput1 : public BinaryOutput
{
public:
    BinaryOutput1() : BinaryOutput{75} {}
    static constexpr const char* name() { return "BinaryOutput1"; };
};

class BinaryOutput2 : public BinaryOutput
{
public:
    BinaryOutput2() : BinaryOutput{76} {}
    static constexpr const char* name() { return "BinaryOutput2"; };
};

class BinaryOutput3 : public BinaryOutput
{
public:
    BinaryOutput3() : BinaryOutput{77} {}
    static constexpr const char* name() { return "BinaryOutput3"; };
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 101 - NMEA Output 1 </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class NmeaOutput1 : public ConfigurationRegister
{
public:
    enum class Port : uint8_t
    {
        None = 0,
        Serial1 = 1,
        Serial2 = 2,
        Serial1And2 = 3,
    };

    enum class Rate : uint8_t
    {
        Rate0Hz = 0,
        Rate1Hz = 1,
        Rate5Hz = 5,
        Rate10Hz = 10,
        Rate20Hz = 20,
    };

    enum class Mode : uint8_t
    {
        V41GPID = 0,
        V23GPID = 1,
        V41SYSID = 2,
    };

    enum class GnssSelect : uint8_t
    {
        GnssA = 0,
        ActiveReceiver = 1,
    };

    struct MsgSelection
    {
        uint32_t rmcGnss : 1;
        uint32_t rmcIns : 1;
        uint32_t ggaGnss : 1;
        uint32_t ggaIns : 1;
        uint32_t gllGnss : 1;
        uint32_t gllIns : 1;
        uint32_t gsaGnss : 1;
        uint32_t gsvGnss : 1;
        uint32_t hdgIns : 1;
        uint32_t hdtIns : 1;
        uint32_t thsIns : 1;
        uint32_t vtgGnss : 1;
        uint32_t vtgIns : 1;
        uint32_t zdaGnss : 1;
        uint32_t zdaIns : 1;
        uint32_t pashrIns : 1;
        uint32_t tss1Ins : 1;
        uint32_t indyn : 1;

        MsgSelection() noexcept = default;
        MsgSelection(uint32_t in) noexcept { std::memcpy(this, &in, sizeof(MsgSelection)); }

        explicit operator uint32_t() const
        {
            uint32_t result;
            std::memcpy(&result, this, sizeof(MsgSelection));
            return result;
        }

        MsgSelection& operator=(const uint32_t& other)
        {
            std::memcpy(this, &other, sizeof(MsgSelection));
            return *this;
        }
    };
    static_assert(sizeof(MsgSelection) == 4);

    Port port = static_cast<Port>(0);
    Rate rate = static_cast<Rate>(0);
    Mode mode = static_cast<Mode>(0);
    GnssSelect gnssSelect = static_cast<GnssSelect>(0);
    MsgSelection msgSelection = static_cast<MsgSelection>(0);

    NmeaOutput1() : ConfigurationRegister(101) {}
    static constexpr const char* name() { return "NmeaOutput1"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const NmeaOutput1& lhs, const NmeaOutput1& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 102 - NMEA Output 2 </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class NmeaOutput2 : public ConfigurationRegister
{
public:
    enum class Port : uint8_t
    {
        None = 0,
        Serial1 = 1,
        Serial2 = 2,
        Serial1And2 = 3,
    };

    enum class Rate : uint8_t
    {
        Rate0Hz = 0,
        Rate1Hz = 1,
        Rate5Hz = 5,
        Rate10Hz = 10,
        Rate20Hz = 20,
    };

    enum class Mode : uint8_t
    {
        V41GPID = 0,
        V23GPID = 1,
        V41SYSID = 2,
    };

    enum class GnssSelect : uint8_t
    {
        GnssA = 0,
        ActiveReceiver = 1,
    };

    struct MsgSelection
    {
        uint32_t rmcGnss : 1;
        uint32_t rmcIns : 1;
        uint32_t ggaGnss : 1;
        uint32_t ggaIns : 1;
        uint32_t gllGnss : 1;
        uint32_t gllIns : 1;
        uint32_t gsaGnss : 1;
        uint32_t gsvGnss : 1;
        uint32_t hdgIns : 1;
        uint32_t hdtIns : 1;
        uint32_t thsIns : 1;
        uint32_t vtgGnss : 1;
        uint32_t vtgIns : 1;
        uint32_t zdaGnss : 1;
        uint32_t zdaIns : 1;
        uint32_t pashrIns : 1;
        uint32_t tss1Ins : 1;
        uint32_t indyn : 1;

        MsgSelection() noexcept = default;
        MsgSelection(uint32_t in) noexcept { std::memcpy(this, &in, sizeof(MsgSelection)); }

        explicit operator uint32_t() const
        {
            uint32_t result;
            std::memcpy(&result, this, sizeof(MsgSelection));
            return result;
        }

        MsgSelection& operator=(const uint32_t& other)
        {
            std::memcpy(this, &other, sizeof(MsgSelection));
            return *this;
        }
    };
    static_assert(sizeof(MsgSelection) == 4);

    Port port = static_cast<Port>(0);
    Rate rate = static_cast<Rate>(0);
    Mode mode = static_cast<Mode>(0);
    GnssSelect gnssSelect = static_cast<GnssSelect>(0);
    MsgSelection msgSelection = static_cast<MsgSelection>(0);

    NmeaOutput2() : ConfigurationRegister(102) {}
    static constexpr const char* name() { return "NmeaOutput2"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const NmeaOutput2& lhs, const NmeaOutput2& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 206 - Legacy Compatibility Settings </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class LegacyCompatibilitySettings : public ConfigurationRegister
{
public:
    enum class InsLegacy : uint8_t
    {
        Current = 0,
        Legacy = 1,
    };

    struct GnssLegacy
    {
        uint8_t legacyGnssFix : 1;
        uint8_t requireReg55Reset : 1;
        uint8_t alwaysPpsPulse : 1;

        GnssLegacy() noexcept = default;
        GnssLegacy(uint8_t in) noexcept { std::memcpy(this, &in, sizeof(GnssLegacy)); }

        explicit operator uint8_t() const
        {
            uint8_t result;
            std::memcpy(&result, this, sizeof(GnssLegacy));
            return result;
        }

        GnssLegacy& operator=(const uint8_t& other)
        {
            std::memcpy(this, &other, sizeof(GnssLegacy));
            return *this;
        }
    };
    static_assert(sizeof(GnssLegacy) == 1);

    enum class ImuLegacy : uint8_t
    {
        Current = 0,
        Legacy = 1,
    };

    enum class HwLegacy : uint8_t
    {
        Current = 0,
        Legacy = 1,
    };

    InsLegacy insLegacy = static_cast<InsLegacy>(0);
    GnssLegacy gnssLegacy = static_cast<GnssLegacy>(0);
    ImuLegacy imuLegacy = static_cast<ImuLegacy>(0);
    HwLegacy hwLegacy = static_cast<HwLegacy>(0);

    LegacyCompatibilitySettings() : ConfigurationRegister(206) {}
    static constexpr const char* name() { return "LegacyCompatibilitySettings"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const LegacyCompatibilitySettings& lhs, const LegacyCompatibilitySettings& rhs);
}  // namespace System

namespace VelocityAiding
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 50 - Velocity Aiding Measurement </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class VelAidingMeas : public ConfigurationRegister
{
public:
    float velocityX = 0;
    float velocityY = 0;
    float velocityZ = 0;

    VelAidingMeas() : ConfigurationRegister(50) {}
    static constexpr const char* name() { return "VelAidingMeas"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const VelAidingMeas& lhs, const VelAidingMeas& rhs);
/**--------------------------------------------------------------------------------------------------
<summary> Register 51 - Velocity Aiding Control </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class VelAidingControl : public ConfigurationRegister
{
public:
    enum class VelAidEnable : uint8_t
    {
        Disable = 0,
        Enable = 1,
    };

    VelAidEnable velAidEnable = static_cast<VelAidEnable>(0);
    float velUncertTuning = 0;
    float resv = 0;

    VelAidingControl() : ConfigurationRegister(51) {}
    static constexpr const char* name() { return "VelAidingControl"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const VelAidingControl& lhs, const VelAidingControl& rhs);
}  // namespace VelocityAiding

namespace WorldMagGravityModel
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 83 - Reference Model Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
class RefModelConfig : public ConfigurationRegister
{
public:
    enum class EnableMagModel : uint8_t
    {
        Disabled = 0,
        Enabled = 1,
    };

    enum class EnableGravityModel : uint8_t
    {
        Disabled = 0,
        Enabled = 1,
    };

    EnableMagModel enableMagModel = static_cast<EnableMagModel>(0);
    EnableGravityModel enableGravityModel = static_cast<EnableGravityModel>(0);
    uint8_t resv1 = 0;
    uint8_t resv2 = 0;
    uint32_t recalcThreshold = 0;
    float year = 0;
    double latitude = 0;
    double longitude = 0;
    double altitude = 0;

    RefModelConfig() : ConfigurationRegister(83) {}
    static constexpr const char* name() { return "RefModelConfig"; };

    bool fromString(const AsciiMessage& sensorResponse) override;
    AsciiMessage toString() const override;
};

bool operator==(const RefModelConfig& lhs, const RefModelConfig& rhs);
}  // namespace WorldMagGravityModel

namespace ById
{
using Reg0 = Registers::System::UserTag;
using Reg1 = Registers::System::Model;
using Reg2 = Registers::System::HwVer;
using Reg3 = Registers::System::Serial;
using Reg4 = Registers::System::FwVer;
using Reg5 = Registers::System::BaudRate;
using Reg6 = Registers::System::AsyncOutputType;
using Reg7 = Registers::System::AsyncOutputFreq;
using Reg8 = Registers::Attitude::YawPitchRoll;
using Reg9 = Registers::Attitude::Quaternion;
using Reg15 = Registers::Attitude::QuatMagAccelRate;
using Reg17 = Registers::IMU::Mag;
using Reg18 = Registers::IMU::Accel;
using Reg19 = Registers::IMU::Gyro;
using Reg20 = Registers::IMU::MagAccelGyro;
using Reg21 = Registers::Attitude::MagGravRefVec;
using Reg23 = Registers::IMU::MagCal;
using Reg25 = Registers::IMU::AccelCal;
using Reg26 = Registers::IMU::RefFrameRot;
using Reg27 = Registers::Attitude::YprMagAccelAngularRates;
using Reg30 = Registers::System::ProtocolControl;
using Reg32 = Registers::System::SyncControl;
using Reg33 = Registers::System::SyncStatus;
using Reg35 = Registers::Attitude::VpeBasicControl;
using Reg36 = Registers::Attitude::VpeMagBasicTuning;
using Reg38 = Registers::Attitude::VpeAccelBasicTuning;
using Reg44 = Registers::HardSoftIronEstimator::RealTimeHsiControl;
using Reg47 = Registers::HardSoftIronEstimator::EstMagCal;
using Reg50 = Registers::VelocityAiding::VelAidingMeas;
using Reg51 = Registers::VelocityAiding::VelAidingControl;
using Reg54 = Registers::IMU::ImuMeas;
using Reg55 = Registers::GNSS::GnssBasicConfig;
using Reg57 = Registers::GNSS::GnssAOffset;
using Reg58 = Registers::GNSS::GnssSolLla;
using Reg59 = Registers::GNSS::GnssSolEcef;
using Reg63 = Registers::INS::InsSolLla;
using Reg64 = Registers::INS::InsSolEcef;
using Reg67 = Registers::INS::InsBasicConfig;
using Reg72 = Registers::INS::InsStateLla;
using Reg73 = Registers::INS::InsStateEcef;
using Reg74 = Registers::INS::FilterStartupBias;
using Reg75 = Registers::System::BinaryOutput1;
using Reg76 = Registers::System::BinaryOutput2;
using Reg77 = Registers::System::BinaryOutput3;
using Reg80 = Registers::IMU::DeltaThetaVelocity;
using Reg82 = Registers::IMU::DeltaThetaVelConfig;
using Reg83 = Registers::WorldMagGravityModel::RefModelConfig;
using Reg84 = Registers::IMU::GyroCal;
using Reg85 = Registers::IMU::ImuFilterControl;
using Reg86 = Registers::GNSSCompass::GnssCompassSignalHealthStatus;
using Reg93 = Registers::GNSSCompass::GnssCompassBaseline;
using Reg97 = Registers::GNSSCompass::GnssCompassEstBaseline;
using Reg98 = Registers::GNSSCompass::GnssCompassStartupStatus;
using Reg99 = Registers::GNSS::GnssSystemConfig;
using Reg100 = Registers::GNSS::GnssSyncConfig;
using Reg101 = Registers::System::NmeaOutput1;
using Reg102 = Registers::System::NmeaOutput2;
using Reg103 = Registers::GNSS::Gnss2SolLla;
using Reg104 = Registers::GNSS::Gnss2SolEcef;
using Reg105 = Registers::INS::InsRefOffset;
using Reg115 = Registers::Heave::HeaveOutputs;
using Reg116 = Registers::Heave::HeaveBasicConfig;
using Reg144 = Registers::INS::InsGnssSelect;
using Reg157 = Registers::GNSS::ExtGnssOffset;
using Reg206 = Registers::System::LegacyCompatibilitySettings;
using Reg239 = Registers::Attitude::YprLinearBodyAccelAngularRates;
using Reg240 = Registers::Attitude::YprLinearInertialAccelAngularRates;
}  // namespace ById

inline bool isConfigEqual(const VN::ConfigurationRegister& lhs, const VN::ConfigurationRegister& rhs)
{
    using namespace VN::Registers::ById;

    if (lhs.id() != rhs.id()) { return false; }

    switch (lhs.id())
    {
        case 0:
            return static_cast<const Reg0&>(lhs) == static_cast<const Reg0&>(rhs);
        case 5:
            return static_cast<const Reg5&>(lhs) == static_cast<const Reg5&>(rhs);
        case 6:
            return static_cast<const Reg6&>(lhs) == static_cast<const Reg6&>(rhs);
        case 7:
            return static_cast<const Reg7&>(lhs) == static_cast<const Reg7&>(rhs);
        case 21:
            return static_cast<const Reg21&>(lhs) == static_cast<const Reg21&>(rhs);
        case 23:
            return static_cast<const Reg23&>(lhs) == static_cast<const Reg23&>(rhs);
        case 25:
            return static_cast<const Reg25&>(lhs) == static_cast<const Reg25&>(rhs);
        case 26:
            return static_cast<const Reg26&>(lhs) == static_cast<const Reg26&>(rhs);
        case 30:
            return static_cast<const Reg30&>(lhs) == static_cast<const Reg30&>(rhs);
        case 32:
            return static_cast<const Reg32&>(lhs) == static_cast<const Reg32&>(rhs);
        case 35:
            return static_cast<const Reg35&>(lhs) == static_cast<const Reg35&>(rhs);
        case 36:
            return static_cast<const Reg36&>(lhs) == static_cast<const Reg36&>(rhs);
        case 38:
            return static_cast<const Reg38&>(lhs) == static_cast<const Reg38&>(rhs);
        case 44:
            return static_cast<const Reg44&>(lhs) == static_cast<const Reg44&>(rhs);
        case 50:
            return static_cast<const Reg50&>(lhs) == static_cast<const Reg50&>(rhs);
        case 51:
            return static_cast<const Reg51&>(lhs) == static_cast<const Reg51&>(rhs);
        case 55:
            return static_cast<const Reg55&>(lhs) == static_cast<const Reg55&>(rhs);
        case 57:
            return static_cast<const Reg57&>(lhs) == static_cast<const Reg57&>(rhs);
        case 67:
            return static_cast<const Reg67&>(lhs) == static_cast<const Reg67&>(rhs);
        case 74:
            return static_cast<const Reg74&>(lhs) == static_cast<const Reg74&>(rhs);
        case 75:
            return static_cast<const VN::Registers::System::BinaryOutput&>(lhs) == static_cast<const VN::Registers::System::BinaryOutput&>(rhs);
        case 76:
            return static_cast<const VN::Registers::System::BinaryOutput&>(lhs) == static_cast<const VN::Registers::System::BinaryOutput&>(rhs);
        case 77:
            return static_cast<const VN::Registers::System::BinaryOutput&>(lhs) == static_cast<const VN::Registers::System::BinaryOutput&>(rhs);
        case 82:
            return static_cast<const Reg82&>(lhs) == static_cast<const Reg82&>(rhs);
        case 83:
            return static_cast<const Reg83&>(lhs) == static_cast<const Reg83&>(rhs);
        case 84:
            return static_cast<const Reg84&>(lhs) == static_cast<const Reg84&>(rhs);
        case 85:
            return static_cast<const Reg85&>(lhs) == static_cast<const Reg85&>(rhs);
        case 93:
            return static_cast<const Reg93&>(lhs) == static_cast<const Reg93&>(rhs);
        case 99:
            return static_cast<const Reg99&>(lhs) == static_cast<const Reg99&>(rhs);
        case 100:
            return static_cast<const Reg100&>(lhs) == static_cast<const Reg100&>(rhs);
        case 101:
            return static_cast<const Reg101&>(lhs) == static_cast<const Reg101&>(rhs);
        case 102:
            return static_cast<const Reg102&>(lhs) == static_cast<const Reg102&>(rhs);
        case 105:
            return static_cast<const Reg105&>(lhs) == static_cast<const Reg105&>(rhs);
        case 116:
            return static_cast<const Reg116&>(lhs) == static_cast<const Reg116&>(rhs);
        case 144:
            return static_cast<const Reg144&>(lhs) == static_cast<const Reg144&>(rhs);
        case 157:
            return static_cast<const Reg157&>(lhs) == static_cast<const Reg157&>(rhs);
        case 206:
            return static_cast<const Reg206&>(lhs) == static_cast<const Reg206&>(rhs);
        default:
            abort();
            break;
    }

    return false;
}  // bool operator==

constexpr static std::optional<const char*> getConfigRegisterName(uint8_t regId)
{
    switch (regId)
    {
        case 0:
        {
            return VN::Registers::ById::Reg0::name();
        }
        case 5:
        {
            return VN::Registers::ById::Reg5::name();
        }
        case 6:
        {
            return VN::Registers::ById::Reg6::name();
        }
        case 7:
        {
            return VN::Registers::ById::Reg7::name();
        }
        case 21:
        {
            return VN::Registers::ById::Reg21::name();
        }
        case 23:
        {
            return VN::Registers::ById::Reg23::name();
        }
        case 25:
        {
            return VN::Registers::ById::Reg25::name();
        }
        case 26:
        {
            return VN::Registers::ById::Reg26::name();
        }
        case 30:
        {
            return VN::Registers::ById::Reg30::name();
        }
        case 32:
        {
            return VN::Registers::ById::Reg32::name();
        }
        case 35:
        {
            return VN::Registers::ById::Reg35::name();
        }
        case 36:
        {
            return VN::Registers::ById::Reg36::name();
        }
        case 38:
        {
            return VN::Registers::ById::Reg38::name();
        }
        case 44:
        {
            return VN::Registers::ById::Reg44::name();
        }
        case 50:
        {
            return VN::Registers::ById::Reg50::name();
        }
        case 51:
        {
            return VN::Registers::ById::Reg51::name();
        }
        case 55:
        {
            return VN::Registers::ById::Reg55::name();
        }
        case 57:
        {
            return VN::Registers::ById::Reg57::name();
        }
        case 67:
        {
            return VN::Registers::ById::Reg67::name();
        }
        case 74:
        {
            return VN::Registers::ById::Reg74::name();
        }
        case 75:
        {
            return VN::Registers::ById::Reg75::name();
        }
        case 76:
        {
            return VN::Registers::ById::Reg76::name();
        }
        case 77:
        {
            return VN::Registers::ById::Reg77::name();
        }
        case 82:
        {
            return VN::Registers::ById::Reg82::name();
        }
        case 83:
        {
            return VN::Registers::ById::Reg83::name();
        }
        case 84:
        {
            return VN::Registers::ById::Reg84::name();
        }
        case 85:
        {
            return VN::Registers::ById::Reg85::name();
        }
        case 93:
        {
            return VN::Registers::ById::Reg93::name();
        }
        case 99:
        {
            return VN::Registers::ById::Reg99::name();
        }
        case 100:
        {
            return VN::Registers::ById::Reg100::name();
        }
        case 101:
        {
            return VN::Registers::ById::Reg101::name();
        }
        case 102:
        {
            return VN::Registers::ById::Reg102::name();
        }
        case 105:
        {
            return VN::Registers::ById::Reg105::name();
        }
        case 116:
        {
            return VN::Registers::ById::Reg116::name();
        }
        case 144:
        {
            return VN::Registers::ById::Reg144::name();
        }
        case 157:
        {
            return VN::Registers::ById::Reg157::name();
        }
        case 206:
        {
            return VN::Registers::ById::Reg206::name();
        }
        default:
        {
            return std::nullopt;
        }
    }
}

constexpr static std::optional<const char*> getMeasurementRegisterName(uint8_t regId)
{
    switch (regId)
    {
        case 1:
        {
            return VN::Registers::ById::Reg1::name();
        }
        case 2:
        {
            return VN::Registers::ById::Reg2::name();
        }
        case 3:
        {
            return VN::Registers::ById::Reg3::name();
        }
        case 4:
        {
            return VN::Registers::ById::Reg4::name();
        }
        case 8:
        {
            return VN::Registers::ById::Reg8::name();
        }
        case 9:
        {
            return VN::Registers::ById::Reg9::name();
        }
        case 15:
        {
            return VN::Registers::ById::Reg15::name();
        }
        case 17:
        {
            return VN::Registers::ById::Reg17::name();
        }
        case 18:
        {
            return VN::Registers::ById::Reg18::name();
        }
        case 19:
        {
            return VN::Registers::ById::Reg19::name();
        }
        case 20:
        {
            return VN::Registers::ById::Reg20::name();
        }
        case 27:
        {
            return VN::Registers::ById::Reg27::name();
        }
        case 33:
        {
            return VN::Registers::ById::Reg33::name();
        }
        case 47:
        {
            return VN::Registers::ById::Reg47::name();
        }
        case 54:
        {
            return VN::Registers::ById::Reg54::name();
        }
        case 58:
        {
            return VN::Registers::ById::Reg58::name();
        }
        case 59:
        {
            return VN::Registers::ById::Reg59::name();
        }
        case 63:
        {
            return VN::Registers::ById::Reg63::name();
        }
        case 64:
        {
            return VN::Registers::ById::Reg64::name();
        }
        case 72:
        {
            return VN::Registers::ById::Reg72::name();
        }
        case 73:
        {
            return VN::Registers::ById::Reg73::name();
        }
        case 80:
        {
            return VN::Registers::ById::Reg80::name();
        }
        case 86:
        {
            return VN::Registers::ById::Reg86::name();
        }
        case 97:
        {
            return VN::Registers::ById::Reg97::name();
        }
        case 98:
        {
            return VN::Registers::ById::Reg98::name();
        }
        case 103:
        {
            return VN::Registers::ById::Reg103::name();
        }
        case 104:
        {
            return VN::Registers::ById::Reg104::name();
        }
        case 115:
        {
            return VN::Registers::ById::Reg115::name();
        }
        case 239:
        {
            return VN::Registers::ById::Reg239::name();
        }
        case 240:
        {
            return VN::Registers::ById::Reg240::name();
        }
        default:
        {
            return std::nullopt;
        }
    }
}

}  // namespace Registers
}  // namespace VN

#endif  // VN_REGISTERS_HPP_
