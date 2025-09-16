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

#ifndef VN_CLI_REGISTERS_HPP_
#define VN_CLI_REGISTERS_HPP_

#define NOMINMAX 1
#include "Windows.h"
#include <string>
#include <conio.h>
#include <msclr/marshal.h>

#include "CLI_GenericCommand.hpp"

#pragma managed(push, off)
#include "vectornav/Interface/Registers.hpp"
#pragma managed(pop)

using namespace System;
using namespace msclr::interop;

namespace VNSDK
{
namespace Registers
{
public ref class Register
{
    protected:
    VN::Register* _reg;
    
    public:
    Register(VN::Register* reg) : _reg(reg) {};
    ~Register() { delete _reg; };
    VN::Register* GetReference() {return _reg;};
    
    VNSDK::GenericCommand^ ToReadCommand()
    {
        VN::GenericCommand* cmd_c = new VN::GenericCommand(_reg->toReadCommand());
        VNSDK::GenericCommand^ cmd = gcnew VNSDK::GenericCommand(cmd_c);
        return cmd;
    };
    
    VNSDK::GenericCommand^ ToWriteCommand()
    {
        VN::ConfigurationRegister* configReg = dynamic_cast<VN::ConfigurationRegister*>(_reg);
        if (configReg != nullptr)
        {
            VN::GenericCommand* cmd_c = new VN::GenericCommand(configReg->toWriteCommand());
            VNSDK::GenericCommand^ cmd = gcnew GenericCommand(cmd_c);
            return cmd;
        }
        else
        {
            throw gcnew InvalidCastException("Cast to ConfigurationRegister failed.");
        }
    };
};

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
public ref class YawPitchRoll : public Register
{
    public:
    YawPitchRoll() : Register(new VN::Registers::Attitude::YawPitchRoll())
    {
    };
    
    property float yaw
    {
        float get()
        {
            return ((VN::Registers::Attitude::YawPitchRoll *)_reg)->yaw;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YawPitchRoll *)_reg)->yaw = value;
        }
        
    };
    
    property float pitch
    {
        float get()
        {
            return ((VN::Registers::Attitude::YawPitchRoll *)_reg)->pitch;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YawPitchRoll *)_reg)->pitch = value;
        }
        
    };
    
    property float roll
    {
        float get()
        {
            return ((VN::Registers::Attitude::YawPitchRoll *)_reg)->roll;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YawPitchRoll *)_reg)->roll = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 9 - Quaternion </summary>
<remarks>

Attitude solution as a quaternion. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class Quaternion : public Register
{
    public:
    Quaternion() : Register(new VN::Registers::Attitude::Quaternion())
    {
    };
    
    property float quatX
    {
        float get()
        {
            return ((VN::Registers::Attitude::Quaternion *)_reg)->quatX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::Quaternion *)_reg)->quatX = value;
        }
        
    };
    
    property float quatY
    {
        float get()
        {
            return ((VN::Registers::Attitude::Quaternion *)_reg)->quatY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::Quaternion *)_reg)->quatY = value;
        }
        
    };
    
    property float quatZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::Quaternion *)_reg)->quatZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::Quaternion *)_reg)->quatZ = value;
        }
        
    };
    
    property float quatS
    {
        float get()
        {
            return ((VN::Registers::Attitude::Quaternion *)_reg)->quatS;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::Quaternion *)_reg)->quatS = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 15 - Quaternion & Compensated IMU </summary>
<remarks>

Quaternion attitude solution, and compensated (Magnetic, Acceleration, Angular Rate) values. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class QuatMagAccelRate : public Register
{
    public:
    QuatMagAccelRate() : Register(new VN::Registers::Attitude::QuatMagAccelRate())
    {
    };
    
    property float quatX
    {
        float get()
        {
            return ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->quatX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->quatX = value;
        }
        
    };
    
    property float quatY
    {
        float get()
        {
            return ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->quatY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->quatY = value;
        }
        
    };
    
    property float quatZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->quatZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->quatZ = value;
        }
        
    };
    
    property float quatS
    {
        float get()
        {
            return ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->quatS;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->quatS = value;
        }
        
    };
    
    property float magX
    {
        float get()
        {
            return ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->magX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->magX = value;
        }
        
    };
    
    property float magY
    {
        float get()
        {
            return ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->magY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->magY = value;
        }
        
    };
    
    property float magZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->magZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->magZ = value;
        }
        
    };
    
    property float accelX
    {
        float get()
        {
            return ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->accelX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->accelX = value;
        }
        
    };
    
    property float accelY
    {
        float get()
        {
            return ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->accelY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->accelY = value;
        }
        
    };
    
    property float accelZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->accelZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->accelZ = value;
        }
        
    };
    
    property float gyroX
    {
        float get()
        {
            return ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->gyroX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->gyroX = value;
        }
        
    };
    
    property float gyroY
    {
        float get()
        {
            return ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->gyroY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->gyroY = value;
        }
        
    };
    
    property float gyroZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->gyroZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::QuatMagAccelRate *)_reg)->gyroZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 21 - Magnetic and Gravity Reference Vectors </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class MagGravRefVec : public Register
{
    public:
    MagGravRefVec() : Register(new VN::Registers::Attitude::MagGravRefVec())
    {
    };
    
    property float magRefN
    {
        float get()
        {
            return ((VN::Registers::Attitude::MagGravRefVec *)_reg)->magRefN;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::MagGravRefVec *)_reg)->magRefN = value;
        }
        
    };
    
    property float magRefE
    {
        float get()
        {
            return ((VN::Registers::Attitude::MagGravRefVec *)_reg)->magRefE;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::MagGravRefVec *)_reg)->magRefE = value;
        }
        
    };
    
    property float magRefD
    {
        float get()
        {
            return ((VN::Registers::Attitude::MagGravRefVec *)_reg)->magRefD;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::MagGravRefVec *)_reg)->magRefD = value;
        }
        
    };
    
    property float gravRefN
    {
        float get()
        {
            return ((VN::Registers::Attitude::MagGravRefVec *)_reg)->gravRefN;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::MagGravRefVec *)_reg)->gravRefN = value;
        }
        
    };
    
    property float gravRefE
    {
        float get()
        {
            return ((VN::Registers::Attitude::MagGravRefVec *)_reg)->gravRefE;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::MagGravRefVec *)_reg)->gravRefE = value;
        }
        
    };
    
    property float gravRefD
    {
        float get()
        {
            return ((VN::Registers::Attitude::MagGravRefVec *)_reg)->gravRefD;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::MagGravRefVec *)_reg)->gravRefD = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 27 - Yaw-Pitch-Roll & Compensated IMU </summary>
<remarks>

Yaw, Pitch, Roll, Accel, and Angular Rates 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class YprMagAccelAngularRates : public Register
{
    public:
    YprMagAccelAngularRates() : Register(new VN::Registers::Attitude::YprMagAccelAngularRates())
    {
    };
    
    property float yaw
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->yaw;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->yaw = value;
        }
        
    };
    
    property float pitch
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->pitch;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->pitch = value;
        }
        
    };
    
    property float roll
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->roll;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->roll = value;
        }
        
    };
    
    property float magX
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->magX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->magX = value;
        }
        
    };
    
    property float magY
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->magY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->magY = value;
        }
        
    };
    
    property float magZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->magZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->magZ = value;
        }
        
    };
    
    property float accelX
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->accelX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->accelX = value;
        }
        
    };
    
    property float accelY
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->accelY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->accelY = value;
        }
        
    };
    
    property float accelZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->accelZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->accelZ = value;
        }
        
    };
    
    property float gyroX
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->gyroX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->gyroX = value;
        }
        
    };
    
    property float gyroY
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->gyroY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->gyroY = value;
        }
        
    };
    
    property float gyroZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->gyroZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprMagAccelAngularRates *)_reg)->gyroZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 35 - VPE Basic Control </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class VpeBasicControl : public Register
{
    public:
    VpeBasicControl() : Register(new VN::Registers::Attitude::VpeBasicControl())
    {
    };
    
    property uint8_t resv
    {
        uint8_t get()
        {
            return ((VN::Registers::Attitude::VpeBasicControl *)_reg)->resv;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::Attitude::VpeBasicControl *)_reg)->resv = value;
        }
        
    };
    
    enum class HeadingMode : uint8_t
    {
        Absolute = 0,
        Relative = 1,
        Indoor = 2,
    };
    property HeadingMode headingMode
    {
        HeadingMode get()
        {
            return static_cast<VpeBasicControl::HeadingMode>(((VN::Registers::Attitude::VpeBasicControl *)_reg)->headingMode);
        }
        
        void set(HeadingMode value)
        {
            ((VN::Registers::Attitude::VpeBasicControl *)_reg)->headingMode = static_cast<VN::Registers::Attitude::VpeBasicControl::HeadingMode>(value);
        }
        
    };
    
    enum class FilteringMode : uint8_t
    {
        Unfiltered = 0,
        AdaptivelyFiltered = 1,
    };
    property FilteringMode filteringMode
    {
        FilteringMode get()
        {
            return static_cast<VpeBasicControl::FilteringMode>(((VN::Registers::Attitude::VpeBasicControl *)_reg)->filteringMode);
        }
        
        void set(FilteringMode value)
        {
            ((VN::Registers::Attitude::VpeBasicControl *)_reg)->filteringMode = static_cast<VN::Registers::Attitude::VpeBasicControl::FilteringMode>(value);
        }
        
    };
    
    enum class TuningMode : uint8_t
    {
        Static = 0,
        Adaptive = 1,
    };
    property TuningMode tuningMode
    {
        TuningMode get()
        {
            return static_cast<VpeBasicControl::TuningMode>(((VN::Registers::Attitude::VpeBasicControl *)_reg)->tuningMode);
        }
        
        void set(TuningMode value)
        {
            ((VN::Registers::Attitude::VpeBasicControl *)_reg)->tuningMode = static_cast<VN::Registers::Attitude::VpeBasicControl::TuningMode>(value);
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 36 - VPE Magnetometer Basic Tuning </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class VpeMagBasicTuning : public Register
{
    public:
    VpeMagBasicTuning() : Register(new VN::Registers::Attitude::VpeMagBasicTuning())
    {
    };
    
    property float baseTuningX
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->baseTuningX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->baseTuningX = value;
        }
        
    };
    
    property float baseTuningY
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->baseTuningY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->baseTuningY = value;
        }
        
    };
    
    property float baseTuningZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->baseTuningZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->baseTuningZ = value;
        }
        
    };
    
    property float adaptiveTuningX
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->adaptiveTuningX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->adaptiveTuningX = value;
        }
        
    };
    
    property float adaptiveTuningY
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->adaptiveTuningY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->adaptiveTuningY = value;
        }
        
    };
    
    property float adaptiveTuningZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->adaptiveTuningZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->adaptiveTuningZ = value;
        }
        
    };
    
    property float adaptiveFilteringX
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->adaptiveFilteringX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->adaptiveFilteringX = value;
        }
        
    };
    
    property float adaptiveFilteringY
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->adaptiveFilteringY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->adaptiveFilteringY = value;
        }
        
    };
    
    property float adaptiveFilteringZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->adaptiveFilteringZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeMagBasicTuning *)_reg)->adaptiveFilteringZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 38 - VPE Accelerometer Basic Tuning </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class VpeAccelBasicTuning : public Register
{
    public:
    VpeAccelBasicTuning() : Register(new VN::Registers::Attitude::VpeAccelBasicTuning())
    {
    };
    
    property float baseTuningX
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->baseTuningX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->baseTuningX = value;
        }
        
    };
    
    property float baseTuningY
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->baseTuningY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->baseTuningY = value;
        }
        
    };
    
    property float baseTuningZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->baseTuningZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->baseTuningZ = value;
        }
        
    };
    
    property float adaptiveTuningX
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->adaptiveTuningX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->adaptiveTuningX = value;
        }
        
    };
    
    property float adaptiveTuningY
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->adaptiveTuningY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->adaptiveTuningY = value;
        }
        
    };
    
    property float adaptiveTuningZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->adaptiveTuningZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->adaptiveTuningZ = value;
        }
        
    };
    
    property float adaptiveFilteringX
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->adaptiveFilteringX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->adaptiveFilteringX = value;
        }
        
    };
    
    property float adaptiveFilteringY
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->adaptiveFilteringY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->adaptiveFilteringY = value;
        }
        
    };
    
    property float adaptiveFilteringZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->adaptiveFilteringZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::VpeAccelBasicTuning *)_reg)->adaptiveFilteringZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 239 - Yaw-Pitch-Roll, Linear Acceleration & Gyro </summary>
<remarks>

Yaw, Pitch, Roll, Linear Body Accel, and Angular Rates. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class YprLinearBodyAccelAngularRates : public Register
{
    public:
    YprLinearBodyAccelAngularRates() : Register(new VN::Registers::Attitude::YprLinearBodyAccelAngularRates())
    {
    };
    
    property float yaw
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->yaw;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->yaw = value;
        }
        
    };
    
    property float pitch
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->pitch;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->pitch = value;
        }
        
    };
    
    property float roll
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->roll;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->roll = value;
        }
        
    };
    
    property float linAccelX
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->linAccelX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->linAccelX = value;
        }
        
    };
    
    property float linAccelY
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->linAccelY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->linAccelY = value;
        }
        
    };
    
    property float linAccelZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->linAccelZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->linAccelZ = value;
        }
        
    };
    
    property float gyroX
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->gyroX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->gyroX = value;
        }
        
    };
    
    property float gyroY
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->gyroY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->gyroY = value;
        }
        
    };
    
    property float gyroZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->gyroZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearBodyAccelAngularRates *)_reg)->gyroZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 240 - Yaw-Pitch-Roll, Inertial Linear Acceleration & Gyro </summary>
<remarks>

Yaw, Pitch, Roll, Linear Inertial Accel, and Angular Rates. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class YprLinearInertialAccelAngularRates : public Register
{
    public:
    YprLinearInertialAccelAngularRates() : Register(new VN::Registers::Attitude::YprLinearInertialAccelAngularRates())
    {
    };
    
    property float yaw
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->yaw;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->yaw = value;
        }
        
    };
    
    property float pitch
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->pitch;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->pitch = value;
        }
        
    };
    
    property float roll
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->roll;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->roll = value;
        }
        
    };
    
    property float linAccelN
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->linAccelN;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->linAccelN = value;
        }
        
    };
    
    property float linAccelE
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->linAccelE;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->linAccelE = value;
        }
        
    };
    
    property float linAccelD
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->linAccelD;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->linAccelD = value;
        }
        
    };
    
    property float gyroX
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->gyroX;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->gyroX = value;
        }
        
    };
    
    property float gyroY
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->gyroY;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->gyroY = value;
        }
        
    };
    
    property float gyroZ
    {
        float get()
        {
            return ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->gyroZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::Attitude::YprLinearInertialAccelAngularRates *)_reg)->gyroZ = value;
        }
        
    };
    
    
};

} // namespace Attitude

namespace GNSS
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 55 - GNSS Basic Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class GnssBasicConfig : public Register
{
    public:
    GnssBasicConfig() : Register(new VN::Registers::GNSS::GnssBasicConfig())
    {
    };
    
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
    property ReceiverEnable receiverEnable
    {
        ReceiverEnable get()
        {
            return static_cast<GnssBasicConfig::ReceiverEnable>(((VN::Registers::GNSS::GnssBasicConfig *)_reg)->receiverEnable);
        }
        
        void set(ReceiverEnable value)
        {
            ((VN::Registers::GNSS::GnssBasicConfig *)_reg)->receiverEnable = static_cast<VN::Registers::GNSS::GnssBasicConfig::ReceiverEnable>(value);
        }
        
    };
    
    enum class PpsSource : uint8_t
    {
        PpsPinRising = 0,
        PpsPinFalling = 1,
        SyncInRising = 2,
        SyncInFalling = 3,
        None = 6,
    };
    property PpsSource ppsSource
    {
        PpsSource get()
        {
            return static_cast<GnssBasicConfig::PpsSource>(((VN::Registers::GNSS::GnssBasicConfig *)_reg)->ppsSource);
        }
        
        void set(PpsSource value)
        {
            ((VN::Registers::GNSS::GnssBasicConfig *)_reg)->ppsSource = static_cast<VN::Registers::GNSS::GnssBasicConfig::PpsSource>(value);
        }
        
    };
    
    enum class Rate : uint8_t
    {
        Rate1Hz = 1,
        Rate5Hz = 5,
    };
    property Rate rate
    {
        Rate get()
        {
            return static_cast<GnssBasicConfig::Rate>(((VN::Registers::GNSS::GnssBasicConfig *)_reg)->rate);
        }
        
        void set(Rate value)
        {
            ((VN::Registers::GNSS::GnssBasicConfig *)_reg)->rate = static_cast<VN::Registers::GNSS::GnssBasicConfig::Rate>(value);
        }
        
    };
    
    property uint8_t resv4
    {
        uint8_t get()
        {
            return ((VN::Registers::GNSS::GnssBasicConfig *)_reg)->resv4;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::GNSS::GnssBasicConfig *)_reg)->resv4 = value;
        }
        
    };
    
    enum class AntPower : uint8_t
    {
        Off = 0,
        Internal = 1,
        External = 2,
    };
    property AntPower antPower
    {
        AntPower get()
        {
            return static_cast<GnssBasicConfig::AntPower>(((VN::Registers::GNSS::GnssBasicConfig *)_reg)->antPower);
        }
        
        void set(AntPower value)
        {
            ((VN::Registers::GNSS::GnssBasicConfig *)_reg)->antPower = static_cast<VN::Registers::GNSS::GnssBasicConfig::AntPower>(value);
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 57 - GNSS Internal A Antenna Offset </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class GnssAOffset : public Register
{
    public:
    GnssAOffset() : Register(new VN::Registers::GNSS::GnssAOffset())
    {
    };
    
    property float positionX
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssAOffset *)_reg)->positionX;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssAOffset *)_reg)->positionX = value;
        }
        
    };
    
    property float positionY
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssAOffset *)_reg)->positionY;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssAOffset *)_reg)->positionY = value;
        }
        
    };
    
    property float positionZ
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssAOffset *)_reg)->positionZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssAOffset *)_reg)->positionZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 58 - GNSS Solution - LLA </summary>
<remarks>

Primary GNSS receiver measurement with lat/lon/alt position and velocity in NED frame. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class GnssSolLla : public Register
{
    public:
    GnssSolLla() : Register(new VN::Registers::GNSS::GnssSolLla())
    {
    };
    
    property double gps1Tow
    {
        double get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->gps1Tow;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->gps1Tow = value;
        }
        
    };
    
    property uint16_t gps1Week
    {
        uint16_t get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->gps1Week;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->gps1Week = value;
        }
        
    };
    
    enum class Gnss1Fix : uint8_t
    {
        NoFix = 0,
        TimeFix = 1,
        Fix2D = 2,
        Fix3D = 3,
        SBAS = 4,
        RtkFloat = 7,
        RtkFix = 8,
    };
    property Gnss1Fix gnss1Fix
    {
        Gnss1Fix get()
        {
            return static_cast<GnssSolLla::Gnss1Fix>(((VN::Registers::GNSS::GnssSolLla *)_reg)->gnss1Fix);
        }
        
        void set(Gnss1Fix value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->gnss1Fix = static_cast<VN::Registers::GNSS::GnssSolLla::Gnss1Fix>(value);
        }
        
    };
    
    property uint8_t gnss1NumSats
    {
        uint8_t get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->gnss1NumSats;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->gnss1NumSats = value;
        }
        
    };
    
    property double lat
    {
        double get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->lat;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->lat = value;
        }
        
    };
    
    property double lon
    {
        double get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->lon;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->lon = value;
        }
        
    };
    
    property double alt
    {
        double get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->alt;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->alt = value;
        }
        
    };
    
    property float velN
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->velN;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->velN = value;
        }
        
    };
    
    property float velE
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->velE;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->velE = value;
        }
        
    };
    
    property float velD
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->velD;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->velD = value;
        }
        
    };
    
    property float posUncertaintyN
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->posUncertaintyN;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->posUncertaintyN = value;
        }
        
    };
    
    property float posUncertaintyE
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->posUncertaintyE;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->posUncertaintyE = value;
        }
        
    };
    
    property float posUncertaintyD
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->posUncertaintyD;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->posUncertaintyD = value;
        }
        
    };
    
    property float gnss1VelUncertainty
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->gnss1VelUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->gnss1VelUncertainty = value;
        }
        
    };
    
    property float gnss1TimeUncertainty
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolLla *)_reg)->gnss1TimeUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolLla *)_reg)->gnss1TimeUncertainty = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 59 - GNSS Solution - ECEF </summary>
<remarks>

Primary GNSS receiver measurement in ECEF frame. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class GnssSolEcef : public Register
{
    public:
    GnssSolEcef() : Register(new VN::Registers::GNSS::GnssSolEcef())
    {
    };
    
    property double gps1Tow
    {
        double get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->gps1Tow;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->gps1Tow = value;
        }
        
    };
    
    property uint16_t gps1Week
    {
        uint16_t get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->gps1Week;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->gps1Week = value;
        }
        
    };
    
    enum class Gnss1Fix : uint8_t
    {
        NoFix = 0,
        TimeFix = 1,
        Fix2D = 2,
        Fix3D = 3,
        SBAS = 4,
        RtkFloat = 7,
        RtkFix = 8,
    };
    property Gnss1Fix gnss1Fix
    {
        Gnss1Fix get()
        {
            return static_cast<GnssSolEcef::Gnss1Fix>(((VN::Registers::GNSS::GnssSolEcef *)_reg)->gnss1Fix);
        }
        
        void set(Gnss1Fix value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->gnss1Fix = static_cast<VN::Registers::GNSS::GnssSolEcef::Gnss1Fix>(value);
        }
        
    };
    
    property uint8_t gnss1NumSats
    {
        uint8_t get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->gnss1NumSats;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->gnss1NumSats = value;
        }
        
    };
    
    property double posX
    {
        double get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->posX;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->posX = value;
        }
        
    };
    
    property double posY
    {
        double get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->posY;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->posY = value;
        }
        
    };
    
    property double posZ
    {
        double get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->posZ;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->posZ = value;
        }
        
    };
    
    property float velX
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->velX;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->velX = value;
        }
        
    };
    
    property float velY
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->velY;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->velY = value;
        }
        
    };
    
    property float velZ
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->velZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->velZ = value;
        }
        
    };
    
    property float posUncertaintyX
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->posUncertaintyX;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->posUncertaintyX = value;
        }
        
    };
    
    property float posUncertaintyY
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->posUncertaintyY;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->posUncertaintyY = value;
        }
        
    };
    
    property float posUncertaintyZ
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->posUncertaintyZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->posUncertaintyZ = value;
        }
        
    };
    
    property float gnss1VelUncertainty
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->gnss1VelUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->gnss1VelUncertainty = value;
        }
        
    };
    
    property float gnss1TimeUncertainty
    {
        float get()
        {
            return ((VN::Registers::GNSS::GnssSolEcef *)_reg)->gnss1TimeUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::GnssSolEcef *)_reg)->gnss1TimeUncertainty = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 99 - GNSS System Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class GnssSystemConfig : public Register
{
    public:
    GnssSystemConfig() : Register(new VN::Registers::GNSS::GnssSystemConfig())
    {
        systems = gcnew Systems(_reg);
        sbasMode = gcnew SbasMode(_reg);
        sbasSelect1 = gcnew SbasSelect1(_reg);
        sbasSelect2 = gcnew SbasSelect2(_reg);
        sbasSelect3 = gcnew SbasSelect3(_reg);
    };
    
    ref class Systems
    {
        VN::Register* _reg;
        
        public:
        Systems(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint16_t _value                ///< BitField value for systems
        {
            uint16_t get()
            {
                return uint16_t(((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems);
            }
            
            void set(uint16_t value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems = value;
            }
            
        };
        
        // GPS
        property bool gps
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.gps;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.gps = value;
            }
            
        };
        
        // SBAS
        property bool sbas
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.sbas;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.sbas = value;
            }
            
        };
        
        // GLONASS
        property bool glonass
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.glonass;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.glonass = value;
            }
            
        };
        
        // Beidou
        property bool beidou
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.beidou;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.beidou = value;
            }
            
        };
        
        // Galileo
        property bool galileo
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.galileo;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.galileo = value;
            }
            
        };
        
        // IMES
        property bool imes
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.imes;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.imes = value;
            }
            
        };
        
        // QZSS_L1_CA
        property bool qzssL1Ca
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.qzssL1Ca;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.qzssL1Ca = value;
            }
            
        };
        
        // QZSS_L1_SAIF
        property bool qzssL1Saif
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.qzssL1Saif;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->systems.qzssL1Saif = value;
            }
            
        };
        
    };
    
    Systems^ systems;
    
    property uint8_t minCno
    {
        uint8_t get()
        {
            return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->minCno;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->minCno = value;
        }
        
    };
    
    property uint8_t minElev
    {
        uint8_t get()
        {
            return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->minElev;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->minElev = value;
        }
        
    };
    
    property uint8_t maxSats
    {
        uint8_t get()
        {
            return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->maxSats;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->maxSats = value;
        }
        
    };
    
    ref class SbasMode
    {
        VN::Register* _reg;
        
        public:
        SbasMode(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint8_t _value                 ///< BitField value for sbasMode
        {
            uint8_t get()
            {
                return uint8_t(((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasMode);
            }
            
            void set(uint8_t value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasMode = value;
            }
            
        };
        
        // Ranging
        property bool ranging
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasMode.ranging;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasMode.ranging = value;
            }
            
        };
        
        // DiffCorr
        property bool diffCorr
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasMode.diffCorr;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasMode.diffCorr = value;
            }
            
        };
        
        // Integrity
        property bool integrity
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasMode.integrity;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasMode.integrity = value;
            }
            
        };
        
        // TestMode
        property bool testMode
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasMode.testMode;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasMode.testMode = value;
            }
            
        };
        
    };
    
    SbasMode^ sbasMode;
    
    ref class SbasSelect1
    {
        VN::Register* _reg;
        
        public:
        SbasSelect1(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint16_t _value                ///< BitField value for sbasSelect1
        {
            uint16_t get()
            {
                return uint16_t(((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1);
            }
            
            void set(uint16_t value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1 = value;
            }
            
        };
        
        // Sbas120
        property bool sbas120
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas120;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas120 = value;
            }
            
        };
        
        // Sbas121
        property bool sbas121
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas121;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas121 = value;
            }
            
        };
        
        // Sbas122
        property bool sbas122
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas122;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas122 = value;
            }
            
        };
        
        // Sbas123
        property bool sbas123
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas123;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas123 = value;
            }
            
        };
        
        // Sbas124
        property bool sbas124
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas124;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas124 = value;
            }
            
        };
        
        // Sbas125
        property bool sbas125
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas125;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas125 = value;
            }
            
        };
        
        // Sbas126
        property bool sbas126
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas126;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas126 = value;
            }
            
        };
        
        // Sbas127
        property bool sbas127
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas127;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas127 = value;
            }
            
        };
        
        // Sbas128
        property bool sbas128
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas128;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas128 = value;
            }
            
        };
        
        // Sbas129
        property bool sbas129
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas129;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas129 = value;
            }
            
        };
        
        // Sbas130
        property bool sbas130
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas130;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas130 = value;
            }
            
        };
        
        // Sbas131
        property bool sbas131
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas131;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas131 = value;
            }
            
        };
        
        // Sbas132
        property bool sbas132
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas132;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas132 = value;
            }
            
        };
        
        // Sbas133
        property bool sbas133
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas133;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas133 = value;
            }
            
        };
        
        // Sbas134
        property bool sbas134
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas134;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas134 = value;
            }
            
        };
        
        // Sbas135
        property bool sbas135
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas135;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect1.sbas135 = value;
            }
            
        };
        
    };
    
    SbasSelect1^ sbasSelect1;
    
    ref class SbasSelect2
    {
        VN::Register* _reg;
        
        public:
        SbasSelect2(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint16_t _value                ///< BitField value for sbasSelect2
        {
            uint16_t get()
            {
                return uint16_t(((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2);
            }
            
            void set(uint16_t value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2 = value;
            }
            
        };
        
        // Sbas136
        property bool sbas136
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas136;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas136 = value;
            }
            
        };
        
        // Sbas137
        property bool sbas137
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas137;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas137 = value;
            }
            
        };
        
        // Sbas138
        property bool sbas138
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas138;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas138 = value;
            }
            
        };
        
        // Sbas139
        property bool sbas139
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas139;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas139 = value;
            }
            
        };
        
        // Sbas140
        property bool sbas140
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas140;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas140 = value;
            }
            
        };
        
        // Sbas141
        property bool sbas141
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas141;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas141 = value;
            }
            
        };
        
        // Sbas142
        property bool sbas142
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas142;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas142 = value;
            }
            
        };
        
        // Sbas143
        property bool sbas143
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas143;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas143 = value;
            }
            
        };
        
        // Sbas144
        property bool sbas144
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas144;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas144 = value;
            }
            
        };
        
        // Sbas145
        property bool sbas145
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas145;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas145 = value;
            }
            
        };
        
        // Sbas146
        property bool sbas146
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas146;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas146 = value;
            }
            
        };
        
        // Sbas147
        property bool sbas147
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas147;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas147 = value;
            }
            
        };
        
        // Sbas148
        property bool sbas148
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas148;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas148 = value;
            }
            
        };
        
        // Sbas149
        property bool sbas149
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas149;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas149 = value;
            }
            
        };
        
        // Sbas150
        property bool sbas150
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas150;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas150 = value;
            }
            
        };
        
        // Sbas151
        property bool sbas151
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas151;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect2.sbas151 = value;
            }
            
        };
        
    };
    
    SbasSelect2^ sbasSelect2;
    
    ref class SbasSelect3
    {
        VN::Register* _reg;
        
        public:
        SbasSelect3(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint16_t _value                ///< BitField value for sbasSelect3
        {
            uint16_t get()
            {
                return uint16_t(((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3);
            }
            
            void set(uint16_t value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3 = value;
            }
            
        };
        
        // Sbas152
        property bool sbas152
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas152;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas152 = value;
            }
            
        };
        
        // Sbas153
        property bool sbas153
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas153;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas153 = value;
            }
            
        };
        
        // Sbas154
        property bool sbas154
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas154;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas154 = value;
            }
            
        };
        
        // Sbas155
        property bool sbas155
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas155;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas155 = value;
            }
            
        };
        
        // Sbas156
        property bool sbas156
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas156;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas156 = value;
            }
            
        };
        
        // Sbas157
        property bool sbas157
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas157;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas157 = value;
            }
            
        };
        
        // Sbas158
        property bool sbas158
        {
            bool get()
            {
                return ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas158;
            }
            
            void set(bool value)
            {
                ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->sbasSelect3.sbas158 = value;
            }
            
        };
        
    };
    
    SbasSelect3^ sbasSelect3;
    
    enum class ReceiverSelect : uint8_t
    {
        GnssAB = 0,
        GnssA = 1,
        GnssB = 2,
    };
    property ReceiverSelect receiverSelect
    {
        ReceiverSelect get()
        {
            return static_cast<GnssSystemConfig::ReceiverSelect>(((VN::Registers::GNSS::GnssSystemConfig *)_reg)->receiverSelect);
        }
        
        void set(ReceiverSelect value)
        {
            ((VN::Registers::GNSS::GnssSystemConfig *)_reg)->receiverSelect = static_cast<VN::Registers::GNSS::GnssSystemConfig::ReceiverSelect>(value);
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 100 - GNSS Sync Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class GnssSyncConfig : public Register
{
    public:
    GnssSyncConfig() : Register(new VN::Registers::GNSS::GnssSyncConfig())
    {
    };
    
    enum class GnssSyncEnable : uint8_t
    {
        Off = 0,
        AlwaysOn = 1,
        OnWhenLocked = 2,
    };
    property GnssSyncEnable gnssSyncEnable
    {
        GnssSyncEnable get()
        {
            return static_cast<GnssSyncConfig::GnssSyncEnable>(((VN::Registers::GNSS::GnssSyncConfig *)_reg)->gnssSyncEnable);
        }
        
        void set(GnssSyncEnable value)
        {
            ((VN::Registers::GNSS::GnssSyncConfig *)_reg)->gnssSyncEnable = static_cast<VN::Registers::GNSS::GnssSyncConfig::GnssSyncEnable>(value);
        }
        
    };
    
    enum class Polarity : uint8_t
    {
        FallingEdge = 0,
        RisingEdge = 1,
    };
    property Polarity polarity
    {
        Polarity get()
        {
            return static_cast<GnssSyncConfig::Polarity>(((VN::Registers::GNSS::GnssSyncConfig *)_reg)->polarity);
        }
        
        void set(Polarity value)
        {
            ((VN::Registers::GNSS::GnssSyncConfig *)_reg)->polarity = static_cast<VN::Registers::GNSS::GnssSyncConfig::Polarity>(value);
        }
        
    };
    
    enum class SpecType : uint8_t
    {
        PeriodPulseWidth = 0,
        FreqDutyCycle = 1,
    };
    property SpecType specType
    {
        SpecType get()
        {
            return static_cast<GnssSyncConfig::SpecType>(((VN::Registers::GNSS::GnssSyncConfig *)_reg)->specType);
        }
        
        void set(SpecType value)
        {
            ((VN::Registers::GNSS::GnssSyncConfig *)_reg)->specType = static_cast<VN::Registers::GNSS::GnssSyncConfig::SpecType>(value);
        }
        
    };
    
    property uint8_t resv
    {
        uint8_t get()
        {
            return ((VN::Registers::GNSS::GnssSyncConfig *)_reg)->resv;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::GNSS::GnssSyncConfig *)_reg)->resv = value;
        }
        
    };
    
    property uint32_t period
    {
        uint32_t get()
        {
            return ((VN::Registers::GNSS::GnssSyncConfig *)_reg)->period;
        }
        
        void set(uint32_t value)
        {
            ((VN::Registers::GNSS::GnssSyncConfig *)_reg)->period = value;
        }
        
    };
    
    property uint32_t pulseWidth
    {
        uint32_t get()
        {
            return ((VN::Registers::GNSS::GnssSyncConfig *)_reg)->pulseWidth;
        }
        
        void set(uint32_t value)
        {
            ((VN::Registers::GNSS::GnssSyncConfig *)_reg)->pulseWidth = value;
        }
        
    };
    
    property int32_t offset
    {
        int32_t get()
        {
            return ((VN::Registers::GNSS::GnssSyncConfig *)_reg)->offset;
        }
        
        void set(int32_t value)
        {
            ((VN::Registers::GNSS::GnssSyncConfig *)_reg)->offset = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 103 - GNSS 2 Solution - LLA </summary>
<remarks>

Estimated GNSS 2 Solution with lat/lon/alt position. This register is deprecated and will be 
removed in future firmware versions. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class Gnss2SolLla : public Register
{
    public:
    Gnss2SolLla() : Register(new VN::Registers::GNSS::Gnss2SolLla())
    {
    };
    
    property double gps2Tow
    {
        double get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->gps2Tow;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->gps2Tow = value;
        }
        
    };
    
    property uint16_t gps2Week
    {
        uint16_t get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->gps2Week;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->gps2Week = value;
        }
        
    };
    
    enum class Gnss2Fix : uint8_t
    {
        NoFix = 0,
        TimeFix = 1,
        Fix2D = 2,
        Fix3D = 3,
        SBAS = 4,
        RtkFloat = 7,
        RtkFix = 8,
    };
    property Gnss2Fix gnss2Fix
    {
        Gnss2Fix get()
        {
            return static_cast<Gnss2SolLla::Gnss2Fix>(((VN::Registers::GNSS::Gnss2SolLla *)_reg)->gnss2Fix);
        }
        
        void set(Gnss2Fix value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->gnss2Fix = static_cast<VN::Registers::GNSS::Gnss2SolLla::Gnss2Fix>(value);
        }
        
    };
    
    property uint8_t gnss2NumSats
    {
        uint8_t get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->gnss2NumSats;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->gnss2NumSats = value;
        }
        
    };
    
    property double lat
    {
        double get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->lat;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->lat = value;
        }
        
    };
    
    property double lon
    {
        double get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->lon;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->lon = value;
        }
        
    };
    
    property double alt
    {
        double get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->alt;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->alt = value;
        }
        
    };
    
    property float velN
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->velN;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->velN = value;
        }
        
    };
    
    property float velE
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->velE;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->velE = value;
        }
        
    };
    
    property float velD
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->velD;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->velD = value;
        }
        
    };
    
    property float posUncertaintyN
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->posUncertaintyN;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->posUncertaintyN = value;
        }
        
    };
    
    property float posUncertaintyE
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->posUncertaintyE;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->posUncertaintyE = value;
        }
        
    };
    
    property float posUncertaintyD
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->posUncertaintyD;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->posUncertaintyD = value;
        }
        
    };
    
    property float gnss2VelUncertainty
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->gnss2VelUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->gnss2VelUncertainty = value;
        }
        
    };
    
    property float gnss2TimeUncertainty
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->gnss2TimeUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolLla *)_reg)->gnss2TimeUncertainty = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 104 - GNSS 2 Solution - ECEF </summary>
<remarks>

Estimated GNSS 2 Solution with ECEF position. This register is deprecated and will be removed in 
future firmware versions. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class Gnss2SolEcef : public Register
{
    public:
    Gnss2SolEcef() : Register(new VN::Registers::GNSS::Gnss2SolEcef())
    {
    };
    
    property double gps2Tow
    {
        double get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->gps2Tow;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->gps2Tow = value;
        }
        
    };
    
    property uint16_t gps2Week
    {
        uint16_t get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->gps2Week;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->gps2Week = value;
        }
        
    };
    
    enum class Gnss2Fix : uint8_t
    {
        NoFix = 0,
        TimeFix = 1,
        Fix2D = 2,
        Fix3D = 3,
        SBAS = 4,
        RtkFloat = 7,
        RtkFix = 8,
    };
    property Gnss2Fix gnss2Fix
    {
        Gnss2Fix get()
        {
            return static_cast<Gnss2SolEcef::Gnss2Fix>(((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->gnss2Fix);
        }
        
        void set(Gnss2Fix value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->gnss2Fix = static_cast<VN::Registers::GNSS::Gnss2SolEcef::Gnss2Fix>(value);
        }
        
    };
    
    property uint8_t gnss2NumSats
    {
        uint8_t get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->gnss2NumSats;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->gnss2NumSats = value;
        }
        
    };
    
    property double posX
    {
        double get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->posX;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->posX = value;
        }
        
    };
    
    property double posY
    {
        double get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->posY;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->posY = value;
        }
        
    };
    
    property double posZ
    {
        double get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->posZ;
        }
        
        void set(double value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->posZ = value;
        }
        
    };
    
    property float velX
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->velX;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->velX = value;
        }
        
    };
    
    property float velY
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->velY;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->velY = value;
        }
        
    };
    
    property float velZ
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->velZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->velZ = value;
        }
        
    };
    
    property float posUncertaintyX
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->posUncertaintyX;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->posUncertaintyX = value;
        }
        
    };
    
    property float posUncertaintyY
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->posUncertaintyY;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->posUncertaintyY = value;
        }
        
    };
    
    property float posUncertaintyZ
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->posUncertaintyZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->posUncertaintyZ = value;
        }
        
    };
    
    property float gnss2VelUncertainty
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->gnss2VelUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->gnss2VelUncertainty = value;
        }
        
    };
    
    property float gnss2TimeUncertainty
    {
        float get()
        {
            return ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->gnss2TimeUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::Gnss2SolEcef *)_reg)->gnss2TimeUncertainty = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 157 - External GNSS Antenna Offset </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class ExtGnssOffset : public Register
{
    public:
    ExtGnssOffset() : Register(new VN::Registers::GNSS::ExtGnssOffset())
    {
    };
    
    property float positionX
    {
        float get()
        {
            return ((VN::Registers::GNSS::ExtGnssOffset *)_reg)->positionX;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::ExtGnssOffset *)_reg)->positionX = value;
        }
        
    };
    
    property float positionY
    {
        float get()
        {
            return ((VN::Registers::GNSS::ExtGnssOffset *)_reg)->positionY;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::ExtGnssOffset *)_reg)->positionY = value;
        }
        
    };
    
    property float positionZ
    {
        float get()
        {
            return ((VN::Registers::GNSS::ExtGnssOffset *)_reg)->positionZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSS::ExtGnssOffset *)_reg)->positionZ = value;
        }
        
    };
    
    
};

} // namespace GNSS

namespace GNSSCompass
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 86 - GNSS Compass Signal Health Status </summary>
<remarks>

Provides several indicators that serve as an overall health status of the GNSS compass subsystem. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class GnssCompassSignalHealthStatus : public Register
{
    public:
    GnssCompassSignalHealthStatus() : Register(new VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus())
    {
    };
    
    property float numSatsPvtA
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->numSatsPvtA;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->numSatsPvtA = value;
        }
        
    };
    
    property float numSatsRtkA
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->numSatsRtkA;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->numSatsRtkA = value;
        }
        
    };
    
    property float highestCn0A
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->highestCn0A;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->highestCn0A = value;
        }
        
    };
    
    property float numSatsPvtB
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->numSatsPvtB;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->numSatsPvtB = value;
        }
        
    };
    
    property float numSatsRtkB
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->numSatsRtkB;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->numSatsRtkB = value;
        }
        
    };
    
    property float highestCn0B
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->highestCn0B;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->highestCn0B = value;
        }
        
    };
    
    property float numComSatsPvt
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->numComSatsPvt;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->numComSatsPvt = value;
        }
        
    };
    
    property float numComSatsRtk
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->numComSatsRtk;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus *)_reg)->numComSatsRtk = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 93 - GNSS Compass Antenna Baseline </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class GnssCompassBaseline : public Register
{
    public:
    GnssCompassBaseline() : Register(new VN::Registers::GNSSCompass::GnssCompassBaseline())
    {
    };
    
    property float positionX
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassBaseline *)_reg)->positionX;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassBaseline *)_reg)->positionX = value;
        }
        
    };
    
    property float positionY
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassBaseline *)_reg)->positionY;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassBaseline *)_reg)->positionY = value;
        }
        
    };
    
    property float positionZ
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassBaseline *)_reg)->positionZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassBaseline *)_reg)->positionZ = value;
        }
        
    };
    
    property float uncertaintyX
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassBaseline *)_reg)->uncertaintyX;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassBaseline *)_reg)->uncertaintyX = value;
        }
        
    };
    
    property float uncertaintyY
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassBaseline *)_reg)->uncertaintyY;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassBaseline *)_reg)->uncertaintyY = value;
        }
        
    };
    
    property float uncertaintyZ
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassBaseline *)_reg)->uncertaintyZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassBaseline *)_reg)->uncertaintyZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 97 - GNSS Compass Estimated Baseline </summary>
<remarks>

Provides the estimated GNSS compass baseline measurement. The estimated position offset and 
measurement uncertainty is for antenna B relative to antenna A in the body reference frame. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class GnssCompassEstBaseline : public Register
{
    public:
    GnssCompassEstBaseline() : Register(new VN::Registers::GNSSCompass::GnssCompassEstBaseline())
    {
    };
    
    property uint8_t estBaselineComplete
    {
        uint8_t get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->estBaselineComplete;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->estBaselineComplete = value;
        }
        
    };
    
    property uint8_t resv
    {
        uint8_t get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->resv;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->resv = value;
        }
        
    };
    
    property uint16_t numMeas
    {
        uint16_t get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->numMeas;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->numMeas = value;
        }
        
    };
    
    property float positionX
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->positionX;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->positionX = value;
        }
        
    };
    
    property float positionY
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->positionY;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->positionY = value;
        }
        
    };
    
    property float positionZ
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->positionZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->positionZ = value;
        }
        
    };
    
    property float uncertaintyX
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->uncertaintyX;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->uncertaintyX = value;
        }
        
    };
    
    property float uncertaintyY
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->uncertaintyY;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->uncertaintyY = value;
        }
        
    };
    
    property float uncertaintyZ
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->uncertaintyZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassEstBaseline *)_reg)->uncertaintyZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 98 - GNSS Compass Startup Status </summary>
<remarks>

Provides status information on the GNSS compass startup process. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class GnssCompassStartupStatus : public Register
{
    public:
    GnssCompassStartupStatus() : Register(new VN::Registers::GNSSCompass::GnssCompassStartupStatus())
    {
    };
    
    property uint8_t percentComplete
    {
        uint8_t get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassStartupStatus *)_reg)->percentComplete;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassStartupStatus *)_reg)->percentComplete = value;
        }
        
    };
    
    property float currentHeading
    {
        float get()
        {
            return ((VN::Registers::GNSSCompass::GnssCompassStartupStatus *)_reg)->currentHeading;
        }
        
        void set(float value)
        {
            ((VN::Registers::GNSSCompass::GnssCompassStartupStatus *)_reg)->currentHeading = value;
        }
        
    };
    
    
};

} // namespace GNSSCompass

namespace HardSoftIronEstimator
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 44 - Real-Time HSI Control </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class RealTimeHsiControl : public Register
{
    public:
    RealTimeHsiControl() : Register(new VN::Registers::HardSoftIronEstimator::RealTimeHsiControl())
    {
    };
    
    enum class Mode : uint8_t
    {
        Off = 0,
        Run = 1,
        Reset = 2,
    };
    property Mode mode
    {
        Mode get()
        {
            return static_cast<RealTimeHsiControl::Mode>(((VN::Registers::HardSoftIronEstimator::RealTimeHsiControl *)_reg)->mode);
        }
        
        void set(Mode value)
        {
            ((VN::Registers::HardSoftIronEstimator::RealTimeHsiControl *)_reg)->mode = static_cast<VN::Registers::HardSoftIronEstimator::RealTimeHsiControl::Mode>(value);
        }
        
    };
    
    enum class ApplyCompensation : uint8_t
    {
        Disable = 1,
        Enable = 3,
    };
    property ApplyCompensation applyCompensation
    {
        ApplyCompensation get()
        {
            return static_cast<RealTimeHsiControl::ApplyCompensation>(((VN::Registers::HardSoftIronEstimator::RealTimeHsiControl *)_reg)->applyCompensation);
        }
        
        void set(ApplyCompensation value)
        {
            ((VN::Registers::HardSoftIronEstimator::RealTimeHsiControl *)_reg)->applyCompensation = static_cast<VN::Registers::HardSoftIronEstimator::RealTimeHsiControl::ApplyCompensation>(value);
        }
        
    };
    
    property uint8_t convergeRate
    {
        uint8_t get()
        {
            return ((VN::Registers::HardSoftIronEstimator::RealTimeHsiControl *)_reg)->convergeRate;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::HardSoftIronEstimator::RealTimeHsiControl *)_reg)->convergeRate = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 47 - Real-Time HSI Results </summary>
<remarks>

Magnetometer calibration values calculated by the real-time HSI calibration filter. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class EstMagCal : public Register
{
    public:
    EstMagCal() : Register(new VN::Registers::HardSoftIronEstimator::EstMagCal())
    {
    };
    
    property float magGain00
    {
        float get()
        {
            return ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain00;
        }
        
        void set(float value)
        {
            ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain00 = value;
        }
        
    };
    
    property float magGain01
    {
        float get()
        {
            return ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain01;
        }
        
        void set(float value)
        {
            ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain01 = value;
        }
        
    };
    
    property float magGain02
    {
        float get()
        {
            return ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain02;
        }
        
        void set(float value)
        {
            ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain02 = value;
        }
        
    };
    
    property float magGain10
    {
        float get()
        {
            return ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain10;
        }
        
        void set(float value)
        {
            ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain10 = value;
        }
        
    };
    
    property float magGain11
    {
        float get()
        {
            return ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain11;
        }
        
        void set(float value)
        {
            ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain11 = value;
        }
        
    };
    
    property float magGain12
    {
        float get()
        {
            return ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain12;
        }
        
        void set(float value)
        {
            ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain12 = value;
        }
        
    };
    
    property float magGain20
    {
        float get()
        {
            return ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain20;
        }
        
        void set(float value)
        {
            ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain20 = value;
        }
        
    };
    
    property float magGain21
    {
        float get()
        {
            return ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain21;
        }
        
        void set(float value)
        {
            ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain21 = value;
        }
        
    };
    
    property float magGain22
    {
        float get()
        {
            return ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain22;
        }
        
        void set(float value)
        {
            ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magGain22 = value;
        }
        
    };
    
    property float magBiasX
    {
        float get()
        {
            return ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magBiasX;
        }
        
        void set(float value)
        {
            ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magBiasX = value;
        }
        
    };
    
    property float magBiasY
    {
        float get()
        {
            return ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magBiasY;
        }
        
        void set(float value)
        {
            ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magBiasY = value;
        }
        
    };
    
    property float magBiasZ
    {
        float get()
        {
            return ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magBiasZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::HardSoftIronEstimator::EstMagCal *)_reg)->magBiasZ = value;
        }
        
    };
    
    
};

} // namespace HardSoftIronEstimator

namespace Heave
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 115 - Heave and Heave Rate </summary>
<remarks>

Real-time heave and heave-rate estimates, plus a delayed-heave estimate. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class HeaveOutputs : public Register
{
    public:
    HeaveOutputs() : Register(new VN::Registers::Heave::HeaveOutputs())
    {
    };
    
    property float heave
    {
        float get()
        {
            return ((VN::Registers::Heave::HeaveOutputs *)_reg)->heave;
        }
        
        void set(float value)
        {
            ((VN::Registers::Heave::HeaveOutputs *)_reg)->heave = value;
        }
        
    };
    
    property float heaveRate
    {
        float get()
        {
            return ((VN::Registers::Heave::HeaveOutputs *)_reg)->heaveRate;
        }
        
        void set(float value)
        {
            ((VN::Registers::Heave::HeaveOutputs *)_reg)->heaveRate = value;
        }
        
    };
    
    property float delayedHeave
    {
        float get()
        {
            return ((VN::Registers::Heave::HeaveOutputs *)_reg)->delayedHeave;
        }
        
        void set(float value)
        {
            ((VN::Registers::Heave::HeaveOutputs *)_reg)->delayedHeave = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 116 - Heave Basic Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class HeaveBasicConfig : public Register
{
    public:
    HeaveBasicConfig() : Register(new VN::Registers::Heave::HeaveBasicConfig())
    {
    };
    
    property float initialWavePeriod
    {
        float get()
        {
            return ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->initialWavePeriod;
        }
        
        void set(float value)
        {
            ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->initialWavePeriod = value;
        }
        
    };
    
    property float initialWaveAmplitude
    {
        float get()
        {
            return ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->initialWaveAmplitude;
        }
        
        void set(float value)
        {
            ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->initialWaveAmplitude = value;
        }
        
    };
    
    property float maxWavePeriod
    {
        float get()
        {
            return ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->maxWavePeriod;
        }
        
        void set(float value)
        {
            ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->maxWavePeriod = value;
        }
        
    };
    
    property float minWaveAmplitude
    {
        float get()
        {
            return ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->minWaveAmplitude;
        }
        
        void set(float value)
        {
            ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->minWaveAmplitude = value;
        }
        
    };
    
    property float delayedHeaveCutoffFreq
    {
        float get()
        {
            return ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->delayedHeaveCutoffFreq;
        }
        
        void set(float value)
        {
            ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->delayedHeaveCutoffFreq = value;
        }
        
    };
    
    property float heaveCutoffFreq
    {
        float get()
        {
            return ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->heaveCutoffFreq;
        }
        
        void set(float value)
        {
            ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->heaveCutoffFreq = value;
        }
        
    };
    
    property float heaveRateCutoffFreq
    {
        float get()
        {
            return ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->heaveRateCutoffFreq;
        }
        
        void set(float value)
        {
            ((VN::Registers::Heave::HeaveBasicConfig *)_reg)->heaveRateCutoffFreq = value;
        }
        
    };
    
    
};

} // namespace Heave

namespace IMU
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 17 - Compensated Magnetometer </summary>
<remarks>

Compensated magnetometer measurements. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class Mag : public Register
{
    public:
    Mag() : Register(new VN::Registers::IMU::Mag())
    {
    };
    
    property float magX
    {
        float get()
        {
            return ((VN::Registers::IMU::Mag *)_reg)->magX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::Mag *)_reg)->magX = value;
        }
        
    };
    
    property float magY
    {
        float get()
        {
            return ((VN::Registers::IMU::Mag *)_reg)->magY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::Mag *)_reg)->magY = value;
        }
        
    };
    
    property float magZ
    {
        float get()
        {
            return ((VN::Registers::IMU::Mag *)_reg)->magZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::Mag *)_reg)->magZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 18 - Compensated Accelerometer </summary>
<remarks>

Compensated acceleration measurements 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class Accel : public Register
{
    public:
    Accel() : Register(new VN::Registers::IMU::Accel())
    {
    };
    
    property float accelX
    {
        float get()
        {
            return ((VN::Registers::IMU::Accel *)_reg)->accelX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::Accel *)_reg)->accelX = value;
        }
        
    };
    
    property float accelY
    {
        float get()
        {
            return ((VN::Registers::IMU::Accel *)_reg)->accelY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::Accel *)_reg)->accelY = value;
        }
        
    };
    
    property float accelZ
    {
        float get()
        {
            return ((VN::Registers::IMU::Accel *)_reg)->accelZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::Accel *)_reg)->accelZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 19 - Compensated Gyro </summary>
<remarks>

Compensated angular rate measurements. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class Gyro : public Register
{
    public:
    Gyro() : Register(new VN::Registers::IMU::Gyro())
    {
    };
    
    property float gyroX
    {
        float get()
        {
            return ((VN::Registers::IMU::Gyro *)_reg)->gyroX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::Gyro *)_reg)->gyroX = value;
        }
        
    };
    
    property float gyroY
    {
        float get()
        {
            return ((VN::Registers::IMU::Gyro *)_reg)->gyroY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::Gyro *)_reg)->gyroY = value;
        }
        
    };
    
    property float gyroZ
    {
        float get()
        {
            return ((VN::Registers::IMU::Gyro *)_reg)->gyroZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::Gyro *)_reg)->gyroZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 20 - Compensated IMU </summary>
<remarks>

Compensated magnetic, acceleration, and angular rate measurements. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class MagAccelGyro : public Register
{
    public:
    MagAccelGyro() : Register(new VN::Registers::IMU::MagAccelGyro())
    {
    };
    
    property float magX
    {
        float get()
        {
            return ((VN::Registers::IMU::MagAccelGyro *)_reg)->magX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagAccelGyro *)_reg)->magX = value;
        }
        
    };
    
    property float magY
    {
        float get()
        {
            return ((VN::Registers::IMU::MagAccelGyro *)_reg)->magY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagAccelGyro *)_reg)->magY = value;
        }
        
    };
    
    property float magZ
    {
        float get()
        {
            return ((VN::Registers::IMU::MagAccelGyro *)_reg)->magZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagAccelGyro *)_reg)->magZ = value;
        }
        
    };
    
    property float accelX
    {
        float get()
        {
            return ((VN::Registers::IMU::MagAccelGyro *)_reg)->accelX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagAccelGyro *)_reg)->accelX = value;
        }
        
    };
    
    property float accelY
    {
        float get()
        {
            return ((VN::Registers::IMU::MagAccelGyro *)_reg)->accelY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagAccelGyro *)_reg)->accelY = value;
        }
        
    };
    
    property float accelZ
    {
        float get()
        {
            return ((VN::Registers::IMU::MagAccelGyro *)_reg)->accelZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagAccelGyro *)_reg)->accelZ = value;
        }
        
    };
    
    property float gyroX
    {
        float get()
        {
            return ((VN::Registers::IMU::MagAccelGyro *)_reg)->gyroX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagAccelGyro *)_reg)->gyroX = value;
        }
        
    };
    
    property float gyroY
    {
        float get()
        {
            return ((VN::Registers::IMU::MagAccelGyro *)_reg)->gyroY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagAccelGyro *)_reg)->gyroY = value;
        }
        
    };
    
    property float gyroZ
    {
        float get()
        {
            return ((VN::Registers::IMU::MagAccelGyro *)_reg)->gyroZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagAccelGyro *)_reg)->gyroZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 23 - Magnetometer Calibration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class MagCal : public Register
{
    public:
    MagCal() : Register(new VN::Registers::IMU::MagCal())
    {
    };
    
    property float magGain00
    {
        float get()
        {
            return ((VN::Registers::IMU::MagCal *)_reg)->magGain00;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagCal *)_reg)->magGain00 = value;
        }
        
    };
    
    property float magGain01
    {
        float get()
        {
            return ((VN::Registers::IMU::MagCal *)_reg)->magGain01;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagCal *)_reg)->magGain01 = value;
        }
        
    };
    
    property float magGain02
    {
        float get()
        {
            return ((VN::Registers::IMU::MagCal *)_reg)->magGain02;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagCal *)_reg)->magGain02 = value;
        }
        
    };
    
    property float magGain10
    {
        float get()
        {
            return ((VN::Registers::IMU::MagCal *)_reg)->magGain10;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagCal *)_reg)->magGain10 = value;
        }
        
    };
    
    property float magGain11
    {
        float get()
        {
            return ((VN::Registers::IMU::MagCal *)_reg)->magGain11;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagCal *)_reg)->magGain11 = value;
        }
        
    };
    
    property float magGain12
    {
        float get()
        {
            return ((VN::Registers::IMU::MagCal *)_reg)->magGain12;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagCal *)_reg)->magGain12 = value;
        }
        
    };
    
    property float magGain20
    {
        float get()
        {
            return ((VN::Registers::IMU::MagCal *)_reg)->magGain20;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagCal *)_reg)->magGain20 = value;
        }
        
    };
    
    property float magGain21
    {
        float get()
        {
            return ((VN::Registers::IMU::MagCal *)_reg)->magGain21;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagCal *)_reg)->magGain21 = value;
        }
        
    };
    
    property float magGain22
    {
        float get()
        {
            return ((VN::Registers::IMU::MagCal *)_reg)->magGain22;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagCal *)_reg)->magGain22 = value;
        }
        
    };
    
    property float magBiasX
    {
        float get()
        {
            return ((VN::Registers::IMU::MagCal *)_reg)->magBiasX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagCal *)_reg)->magBiasX = value;
        }
        
    };
    
    property float magBiasY
    {
        float get()
        {
            return ((VN::Registers::IMU::MagCal *)_reg)->magBiasY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagCal *)_reg)->magBiasY = value;
        }
        
    };
    
    property float magBiasZ
    {
        float get()
        {
            return ((VN::Registers::IMU::MagCal *)_reg)->magBiasZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::MagCal *)_reg)->magBiasZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 25 - Accel Calibration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class AccelCal : public Register
{
    public:
    AccelCal() : Register(new VN::Registers::IMU::AccelCal())
    {
    };
    
    property float accelGain00
    {
        float get()
        {
            return ((VN::Registers::IMU::AccelCal *)_reg)->accelGain00;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::AccelCal *)_reg)->accelGain00 = value;
        }
        
    };
    
    property float accelGain01
    {
        float get()
        {
            return ((VN::Registers::IMU::AccelCal *)_reg)->accelGain01;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::AccelCal *)_reg)->accelGain01 = value;
        }
        
    };
    
    property float accelGain02
    {
        float get()
        {
            return ((VN::Registers::IMU::AccelCal *)_reg)->accelGain02;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::AccelCal *)_reg)->accelGain02 = value;
        }
        
    };
    
    property float accelGain10
    {
        float get()
        {
            return ((VN::Registers::IMU::AccelCal *)_reg)->accelGain10;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::AccelCal *)_reg)->accelGain10 = value;
        }
        
    };
    
    property float accelGain11
    {
        float get()
        {
            return ((VN::Registers::IMU::AccelCal *)_reg)->accelGain11;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::AccelCal *)_reg)->accelGain11 = value;
        }
        
    };
    
    property float accelGain12
    {
        float get()
        {
            return ((VN::Registers::IMU::AccelCal *)_reg)->accelGain12;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::AccelCal *)_reg)->accelGain12 = value;
        }
        
    };
    
    property float accelGain20
    {
        float get()
        {
            return ((VN::Registers::IMU::AccelCal *)_reg)->accelGain20;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::AccelCal *)_reg)->accelGain20 = value;
        }
        
    };
    
    property float accelGain21
    {
        float get()
        {
            return ((VN::Registers::IMU::AccelCal *)_reg)->accelGain21;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::AccelCal *)_reg)->accelGain21 = value;
        }
        
    };
    
    property float accelGain22
    {
        float get()
        {
            return ((VN::Registers::IMU::AccelCal *)_reg)->accelGain22;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::AccelCal *)_reg)->accelGain22 = value;
        }
        
    };
    
    property float accelBiasX
    {
        float get()
        {
            return ((VN::Registers::IMU::AccelCal *)_reg)->accelBiasX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::AccelCal *)_reg)->accelBiasX = value;
        }
        
    };
    
    property float accelBiasY
    {
        float get()
        {
            return ((VN::Registers::IMU::AccelCal *)_reg)->accelBiasY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::AccelCal *)_reg)->accelBiasY = value;
        }
        
    };
    
    property float accelBiasZ
    {
        float get()
        {
            return ((VN::Registers::IMU::AccelCal *)_reg)->accelBiasZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::AccelCal *)_reg)->accelBiasZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 26 - Reference Frame Rotation </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class RefFrameRot : public Register
{
    public:
    RefFrameRot() : Register(new VN::Registers::IMU::RefFrameRot())
    {
    };
    
    property float rfr00
    {
        float get()
        {
            return ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr00;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr00 = value;
        }
        
    };
    
    property float rfr01
    {
        float get()
        {
            return ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr01;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr01 = value;
        }
        
    };
    
    property float rfr02
    {
        float get()
        {
            return ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr02;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr02 = value;
        }
        
    };
    
    property float rfr10
    {
        float get()
        {
            return ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr10;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr10 = value;
        }
        
    };
    
    property float rfr11
    {
        float get()
        {
            return ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr11;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr11 = value;
        }
        
    };
    
    property float rfr12
    {
        float get()
        {
            return ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr12;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr12 = value;
        }
        
    };
    
    property float rfr20
    {
        float get()
        {
            return ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr20;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr20 = value;
        }
        
    };
    
    property float rfr21
    {
        float get()
        {
            return ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr21;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr21 = value;
        }
        
    };
    
    property float rfr22
    {
        float get()
        {
            return ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr22;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::RefFrameRot *)_reg)->rfr22 = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 54 - IMU Measurements </summary>
<remarks>

Provides the calibrated IMU measurements including barometric pressure. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class ImuMeas : public Register
{
    public:
    ImuMeas() : Register(new VN::Registers::IMU::ImuMeas())
    {
    };
    
    property float uncompMagX
    {
        float get()
        {
            return ((VN::Registers::IMU::ImuMeas *)_reg)->uncompMagX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::ImuMeas *)_reg)->uncompMagX = value;
        }
        
    };
    
    property float uncompMagY
    {
        float get()
        {
            return ((VN::Registers::IMU::ImuMeas *)_reg)->uncompMagY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::ImuMeas *)_reg)->uncompMagY = value;
        }
        
    };
    
    property float uncompMagZ
    {
        float get()
        {
            return ((VN::Registers::IMU::ImuMeas *)_reg)->uncompMagZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::ImuMeas *)_reg)->uncompMagZ = value;
        }
        
    };
    
    property float uncompAccX
    {
        float get()
        {
            return ((VN::Registers::IMU::ImuMeas *)_reg)->uncompAccX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::ImuMeas *)_reg)->uncompAccX = value;
        }
        
    };
    
    property float uncompAccY
    {
        float get()
        {
            return ((VN::Registers::IMU::ImuMeas *)_reg)->uncompAccY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::ImuMeas *)_reg)->uncompAccY = value;
        }
        
    };
    
    property float uncompAccZ
    {
        float get()
        {
            return ((VN::Registers::IMU::ImuMeas *)_reg)->uncompAccZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::ImuMeas *)_reg)->uncompAccZ = value;
        }
        
    };
    
    property float uncompGyroX
    {
        float get()
        {
            return ((VN::Registers::IMU::ImuMeas *)_reg)->uncompGyroX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::ImuMeas *)_reg)->uncompGyroX = value;
        }
        
    };
    
    property float uncompGyroY
    {
        float get()
        {
            return ((VN::Registers::IMU::ImuMeas *)_reg)->uncompGyroY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::ImuMeas *)_reg)->uncompGyroY = value;
        }
        
    };
    
    property float uncompGyroZ
    {
        float get()
        {
            return ((VN::Registers::IMU::ImuMeas *)_reg)->uncompGyroZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::ImuMeas *)_reg)->uncompGyroZ = value;
        }
        
    };
    
    property float temperature
    {
        float get()
        {
            return ((VN::Registers::IMU::ImuMeas *)_reg)->temperature;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::ImuMeas *)_reg)->temperature = value;
        }
        
    };
    
    property float pressure
    {
        float get()
        {
            return ((VN::Registers::IMU::ImuMeas *)_reg)->pressure;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::ImuMeas *)_reg)->pressure = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 80 - Delta Theta and Delta Velocity </summary>
<remarks>

This register contains the output values of the onboard coning and sculling algorithm. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class DeltaThetaVelocity : public Register
{
    public:
    DeltaThetaVelocity() : Register(new VN::Registers::IMU::DeltaThetaVelocity())
    {
    };
    
    property float deltaTime
    {
        float get()
        {
            return ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaTime;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaTime = value;
        }
        
    };
    
    property float deltaThetaX
    {
        float get()
        {
            return ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaThetaX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaThetaX = value;
        }
        
    };
    
    property float deltaThetaY
    {
        float get()
        {
            return ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaThetaY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaThetaY = value;
        }
        
    };
    
    property float deltaThetaZ
    {
        float get()
        {
            return ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaThetaZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaThetaZ = value;
        }
        
    };
    
    property float deltaVelX
    {
        float get()
        {
            return ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaVelX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaVelX = value;
        }
        
    };
    
    property float deltaVelY
    {
        float get()
        {
            return ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaVelY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaVelY = value;
        }
        
    };
    
    property float deltaVelZ
    {
        float get()
        {
            return ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaVelZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::DeltaThetaVelocity *)_reg)->deltaVelZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 82 - Delta Theta and Delta Velocity Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class DeltaThetaVelConfig : public Register
{
    public:
    DeltaThetaVelConfig() : Register(new VN::Registers::IMU::DeltaThetaVelConfig())
    {
    };
    
    enum class IntegrationFrame : uint8_t
    {
        Body = 0,
        NED = 1,
    };
    property IntegrationFrame integrationFrame
    {
        IntegrationFrame get()
        {
            return static_cast<DeltaThetaVelConfig::IntegrationFrame>(((VN::Registers::IMU::DeltaThetaVelConfig *)_reg)->integrationFrame);
        }
        
        void set(IntegrationFrame value)
        {
            ((VN::Registers::IMU::DeltaThetaVelConfig *)_reg)->integrationFrame = static_cast<VN::Registers::IMU::DeltaThetaVelConfig::IntegrationFrame>(value);
        }
        
    };
    
    enum class GyroCompensation : uint8_t
    {
        None = 0,
        Bias = 1,
    };
    property GyroCompensation gyroCompensation
    {
        GyroCompensation get()
        {
            return static_cast<DeltaThetaVelConfig::GyroCompensation>(((VN::Registers::IMU::DeltaThetaVelConfig *)_reg)->gyroCompensation);
        }
        
        void set(GyroCompensation value)
        {
            ((VN::Registers::IMU::DeltaThetaVelConfig *)_reg)->gyroCompensation = static_cast<VN::Registers::IMU::DeltaThetaVelConfig::GyroCompensation>(value);
        }
        
    };
    
    enum class AccelCompensation : uint8_t
    {
        None = 0,
        Gravity = 1,
        Bias = 2,
        BiasAndGravity = 3,
    };
    property AccelCompensation accelCompensation
    {
        AccelCompensation get()
        {
            return static_cast<DeltaThetaVelConfig::AccelCompensation>(((VN::Registers::IMU::DeltaThetaVelConfig *)_reg)->accelCompensation);
        }
        
        void set(AccelCompensation value)
        {
            ((VN::Registers::IMU::DeltaThetaVelConfig *)_reg)->accelCompensation = static_cast<VN::Registers::IMU::DeltaThetaVelConfig::AccelCompensation>(value);
        }
        
    };
    
    enum class EarthRateCompensation : uint8_t
    {
        None = 0,
        GyroRate = 1,
        CoriolisAccel = 2,
        RateAndCoriolis = 3,
    };
    property EarthRateCompensation earthRateCompensation
    {
        EarthRateCompensation get()
        {
            return static_cast<DeltaThetaVelConfig::EarthRateCompensation>(((VN::Registers::IMU::DeltaThetaVelConfig *)_reg)->earthRateCompensation);
        }
        
        void set(EarthRateCompensation value)
        {
            ((VN::Registers::IMU::DeltaThetaVelConfig *)_reg)->earthRateCompensation = static_cast<VN::Registers::IMU::DeltaThetaVelConfig::EarthRateCompensation>(value);
        }
        
    };
    
    property uint16_t resv
    {
        uint16_t get()
        {
            return ((VN::Registers::IMU::DeltaThetaVelConfig *)_reg)->resv;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::IMU::DeltaThetaVelConfig *)_reg)->resv = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 84 - Gyro Calibration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class GyroCal : public Register
{
    public:
    GyroCal() : Register(new VN::Registers::IMU::GyroCal())
    {
    };
    
    property float gyroGain00
    {
        float get()
        {
            return ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain00;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain00 = value;
        }
        
    };
    
    property float gyroGain01
    {
        float get()
        {
            return ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain01;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain01 = value;
        }
        
    };
    
    property float gyroGain02
    {
        float get()
        {
            return ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain02;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain02 = value;
        }
        
    };
    
    property float gyroGain10
    {
        float get()
        {
            return ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain10;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain10 = value;
        }
        
    };
    
    property float gyroGain11
    {
        float get()
        {
            return ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain11;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain11 = value;
        }
        
    };
    
    property float gyroGain12
    {
        float get()
        {
            return ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain12;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain12 = value;
        }
        
    };
    
    property float gyroGain20
    {
        float get()
        {
            return ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain20;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain20 = value;
        }
        
    };
    
    property float gyroGain21
    {
        float get()
        {
            return ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain21;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain21 = value;
        }
        
    };
    
    property float gyroGain22
    {
        float get()
        {
            return ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain22;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::GyroCal *)_reg)->gyroGain22 = value;
        }
        
    };
    
    property float gyroBiasX
    {
        float get()
        {
            return ((VN::Registers::IMU::GyroCal *)_reg)->gyroBiasX;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::GyroCal *)_reg)->gyroBiasX = value;
        }
        
    };
    
    property float gyroBiasY
    {
        float get()
        {
            return ((VN::Registers::IMU::GyroCal *)_reg)->gyroBiasY;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::GyroCal *)_reg)->gyroBiasY = value;
        }
        
    };
    
    property float gyroBiasZ
    {
        float get()
        {
            return ((VN::Registers::IMU::GyroCal *)_reg)->gyroBiasZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::IMU::GyroCal *)_reg)->gyroBiasZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 85 - IMU Filtering Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class ImuFilterControl : public Register
{
    public:
    ImuFilterControl() : Register(new VN::Registers::IMU::ImuFilterControl())
    {
        magFilterMode = gcnew MagFilterMode(_reg);
        accelFilterMode = gcnew AccelFilterMode(_reg);
        gyroFilterMode = gcnew GyroFilterMode(_reg);
        tempFilterMode = gcnew TempFilterMode(_reg);
        presFilterMode = gcnew PresFilterMode(_reg);
    };
    
    property uint16_t magWindowSize
    {
        uint16_t get()
        {
            return ((VN::Registers::IMU::ImuFilterControl *)_reg)->magWindowSize;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::IMU::ImuFilterControl *)_reg)->magWindowSize = value;
        }
        
    };
    
    property uint16_t accelWindowSize
    {
        uint16_t get()
        {
            return ((VN::Registers::IMU::ImuFilterControl *)_reg)->accelWindowSize;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::IMU::ImuFilterControl *)_reg)->accelWindowSize = value;
        }
        
    };
    
    property uint16_t gyroWindowSize
    {
        uint16_t get()
        {
            return ((VN::Registers::IMU::ImuFilterControl *)_reg)->gyroWindowSize;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::IMU::ImuFilterControl *)_reg)->gyroWindowSize = value;
        }
        
    };
    
    property uint16_t tempWindowSize
    {
        uint16_t get()
        {
            return ((VN::Registers::IMU::ImuFilterControl *)_reg)->tempWindowSize;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::IMU::ImuFilterControl *)_reg)->tempWindowSize = value;
        }
        
    };
    
    property uint16_t presWindowSize
    {
        uint16_t get()
        {
            return ((VN::Registers::IMU::ImuFilterControl *)_reg)->presWindowSize;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::IMU::ImuFilterControl *)_reg)->presWindowSize = value;
        }
        
    };
    
    ref class MagFilterMode
    {
        VN::Register* _reg;
        
        public:
        MagFilterMode(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint8_t _value                 ///< BitField value for magFilterMode
        {
            uint8_t get()
            {
                return uint8_t(((VN::Registers::IMU::ImuFilterControl *)_reg)->magFilterMode);
            }
            
            void set(uint8_t value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->magFilterMode = value;
            }
            
        };
        
        // Uncomp
        property bool uncomp
        {
            bool get()
            {
                return ((VN::Registers::IMU::ImuFilterControl *)_reg)->magFilterMode.uncomp;
            }
            
            void set(bool value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->magFilterMode.uncomp = value;
            }
            
        };
        
        // Comp
        property bool comp
        {
            bool get()
            {
                return ((VN::Registers::IMU::ImuFilterControl *)_reg)->magFilterMode.comp;
            }
            
            void set(bool value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->magFilterMode.comp = value;
            }
            
        };
        
    };
    
    MagFilterMode^ magFilterMode;
    
    ref class AccelFilterMode
    {
        VN::Register* _reg;
        
        public:
        AccelFilterMode(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint8_t _value                 ///< BitField value for accelFilterMode
        {
            uint8_t get()
            {
                return uint8_t(((VN::Registers::IMU::ImuFilterControl *)_reg)->accelFilterMode);
            }
            
            void set(uint8_t value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->accelFilterMode = value;
            }
            
        };
        
        // Uncomp
        property bool uncomp
        {
            bool get()
            {
                return ((VN::Registers::IMU::ImuFilterControl *)_reg)->accelFilterMode.uncomp;
            }
            
            void set(bool value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->accelFilterMode.uncomp = value;
            }
            
        };
        
        // Comp
        property bool comp
        {
            bool get()
            {
                return ((VN::Registers::IMU::ImuFilterControl *)_reg)->accelFilterMode.comp;
            }
            
            void set(bool value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->accelFilterMode.comp = value;
            }
            
        };
        
    };
    
    AccelFilterMode^ accelFilterMode;
    
    ref class GyroFilterMode
    {
        VN::Register* _reg;
        
        public:
        GyroFilterMode(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint8_t _value                 ///< BitField value for gyroFilterMode
        {
            uint8_t get()
            {
                return uint8_t(((VN::Registers::IMU::ImuFilterControl *)_reg)->gyroFilterMode);
            }
            
            void set(uint8_t value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->gyroFilterMode = value;
            }
            
        };
        
        // Uncomp
        property bool uncomp
        {
            bool get()
            {
                return ((VN::Registers::IMU::ImuFilterControl *)_reg)->gyroFilterMode.uncomp;
            }
            
            void set(bool value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->gyroFilterMode.uncomp = value;
            }
            
        };
        
        // Comp
        property bool comp
        {
            bool get()
            {
                return ((VN::Registers::IMU::ImuFilterControl *)_reg)->gyroFilterMode.comp;
            }
            
            void set(bool value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->gyroFilterMode.comp = value;
            }
            
        };
        
    };
    
    GyroFilterMode^ gyroFilterMode;
    
    ref class TempFilterMode
    {
        VN::Register* _reg;
        
        public:
        TempFilterMode(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint8_t _value                 ///< BitField value for tempFilterMode
        {
            uint8_t get()
            {
                return uint8_t(((VN::Registers::IMU::ImuFilterControl *)_reg)->tempFilterMode);
            }
            
            void set(uint8_t value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->tempFilterMode = value;
            }
            
        };
        
        // Uncomp
        property bool uncomp
        {
            bool get()
            {
                return ((VN::Registers::IMU::ImuFilterControl *)_reg)->tempFilterMode.uncomp;
            }
            
            void set(bool value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->tempFilterMode.uncomp = value;
            }
            
        };
        
        // Comp
        property bool comp
        {
            bool get()
            {
                return ((VN::Registers::IMU::ImuFilterControl *)_reg)->tempFilterMode.comp;
            }
            
            void set(bool value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->tempFilterMode.comp = value;
            }
            
        };
        
    };
    
    TempFilterMode^ tempFilterMode;
    
    ref class PresFilterMode
    {
        VN::Register* _reg;
        
        public:
        PresFilterMode(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint8_t _value                 ///< BitField value for presFilterMode
        {
            uint8_t get()
            {
                return uint8_t(((VN::Registers::IMU::ImuFilterControl *)_reg)->presFilterMode);
            }
            
            void set(uint8_t value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->presFilterMode = value;
            }
            
        };
        
        // Uncomp
        property bool uncomp
        {
            bool get()
            {
                return ((VN::Registers::IMU::ImuFilterControl *)_reg)->presFilterMode.uncomp;
            }
            
            void set(bool value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->presFilterMode.uncomp = value;
            }
            
        };
        
        // Comp
        property bool comp
        {
            bool get()
            {
                return ((VN::Registers::IMU::ImuFilterControl *)_reg)->presFilterMode.comp;
            }
            
            void set(bool value)
            {
                ((VN::Registers::IMU::ImuFilterControl *)_reg)->presFilterMode.comp = value;
            }
            
        };
        
    };
    
    PresFilterMode^ presFilterMode;
    
    
};

} // namespace IMU

namespace INS
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 63 - INS Solution - LLA </summary>
<remarks>

Estimated INS solution with lat/lon/alt position. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class InsSolLla : public Register
{
    public:
    InsSolLla() : Register(new VN::Registers::INS::InsSolLla())
    {
    };
    
    property double timeGpsTow
    {
        double get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->timeGpsTow;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->timeGpsTow = value;
        }
        
    };
    
    property uint16_t timeGpsWeek
    {
        uint16_t get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->timeGpsWeek;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->timeGpsWeek = value;
        }
        
    };
    
    property VN::InsStatus insStatus
    {
        VN::InsStatus get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->insStatus;
        }
        
        void set(VN::InsStatus value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->insStatus = value;
        }
        
    };
    
    property float yaw
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->yaw;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->yaw = value;
        }
        
    };
    
    property float pitch
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->pitch;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->pitch = value;
        }
        
    };
    
    property float roll
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->roll;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->roll = value;
        }
        
    };
    
    property double posLat
    {
        double get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->posLat;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->posLat = value;
        }
        
    };
    
    property double posLon
    {
        double get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->posLon;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->posLon = value;
        }
        
    };
    
    property double posAlt
    {
        double get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->posAlt;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->posAlt = value;
        }
        
    };
    
    property float velN
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->velN;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->velN = value;
        }
        
    };
    
    property float velE
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->velE;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->velE = value;
        }
        
    };
    
    property float velD
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->velD;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->velD = value;
        }
        
    };
    
    property float attUncertainty
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->attUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->attUncertainty = value;
        }
        
    };
    
    property float posUncertainty
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->posUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->posUncertainty = value;
        }
        
    };
    
    property float velUncertainty
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolLla *)_reg)->velUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolLla *)_reg)->velUncertainty = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 64 - INS Solution - ECEF </summary>
<remarks>

Estimated INS Solution with ECEF position 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class InsSolEcef : public Register
{
    public:
    InsSolEcef() : Register(new VN::Registers::INS::InsSolEcef())
    {
    };
    
    property double timeGpsTow
    {
        double get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->timeGpsTow;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->timeGpsTow = value;
        }
        
    };
    
    property uint16_t timeGpsWeek
    {
        uint16_t get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->timeGpsWeek;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->timeGpsWeek = value;
        }
        
    };
    
    property VN::InsStatus insStatus
    {
        VN::InsStatus get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->insStatus;
        }
        
        void set(VN::InsStatus value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->insStatus = value;
        }
        
    };
    
    property float yaw
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->yaw;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->yaw = value;
        }
        
    };
    
    property float pitch
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->pitch;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->pitch = value;
        }
        
    };
    
    property float roll
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->roll;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->roll = value;
        }
        
    };
    
    property double posEx
    {
        double get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->posEx;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->posEx = value;
        }
        
    };
    
    property double posEy
    {
        double get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->posEy;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->posEy = value;
        }
        
    };
    
    property double posEz
    {
        double get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->posEz;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->posEz = value;
        }
        
    };
    
    property float velEx
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->velEx;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->velEx = value;
        }
        
    };
    
    property float velEy
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->velEy;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->velEy = value;
        }
        
    };
    
    property float velEz
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->velEz;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->velEz = value;
        }
        
    };
    
    property float attUncertainty
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->attUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->attUncertainty = value;
        }
        
    };
    
    property float posUncertainty
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->posUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->posUncertainty = value;
        }
        
    };
    
    property float velUncertainty
    {
        float get()
        {
            return ((VN::Registers::INS::InsSolEcef *)_reg)->velUncertainty;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsSolEcef *)_reg)->velUncertainty = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 67 - INS Basic Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class InsBasicConfig : public Register
{
    public:
    InsBasicConfig() : Register(new VN::Registers::INS::InsBasicConfig())
    {
    };
    
    enum class Scenario : uint8_t
    {
        Ahrs = 0,
        GnssInsWithPressure = 1,
        GnssInsNoPressure = 2,
        DualGnssNoPressure = 3,
        DualGnssWithPressure = 5,
    };
    property Scenario scenario
    {
        Scenario get()
        {
            return static_cast<InsBasicConfig::Scenario>(((VN::Registers::INS::InsBasicConfig *)_reg)->scenario);
        }
        
        void set(Scenario value)
        {
            ((VN::Registers::INS::InsBasicConfig *)_reg)->scenario = static_cast<VN::Registers::INS::InsBasicConfig::Scenario>(value);
        }
        
    };
    
    enum class AhrsAiding : uint8_t
    {
        Disable = 0,
        Enable = 1,
    };
    property AhrsAiding ahrsAiding
    {
        AhrsAiding get()
        {
            return static_cast<InsBasicConfig::AhrsAiding>(((VN::Registers::INS::InsBasicConfig *)_reg)->ahrsAiding);
        }
        
        void set(AhrsAiding value)
        {
            ((VN::Registers::INS::InsBasicConfig *)_reg)->ahrsAiding = static_cast<VN::Registers::INS::InsBasicConfig::AhrsAiding>(value);
        }
        
    };
    
    enum class EstBaseline : uint8_t
    {
        Disable = 0,
        Enable = 1,
    };
    property EstBaseline estBaseline
    {
        EstBaseline get()
        {
            return static_cast<InsBasicConfig::EstBaseline>(((VN::Registers::INS::InsBasicConfig *)_reg)->estBaseline);
        }
        
        void set(EstBaseline value)
        {
            ((VN::Registers::INS::InsBasicConfig *)_reg)->estBaseline = static_cast<VN::Registers::INS::InsBasicConfig::EstBaseline>(value);
        }
        
    };
    
    property uint8_t resv
    {
        uint8_t get()
        {
            return ((VN::Registers::INS::InsBasicConfig *)_reg)->resv;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::INS::InsBasicConfig *)_reg)->resv = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 72 - INS State - LLA </summary>
<remarks>

Estimated INS state with lat/lon/alt position. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class InsStateLla : public Register
{
    public:
    InsStateLla() : Register(new VN::Registers::INS::InsStateLla())
    {
    };
    
    property float yaw
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->yaw;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->yaw = value;
        }
        
    };
    
    property float pitch
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->pitch;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->pitch = value;
        }
        
    };
    
    property float roll
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->roll;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->roll = value;
        }
        
    };
    
    property double posLat
    {
        double get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->posLat;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->posLat = value;
        }
        
    };
    
    property double posLon
    {
        double get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->posLon;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->posLon = value;
        }
        
    };
    
    property double posAlt
    {
        double get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->posAlt;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->posAlt = value;
        }
        
    };
    
    property float velN
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->velN;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->velN = value;
        }
        
    };
    
    property float velE
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->velE;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->velE = value;
        }
        
    };
    
    property float velD
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->velD;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->velD = value;
        }
        
    };
    
    property float accelX
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->accelX;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->accelX = value;
        }
        
    };
    
    property float accelY
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->accelY;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->accelY = value;
        }
        
    };
    
    property float accelZ
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->accelZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->accelZ = value;
        }
        
    };
    
    property float gyroX
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->gyroX;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->gyroX = value;
        }
        
    };
    
    property float gyroY
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->gyroY;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->gyroY = value;
        }
        
    };
    
    property float gyroZ
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateLla *)_reg)->gyroZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateLla *)_reg)->gyroZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 73 - INS State - ECEF </summary>
<remarks>

Estimated INS state with ECEF position. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class InsStateEcef : public Register
{
    public:
    InsStateEcef() : Register(new VN::Registers::INS::InsStateEcef())
    {
    };
    
    property float yaw
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->yaw;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->yaw = value;
        }
        
    };
    
    property float pitch
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->pitch;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->pitch = value;
        }
        
    };
    
    property float roll
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->roll;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->roll = value;
        }
        
    };
    
    property double posEx
    {
        double get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->posEx;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->posEx = value;
        }
        
    };
    
    property double posEy
    {
        double get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->posEy;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->posEy = value;
        }
        
    };
    
    property double posEz
    {
        double get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->posEz;
        }
        
        void set(double value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->posEz = value;
        }
        
    };
    
    property float velEx
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->velEx;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->velEx = value;
        }
        
    };
    
    property float velEy
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->velEy;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->velEy = value;
        }
        
    };
    
    property float velEz
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->velEz;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->velEz = value;
        }
        
    };
    
    property float accelX
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->accelX;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->accelX = value;
        }
        
    };
    
    property float accelY
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->accelY;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->accelY = value;
        }
        
    };
    
    property float accelZ
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->accelZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->accelZ = value;
        }
        
    };
    
    property float gyroX
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->gyroX;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->gyroX = value;
        }
        
    };
    
    property float gyroY
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->gyroY;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->gyroY = value;
        }
        
    };
    
    property float gyroZ
    {
        float get()
        {
            return ((VN::Registers::INS::InsStateEcef *)_reg)->gyroZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsStateEcef *)_reg)->gyroZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 74 - Filter Startup Bias </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class FilterStartupBias : public Register
{
    public:
    FilterStartupBias() : Register(new VN::Registers::INS::FilterStartupBias())
    {
    };
    
    property float gyroBiasX
    {
        float get()
        {
            return ((VN::Registers::INS::FilterStartupBias *)_reg)->gyroBiasX;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::FilterStartupBias *)_reg)->gyroBiasX = value;
        }
        
    };
    
    property float gyroBiasY
    {
        float get()
        {
            return ((VN::Registers::INS::FilterStartupBias *)_reg)->gyroBiasY;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::FilterStartupBias *)_reg)->gyroBiasY = value;
        }
        
    };
    
    property float gyroBiasZ
    {
        float get()
        {
            return ((VN::Registers::INS::FilterStartupBias *)_reg)->gyroBiasZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::FilterStartupBias *)_reg)->gyroBiasZ = value;
        }
        
    };
    
    property float accelBiasX
    {
        float get()
        {
            return ((VN::Registers::INS::FilterStartupBias *)_reg)->accelBiasX;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::FilterStartupBias *)_reg)->accelBiasX = value;
        }
        
    };
    
    property float accelBiasY
    {
        float get()
        {
            return ((VN::Registers::INS::FilterStartupBias *)_reg)->accelBiasY;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::FilterStartupBias *)_reg)->accelBiasY = value;
        }
        
    };
    
    property float accelBiasZ
    {
        float get()
        {
            return ((VN::Registers::INS::FilterStartupBias *)_reg)->accelBiasZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::FilterStartupBias *)_reg)->accelBiasZ = value;
        }
        
    };
    
    property float presBias
    {
        float get()
        {
            return ((VN::Registers::INS::FilterStartupBias *)_reg)->presBias;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::FilterStartupBias *)_reg)->presBias = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 105 - INS Reference Point Offset </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class InsRefOffset : public Register
{
    public:
    InsRefOffset() : Register(new VN::Registers::INS::InsRefOffset())
    {
    };
    
    property float refOffsetX
    {
        float get()
        {
            return ((VN::Registers::INS::InsRefOffset *)_reg)->refOffsetX;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsRefOffset *)_reg)->refOffsetX = value;
        }
        
    };
    
    property float refOffsetY
    {
        float get()
        {
            return ((VN::Registers::INS::InsRefOffset *)_reg)->refOffsetY;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsRefOffset *)_reg)->refOffsetY = value;
        }
        
    };
    
    property float refOffsetZ
    {
        float get()
        {
            return ((VN::Registers::INS::InsRefOffset *)_reg)->refOffsetZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsRefOffset *)_reg)->refOffsetZ = value;
        }
        
    };
    
    property float refUncertX
    {
        float get()
        {
            return ((VN::Registers::INS::InsRefOffset *)_reg)->refUncertX;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsRefOffset *)_reg)->refUncertX = value;
        }
        
    };
    
    property float refUncertY
    {
        float get()
        {
            return ((VN::Registers::INS::InsRefOffset *)_reg)->refUncertY;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsRefOffset *)_reg)->refUncertY = value;
        }
        
    };
    
    property float refUncertZ
    {
        float get()
        {
            return ((VN::Registers::INS::InsRefOffset *)_reg)->refUncertZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::INS::InsRefOffset *)_reg)->refUncertZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 144 - INS GNSS Select </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class InsGnssSelect : public Register
{
    public:
    InsGnssSelect() : Register(new VN::Registers::INS::InsGnssSelect())
    {
    };
    
    enum class ActiveReceiverSelect : uint8_t
    {
        None = 0,
        PrimaryReceiver = 1,
        SecondaryReceiver = 2,
        TertiaryReceiver = 3,
        FallbackOnFailure = 5,
    };
    property ActiveReceiverSelect activeReceiverSelect
    {
        ActiveReceiverSelect get()
        {
            return static_cast<InsGnssSelect::ActiveReceiverSelect>(((VN::Registers::INS::InsGnssSelect *)_reg)->activeReceiverSelect);
        }
        
        void set(ActiveReceiverSelect value)
        {
            ((VN::Registers::INS::InsGnssSelect *)_reg)->activeReceiverSelect = static_cast<VN::Registers::INS::InsGnssSelect::ActiveReceiverSelect>(value);
        }
        
    };
    
    property uint8_t usedForNavTime
    {
        uint8_t get()
        {
            return ((VN::Registers::INS::InsGnssSelect *)_reg)->usedForNavTime;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::INS::InsGnssSelect *)_reg)->usedForNavTime = value;
        }
        
    };
    
    property uint8_t hysteresisTime
    {
        uint8_t get()
        {
            return ((VN::Registers::INS::InsGnssSelect *)_reg)->hysteresisTime;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::INS::InsGnssSelect *)_reg)->hysteresisTime = value;
        }
        
    };
    
    enum class UseGnssCompass : uint8_t
    {
        Off = 0,
        On = 1,
    };
    property UseGnssCompass useGnssCompass
    {
        UseGnssCompass get()
        {
            return static_cast<InsGnssSelect::UseGnssCompass>(((VN::Registers::INS::InsGnssSelect *)_reg)->useGnssCompass);
        }
        
        void set(UseGnssCompass value)
        {
            ((VN::Registers::INS::InsGnssSelect *)_reg)->useGnssCompass = static_cast<VN::Registers::INS::InsGnssSelect::UseGnssCompass>(value);
        }
        
    };
    
    property uint8_t resv1
    {
        uint8_t get()
        {
            return ((VN::Registers::INS::InsGnssSelect *)_reg)->resv1;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::INS::InsGnssSelect *)_reg)->resv1 = value;
        }
        
    };
    
    property uint8_t resv2
    {
        uint8_t get()
        {
            return ((VN::Registers::INS::InsGnssSelect *)_reg)->resv2;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::INS::InsGnssSelect *)_reg)->resv2 = value;
        }
        
    };
    
    
};

} // namespace INS

namespace System
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 0 - User Tag </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class UserTag : public Register
{
    public:
    UserTag() : Register(new VN::Registers::System::UserTag())
    {
    };
    
    property String^ tag
    {
        String^ get()
        {
            marshal_context^ context = gcnew marshal_context();
            String^ value = context->marshal_as<String^>(((VN::Registers::System::UserTag *)_reg)->tag.c_str());
            delete context;
            return value;
        }
        
        void set(String^ value)
        {
            marshal_context^ context = gcnew marshal_context();
            ((VN::Registers::System::UserTag *)_reg)->tag = context->marshal_as<const char*>(value);
            delete context;
        }
        
    }
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 1 - Model </summary>
<remarks>

Product model string. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class Model : public Register
{
    public:
    Model() : Register(new VN::Registers::System::Model())
    {
    };
    
    property String^ model
    {
        String^ get()
        {
            marshal_context^ context = gcnew marshal_context();
            String^ value = context->marshal_as<String^>(((VN::Registers::System::Model *)_reg)->model.c_str());
            delete context;
            return value;
        }
        
        void set(String^ value)
        {
            marshal_context^ context = gcnew marshal_context();
            ((VN::Registers::System::Model *)_reg)->model = context->marshal_as<const char*>(value);
            delete context;
        }
        
    }
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 2 - Hardware Version </summary>
<remarks>

Hardware version number. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class HwVer : public Register
{
    public:
    HwVer() : Register(new VN::Registers::System::HwVer())
    {
    };
    
    property uint32_t hwVer
    {
        uint32_t get()
        {
            return ((VN::Registers::System::HwVer *)_reg)->hwVer;
        }
        
        void set(uint32_t value)
        {
            ((VN::Registers::System::HwVer *)_reg)->hwVer = value;
        }
        
    };
    
    property uint32_t hwMinVer
    {
        uint32_t get()
        {
            return ((VN::Registers::System::HwVer *)_reg)->hwMinVer;
        }
        
        void set(uint32_t value)
        {
            ((VN::Registers::System::HwVer *)_reg)->hwMinVer = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 3 - Serial Number </summary>
<remarks>

Device serial number. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class Serial : public Register
{
    public:
    Serial() : Register(new VN::Registers::System::Serial())
    {
    };
    
    property uint32_t serialNum
    {
        uint32_t get()
        {
            return ((VN::Registers::System::Serial *)_reg)->serialNum;
        }
        
        void set(uint32_t value)
        {
            ((VN::Registers::System::Serial *)_reg)->serialNum = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 4 - Firmware Version </summary>
<remarks>

Firmware version number. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class FwVer : public Register
{
    public:
    FwVer() : Register(new VN::Registers::System::FwVer())
    {
    };
    
    property String^ fwVer
    {
        String^ get()
        {
            marshal_context^ context = gcnew marshal_context();
            String^ value = context->marshal_as<String^>(((VN::Registers::System::FwVer *)_reg)->fwVer.c_str());
            delete context;
            return value;
        }
        
        void set(String^ value)
        {
            marshal_context^ context = gcnew marshal_context();
            ((VN::Registers::System::FwVer *)_reg)->fwVer = context->marshal_as<const char*>(value);
            delete context;
        }
        
    }
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 5 - Baud Rate </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class BaudRate : public Register
{
    public:
    BaudRate() : Register(new VN::Registers::System::BaudRate())
    {
    };
    
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
    property BaudRates baudRate
    {
        BaudRates get()
        {
            return static_cast<BaudRate::BaudRates>(((VN::Registers::System::BaudRate *)_reg)->baudRate);
        }
        
        void set(BaudRates value)
        {
            ((VN::Registers::System::BaudRate *)_reg)->baudRate = static_cast<VN::Registers::System::BaudRate::BaudRates>(value);
        }
        
    };
    
    enum class SerialPort : uint8_t
    {
        ActiveSerial = 0,                       ///< Set value on active serial port.
        Serial1 = 1,
        Serial2 = 2,
        Poll = static_cast<uint8_t>('?')
    };
    property SerialPort serialPort
    {
        SerialPort get()
        {
            return static_cast<BaudRate::SerialPort>(((VN::Registers::System::BaudRate *)_reg)->serialPort);
        }
        
        void set(SerialPort value)
        {
            ((VN::Registers::System::BaudRate *)_reg)->serialPort = static_cast<VN::Registers::System::BaudRate::SerialPort>(value);
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 6 - Async Data Output Type </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class AsyncOutputType : public Register
{
    public:
    AsyncOutputType() : Register(new VN::Registers::System::AsyncOutputType())
    {
    };
    
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
    property Ador ador
    {
        Ador get()
        {
            return static_cast<AsyncOutputType::Ador>(((VN::Registers::System::AsyncOutputType *)_reg)->ador);
        }
        
        void set(Ador value)
        {
            ((VN::Registers::System::AsyncOutputType *)_reg)->ador = static_cast<VN::Registers::System::AsyncOutputType::Ador>(value);
        }
        
    };
    
    enum class SerialPort : uint8_t
    {
        ActiveSerial = 0,                       ///< Set value on active serial port.
        Serial1 = 1,
        Serial2 = 2,
        Poll = static_cast<uint8_t>('?')
    };
    property SerialPort serialPort
    {
        SerialPort get()
        {
            return static_cast<AsyncOutputType::SerialPort>(((VN::Registers::System::AsyncOutputType *)_reg)->serialPort);
        }
        
        void set(SerialPort value)
        {
            ((VN::Registers::System::AsyncOutputType *)_reg)->serialPort = static_cast<VN::Registers::System::AsyncOutputType::SerialPort>(value);
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 7 - Async Data Output Freq </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class AsyncOutputFreq : public Register
{
    public:
    AsyncOutputFreq() : Register(new VN::Registers::System::AsyncOutputFreq())
    {
    };
    
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
    property Adof adof
    {
        Adof get()
        {
            return static_cast<AsyncOutputFreq::Adof>(((VN::Registers::System::AsyncOutputFreq *)_reg)->adof);
        }
        
        void set(Adof value)
        {
            ((VN::Registers::System::AsyncOutputFreq *)_reg)->adof = static_cast<VN::Registers::System::AsyncOutputFreq::Adof>(value);
        }
        
    };
    
    enum class SerialPort : uint8_t
    {
        ActiveSerial = 0,                       ///< Set value on active serial port.
        Serial1 = 1,
        Serial2 = 2,
        Poll = static_cast<uint8_t>('?')
    };
    property SerialPort serialPort
    {
        SerialPort get()
        {
            return static_cast<AsyncOutputFreq::SerialPort>(((VN::Registers::System::AsyncOutputFreq *)_reg)->serialPort);
        }
        
        void set(SerialPort value)
        {
            ((VN::Registers::System::AsyncOutputFreq *)_reg)->serialPort = static_cast<VN::Registers::System::AsyncOutputFreq::SerialPort>(value);
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 30 - Communication Protocol Control </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class ProtocolControl : public Register
{
    public:
    ProtocolControl() : Register(new VN::Registers::System::ProtocolControl())
    {
    };
    
    enum class AsciiAppendCount : uint8_t
    {
        None = 0,
        SyncInCount = 1,
        SyncInTime = 2,
        SyncOutCount = 3,
        GpsPps = 4,
        GpsTow = 5,
    };
    property AsciiAppendCount asciiAppendCount
    {
        AsciiAppendCount get()
        {
            return static_cast<ProtocolControl::AsciiAppendCount>(((VN::Registers::System::ProtocolControl *)_reg)->asciiAppendCount);
        }
        
        void set(AsciiAppendCount value)
        {
            ((VN::Registers::System::ProtocolControl *)_reg)->asciiAppendCount = static_cast<VN::Registers::System::ProtocolControl::AsciiAppendCount>(value);
        }
        
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
    property AsciiAppendStatus asciiAppendStatus
    {
        AsciiAppendStatus get()
        {
            return static_cast<ProtocolControl::AsciiAppendStatus>(((VN::Registers::System::ProtocolControl *)_reg)->asciiAppendStatus);
        }
        
        void set(AsciiAppendStatus value)
        {
            ((VN::Registers::System::ProtocolControl *)_reg)->asciiAppendStatus = static_cast<VN::Registers::System::ProtocolControl::AsciiAppendStatus>(value);
        }
        
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
    property SpiAppendCount spiAppendCount
    {
        SpiAppendCount get()
        {
            return static_cast<ProtocolControl::SpiAppendCount>(((VN::Registers::System::ProtocolControl *)_reg)->spiAppendCount);
        }
        
        void set(SpiAppendCount value)
        {
            ((VN::Registers::System::ProtocolControl *)_reg)->spiAppendCount = static_cast<VN::Registers::System::ProtocolControl::SpiAppendCount>(value);
        }
        
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
    property SpiAppendStatus spiAppendStatus
    {
        SpiAppendStatus get()
        {
            return static_cast<ProtocolControl::SpiAppendStatus>(((VN::Registers::System::ProtocolControl *)_reg)->spiAppendStatus);
        }
        
        void set(SpiAppendStatus value)
        {
            ((VN::Registers::System::ProtocolControl *)_reg)->spiAppendStatus = static_cast<VN::Registers::System::ProtocolControl::SpiAppendStatus>(value);
        }
        
    };
    
    enum class AsciiChecksum : uint8_t
    {
        Checksum8bit = 1,
        Crc16bit = 3,
    };
    property AsciiChecksum asciiChecksum
    {
        AsciiChecksum get()
        {
            return static_cast<ProtocolControl::AsciiChecksum>(((VN::Registers::System::ProtocolControl *)_reg)->asciiChecksum);
        }
        
        void set(AsciiChecksum value)
        {
            ((VN::Registers::System::ProtocolControl *)_reg)->asciiChecksum = static_cast<VN::Registers::System::ProtocolControl::AsciiChecksum>(value);
        }
        
    };
    
    enum class SpiChecksum : uint8_t
    {
        Off = 0,
        Checksum8bit = 1,
        Crc16bit = 3,
    };
    property SpiChecksum spiChecksum
    {
        SpiChecksum get()
        {
            return static_cast<ProtocolControl::SpiChecksum>(((VN::Registers::System::ProtocolControl *)_reg)->spiChecksum);
        }
        
        void set(SpiChecksum value)
        {
            ((VN::Registers::System::ProtocolControl *)_reg)->spiChecksum = static_cast<VN::Registers::System::ProtocolControl::SpiChecksum>(value);
        }
        
    };
    
    enum class ErrorMode : uint8_t
    {
        Ignore = 0,
        SendError = 1,
        AdorOff = 2,
    };
    property ErrorMode errorMode
    {
        ErrorMode get()
        {
            return static_cast<ProtocolControl::ErrorMode>(((VN::Registers::System::ProtocolControl *)_reg)->errorMode);
        }
        
        void set(ErrorMode value)
        {
            ((VN::Registers::System::ProtocolControl *)_reg)->errorMode = static_cast<VN::Registers::System::ProtocolControl::ErrorMode>(value);
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 32 - Synchronization Control </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class SyncControl : public Register
{
    public:
    SyncControl() : Register(new VN::Registers::System::SyncControl())
    {
    };
    
    enum class SyncInMode : uint8_t
    {
        Disable = 0,
        Count = 3,
        ImuSample = 4,
        AsyncAll = 5,
        Async0 = 6,
    };
    property SyncInMode syncInMode
    {
        SyncInMode get()
        {
            return static_cast<SyncControl::SyncInMode>(((VN::Registers::System::SyncControl *)_reg)->syncInMode);
        }
        
        void set(SyncInMode value)
        {
            ((VN::Registers::System::SyncControl *)_reg)->syncInMode = static_cast<VN::Registers::System::SyncControl::SyncInMode>(value);
        }
        
    };
    
    enum class SyncInEdge : uint8_t
    {
        RisingEdge = 0,
        FallingEdge = 1,
    };
    property SyncInEdge syncInEdge
    {
        SyncInEdge get()
        {
            return static_cast<SyncControl::SyncInEdge>(((VN::Registers::System::SyncControl *)_reg)->syncInEdge);
        }
        
        void set(SyncInEdge value)
        {
            ((VN::Registers::System::SyncControl *)_reg)->syncInEdge = static_cast<VN::Registers::System::SyncControl::SyncInEdge>(value);
        }
        
    };
    
    property uint16_t syncInSkipFactor
    {
        uint16_t get()
        {
            return ((VN::Registers::System::SyncControl *)_reg)->syncInSkipFactor;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::System::SyncControl *)_reg)->syncInSkipFactor = value;
        }
        
    };
    
    property uint32_t resv1
    {
        uint32_t get()
        {
            return ((VN::Registers::System::SyncControl *)_reg)->resv1;
        }
        
        void set(uint32_t value)
        {
            ((VN::Registers::System::SyncControl *)_reg)->resv1 = value;
        }
        
    };
    
    enum class SyncOutMode : uint8_t
    {
        None = 0,
        ImuStart = 1,
        ImuReady = 2,
        NavReady = 3,
        GpsPps = 6,
    };
    property SyncOutMode syncOutMode
    {
        SyncOutMode get()
        {
            return static_cast<SyncControl::SyncOutMode>(((VN::Registers::System::SyncControl *)_reg)->syncOutMode);
        }
        
        void set(SyncOutMode value)
        {
            ((VN::Registers::System::SyncControl *)_reg)->syncOutMode = static_cast<VN::Registers::System::SyncControl::SyncOutMode>(value);
        }
        
    };
    
    enum class SyncOutPolarity : uint8_t
    {
        NegativePulse = 0,
        PositivePulse = 1,
    };
    property SyncOutPolarity syncOutPolarity
    {
        SyncOutPolarity get()
        {
            return static_cast<SyncControl::SyncOutPolarity>(((VN::Registers::System::SyncControl *)_reg)->syncOutPolarity);
        }
        
        void set(SyncOutPolarity value)
        {
            ((VN::Registers::System::SyncControl *)_reg)->syncOutPolarity = static_cast<VN::Registers::System::SyncControl::SyncOutPolarity>(value);
        }
        
    };
    
    property uint16_t syncOutSkipFactor
    {
        uint16_t get()
        {
            return ((VN::Registers::System::SyncControl *)_reg)->syncOutSkipFactor;
        }
        
        void set(uint16_t value)
        {
            ((VN::Registers::System::SyncControl *)_reg)->syncOutSkipFactor = value;
        }
        
    };
    
    property uint32_t syncOutPulseWidth
    {
        uint32_t get()
        {
            return ((VN::Registers::System::SyncControl *)_reg)->syncOutPulseWidth;
        }
        
        void set(uint32_t value)
        {
            ((VN::Registers::System::SyncControl *)_reg)->syncOutPulseWidth = value;
        }
        
    };
    
    property uint32_t resv2
    {
        uint32_t get()
        {
            return ((VN::Registers::System::SyncControl *)_reg)->resv2;
        }
        
        void set(uint32_t value)
        {
            ((VN::Registers::System::SyncControl *)_reg)->resv2 = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 33 - Synchronization Status </summary>
<remarks>

Contains counters based on the SyncIn and SyncOut events. 

</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class SyncStatus : public Register
{
    public:
    SyncStatus() : Register(new VN::Registers::System::SyncStatus())
    {
    };
    
    property uint32_t syncInCount
    {
        uint32_t get()
        {
            return ((VN::Registers::System::SyncStatus *)_reg)->syncInCount;
        }
        
        void set(uint32_t value)
        {
            ((VN::Registers::System::SyncStatus *)_reg)->syncInCount = value;
        }
        
    };
    
    property uint32_t syncInTime
    {
        uint32_t get()
        {
            return ((VN::Registers::System::SyncStatus *)_reg)->syncInTime;
        }
        
        void set(uint32_t value)
        {
            ((VN::Registers::System::SyncStatus *)_reg)->syncInTime = value;
        }
        
    };
    
    property uint32_t syncOutCount
    {
        uint32_t get()
        {
            return ((VN::Registers::System::SyncStatus *)_reg)->syncOutCount;
        }
        
        void set(uint32_t value)
        {
            ((VN::Registers::System::SyncStatus *)_reg)->syncOutCount = value;
        }
        
    };
    
    
};

public ref class BinaryOutput : public Register
{
    
    public:
    BinaryOutput(VN::Register* reg) : Register(reg)
    {
        asyncMode = gcnew AsyncMode(reg);
        common = gcnew CommonGroup(reg);
        time = gcnew TimeGroup(reg);
        imu = gcnew ImuGroup(reg);
        gnss = gcnew GnssGroup(reg);
        attitude = gcnew AttitudeGroup(reg);
        ins = gcnew InsGroup(reg);
        gnss2 = gcnew Gnss2Group(reg);
        gnss3 = gcnew Gnss3Group(reg);
        
    };
    
    ref class AsyncMode
    {
        VN::Register* _reg;
        
        public:
        AsyncMode(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint16_t _value                ///< BitField value for asyncMode
        {
            uint16_t get()
            {
                return uint16_t(((VN::Registers::System::BinaryOutput1 *)_reg)->asyncMode);
            }
            
            void set(uint16_t value)
            {
                ((VN::Registers::System::BinaryOutput1 *)_reg)->asyncMode = value;
            }
            
        };
        
        // Serial1
        property bool serial1
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput1 *)_reg)->asyncMode.serial1;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput1 *)_reg)->asyncMode.serial1 = value;
            }
            
        };
        
        // Serial2
        property bool serial2
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput1 *)_reg)->asyncMode.serial2;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput1 *)_reg)->asyncMode.serial2 = value;
            }
            
        };
        
        // Spi
        property bool spi
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput1 *)_reg)->asyncMode.spi;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput1 *)_reg)->asyncMode.spi = value;
            }
            
        };
        
    };
    
    AsyncMode^ asyncMode;
    
    property uint16_t rateDivisor
    {
        uint16_t get()
        {
            return ((VN::Registers::System::BinaryOutput *)_reg)->rateDivisor;
        };
        
        void set(uint16_t value)
        {
            ((VN::Registers::System::BinaryOutput *)_reg)->rateDivisor = value;
        };
        
    };
    
    VN::BinaryHeader toBinaryHeader()
    {
        return ((VN::Registers::System::BinaryOutput *)_reg)->toBinaryHeader();
    }
    
    ref class CommonGroup
    {
        VN::Register* _reg;
        
        public:
        CommonGroup(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint32_t _value
        {
            uint32_t get()
            {
                return uint32_t(((VN::Registers::System::BinaryOutput *)_reg)->common);
            };
            
            void set(uint32_t value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common = value;
            };
            
        }
        
        // TimeStartup
        property bool timeStartup
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.timeStartup;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.timeStartup = value;
            };
            
        }
        
        // TimeGps
        property bool timeGps
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.timeGps;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.timeGps = value;
            };
            
        }
        
        // TimeSyncIn
        property bool timeSyncIn
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.timeSyncIn;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.timeSyncIn = value;
            };
            
        }
        
        // Ypr
        property bool ypr
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.ypr;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.ypr = value;
            };
            
        }
        
        // Quaternion
        property bool quaternion
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.quaternion;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.quaternion = value;
            };
            
        }
        
        // AngularRate
        property bool angularRate
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.angularRate;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.angularRate = value;
            };
            
        }
        
        // PosLla
        property bool posLla
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.posLla;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.posLla = value;
            };
            
        }
        
        // VelNed
        property bool velNed
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.velNed;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.velNed = value;
            };
            
        }
        
        // Accel
        property bool accel
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.accel;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.accel = value;
            };
            
        }
        
        // Imu
        property bool imu
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.imu;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.imu = value;
            };
            
        }
        
        // MagPres
        property bool magPres
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.magPres;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.magPres = value;
            };
            
        }
        
        // Deltas
        property bool deltas
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.deltas;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.deltas = value;
            };
            
        }
        
        // InsStatus
        property bool insStatus
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.insStatus;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.insStatus = value;
            };
            
        }
        
        // SyncInCnt
        property bool syncInCnt
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.syncInCnt;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.syncInCnt = value;
            };
            
        }
        
        // TimeGpsPps
        property bool timeGpsPps
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->common.timeGpsPps;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->common.timeGpsPps = value;
            };
            
        }
        
    };
    
    CommonGroup^ common;
    
    ref class TimeGroup
    {
        VN::Register* _reg;
        
        public:
        TimeGroup(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint32_t _value
        {
            uint32_t get()
            {
                return uint32_t(((VN::Registers::System::BinaryOutput *)_reg)->time);
            };
            
            void set(uint32_t value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->time = value;
            };
            
        }
        
        // TimeStartup
        property bool timeStartup
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->time.timeStartup;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->time.timeStartup = value;
            };
            
        }
        
        // TimeGps
        property bool timeGps
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->time.timeGps;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->time.timeGps = value;
            };
            
        }
        
        // TimeGpsTow
        property bool timeGpsTow
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->time.timeGpsTow;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->time.timeGpsTow = value;
            };
            
        }
        
        // TimeGpsWeek
        property bool timeGpsWeek
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->time.timeGpsWeek;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->time.timeGpsWeek = value;
            };
            
        }
        
        // TimeSyncIn
        property bool timeSyncIn
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->time.timeSyncIn;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->time.timeSyncIn = value;
            };
            
        }
        
        // TimeGpsPps
        property bool timeGpsPps
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->time.timeGpsPps;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->time.timeGpsPps = value;
            };
            
        }
        
        // TimeUtc
        property bool timeUtc
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->time.timeUtc;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->time.timeUtc = value;
            };
            
        }
        
        // SyncInCnt
        property bool syncInCnt
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->time.syncInCnt;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->time.syncInCnt = value;
            };
            
        }
        
        // SyncOutCnt
        property bool syncOutCnt
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->time.syncOutCnt;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->time.syncOutCnt = value;
            };
            
        }
        
        // TimeStatus
        property bool timeStatus
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->time.timeStatus;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->time.timeStatus = value;
            };
            
        }
        
    };
    
    TimeGroup^ time;
    
    ref class ImuGroup
    {
        VN::Register* _reg;
        
        public:
        ImuGroup(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint32_t _value
        {
            uint32_t get()
            {
                return uint32_t(((VN::Registers::System::BinaryOutput *)_reg)->imu);
            };
            
            void set(uint32_t value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->imu = value;
            };
            
        }
        
        // ImuStatus
        property bool imuStatus
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->imu.imuStatus;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->imu.imuStatus = value;
            };
            
        }
        
        // UncompMag
        property bool uncompMag
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->imu.uncompMag;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->imu.uncompMag = value;
            };
            
        }
        
        // UncompAccel
        property bool uncompAccel
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->imu.uncompAccel;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->imu.uncompAccel = value;
            };
            
        }
        
        // UncompGyro
        property bool uncompGyro
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->imu.uncompGyro;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->imu.uncompGyro = value;
            };
            
        }
        
        // Temperature
        property bool temperature
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->imu.temperature;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->imu.temperature = value;
            };
            
        }
        
        // Pressure
        property bool pressure
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->imu.pressure;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->imu.pressure = value;
            };
            
        }
        
        // DeltaTheta
        property bool deltaTheta
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->imu.deltaTheta;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->imu.deltaTheta = value;
            };
            
        }
        
        // DeltaVel
        property bool deltaVel
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->imu.deltaVel;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->imu.deltaVel = value;
            };
            
        }
        
        // Mag
        property bool mag
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->imu.mag;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->imu.mag = value;
            };
            
        }
        
        // Accel
        property bool accel
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->imu.accel;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->imu.accel = value;
            };
            
        }
        
        // AngularRate
        property bool angularRate
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->imu.angularRate;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->imu.angularRate = value;
            };
            
        }
        
        // SensSat
        property bool sensSat
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->imu.sensSat;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->imu.sensSat = value;
            };
            
        }
        
    };
    
    ImuGroup^ imu;
    
    ref class GnssGroup
    {
        VN::Register* _reg;
        
        public:
        GnssGroup(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint32_t _value
        {
            uint32_t get()
            {
                return uint32_t(((VN::Registers::System::BinaryOutput *)_reg)->gnss);
            };
            
            void set(uint32_t value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss = value;
            };
            
        }
        
        // Gnss1TimeUtc
        property bool gnss1TimeUtc
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1TimeUtc;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1TimeUtc = value;
            };
            
        }
        
        // Gps1Tow
        property bool gps1Tow
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gps1Tow;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gps1Tow = value;
            };
            
        }
        
        // Gps1Week
        property bool gps1Week
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gps1Week;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gps1Week = value;
            };
            
        }
        
        // Gnss1NumSats
        property bool gnss1NumSats
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1NumSats;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1NumSats = value;
            };
            
        }
        
        // Gnss1Fix
        property bool gnss1Fix
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1Fix;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1Fix = value;
            };
            
        }
        
        // Gnss1PosLla
        property bool gnss1PosLla
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1PosLla;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1PosLla = value;
            };
            
        }
        
        // Gnss1PosEcef
        property bool gnss1PosEcef
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1PosEcef;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1PosEcef = value;
            };
            
        }
        
        // Gnss1VelNed
        property bool gnss1VelNed
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1VelNed;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1VelNed = value;
            };
            
        }
        
        // Gnss1VelEcef
        property bool gnss1VelEcef
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1VelEcef;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1VelEcef = value;
            };
            
        }
        
        // Gnss1PosUncertainty
        property bool gnss1PosUncertainty
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1PosUncertainty;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1PosUncertainty = value;
            };
            
        }
        
        // Gnss1VelUncertainty
        property bool gnss1VelUncertainty
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1VelUncertainty;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1VelUncertainty = value;
            };
            
        }
        
        // Gnss1TimeUncertainty
        property bool gnss1TimeUncertainty
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1TimeUncertainty;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1TimeUncertainty = value;
            };
            
        }
        
        // Gnss1TimeInfo
        property bool gnss1TimeInfo
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1TimeInfo;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1TimeInfo = value;
            };
            
        }
        
        // Gnss1Dop
        property bool gnss1Dop
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1Dop;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1Dop = value;
            };
            
        }
        
        // Gnss1SatInfo
        property bool gnss1SatInfo
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1SatInfo;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1SatInfo = value;
            };
            
        }
        
        // Gnss1RawMeas
        property bool gnss1RawMeas
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1RawMeas;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1RawMeas = value;
            };
            
        }
        
        // Gnss1Status
        property bool gnss1Status
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1Status;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1Status = value;
            };
            
        }
        
        // Gnss1AltMSL
        property bool gnss1AltMsl
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1AltMsl;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss.gnss1AltMsl = value;
            };
            
        }
        
    };
    
    GnssGroup^ gnss;
    
    ref class AttitudeGroup
    {
        VN::Register* _reg;
        
        public:
        AttitudeGroup(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint32_t _value
        {
            uint32_t get()
            {
                return uint32_t(((VN::Registers::System::BinaryOutput *)_reg)->attitude);
            };
            
            void set(uint32_t value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->attitude = value;
            };
            
        }
        
        // Ypr
        property bool ypr
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->attitude.ypr;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->attitude.ypr = value;
            };
            
        }
        
        // Quaternion
        property bool quaternion
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->attitude.quaternion;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->attitude.quaternion = value;
            };
            
        }
        
        // Dcm
        property bool dcm
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->attitude.dcm;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->attitude.dcm = value;
            };
            
        }
        
        // MagNed
        property bool magNed
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->attitude.magNed;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->attitude.magNed = value;
            };
            
        }
        
        // AccelNed
        property bool accelNed
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->attitude.accelNed;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->attitude.accelNed = value;
            };
            
        }
        
        // LinBodyAcc
        property bool linBodyAcc
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->attitude.linBodyAcc;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->attitude.linBodyAcc = value;
            };
            
        }
        
        // LinAccelNed
        property bool linAccelNed
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->attitude.linAccelNed;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->attitude.linAccelNed = value;
            };
            
        }
        
        // YprU
        property bool yprU
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->attitude.yprU;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->attitude.yprU = value;
            };
            
        }
        
        // Heave
        property bool heave
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->attitude.heave;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->attitude.heave = value;
            };
            
        }
        
        // AttU
        property bool attU
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->attitude.attU;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->attitude.attU = value;
            };
            
        }
        
    };
    
    AttitudeGroup^ attitude;
    
    ref class InsGroup
    {
        VN::Register* _reg;
        
        public:
        InsGroup(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint32_t _value
        {
            uint32_t get()
            {
                return uint32_t(((VN::Registers::System::BinaryOutput *)_reg)->ins);
            };
            
            void set(uint32_t value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->ins = value;
            };
            
        }
        
        // InsStatus
        property bool insStatus
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->ins.insStatus;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->ins.insStatus = value;
            };
            
        }
        
        // PosLla
        property bool posLla
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->ins.posLla;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->ins.posLla = value;
            };
            
        }
        
        // PosEcef
        property bool posEcef
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->ins.posEcef;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->ins.posEcef = value;
            };
            
        }
        
        // VelBody
        property bool velBody
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->ins.velBody;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->ins.velBody = value;
            };
            
        }
        
        // VelNed
        property bool velNed
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->ins.velNed;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->ins.velNed = value;
            };
            
        }
        
        // VelEcef
        property bool velEcef
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->ins.velEcef;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->ins.velEcef = value;
            };
            
        }
        
        // MagEcef
        property bool magEcef
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->ins.magEcef;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->ins.magEcef = value;
            };
            
        }
        
        // AccelEcef
        property bool accelEcef
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->ins.accelEcef;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->ins.accelEcef = value;
            };
            
        }
        
        // LinAccelEcef
        property bool linAccelEcef
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->ins.linAccelEcef;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->ins.linAccelEcef = value;
            };
            
        }
        
        // PosU
        property bool posU
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->ins.posU;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->ins.posU = value;
            };
            
        }
        
        // VelU
        property bool velU
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->ins.velU;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->ins.velU = value;
            };
            
        }
        
    };
    
    InsGroup^ ins;
    
    ref class Gnss2Group
    {
        VN::Register* _reg;
        
        public:
        Gnss2Group(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint32_t _value
        {
            uint32_t get()
            {
                return uint32_t(((VN::Registers::System::BinaryOutput *)_reg)->gnss2);
            };
            
            void set(uint32_t value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2 = value;
            };
            
        }
        
        // Gnss2TimeUtc
        property bool gnss2TimeUtc
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2TimeUtc;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2TimeUtc = value;
            };
            
        }
        
        // Gps2Tow
        property bool gps2Tow
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gps2Tow;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gps2Tow = value;
            };
            
        }
        
        // Gps2Week
        property bool gps2Week
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gps2Week;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gps2Week = value;
            };
            
        }
        
        // Gnss2NumSats
        property bool gnss2NumSats
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2NumSats;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2NumSats = value;
            };
            
        }
        
        // Gnss2Fix
        property bool gnss2Fix
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2Fix;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2Fix = value;
            };
            
        }
        
        // Gnss2PosLla
        property bool gnss2PosLla
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2PosLla;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2PosLla = value;
            };
            
        }
        
        // Gnss2PosEcef
        property bool gnss2PosEcef
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2PosEcef;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2PosEcef = value;
            };
            
        }
        
        // Gnss2VelNed
        property bool gnss2VelNed
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2VelNed;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2VelNed = value;
            };
            
        }
        
        // Gnss2VelEcef
        property bool gnss2VelEcef
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2VelEcef;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2VelEcef = value;
            };
            
        }
        
        // Gnss2PosUncertainty
        property bool gnss2PosUncertainty
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2PosUncertainty;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2PosUncertainty = value;
            };
            
        }
        
        // Gnss2VelUncertainty
        property bool gnss2VelUncertainty
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2VelUncertainty;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2VelUncertainty = value;
            };
            
        }
        
        // Gnss2TimeUncertainty
        property bool gnss2TimeUncertainty
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2TimeUncertainty;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2TimeUncertainty = value;
            };
            
        }
        
        // Gnss2TimeInfo
        property bool gnss2TimeInfo
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2TimeInfo;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2TimeInfo = value;
            };
            
        }
        
        // Gnss2Dop
        property bool gnss2Dop
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2Dop;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2Dop = value;
            };
            
        }
        
        // Gnss2SatInfo
        property bool gnss2SatInfo
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2SatInfo;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2SatInfo = value;
            };
            
        }
        
        // Gnss2RawMeas
        property bool gnss2RawMeas
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2RawMeas;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2RawMeas = value;
            };
            
        }
        
        // Gnss2Status
        property bool gnss2Status
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2Status;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2Status = value;
            };
            
        }
        
        // Gnss2AltMSL
        property bool gnss2AltMsl
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2AltMsl;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss2.gnss2AltMsl = value;
            };
            
        }
        
    };
    
    Gnss2Group^ gnss2;
    
    ref class Gnss3Group
    {
        VN::Register* _reg;
        
        public:
        Gnss3Group(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint32_t _value
        {
            uint32_t get()
            {
                return uint32_t(((VN::Registers::System::BinaryOutput *)_reg)->gnss3);
            };
            
            void set(uint32_t value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3 = value;
            };
            
        }
        
        // Gnss3TimeUtc
        property bool gnss3TimeUtc
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3TimeUtc;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3TimeUtc = value;
            };
            
        }
        
        // Gps3Tow
        property bool gps3Tow
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gps3Tow;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gps3Tow = value;
            };
            
        }
        
        // Gps3Week
        property bool gps3Week
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gps3Week;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gps3Week = value;
            };
            
        }
        
        // Gnss3NumSats
        property bool gnss3NumSats
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3NumSats;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3NumSats = value;
            };
            
        }
        
        // Gnss3Fix
        property bool gnss3Fix
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3Fix;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3Fix = value;
            };
            
        }
        
        // Gnss3PosLla
        property bool gnss3PosLla
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3PosLla;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3PosLla = value;
            };
            
        }
        
        // Gnss3PosEcef
        property bool gnss3PosEcef
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3PosEcef;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3PosEcef = value;
            };
            
        }
        
        // Gnss3VelNed
        property bool gnss3VelNed
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3VelNed;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3VelNed = value;
            };
            
        }
        
        // Gnss3VelEcef
        property bool gnss3VelEcef
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3VelEcef;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3VelEcef = value;
            };
            
        }
        
        // Gnss3PosUncertainty
        property bool gnss3PosUncertainty
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3PosUncertainty;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3PosUncertainty = value;
            };
            
        }
        
        // Gnss3VelUncertainty
        property bool gnss3VelUncertainty
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3VelUncertainty;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3VelUncertainty = value;
            };
            
        }
        
        // Gnss3TimeUncertainty
        property bool gnss3TimeUncertainty
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3TimeUncertainty;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3TimeUncertainty = value;
            };
            
        }
        
        // Gnss3TimeInfo
        property bool gnss3TimeInfo
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3TimeInfo;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3TimeInfo = value;
            };
            
        }
        
        // Gnss3Dop
        property bool gnss3Dop
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3Dop;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3Dop = value;
            };
            
        }
        
        // Gnss3SatInfo
        property bool gnss3SatInfo
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3SatInfo;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3SatInfo = value;
            };
            
        }
        
        // Gnss3RawMeas
        property bool gnss3RawMeas
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3RawMeas;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3RawMeas = value;
            };
            
        }
        
        // Gnss3Status
        property bool gnss3Status
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3Status;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3Status = value;
            };
            
        }
        
        // Gnss3AltMSL
        property bool gnss3AltMsl
        {
            bool get()
            {
                return ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3AltMsl;
            };
            
            void set(bool value)
            {
                ((VN::Registers::System::BinaryOutput *)_reg)->gnss3.gnss3AltMsl = value;
            };
            
        }
        
    };
    
    Gnss3Group^ gnss3;
    
    
};

public ref class BinaryOutput1 : public BinaryOutput
{
    public:
    BinaryOutput1() : BinaryOutput(new VN::Registers::System::BinaryOutput1()) {};
};

public ref class BinaryOutput2 : public BinaryOutput
{
    public:
    BinaryOutput2() : BinaryOutput(new VN::Registers::System::BinaryOutput2()) {};
};

public ref class BinaryOutput3 : public BinaryOutput
{
    public:
    BinaryOutput3() : BinaryOutput(new VN::Registers::System::BinaryOutput3()) {};
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 101 - NMEA Output 1 </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class NmeaOutput1 : public Register
{
    public:
    NmeaOutput1() : Register(new VN::Registers::System::NmeaOutput1())
    {
        msgSelection = gcnew MsgSelection(_reg);
    };
    
    enum class Port : uint8_t
    {
        None = 0,
        Serial1 = 1,
        Serial2 = 2,
        Serial1And2 = 3,
    };
    property Port port
    {
        Port get()
        {
            return static_cast<NmeaOutput1::Port>(((VN::Registers::System::NmeaOutput1 *)_reg)->port);
        }
        
        void set(Port value)
        {
            ((VN::Registers::System::NmeaOutput1 *)_reg)->port = static_cast<VN::Registers::System::NmeaOutput1::Port>(value);
        }
        
    };
    
    enum class Rate : uint8_t
    {
        Rate0Hz = 0,
        Rate1Hz = 1,
        Rate5Hz = 5,
        Rate10Hz = 10,
        Rate20Hz = 20,
    };
    property Rate rate
    {
        Rate get()
        {
            return static_cast<NmeaOutput1::Rate>(((VN::Registers::System::NmeaOutput1 *)_reg)->rate);
        }
        
        void set(Rate value)
        {
            ((VN::Registers::System::NmeaOutput1 *)_reg)->rate = static_cast<VN::Registers::System::NmeaOutput1::Rate>(value);
        }
        
    };
    
    enum class Mode : uint8_t
    {
        V41GPID = 0,
        V23GPID = 1,
        V41SYSID = 2,
    };
    property Mode mode
    {
        Mode get()
        {
            return static_cast<NmeaOutput1::Mode>(((VN::Registers::System::NmeaOutput1 *)_reg)->mode);
        }
        
        void set(Mode value)
        {
            ((VN::Registers::System::NmeaOutput1 *)_reg)->mode = static_cast<VN::Registers::System::NmeaOutput1::Mode>(value);
        }
        
    };
    
    enum class GnssSelect : uint8_t
    {
        GnssA = 0,
        ActiveReceiver = 1,
    };
    property GnssSelect gnssSelect
    {
        GnssSelect get()
        {
            return static_cast<NmeaOutput1::GnssSelect>(((VN::Registers::System::NmeaOutput1 *)_reg)->gnssSelect);
        }
        
        void set(GnssSelect value)
        {
            ((VN::Registers::System::NmeaOutput1 *)_reg)->gnssSelect = static_cast<VN::Registers::System::NmeaOutput1::GnssSelect>(value);
        }
        
    };
    
    ref class MsgSelection
    {
        VN::Register* _reg;
        
        public:
        MsgSelection(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint32_t _value                ///< BitField value for msgSelection
        {
            uint32_t get()
            {
                return uint32_t(((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection);
            }
            
            void set(uint32_t value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection = value;
            }
            
        };
        
        // RMC_GNSS
        property bool rmcGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.rmcGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.rmcGnss = value;
            }
            
        };
        
        // RMC_INS
        property bool rmcIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.rmcIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.rmcIns = value;
            }
            
        };
        
        // GGA_GNSS
        property bool ggaGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.ggaGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.ggaGnss = value;
            }
            
        };
        
        // GGA_INS
        property bool ggaIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.ggaIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.ggaIns = value;
            }
            
        };
        
        // GLL_GNSS
        property bool gllGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.gllGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.gllGnss = value;
            }
            
        };
        
        // GLL_INS
        property bool gllIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.gllIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.gllIns = value;
            }
            
        };
        
        // GSA_GNSS
        property bool gsaGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.gsaGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.gsaGnss = value;
            }
            
        };
        
        // GSV_GNSS
        property bool gsvGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.gsvGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.gsvGnss = value;
            }
            
        };
        
        // HDG_INS
        property bool hdgIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.hdgIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.hdgIns = value;
            }
            
        };
        
        // HDT_INS
        property bool hdtIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.hdtIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.hdtIns = value;
            }
            
        };
        
        // THS_INS
        property bool thsIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.thsIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.thsIns = value;
            }
            
        };
        
        // VTG_GNSS
        property bool vtgGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.vtgGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.vtgGnss = value;
            }
            
        };
        
        // VTG_INS
        property bool vtgIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.vtgIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.vtgIns = value;
            }
            
        };
        
        // ZDA_GNSS
        property bool zdaGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.zdaGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.zdaGnss = value;
            }
            
        };
        
        // ZDA_INS
        property bool zdaIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.zdaIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.zdaIns = value;
            }
            
        };
        
        // PASHR_INS
        property bool pashrIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.pashrIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.pashrIns = value;
            }
            
        };
        
        // TSS1_INS
        property bool tss1Ins
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.tss1Ins;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.tss1Ins = value;
            }
            
        };
        
        // INDYN
        property bool indyn
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.indyn;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput1 *)_reg)->msgSelection.indyn = value;
            }
            
        };
        
    };
    
    MsgSelection^ msgSelection;
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 102 - NMEA Output 2 </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class NmeaOutput2 : public Register
{
    public:
    NmeaOutput2() : Register(new VN::Registers::System::NmeaOutput2())
    {
        msgSelection = gcnew MsgSelection(_reg);
    };
    
    enum class Port : uint8_t
    {
        None = 0,
        Serial1 = 1,
        Serial2 = 2,
        Serial1And2 = 3,
    };
    property Port port
    {
        Port get()
        {
            return static_cast<NmeaOutput2::Port>(((VN::Registers::System::NmeaOutput2 *)_reg)->port);
        }
        
        void set(Port value)
        {
            ((VN::Registers::System::NmeaOutput2 *)_reg)->port = static_cast<VN::Registers::System::NmeaOutput2::Port>(value);
        }
        
    };
    
    enum class Rate : uint8_t
    {
        Rate0Hz = 0,
        Rate1Hz = 1,
        Rate5Hz = 5,
        Rate10Hz = 10,
        Rate20Hz = 20,
    };
    property Rate rate
    {
        Rate get()
        {
            return static_cast<NmeaOutput2::Rate>(((VN::Registers::System::NmeaOutput2 *)_reg)->rate);
        }
        
        void set(Rate value)
        {
            ((VN::Registers::System::NmeaOutput2 *)_reg)->rate = static_cast<VN::Registers::System::NmeaOutput2::Rate>(value);
        }
        
    };
    
    enum class Mode : uint8_t
    {
        V41GPID = 0,
        V23GPID = 1,
        V41SYSID = 2,
    };
    property Mode mode
    {
        Mode get()
        {
            return static_cast<NmeaOutput2::Mode>(((VN::Registers::System::NmeaOutput2 *)_reg)->mode);
        }
        
        void set(Mode value)
        {
            ((VN::Registers::System::NmeaOutput2 *)_reg)->mode = static_cast<VN::Registers::System::NmeaOutput2::Mode>(value);
        }
        
    };
    
    enum class GnssSelect : uint8_t
    {
        GnssA = 0,
        ActiveReceiver = 1,
    };
    property GnssSelect gnssSelect
    {
        GnssSelect get()
        {
            return static_cast<NmeaOutput2::GnssSelect>(((VN::Registers::System::NmeaOutput2 *)_reg)->gnssSelect);
        }
        
        void set(GnssSelect value)
        {
            ((VN::Registers::System::NmeaOutput2 *)_reg)->gnssSelect = static_cast<VN::Registers::System::NmeaOutput2::GnssSelect>(value);
        }
        
    };
    
    ref class MsgSelection
    {
        VN::Register* _reg;
        
        public:
        MsgSelection(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint32_t _value                ///< BitField value for msgSelection
        {
            uint32_t get()
            {
                return uint32_t(((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection);
            }
            
            void set(uint32_t value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection = value;
            }
            
        };
        
        // RMC_GNSS
        property bool rmcGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.rmcGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.rmcGnss = value;
            }
            
        };
        
        // RMC_INS
        property bool rmcIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.rmcIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.rmcIns = value;
            }
            
        };
        
        // GGA_GNSS
        property bool ggaGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.ggaGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.ggaGnss = value;
            }
            
        };
        
        // GGA_INS
        property bool ggaIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.ggaIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.ggaIns = value;
            }
            
        };
        
        // GLL_GNSS
        property bool gllGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.gllGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.gllGnss = value;
            }
            
        };
        
        // GLL_INS
        property bool gllIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.gllIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.gllIns = value;
            }
            
        };
        
        // GSA_GNSS
        property bool gsaGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.gsaGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.gsaGnss = value;
            }
            
        };
        
        // GSV_GNSS
        property bool gsvGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.gsvGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.gsvGnss = value;
            }
            
        };
        
        // HDG_INS
        property bool hdgIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.hdgIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.hdgIns = value;
            }
            
        };
        
        // HDT_INS
        property bool hdtIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.hdtIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.hdtIns = value;
            }
            
        };
        
        // THS_INS
        property bool thsIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.thsIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.thsIns = value;
            }
            
        };
        
        // VTG_GNSS
        property bool vtgGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.vtgGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.vtgGnss = value;
            }
            
        };
        
        // VTG_INS
        property bool vtgIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.vtgIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.vtgIns = value;
            }
            
        };
        
        // ZDA_GNSS
        property bool zdaGnss
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.zdaGnss;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.zdaGnss = value;
            }
            
        };
        
        // ZDA_INS
        property bool zdaIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.zdaIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.zdaIns = value;
            }
            
        };
        
        // PASHR_INS
        property bool pashrIns
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.pashrIns;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.pashrIns = value;
            }
            
        };
        
        // TSS1_INS
        property bool tss1Ins
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.tss1Ins;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.tss1Ins = value;
            }
            
        };
        
        // INDYN
        property bool indyn
        {
            bool get()
            {
                return ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.indyn;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::NmeaOutput2 *)_reg)->msgSelection.indyn = value;
            }
            
        };
        
    };
    
    MsgSelection^ msgSelection;
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 206 - Legacy Compatibility Settings </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class LegacyCompatibilitySettings : public Register
{
    public:
    LegacyCompatibilitySettings() : Register(new VN::Registers::System::LegacyCompatibilitySettings())
    {
        gnssLegacy = gcnew GnssLegacy(_reg);
    };
    
    enum class InsLegacy : uint8_t
    {
        Current = 0,
        Legacy = 1,
    };
    property InsLegacy insLegacy
    {
        InsLegacy get()
        {
            return static_cast<LegacyCompatibilitySettings::InsLegacy>(((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->insLegacy);
        }
        
        void set(InsLegacy value)
        {
            ((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->insLegacy = static_cast<VN::Registers::System::LegacyCompatibilitySettings::InsLegacy>(value);
        }
        
    };
    
    ref class GnssLegacy
    {
        VN::Register* _reg;
        
        public:
        GnssLegacy(VN::Register* reg)
        {
            _reg = reg;
        }
        
        property uint8_t _value                 ///< BitField value for gnssLegacy
        {
            uint8_t get()
            {
                return uint8_t(((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->gnssLegacy);
            }
            
            void set(uint8_t value)
            {
                ((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->gnssLegacy = value;
            }
            
        };
        
        // LegacyGnssFix
        property bool legacyGnssFix
        {
            bool get()
            {
                return ((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->gnssLegacy.legacyGnssFix;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->gnssLegacy.legacyGnssFix = value;
            }
            
        };
        
        // RequireReg55Reset
        property bool requireReg55Reset
        {
            bool get()
            {
                return ((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->gnssLegacy.requireReg55Reset;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->gnssLegacy.requireReg55Reset = value;
            }
            
        };
        
        // alwaysPpsPulse
        property bool alwaysPpsPulse
        {
            bool get()
            {
                return ((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->gnssLegacy.alwaysPpsPulse;
            }
            
            void set(bool value)
            {
                ((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->gnssLegacy.alwaysPpsPulse = value;
            }
            
        };
        
    };
    
    GnssLegacy^ gnssLegacy;
    
    enum class ImuLegacy : uint8_t
    {
        Current = 0,
        Legacy = 1,
    };
    property ImuLegacy imuLegacy
    {
        ImuLegacy get()
        {
            return static_cast<LegacyCompatibilitySettings::ImuLegacy>(((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->imuLegacy);
        }
        
        void set(ImuLegacy value)
        {
            ((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->imuLegacy = static_cast<VN::Registers::System::LegacyCompatibilitySettings::ImuLegacy>(value);
        }
        
    };
    
    enum class HwLegacy : uint8_t
    {
        Current = 0,
        Legacy = 1,
    };
    property HwLegacy hwLegacy
    {
        HwLegacy get()
        {
            return static_cast<LegacyCompatibilitySettings::HwLegacy>(((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->hwLegacy);
        }
        
        void set(HwLegacy value)
        {
            ((VN::Registers::System::LegacyCompatibilitySettings *)_reg)->hwLegacy = static_cast<VN::Registers::System::LegacyCompatibilitySettings::HwLegacy>(value);
        }
        
    };
    
    
};

} // namespace System

namespace VelocityAiding
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 50 - Velocity Aiding Measurement </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class VelAidingMeas : public Register
{
    public:
    VelAidingMeas() : Register(new VN::Registers::VelocityAiding::VelAidingMeas())
    {
    };
    
    property float velocityX
    {
        float get()
        {
            return ((VN::Registers::VelocityAiding::VelAidingMeas *)_reg)->velocityX;
        }
        
        void set(float value)
        {
            ((VN::Registers::VelocityAiding::VelAidingMeas *)_reg)->velocityX = value;
        }
        
    };
    
    property float velocityY
    {
        float get()
        {
            return ((VN::Registers::VelocityAiding::VelAidingMeas *)_reg)->velocityY;
        }
        
        void set(float value)
        {
            ((VN::Registers::VelocityAiding::VelAidingMeas *)_reg)->velocityY = value;
        }
        
    };
    
    property float velocityZ
    {
        float get()
        {
            return ((VN::Registers::VelocityAiding::VelAidingMeas *)_reg)->velocityZ;
        }
        
        void set(float value)
        {
            ((VN::Registers::VelocityAiding::VelAidingMeas *)_reg)->velocityZ = value;
        }
        
    };
    
    
};

/**--------------------------------------------------------------------------------------------------
<summary> Register 51 - Velocity Aiding Control </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class VelAidingControl : public Register
{
    public:
    VelAidingControl() : Register(new VN::Registers::VelocityAiding::VelAidingControl())
    {
    };
    
    enum class VelAidEnable : uint8_t
    {
        Disable = 0,
        Enable = 1,
    };
    property VelAidEnable velAidEnable
    {
        VelAidEnable get()
        {
            return static_cast<VelAidingControl::VelAidEnable>(((VN::Registers::VelocityAiding::VelAidingControl *)_reg)->velAidEnable);
        }
        
        void set(VelAidEnable value)
        {
            ((VN::Registers::VelocityAiding::VelAidingControl *)_reg)->velAidEnable = static_cast<VN::Registers::VelocityAiding::VelAidingControl::VelAidEnable>(value);
        }
        
    };
    
    property float velUncertTuning
    {
        float get()
        {
            return ((VN::Registers::VelocityAiding::VelAidingControl *)_reg)->velUncertTuning;
        }
        
        void set(float value)
        {
            ((VN::Registers::VelocityAiding::VelAidingControl *)_reg)->velUncertTuning = value;
        }
        
    };
    
    property float resv
    {
        float get()
        {
            return ((VN::Registers::VelocityAiding::VelAidingControl *)_reg)->resv;
        }
        
        void set(float value)
        {
            ((VN::Registers::VelocityAiding::VelAidingControl *)_reg)->resv = value;
        }
        
    };
    
    
};

} // namespace VelocityAiding

namespace WorldMagGravityModel
{
/**--------------------------------------------------------------------------------------------------
<summary> Register 83 - Reference Model Configuration </summary>
<remarks>
</remarks>
*-----------------------------------------------------------------------------------------------**/
public ref class RefModelConfig : public Register
{
    public:
    RefModelConfig() : Register(new VN::Registers::WorldMagGravityModel::RefModelConfig())
    {
    };
    
    enum class EnableMagModel : uint8_t
    {
        Disabled = 0,
        Enabled = 1,
    };
    property EnableMagModel enableMagModel
    {
        EnableMagModel get()
        {
            return static_cast<RefModelConfig::EnableMagModel>(((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->enableMagModel);
        }
        
        void set(EnableMagModel value)
        {
            ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->enableMagModel = static_cast<VN::Registers::WorldMagGravityModel::RefModelConfig::EnableMagModel>(value);
        }
        
    };
    
    enum class EnableGravityModel : uint8_t
    {
        Disabled = 0,
        Enabled = 1,
    };
    property EnableGravityModel enableGravityModel
    {
        EnableGravityModel get()
        {
            return static_cast<RefModelConfig::EnableGravityModel>(((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->enableGravityModel);
        }
        
        void set(EnableGravityModel value)
        {
            ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->enableGravityModel = static_cast<VN::Registers::WorldMagGravityModel::RefModelConfig::EnableGravityModel>(value);
        }
        
    };
    
    property uint8_t resv1
    {
        uint8_t get()
        {
            return ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->resv1;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->resv1 = value;
        }
        
    };
    
    property uint8_t resv2
    {
        uint8_t get()
        {
            return ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->resv2;
        }
        
        void set(uint8_t value)
        {
            ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->resv2 = value;
        }
        
    };
    
    property uint32_t recalcThreshold
    {
        uint32_t get()
        {
            return ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->recalcThreshold;
        }
        
        void set(uint32_t value)
        {
            ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->recalcThreshold = value;
        }
        
    };
    
    property float year
    {
        float get()
        {
            return ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->year;
        }
        
        void set(float value)
        {
            ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->year = value;
        }
        
    };
    
    property double latitude
    {
        double get()
        {
            return ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->latitude;
        }
        
        void set(double value)
        {
            ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->latitude = value;
        }
        
    };
    
    property double longitude
    {
        double get()
        {
            return ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->longitude;
        }
        
        void set(double value)
        {
            ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->longitude = value;
        }
        
    };
    
    property double altitude
    {
        double get()
        {
            return ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->altitude;
        }
        
        void set(double value)
        {
            ((VN::Registers::WorldMagGravityModel::RefModelConfig *)_reg)->altitude = value;
        }
        
    };
    
    
};

} // namespace WorldMagGravityModel

namespace ById {
using reg0 = VNSDK::Registers::System::UserTag;
using reg1 = VNSDK::Registers::System::Model;
using reg2 = VNSDK::Registers::System::HwVer;
using reg3 = VNSDK::Registers::System::Serial;
using reg4 = VNSDK::Registers::System::FwVer;
using reg5 = VNSDK::Registers::System::BaudRate;
using reg6 = VNSDK::Registers::System::AsyncOutputType;
using reg7 = VNSDK::Registers::System::AsyncOutputFreq;
using reg8 = VNSDK::Registers::Attitude::YawPitchRoll;
using reg9 = VNSDK::Registers::Attitude::Quaternion;
using reg15 = VNSDK::Registers::Attitude::QuatMagAccelRate;
using reg17 = VNSDK::Registers::IMU::Mag;
using reg18 = VNSDK::Registers::IMU::Accel;
using reg19 = VNSDK::Registers::IMU::Gyro;
using reg20 = VNSDK::Registers::IMU::MagAccelGyro;
using reg21 = VNSDK::Registers::Attitude::MagGravRefVec;
using reg23 = VNSDK::Registers::IMU::MagCal;
using reg25 = VNSDK::Registers::IMU::AccelCal;
using reg26 = VNSDK::Registers::IMU::RefFrameRot;
using reg27 = VNSDK::Registers::Attitude::YprMagAccelAngularRates;
using reg30 = VNSDK::Registers::System::ProtocolControl;
using reg32 = VNSDK::Registers::System::SyncControl;
using reg33 = VNSDK::Registers::System::SyncStatus;
using reg35 = VNSDK::Registers::Attitude::VpeBasicControl;
using reg36 = VNSDK::Registers::Attitude::VpeMagBasicTuning;
using reg38 = VNSDK::Registers::Attitude::VpeAccelBasicTuning;
using reg44 = VNSDK::Registers::HardSoftIronEstimator::RealTimeHsiControl;
using reg47 = VNSDK::Registers::HardSoftIronEstimator::EstMagCal;
using reg50 = VNSDK::Registers::VelocityAiding::VelAidingMeas;
using reg51 = VNSDK::Registers::VelocityAiding::VelAidingControl;
using reg54 = VNSDK::Registers::IMU::ImuMeas;
using reg55 = VNSDK::Registers::GNSS::GnssBasicConfig;
using reg57 = VNSDK::Registers::GNSS::GnssAOffset;
using reg58 = VNSDK::Registers::GNSS::GnssSolLla;
using reg59 = VNSDK::Registers::GNSS::GnssSolEcef;
using reg63 = VNSDK::Registers::INS::InsSolLla;
using reg64 = VNSDK::Registers::INS::InsSolEcef;
using reg67 = VNSDK::Registers::INS::InsBasicConfig;
using reg72 = VNSDK::Registers::INS::InsStateLla;
using reg73 = VNSDK::Registers::INS::InsStateEcef;
using reg74 = VNSDK::Registers::INS::FilterStartupBias;
using reg75 = VNSDK::Registers::System::BinaryOutput1;
using reg76 = VNSDK::Registers::System::BinaryOutput2;
using reg77 = VNSDK::Registers::System::BinaryOutput3;
using reg80 = VNSDK::Registers::IMU::DeltaThetaVelocity;
using reg82 = VNSDK::Registers::IMU::DeltaThetaVelConfig;
using reg83 = VNSDK::Registers::WorldMagGravityModel::RefModelConfig;
using reg84 = VNSDK::Registers::IMU::GyroCal;
using reg85 = VNSDK::Registers::IMU::ImuFilterControl;
using reg86 = VNSDK::Registers::GNSSCompass::GnssCompassSignalHealthStatus;
using reg93 = VNSDK::Registers::GNSSCompass::GnssCompassBaseline;
using reg97 = VNSDK::Registers::GNSSCompass::GnssCompassEstBaseline;
using reg98 = VNSDK::Registers::GNSSCompass::GnssCompassStartupStatus;
using reg99 = VNSDK::Registers::GNSS::GnssSystemConfig;
using reg100 = VNSDK::Registers::GNSS::GnssSyncConfig;
using reg101 = VNSDK::Registers::System::NmeaOutput1;
using reg102 = VNSDK::Registers::System::NmeaOutput2;
using reg103 = VNSDK::Registers::GNSS::Gnss2SolLla;
using reg104 = VNSDK::Registers::GNSS::Gnss2SolEcef;
using reg105 = VNSDK::Registers::INS::InsRefOffset;
using reg115 = VNSDK::Registers::Heave::HeaveOutputs;
using reg116 = VNSDK::Registers::Heave::HeaveBasicConfig;
using reg144 = VNSDK::Registers::INS::InsGnssSelect;
using reg157 = VNSDK::Registers::GNSS::ExtGnssOffset;
using reg206 = VNSDK::Registers::System::LegacyCompatibilitySettings;
using reg239 = VNSDK::Registers::Attitude::YprLinearBodyAccelAngularRates;
using reg240 = VNSDK::Registers::Attitude::YprLinearInertialAccelAngularRates;
} // namespace ById
} // namespace Registers
} // namespace VNSDK


#endif //VN_CLI_REGISTERS_HPP_


