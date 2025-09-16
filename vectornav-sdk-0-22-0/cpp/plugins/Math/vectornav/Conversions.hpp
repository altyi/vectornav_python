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

#ifndef VN_CONVERSIONS_HPP_
#define VN_CONVERSIONS_HPP_

#define _USE_MATH_DEFINES
#include <math.h>

#include "vectornav/Implementation/MeasurementDatatypes.hpp"
#include "vectornav/TemplateLibrary/Matrix.hpp"

namespace VN
{
namespace Math
{

// ******
// Angles
// ******

inline float rad2deg(float angleInRads) noexcept { return angleInRads * 180.0f / static_cast<float>(M_PI); }

inline double rad2deg(double angleInRads) noexcept { return angleInRads * 180.0 / static_cast<double>(M_PI); }

inline float deg2rad(float angleInDegs) noexcept { return angleInDegs * static_cast<float>(M_PI) / 180.0f; }

inline double deg2rad(double angleInDegs) noexcept { return angleInDegs * static_cast<double>(M_PI) / 180.0; }

template <uint16_t n, uint16_t m, typename T>
inline Matrix<n, m, T> rad2deg(Matrix<n, m, T> vec) noexcept
{
    return vec * (static_cast<T>(180.0) / static_cast<T>(M_PI));
}

template <uint16_t n, uint16_t m, typename T>
inline Matrix<n, m, T> deg2rad(Matrix<n, m, T> vec) noexcept
{
    return vec * (static_cast<T>(M_PI) / static_cast<T>(180.0));
}

inline Ypr deg2rad(Ypr ypr) noexcept
{
    ypr.yaw *= (static_cast<float>(M_PI) / 180.0f);
    ypr.pitch *= (static_cast<float>(M_PI) / 180.0f);
    ypr.roll *= (static_cast<float>(M_PI) / 180.0f);
    return ypr;
}

inline Ypr rad2deg(Ypr ypr) noexcept
{
    ypr.yaw *= (180.0f / static_cast<float>(M_PI));
    ypr.pitch *= (180.0f / static_cast<float>(M_PI));
    ypr.roll *= (180.0f / static_cast<float>(M_PI));
    return ypr;
}

// ***********
// Temperature
// ***********

inline float celsius2fahren(float tempInCelsius) noexcept { return (tempInCelsius * 9.0f) / 5.0f + 32.0f; }

inline double celsius2fahren(double tempInCelsius) noexcept { return (tempInCelsius * 9.0) / 5.0 + 32.0; }

inline float fahren2celsius(float tempInFahren) noexcept { return (tempInFahren - 32.0f) * 5.0f / 9.0f; }

inline double fahren2celsius(double tempInFahren) noexcept { return (tempInFahren - 32.0) * 5.0 / 9.0; }

inline float celsius2kelvin(float tempInCelsius) noexcept { return tempInCelsius + 273.15f; }

inline double celsius2kelvin(double tempInCelsius) noexcept { return tempInCelsius + 273.15; }

inline float kelvin2celsius(float tempInKelvin) noexcept { return tempInKelvin - 273.15f; }

inline double kelvin2celsius(double tempInKelvin) noexcept { return tempInKelvin - 273.15; }

inline float fahren2kelvin(float tempInFahren) noexcept { return celsius2kelvin(fahren2celsius(tempInFahren)); }

inline double fahren2kelvin(double tempInFahren) noexcept { return celsius2kelvin(fahren2celsius(tempInFahren)); }

inline float kelvin2fahren(float tempInKelvin) noexcept { return celsius2fahren(tempInKelvin - 273.15f); }

inline double kelvin2fahren(double tempInKelvin) noexcept { return celsius2fahren(tempInKelvin - 273.15); }

}  // namespace Math
}  // namespace VN

#endif  // VN_CONVERSIONS_HPP_
