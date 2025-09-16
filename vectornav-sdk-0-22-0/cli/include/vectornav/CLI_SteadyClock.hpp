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

#ifndef VN_CLI_STEADYCLOCK_HPP_
#define VN_CLI_STEADYCLOCK_HPP_

#include "vectornav/HAL/Timer.hpp"

namespace VNSDK
{
	public ref class SteadyClock
	{
		public:
		
	    SteadyClock()
        {
            _t0cli = System::DateTime::Now;
            _t0cpp = new VN::time_point(VN::now());
        }

	    ~SteadyClock() { delete _t0cpp; };

        System::DateTime ConvertTime(std::chrono::steady_clock::time_point const & tCPP)
        {
            auto const milliSecsUntilT = std::chrono::duration_cast<std::chrono::milliseconds>(tCPP - *_t0cpp).count();
            return _t0cli.AddMilliseconds(milliSecsUntilT);
        }

        System::DateTime now() { return ConvertTime(VN::now()); }

        private:

        System::DateTime _t0cli;
        VN::time_point *_t0cpp;

	}; // ref class SteadyClock
} // namespace VNSDK

#endif  // VN_CLI_STEADYCLOCK_HPP_
