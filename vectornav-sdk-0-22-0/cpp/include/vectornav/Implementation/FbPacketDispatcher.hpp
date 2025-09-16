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

#ifndef VN_FBPACKETDISPATCHER_HPP_
#define VN_FBPACKETDISPATCHER_HPP_

#include <cstdint>

#include "vectornav/Config.hpp"
#include "vectornav/Implementation/BinaryHeader.hpp"
#include "vectornav/Implementation/FaPacketDispatcher.hpp"
#include "vectornav/Implementation/FbPacketProtocol.hpp"
#include "vectornav/Implementation/PacketDispatcher.hpp"
#include "vectornav/TemplateLibrary/ByteBuffer.hpp"

namespace VN
{

class FbPacketDispatcher : public PacketDispatcher
{
public:
    FbPacketDispatcher(FaPacketDispatcher* faSubsciberInvoker, const size_t byteBufferCapacity)
        : PacketDispatcher({0xFB}), _faPacketDispatcher(faSubsciberInvoker), _fbByteBuffer(byteBufferCapacity)
    {
    }

    FbPacketDispatcher(FaPacketDispatcher* faSubsciberInvoker, uint8_t* bufferHead, size_t bufferCapacity)
        : PacketDispatcher({0xFB}), _faPacketDispatcher(faSubsciberInvoker), _fbByteBuffer(bufferHead, bufferCapacity)
    {
    }

    PacketDispatcher::FindPacketRetVal findPacket(const ByteBuffer& byteBuffer, const size_t syncByteIndex) noexcept override;

    void dispatchPacket(const ByteBuffer& byteBuffer, const size_t syncByteIndex) noexcept override;

private:
    void _resetFbBuffer() noexcept;
    void _addFaPacketCrc() noexcept;
    bool _moveBytesFromMainBufferToFbBuffer(SplitPacketDetails splitPacketDetails, const ByteBuffer& byteBuffer, const uint16_t numOfBytesToMove,
                                            const size_t startingIndex) noexcept;

    FaPacketDispatcher* _faPacketDispatcher;
    ByteBuffer _fbByteBuffer;
    FbPacketProtocol::Metadata _latestPacketMetadata{};
    FbPacketProtocol::Metadata _previousPacketMetadata{};
};
}  // namespace VN

#endif  // VN_FBPACKETDISPATCHER_HPP_
