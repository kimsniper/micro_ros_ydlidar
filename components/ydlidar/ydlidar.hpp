/*
 * Copyright (c) 2026, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "ydlidar_hal.hpp"
#include <vector>
#include <memory>

namespace YDLIDAR {

static constexpr uint16_t PACKET_HEADER = 0x55AA;
static constexpr uint8_t  CT_NORMAL     = 0x00;
static constexpr uint8_t  CT_ZERO       = 0x01;

struct PointData {
    float angle_rad;
    float distance_m;
    float intensity;
};

class YDLidar_Driver {
public:
    YDLidar_Driver(Device dev, Config cfg);
    LidarError_t Init();
    LidarError_t PollScanChunk(std::vector<PointData>& outPoints, bool& isFullTurnCompleted);

private:
    enum class ParserState {
        WAIT_PH_LOW,
        WAIT_PH_HIGH,
        WAIT_CT,
        WAIT_LSN,
        WAIT_FSA_LOW,
        WAIT_FSA_HIGH,
        WAIT_LSA_LOW,
        WAIT_LSA_HIGH,
        WAIT_CS_LOW,
        WAIT_CS_HIGH,
        WAIT_DATA
    };

    Device device;
    Config config;
    ParserState state;

    // Local packet tracking structure
    struct RawPacket {
        uint8_t ct;
        uint8_t lsn;
        uint16_t fsa;
        uint16_t lsa;
        uint16_t cs;
        uint8_t data_buffer[160]; 
        uint16_t expected_data_len;
        uint16_t bytes_read;
    } pkt;

    LidarError_t ValidateChecksum();
    void ParsePacketPayload(std::vector<PointData>& outPoints);
};

} // namespace YDLIDAR