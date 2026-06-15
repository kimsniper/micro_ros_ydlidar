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

#include "ydlidar.hpp"
#include <cmath>
#include <cstring>
#include "esp_log.h"

namespace YDLIDAR {

YDLidar_Driver::YDLidar_Driver(Device dev, Config cfg) 
    : device(dev), config(cfg), state(ParserState::WAIT_PH_LOW) {
    std::memset(&pkt, 0, sizeof(pkt));
}

LidarError_t YDLidar_Driver::Init() {
    return ydlidar_hal_init(&device);
}

LidarError_t YDLidar_Driver::ValidateChecksum() {
    uint16_t checksum_calc = PACKET_HEADER;
    checksum_calc ^= pkt.fsa;

    if (!config.hasIntensity) {
        for (int i = 0; i < 2 * pkt.lsn; i += 2) {
            checksum_calc ^= static_cast<uint16_t>((pkt.data_buffer[i + 1] << 8) | pkt.data_buffer[i]);
        }
    } else {
        for (int i = 0; i < 3 * pkt.lsn; i += 3) {
            checksum_calc ^= pkt.data_buffer[i];
            checksum_calc ^= static_cast<uint16_t>((pkt.data_buffer[i + 2] << 8) | pkt.data_buffer[i + 1]);
        }
    }

    checksum_calc ^= static_cast<uint16_t>((pkt.lsn << 8) | pkt.ct);
    checksum_calc ^= pkt.lsa;

    return (checksum_calc == pkt.cs) ? LidarError_t::LIDAR_OK : LidarError_t::LIDAR_ERR_CHECKSUM;
}

void YDLidar_Driver::ParsePacketPayload(std::vector<PointData>& outPoints) {
    double angle_fsa = static_cast<double>(pkt.fsa >> 1) / 64.0;
    double angle_lsa = static_cast<double>(pkt.lsa >> 1) / 64.0;
    double angle_diff = angle_lsa - angle_fsa;

    if (angle_diff < 0.0) {
        angle_diff += 360.0;
    }

    for (int i = 0; i < pkt.lsn; i++) {
        double distance = 0.0;
        double intensity = 0.0;

        if (!config.hasIntensity) {
            uint16_t raw_dist = static_cast<uint16_t>((pkt.data_buffer[i * 2 + 1] << 8) | pkt.data_buffer[i * 2]);
            distance = (config.model == LidarModel_t::TRIANGLE_LIDAR) ? (static_cast<double>(raw_dist) / 4.0) : static_cast<double>(raw_dist);
        } else {
            int base = i * 3;
            uint16_t raw_data = static_cast<uint16_t>((pkt.data_buffer[base + 2] << 8) | pkt.data_buffer[base + 1]);
            intensity = static_cast<double>(((pkt.data_buffer[base + 1] & 0x03) << 8) | pkt.data_buffer[base]);
            distance = (config.model == LidarModel_t::TRIANGLE_LIDAR) ? static_cast<double>(raw_data >> 2) : static_cast<double>(raw_data);
        }

        double angle = (pkt.lsn > 1) ? ((angle_diff / (pkt.lsn - 1)) * i + angle_fsa) : angle_fsa;

        // Trigonometric Error Correction for Triangle Lidar models
        if (config.model == LidarModel_t::TRIANGLE_LIDAR && distance > 0.0) {
            double ang_correct = std::atan(21.8 * (155.3 - distance) / (155.3 * distance));
            angle += ang_correct * (180.0 / M_PI);
        }

        while (angle >= 360.0) angle -= 360.0;
        while (angle < 0.0) angle += 360.0;

        PointData pt;
        pt.distance_m = static_cast<float>(distance / 1000.0); // mm to meters
        pt.angle_rad = static_cast<float>(angle * M_PI / 180.0); // degrees to radians
        pt.intensity = static_cast<float>(intensity);
        
        outPoints.push_back(pt);
    }
}

LidarError_t YDLidar_Driver::PollScanChunk(std::vector<PointData>& outPoints, bool& isFullTurnCompleted) {
    uint8_t b = 0;
    isFullTurnCompleted = false;

    // Dynamic processing stream state machine loop
    while (ydlidar_hal_read_byte(device.uartPort, &b, 2) == LidarError_t::LIDAR_OK) {
        switch (state) {
            case ParserState::WAIT_PH_LOW:
                if (b == 0xAA) state = ParserState::WAIT_PH_HIGH;
                break;
            case ParserState::WAIT_PH_HIGH:
                state = (b == 0x55) ? ParserState::WAIT_CT : ParserState::WAIT_PH_LOW;
                break;
            case ParserState::WAIT_CT:
                pkt.ct = b;
                state = ParserState::WAIT_LSN;
                break;
            case ParserState::WAIT_LSN:
                pkt.lsn = b;
                pkt.expected_data_len = pkt.lsn * (config.hasIntensity ? 3 : 2);
                pkt.bytes_read = 0;
                state = ParserState::WAIT_FSA_LOW;
                break;
            case ParserState::WAIT_FSA_LOW:
                pkt.fsa = b;
                state = ParserState::WAIT_FSA_HIGH;
                break;
            case ParserState::WAIT_FSA_HIGH:
                pkt.fsa |= (b << 8);
                state = ParserState::WAIT_LSA_LOW;
                break;
            case ParserState::WAIT_LSA_LOW:
                pkt.lsa = b;
                state = ParserState::WAIT_LSA_HIGH;
                break;
            case ParserState::WAIT_LSA_HIGH:
                pkt.lsa |= (b << 8);
                state = ParserState::WAIT_CS_LOW;
                break;
            case ParserState::WAIT_CS_LOW:
                pkt.cs = b;
                state = ParserState::WAIT_CS_HIGH;
                break;
            case ParserState::WAIT_CS_HIGH:
                pkt.cs |= (b << 8);
                if (pkt.expected_data_len == 0) {
                    state = ParserState::WAIT_PH_LOW;
                    if ((pkt.ct & 0x01) == CT_ZERO) isFullTurnCompleted = true;
                    return LidarError_t::LIDAR_OK;
                }
                state = ParserState::WAIT_DATA;
                break;
            case ParserState::WAIT_DATA:
                pkt.data_buffer[pkt.bytes_read++] = b;
                if (pkt.bytes_read >= pkt.expected_data_len) {
                    state = ParserState::WAIT_PH_LOW;
                    if (ValidateChecksum() == LidarError_t::LIDAR_OK) {
                        if ((pkt.ct & 0x01) == CT_ZERO) {
                            isFullTurnCompleted = true;
                        } else {
                            ParsePacketPayload(outPoints);
                        }
                        return LidarError_t::LIDAR_OK;
                    } else {
                        return LidarError_t::LIDAR_ERR_CHECKSUM;
                    }
                }
                break;
        }
    }
    return LidarError_t::LIDAR_OK;
}

} // namespace YDLIDAR