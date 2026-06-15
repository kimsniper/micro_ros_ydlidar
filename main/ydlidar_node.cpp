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
#include "ydlidar_node.hpp"
#include <cstring>
#include <ctime>
#include "esp_log.h"

#define TAG "ydlidar_node"

static YDLIDAR::Device TargetLidarDevice = {
    .uartPort = 1,
    .txPin = 22,
    .rxPin = 21,
    .baudRate = 128000
};

static YDLIDAR::Config TargetLidarConfig = {
    .model = YDLIDAR::LidarModel_t::TRIANGLE_LIDAR,
    .hasIntensity = false,
    .maxPointsPerTurn = 720
};

void YdLidarNode::init(rclc_support_t* support, rcl_allocator_t* allocator) {
    rcl_node_t node;
    rclc_node_init_default(&node, "ydlidar_node", "", support);

    rclc_publisher_init_default(
        &pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "/scan"
    );

    rangesCache.resize(totalBins, 0.0f);
    intensitiesCache.resize(totalBins, 0.0f);

    std::memset(&msg, 0, sizeof(msg));
    msg.header.frame_id.data = (char*)"laser_frame";
    msg.header.frame_id.size = std::strlen("laser_frame");
    msg.header.frame_id.capacity = msg.header.frame_id.size + 1;

    lidarDriver = new YDLIDAR::YDLidar_Driver(TargetLidarDevice, TargetLidarConfig);
    if (lidarDriver->Init() != YDLIDAR::LidarError_t::LIDAR_OK) {
        ESP_LOGE(TAG, "Failed to build UART serial connection to YDLidar hardware peripheral map target pipeline.");
    }
}

void YdLidarNode::flushScanMessage() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    msg.header.stamp.sec = ts.tv_sec;
    msg.header.stamp.nanosec = ts.tv_nsec;

    msg.angle_min = 0.0f;
    msg.angle_max = static_cast<float>(2.0 * M_PI);
    msg.angle_increment = msg.angle_max / static_cast<float>(totalBins);
    msg.time_increment = 0.0f;
    msg.scan_time = 0.1f; // Target 10Hz base velocity
    msg.range_min = 0.15f;
    msg.range_max = 12.0f;

    msg.ranges.size = totalBins;
    msg.ranges.data = rangesCache.data();

    if (TargetLidarConfig.hasIntensity) {
        msg.intensities.size = totalBins;
        msg.intensities.data = intensitiesCache.data();
    } else {
        msg.intensities.size = 0;
        msg.intensities.data = nullptr;
    }

    rcl_publish(&pub, &msg, NULL);

    // Clear buffer vectors for subsequent sweeping cycles
    std::fill(rangesCache.begin(), rangesCache.end(), 0.0f);
    std::fill(intensitiesCache.begin(), intensitiesCache.end(), 0.0f);
}

void YdLidarNode::spin() {
    std::vector<YDLIDAR::PointData> pointChunk;
    bool isFullTurnCompleted = false;

    while (true) {
        pointChunk.clear();
        YDLIDAR::LidarError_t res = lidarDriver->PollScanChunk(pointChunk, isFullTurnCompleted);

        if (res == YDLIDAR::LidarError_t::LIDAR_OK && !pointChunk.empty()) {
            for (const auto& pt : pointChunk) {
                // Determine layout indexes relative to global angle arrays
                int index = static_cast<int>((pt.angle_rad / (2.0f * M_PI)) * totalBins);
                if (index >= 0 && index < totalBins) {
                    rangesCache[index] = pt.distance_m;
                    if (TargetLidarConfig.hasIntensity) {
                        intensitiesCache[index] = pt.intensity;
                    }
                }
            }
        }

        if (isFullTurnCompleted) {
            flushScanMessage();
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Context-switching optimization gap
    }
}