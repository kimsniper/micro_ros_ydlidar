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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "ydlidar_node"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

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

    static const char frame[] = "laser_frame";
    msg.header.frame_id.data = const_cast<char*>(frame);
    msg.header.frame_id.size = strlen(frame);
    msg.header.frame_id.capacity = sizeof(frame);

    lidarDriver = new YDLIDAR::YDLidar_Driver(TargetLidarDevice, TargetLidarConfig);
    if (lidarDriver->Init() != YDLIDAR::LidarError_t::LIDAR_OK) {
        ESP_LOGE(TAG, "Failed to initialize YDLidar driver");
    }

    scanReady = false;
    lastPublishMs = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void YdLidarNode::flushScanMessage() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    msg.header.stamp.sec = ts.tv_sec;
    msg.header.stamp.nanosec = ts.tv_nsec;

    msg.angle_min = 0.0f;
    msg.angle_max = 2.0f * M_PI;
    msg.angle_increment = msg.angle_max / totalBins;

    msg.time_increment = 0.0f;
    msg.scan_time = 0.1f;

    msg.range_min = 0.15f;
    msg.range_max = 12.0f;

    msg.ranges.size = totalBins;
    msg.ranges.capacity = totalBins;
    msg.ranges.data = rangesCache.data();

    msg.intensities.size = TargetLidarConfig.hasIntensity ? totalBins : 0;
    msg.intensities.capacity = totalBins;
    msg.intensities.data = TargetLidarConfig.hasIntensity ? intensitiesCache.data() : nullptr;

    rcl_ret_t rc = rcl_publish(&pub, &msg, NULL);
    (void)rc;

    std::fill(rangesCache.begin(), rangesCache.end(), 0.0f);
    std::fill(intensitiesCache.begin(), intensitiesCache.end(), 0.0f);
}

void YdLidarNode::spin() {
    std::vector<YDLIDAR::PointData> pointChunk;
    bool isFullTurnCompleted = false;
    const uint32_t PUBLISH_PERIOD_MS = 100;

    while (true) {
        pointChunk.clear();
        isFullTurnCompleted = false;

        auto res = lidarDriver->PollScanChunk(pointChunk, isFullTurnCompleted);

        if (res == YDLIDAR::LidarError_t::LIDAR_OK && !pointChunk.empty()) {
            for (const auto& pt : pointChunk) {

                int index = static_cast<int>(
                    (pt.angle_rad / (2.0f * M_PI)) * totalBins
                );

                if (index >= 0 && index < totalBins) {
                    rangesCache[index] = pt.distance_m;
                    if (TargetLidarConfig.hasIntensity) {
                        intensitiesCache[index] = pt.intensity;
                    }
                }
            }

            scanReady = isFullTurnCompleted;
        }

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (scanReady && (now - lastPublishMs >= PUBLISH_PERIOD_MS)) {
            scanReady = false;
            lastPublishMs = now;
            flushScanMessage();
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}