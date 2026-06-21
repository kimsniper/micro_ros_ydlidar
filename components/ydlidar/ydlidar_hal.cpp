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

#include "ydlidar_hal.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace YDLIDAR {

LidarError_t ydlidar_hal_init(const Device* dev) {
    if (!dev) return LidarError_t::LIDAR_ERR;

    uart_config_t uart_config = {};
    uart_config.baud_rate = dev->baudRate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    if (uart_driver_install((uart_port_t)dev->uartPort, 2048, 0, 0, nullptr, 0) != ESP_OK)
        return LidarError_t::LIDAR_ERR;

    if (uart_param_config((uart_port_t)dev->uartPort, &uart_config) != ESP_OK)
        return LidarError_t::LIDAR_ERR;

    if (uart_set_pin((uart_port_t)dev->uartPort,
                     dev->txPin,
                     dev->rxPin,
                     UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE) != ESP_OK)
        return LidarError_t::LIDAR_ERR;

    return LidarError_t::LIDAR_OK;
}

LidarError_t ydlidar_hal_read_byte(uint8_t uartPort, uint8_t* byteBuffer, uint32_t timeoutMs) {
    if (!byteBuffer) return LidarError_t::LIDAR_ERR;

    int len = uart_read_bytes((uart_port_t)uartPort,
                              byteBuffer,
                              1,
                              pdMS_TO_TICKS(timeoutMs));

    if (len <= 0)
        return LidarError_t::LIDAR_ERR_TIMEOUT;

    return LidarError_t::LIDAR_OK;
}

void ydlidar_hal_ms_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

} // namespace YDLIDAR