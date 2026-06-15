# micro_ros_ydlidar

A robust, object-oriented Micro-ROS subsystem for ESP-IDF designed to interface directly with YDLidar laser scanners. It processes raw lidar point streams in real-time across a dedicated FreeRTOS task to publish standard ROS 2 `sensor_msgs/msg/LaserScan` messages over UDP.

---

## Overview

This project provides a clean C++ implementation for:

-   **YDLidar (Triangle/TOF Models)** integration via a high-performance streaming UART serial state machine parser.
-   **Level-2 Geometric Error Correction** to handle trigonometric variations inherent in triangulation lidar models.
-   **Micro-ROS on ESP-IDF** natively running an asynchronous laser scan node.
-   **ROS 2 Compatibility** publishing standard laser scans directly over network transport layers.

---

## Features

-   **Byte-by-Byte Hardware State Machine:** A non-blocking streaming parser that validates 16-bit XOR packet checksums on the fly.
-   **Angle Bin Transformation:** Seamlessly interpolates incoming data chunks and aggregates them into a synchronized, stable 360° `sensor_msgs/msg/LaserScan` matrix.
-   **Trigonometric Range Correction:** Implements internal correction equations to counteract triangulation hardware offsets at varying distances.
-   **Configurable Resolution:** Easily maps raw points to distinct angular bins (e.g., 720 bins for 0.5° resolution).

---

## Hardware Requirements

-   ESP32 / ESP32-S3 / ESP32-C3 development board
-   YDLidar Sensor (e.g., X4, G4, or similar TOF/Triangulation variants)
-   Dedicated external 5V power distribution (highly recommended as lidar motor draw can brown out MCU rails)

### Wiring (Default Pin Mapping Example)

| YDLidar Signal | ESP32 GPIO (Typical) | Notes |
| :--- | :--- | :--- |
| **TX** | GPIO 21 | Connects to ESP RX |
| **RX** | GPIO 22 | Connects to ESP TX |
| **VCC / M_EN** | 5V / 3.3V | External power rails recommended |
| **GND** | GND | Must share a common ground with ESP32 |

---

## Micro-ROS Architecture

The ESP32 firmware initializes a single functional wrapper node instance:

* **Transport Layer:** Native UDP client connection binding over standard 802.11 b/g/n WiFi.
* **Published Topic:** * `/scan` (`sensor_msgs/msg/LaserScan`): Broadcasts complete, structured planar laser range-finding arrays.

---

## Setup Instructions

### 1. Clone Project

``` bash
cd ~
git clone --recursive [https://github.com/kimsniper/micro_ros_ydlidar.git](https://github.com/kimsniper/micro_ros_ydlidar.git)
cd ~/micro_ros_ydlidar
```

------------------------------------------------------------------------

### 2. Install ESP-IDF

Follow official ESP-IDF setup guide:
https://docs.espressif.com/projects/esp-idf/en/latest/

------------------------------------------------------------------------

### 3. Set Target

``` bash
idf.py set-target esp32
# or esp32s3 / esp32c3 depending on board
```

------------------------------------------------------------------------

### 4. Configure Project

``` bash
idf.py menuconfig
```

------------------------------------------------------------------------

## Micro-ROS Configuration (menuconfig)

### Micro-ROS YDLidar Setting

    Menu: Micro-ROS YDLidar Setting

-   UART Port Number: 1
-   UART TX GPIO: Pin 22
-   UART RX GPIO: Pin 21
-   Lidar Baud Rate: 128000 (or 115200/230400 depending on your specific lidar model)

------------------------------------------------------------------------

## micro-ROS Network Settings

Inside menuconfig:

    Micro-ROS Settings → WiFi Configuration

Set:

-   WiFi SSID
-   WiFi Password
-   micro-ROS Agent IP (PC running ROS2)
-   micro-ROS Agent Port → 8888

Example:

    Agent IP: 192.168.1.10
    Port: 8888

------------------------------------------------------------------------

## Build & Flash

``` bash
idf.py build
idf.py flash
idf.py monitor
```

------------------------------------------------------------------------

## Run micro-ROS Agent

Before running ESP32, start the agent:

``` bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

------------------------------------------------------------------------

## ROS 2 Usage

After successful connection:

### Check topic

``` bash
ros2 topic list
```

### Monitor the 2D Laser Scan range-finding array

``` bash
ros2 topic echo /scan
```

------------------------------------------------------------------------

## Output Data

Published message:

sensor_msgs/msg/LaserScan

-   Ranges: An array filled with exactly 720 bins mapping an entire 360° field of view (0.5° angular resolution). Distance measurements are provided in meters $[\text{m}]$
-   Intensities: Optional array containing signal return amplitudes (returns empty if the chosen Lidar model configuration has intensity tracking disabled).
-   Scan Parameters: -   Fixed boundary metrics tracking from $0$ to $2\pi$ radians at a target refresh velocity of 10Hz.

------------------------------------------------------------------------

## Notes

Lidar Power Supply: Lasers and high-speed spin motors generate noticeable current spikes. If the sensor drops packets or brown-outs occur, route an isolated external 5V power supply directly to the Lidar module while maintaining a common ground link with the ESP32.

------------------------------------------------------------------------

## Troubleshooting

### 1. No Data Received on Host Machine (`ros2 topic echo /scan` is blank)
* **Check the micro-ROS Agent:** Ensure the Docker agent container is actively running on the host PC and listening on the correct UDP port (`8888`). You should see connection logs (`Session established...`) when the ESP32 boots.
* **Verify IP/Network Configurations:** Double-check that your ESP32 and host PC are on the *exact same* local WiFi subnet. Verify the `micro-ROS Agent IP` inside `menuconfig`.
* **Monitor ESP32 Serial Output:** Run `idf.py monitor`. If you see `"Failed to build UART serial connection..."`, the ESP32 cannot open the hardware port. If it panics or resets, check your task stack allocations.

### 2. Missing Packets or Continuous Checksum Errors
* **Baud Rate Mismatch:** Different YDLidar models use distinct default baud rates (e.g., X4 uses `128000`, while others might use `115200` or `230400`). Ensure your `TargetLidarDevice.baudRate` matches your specific hardware datasheet.
* **Lidar Power Instability (Most Common):** The spinning motor draws high current spikes during startup and continuous sweeping. If powered solely via the ESP32’s 5V/3.3V development board pins, the UART transmission will glitch, causing continuous `LIDAR_ERR_CHECKSUM` flags. 
    * *Solution:* Use an external, stabilized 5V power supply for the Lidar motor and ensure its ground (`GND`) is tied directly to the ESP32 `GND`.

### 3. Laser Scan Visualization Looks Distorted or Shifted
* **Incorrect Model Type Flag:** Triangulation models (`TRIANGLE_LIDAR`) process raw distances through an inverse trigonometric formula to yield correct Cartesian spacing, whereas Time-of-Flight (`TOF_LIDAR`) models do not. Ensure `TargetLidarConfig.model` is set correctly.
* **TX/RX Pin Inversion:** Ensure your physical wires cross over properly: ESP32 TX pin connects to YDLidar RX, and ESP32 RX pin connects to YDLidar TX.

### 4. ESP32 FreeRTOS Task Crashes (Watchdog Reset / Stack Overflow)
* **Increase Stack Sizes:** The `LaserScan` message handles large heap/stack arrays to cache the 720 tracking bins. If you experience random crashes when a full turn completes, increase the `CONFIG_MICRO_ROS_APP_STACK` allocation inside `menuconfig` to `20000` or higher.
* **WDT (Watchdog Timer) Starvation:** The state machine processes raw bytes sequentially. If the loop blocks context shifting, ensure the `vTaskDelay(pdMS_TO_TICKS(1))` optimization gap is active inside your `spin()` method to yield time back to the FreeRTOS scheduler.

------------------------------------------------------------------------

## Integrate ROS2 YDLidar Demo

------------------------------------------------------------------------

## License

BSD 3-Clause License

Copyright (c) 2026 Mezael Docoy
