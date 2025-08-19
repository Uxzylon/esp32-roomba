# ESP32 Roomba

ESP32 Controlled iRobot Roomba using iRobot® Create® 2 Open Interface

## Features

- Control Roomba using ESP32 over WiFi with websocket
- Retrieve sensor data from Roomba
- Control Roomba using Web Interface
- Video streaming from ESP32 camera

## Requirements

- ESP32 (I used ESP32-CAM OV5640 REV1.2.1)
- Compatible Roomba (I used Roomba 564)
- Voltage Regulator (To use unregulated battery voltage from Roomba to 5V for ESP32)
- PlatformIO

## Wiring

![Roomba Pinouts](images/roomba-pinout.jpg)

- Connect Roomba BRC (Pin 5) to ESP32 (Pin 19)
- Connect Roomba RXD (Pin 3) to ESP32 (Pin 16)
- Connect Roomba TXD (Pin 4) to ESP32 (Pin 17)
- Connect Roomba GND (Pin 6 or 7) to Voltage Regulator Input GND and Output GND to ESP32 GND
- Connect Roomba Vpwr (Pin 1 or 2) to Voltage Regulator Input Vin and Output Vout to ESP32 5V

For video streaming, use an ESP32 board with camera support (such as OV2640 or OV5640). You can solder wires directly to the underside of the Roomba's serial port and mount the ESP32 beneath the front side of the Roomba motherboard. Connect the camera to the ESP32 using a long ribbon cable. To install the camera, secure it at the front of the Roomba behind the bumper, and drill a small hole in the lower part of the bumper to allow the camera lens to capture video.

## Configuration

Create a config.h file in the src directory and add your WiFi SSID and Password:

```cpp
#ifndef CONFIG_H
#define CONFIG_H

const char* ssid = "YourSSID";
const char* password = "YourPassword";

#endif // CONFIG_H
```

## Installation

Use PlatformIO on Visual Studio Code to install the required libraries and upload the code to your ESP32.

1. **First Time Setup**
    - Select your ESP32 and update the `platformio.ini` file if necessary

2. **Build the Project**
    - Click the checkmark (**Build**) in the PlatformIO toolbar or run `PlatformIO: Build` from the command palette.

3. **Build Filesystem Image**
    - In the PlatformIO sidebar, expand **Project Tasks** under your project and select your ESP32 platform. Click on **Build Filesystem Image** to generate the image or run `PlatformIO: Build Filesystem Image` from the command palette.

4. **Upload the Firmware**
    - Click the right arrow (**Upload**) in the PlatformIO toolbar or run `PlatformIO: Upload`.

5. **Upload Filesystem Image**
    - In the PlatformIO sidebar, under **Project Tasks**, select your ESP32 platform and click on **Upload Filesystem Image** to upload the generated filesystem image or run `PlatformIO: Upload Filesystem Image` from the command palette.

6. **Monitor Serial Output (Optional)**
    - Click the plug icon (**Monitor**) or run `PlatformIO: Monitor` to view ESP32 logs.

Your ESP32 should now be running the Roomba control firmware with the required filesystem image.

## Usage

Connect to the ESP32 WiFi network, open a web browser and navigate to `http://<ESP32_IP_ADDRESS>` to access the Roomba control interface.

Press the Connect button to establish a connection to the ESP32 websocket and camera feed. Then press the Wakeup button to wake up the Roomba.
