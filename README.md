# Autonomous Delivery Bot

![Project Status](https://img.shields.io/badge/status-prototype-brightgreen)
![Platform](https://img.shields.io/badge/platform-ESP32-blue)
![Language](https://img.shields.io/badge/language-C%2B%2B-orange)

## Overview
This project is an **Autonomous Line-Following Delivery Bot** built using an ESP32 microcontroller. The robot is designed to navigate a predefined path using an array of IR sensors with **PID control** for smooth line following. It utilizes **RFID technology** to identify key waypoints (such as corners and delivery tables) and can be controlled and tuned remotely via a **Mobile Serial Monitor app** over Bluetooth. 

This project was developed alongside a research paper detailing the design, implementation, and routing algorithm of the bot.

## 📸 Media & Demos

*Note: Add your prototype photos, map photo, demo video, and research paper into the `assets` folder and link them below!*

- **Prototype Photo:** `![Prototype](assets/prototype.jpg)`
- **Prototype Photo:** `![Prototype](assets/prototype2.jpg)`
- **Path/Map Photo:** `![Map](assets/map.jpg)`
- **Demo Video:** `[Watch Demo Video](assets/demo_video.mp4)`
- **Research Paper:** `[Read the Paper](assets/research_paper.pdf)`

## ✨ Features
- **PID Line Following:** Uses a 7-sensor IR array to calculate weighted error and smoothly follow a line using Proportional-Integral-Derivative control.
- **RFID Waypoint Navigation:** Detects specific MFRC522 RFID tags placed on the track to recognize table locations and corners, making navigation decisions autonomously.
- **Bluetooth Control & Tuning:** Connects to a Mobile Serial Monitor app (device name: `PID_TUNING`) to receive routing commands (Table 1, Table 2, Stop) and dynamically tune PID constants (`Kp`, `Ki`, `Kd`) on the fly.
- **State Machine Architecture:** Employs a robust non-blocking state machine for turning, path correction, and delivery stops.

## 🛠️ Hardware Requirements
- **Microcontroller:** ESP32 (due to built-in Bluetooth and ample GPIOs)
- **Motor Driver:** L298N (or similar) to control left and right DC motors
- **Line Sensor:** 7-channel IR sensor array
- **RFID Reader:** MFRC522 module
- **Chassis:** Differential drive chassis with 2 DC motors and caster wheel(s)
- **Power Supply:** Appropriate battery pack (e.g., 2S or 3S LiPo)

## 📌 Pin Configuration
Based on the current ESP32 setup:
| Component | ESP32 Pin |
|-----------|-----------|
| IR Sensors | 12, 13, 14, 15, 16, 17, 18 (Sequential starting at GPIO 12) |
| Left Motor PWM | 11 |
| Right Motor PWM | 23 |
| Left Motor IN1 | 12 |
| Left Motor IN2 | 13 |
| Right Motor IN3 | 18 |
| Right Motor IN4 | 19 |
| RFID SS / SDA | 21 |
| RFID RST | 22 |
| SPI Pins | Default ESP32 VSPI (SCK=18, MISO=19, MOSI=23)* |

*(Note: Double check overlapping pins in your specific hardware layout, as `main.cpp` reuses some pins like 12, 13, 18, 19 between IR, Motors, and SPI depending on your exact board variant. Please adapt if necessary.)*

## 📱 Mobile App Control
To control the bot, download a standard **Serial Bluetooth Terminal** app on your phone.
1. Pair your phone with the ESP32 (Device Name: `PID_TUNING`).
2. Open the serial app and connect.
3. Send the following commands:
   - `1`: Go to Table 1
   - `2`: Go to Table 2
   - `3`: Stop the Bot
   - `q`/`a`: Increase/Decrease `Kp`
   - `w`/`s`: Increase/Decrease `Kd`
   - `e`/`d`: Increase/Decrease `Ki`

## 🚀 Setup & Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/autonomous-delivery-bot.git
   ```
2. Open the `Source_code` folder in **Arduino IDE** or **PlatformIO**.
3. Install the required libraries via the Library Manager:
   - `MFRC522` by GithubCommunity
4. Select your ESP32 board and compile/upload `main.cpp` to the microcontroller.
5. Place the RFID tags at the designated intersections and update the UID strings in the code (`TABLE1_CORNER_UID`, etc.) if needed.

## 📄 License
This project is open-source and available under the MIT License.
