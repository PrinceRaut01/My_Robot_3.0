# Voice Controlled Humanoid Robot using Arduino Mega 2560

This project demonstrates a humanoid robot that responds to offline voice commands and performs expressive movements using multiple servo motors. The system is built around an Arduino Mega 2560, a PCA9685 16‑channel servo driver, and the DFRobot DF2301Q voice recognition module.

The robot animates arms, shoulders, eyes, and mouth based on recognized voice command IDs. GPIO outputs provide visual indicators for each triggered action.

# Hardware Used

* Arduino Mega 2560
* PCA9685 16‑Channel Servo Driver (I2C)
* DFRobot DF2301Q Offline Voice Module
* Servo Motors (Eyes, Mouth, Arms, Shoulders)
* Jumper wires & external 5V servo power supply

# Pin Configuration

**Servo Driver (PCA9685 via I2C)**

* SDA → Mega Pin 20
* SCL → Mega Pin 21
* VCC → 5V
* GND → GND

**Voice Module (UART)**

* Connected to Serial1 on Arduino Mega
* TX → RX1 (19)
* RX → TX1 (18)

**GPIO Indicators**

* m1–m8 connected to pins 2–9

# Features

* Offline voice recognition (no internet required)
* Multi-servo humanoid expressions
* Eye blinking animation
* Talking mouth movement
* Arm waving and gesture loops
* Shoulder rotations
* GPIO-based action indicators

# Voice Commands Mapping

| CMD ID | Action                                     |
| ------ | ------------------------------------------ |
| 5      | Full gesture + talking + blinking sequence |
| 6      | Left arm talking loop                      |
| 7      | Short talking gesture                      |
| 8      | Arm + mouth animation                      |
| 9      | Medium talking loop                        |
| 10     | Short loop gesture                         |
| 11     | Another gesture loop                       |
| 12     | Final gesture loop                         |

# Required Libraries

Install from Arduino Library Manager:

* Adafruit PWM Servo Driver
* DFRobot DF2301Q
* Wire (built-in)

# How It Works

1. Arduino initializes PCA9685 to control servos at 60Hz.
2. DF2301Q listens for trained voice commands.
3. Each recognized CMD ID triggers a unique servo motion routine.
4. Robot performs blinking, arm waving, and mouth movement sequences.
5. GPIO pins toggle LOW to indicate which command was executed.

# Power Notes

* Servos must be powered from an external 5V supply.
* Do NOT power servos directly from Arduino 5V.
* Common ground between Arduino and PCA9685 is required.

# Purpose of Project

* Embedded robotics learning
* Voice-based human interaction
* Servo coordination practice
* Real-time hardware control with Arduino Mega

# Author

Prince Raut
Firmware & Embedded Systems Enthusiast
