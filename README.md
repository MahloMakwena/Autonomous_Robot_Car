# Autonomous_Robot_Car
### Autonomous Path-Tracking Robot (RPLIDAR C1)

### Project Overview

This project focuses on designing and programming an **autonomous robotic vehicle** capable of dynamic path tracking, barrier mapping, and high-speed obstacle avoidance. Built on the **ESP32-S3 microcontroller architecture**, the robot uses real-time laser scanning data to safely navigate complex tracks without human intervention. 

###  How It Works

### 1. Environmental Mapping (The "Eyes")

* **Component:** Slamtec RPLIDAR C1 (2D DTOF Laser Scanner)
* **Execution:** The LiDAR emits thousands of laser pulses per second in a continuous **360-degree flat sweep**. It creates a real-time 2D coordinates map of track boundaries, walls, and upcoming curves up to 12 metres away.

### 2. Intelligent Path Decision (The "Brain")

* **Component:** ESP32-S3 Dual-Core Processor
* **Execution:** The processor ingests the live LiDAR data stream to calculate clear pathways ("gap finding"). Using an optimized navigation algorithm, the software dynamically adjusts steering vectors to guide the vehicle along the safest, fastest racing line.

### 3. Fail-Safe Emergency Cutoff (The "Shield")

* **Component:** 3-Button Hardware Latching Circuit & Interrupt Watchdog [disclaimer]
* **Execution:** For ultimate physical safety, the robot implements a true **hardware-level E-Stop loop**. Pressing the manual E-stop button breaks a self-holding relay circuit to instantly cut 100% of electrical power to the drive motors. Simultaneously, an isolated **3.3V logic signal** triggers a high-priority hardware interrupt on the ESP32-S3, forcing the software into a permanent safe halt mode until an engineer performs a manual system reset.

###  Tech Stack & Tools

* **Programming Language:** C++ (Embedded)
* **Framework:** Arduino IDE (ESP32 Arduino Core)
* **Hardware Design & Simulation:** Cirkit Designer

###  System Simulation & Architecture

To ensure high-fidelity safety testing before deploying code onto physical hardware, the system architecture was completely mapped out and simulated using **Cirkit Designer**: 

1. **The Latching Safety Loop:** We simulated a self-holding hardware latch circuit using standard DC relay modules, a Normally Closed (NC) button, and a Normally Open (NO) button. This layout guarantees power is cut mechanically to the motors during an emergency, entirely bypassing the processor.
2. **Logic Monitoring:** An isolated 3.3V logic line is mapped from the ESP32-S3 through a separate signal switch to GPIO 4, stabilized by a physical 10kΩ pull-down resistor to prevent pin floatation.
3. **Firmware Integration:** The Arduino sketch utilizes the Xtensa dual-core architecture of the ESP32-S3 to register a high-priority hardware interrupt (IRAM_ATTR). The software instantly drops the vehicle into a permanent safe state the exact millisecond a hardware shutdown occurs.

### ⚙️ Repository Structure

* **/src**: Core C++ firmware for the ESP32-S3, managing the hardware interrupt loops, LiDAR data parsing, and path-tracking logic.
* **/schematics**: Complete visual wiring blueprints and architecture maps designed in **Cirkit Designer**.