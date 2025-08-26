#  LoRa Based Electric Distribution Line Monitoring System  

A real-time **grid monitoring and fault detection system** utilizing **ESP32**, **Current Transformers (CTs)**, **Voltage sensors**, and **LoRa communication** for pole-to-substation data transfer.  
The system integrates with **MQTT** and a **Web GUI Dashboard** for monitoring, fault visualization, and system restoration.  

Built as part of a Hardware hackathon to address **automated detection of fallen power lines and distribution line faults**, this project demonstrates a **safe scale model (12V)** that replicates real-world conditions of power distribution monitoring.

---

## Problem Statement
**Automated Detection System for Fallen Power Lines**  
- **Scenario:** Storms, falling trees, and monsoon floods cause live power lines to snap and fall, creating silent hazards for pedestrians and vehicles. These incidents often remain undetected for hours.  
- **Challenge:** Existing protection systems mainly cover **transmission lines (3-phase, high-voltage)** but are **absent in local distribution networks** (230V single-phase).  
- **Objective:** Build a **distributed IoT system** to:  
  - Detect **line-to-line** and **line-to-ground faults** automatically.  
  - Detect **line breakages** and **power restoration** events.  
  - Provide **real-time alerts** to utilities and public dashboards.  

---

## Proposed Solution
- **Substation Node (ESP32 + CTs + Relays):**  
  - Continuously monitors live and neutral currents.  
  - Detects **line-to-line** and **line-to-ground faults**.  
  - Trips the breaker (relay) during faults.  
  - Sends JSON-formatted MQTT data to the Web Dashboard.  

- **Pole Node (ESP32 + CT + Voltage Sensor + LoRa):**  
  - Measures **current and voltage** at the pole.  
  - Detects **line breakages**, **reconnections**, and confirms **substation trips**.  
  - Reports status via LoRa to the Substation node.  

- **Web GUI Dashboard:**  
  - Displays a **map with substation and poles**.  
  - Shows real-time **alerts**, **ANSI fault codes**, breaker status, and device health.  
  - Provides reset controls via MQTT.  
  - Designed with an **industry-grade look and feel**.  

---

## Hardware Components
- **Microcontrollers**: ESP32 NodeMCU (x2)  
- **Communication**: E32-900T30D LoRa Modules  
- **Sensors**:  
  - CT current sensors (ZMCT103C)  
  - AC voltage sensors (ZMPT101B)  
- **Protection & Control**: Relay Module, Step-down Transformer (230V to 12V for demo)  
- **Miscellaneous**: Resistors, wiring, poles (scale model), power supply  

---

## Software Stack
- **ESP32 Firmware:** Arduino framework (C++)  
- **Protocols:** LoRa (pole → substation), MQTT (substation → Web GUI)  
- **Data Format:** JSON messages  
- **Web Dashboard:** Custom frontend (HTML/JS/React) + MQTT integration  

---

