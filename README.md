# STM32 Hardware-in-the-Loop (HIL) Battery Simulator

**A real-time HIL test bench simulating a Li-Ion battery management system (BMS).**
The system uses an STM32L4 microcontroller to model battery physics and a Python dashboard for telemetry and fault injection.

---

## 🎯 Project Overview
This project demonstrates **Embedded Systems** and **HIL Testing** concepts by simulating a battery that doesn't physically exist. The STM32 computes the voltage based on mathematical models (Charge, Discharge, Drain), while a PC dashboard acts as the external tester.

* **Hardware:** STM32 Nucleo-L476RG
* **Communication:** UART (115200 Baud)
* **RTOS:** FreeRTOS (Tasks for Simulation, Filtering, and Comms)
* **Software:** Python (Matplotlib, PySerial)

---

## 📸 Demo & Test Scenarios

### 1. Closed-Loop Control (Charge/Discharge)
*The PC sends 'C' (Charge) and 'D' (Discharge) commands. The firmware intercepts these via UART Interrupts and adjusts the internal voltage model.*
![Insert GIF of Charging/Discharging here]

### 2. Safety & Fault Injection
*Simulating a critical "Cut Wire" failure by sending command 'X'. The firmware detects the 0V condition and triggers the Hardware Alarm (LED) within 10ms.*
![Insert GIF of Fault Injection here]

### 3. Endurance Simulation
*A realistic discharge curve modeling a constant load over time.*
![Insert Image of Slow Drain here]

---

## 🛠️ Technical Architecture

### Firmware (STM32)
* **Task_Sim (High Priority):** Runs the physics model (Sine wave, Linear charge, Instant fault).
* **Task_Filter (Normal Priority):** Implements a Moving Average Filter to smooth sensor noise.
* **Task_UART (Low Priority):** Transmits telemetry data (`Bat: 3.72 V`) to the PC.
* **Interrupts:** UART RX interrupt handles incoming control commands asynchronously.

### Software (Python)
* Real-time plotting using `matplotlib.animation`.
* Thread-safe serial parsing using `pyserial`.
* Keyboard event listeners for user control.

---

## 🚀 How to Run
1.  Flash the firmware located in `/Firmware` to a Nucleo-L476RG.
2.  Install Python dependencies: `pip install pyserial matplotlib`
3.  Run the dashboard: `python dashboard.py`
4.  Controls:
    * **C**: Charge
    * **D**: Discharge
    * **X**: Short Circuit (Fault)
    * **E**: Endurance Test
    * **N**: Sine Wave Mode
