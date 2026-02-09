# STM32 Battery HIL Simulator

I built this to test Battery Management System (BMS) logic without actually having to wire up (and potentially blow up) real Li-Ion cells. It’s a **Hardware-in-the-Loop (HIL)** setup where an STM32 simulates battery physics in real-time, and a Python dashboard handles the telemetry and control.

## What it does

Basically, the STM32 "pretends" to be a battery. It calculates voltage state based on charge/discharge math, and you can mess with it in real-time using a PC dashboard.

* **Real-time Physics:** It’s not just static numbers. The voltage moves realistically based on the load.
* **Fault Injection:** You can trigger a "short circuit" from the dashboard to see if the firmware catches the drop and triggers an alarm fast enough.
* **Filtering:** I added a moving average filter on the firmware side to handle the "sensor noise" simulated in the model.

## The Setup

* **Hardware:** STM32 Nucleo-L476RG
* **RTOS:** FreeRTOS (I split the physics, filtering, and UART comms into separate tasks).
* **Dashboard:** Python (Matplotlib for the live graphs and PySerial for the link).

## How it's structured

The firmware uses three main tasks to keep things responsive:

1. **Simulation Task (High Priority):** Handles the actual battery model (linear charge, discharge, or sine waves).
2. **Filter Task:** Smooths out the raw data using a moving average.
3. **UART Task:** Sends the telemetry strings (e.g., `Bat: 3.72 V`) to the PC.

I used a UART RX interrupt for the control commands ('C', 'D', 'X', etc.) so the simulation reacts instantly the moment you hit a key.

## Running it

1. Flash the firmware from `/Firmware` to the Nucleo.
2. Install the Python requirements: `pip install pyserial matplotlib`.
3. Run `python dashboard.py`.

**Controls:**

* `C` / `D` : Charge or Discharge.
* `X` : Trigger a fault (Short circuit).
* `E` : Endurance test (slow drain).
* `N` : Sine wave mode (mostly to test the filter).
