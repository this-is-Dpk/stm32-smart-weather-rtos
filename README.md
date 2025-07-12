# stm32-smart-weather-rtos
# 🌤️ Smart Weather Monitoring System (STM32 + FreeRTOS)

This is a simulation-based embedded systems project built on **STM32 microcontroller** using **FreeRTOS**. The project is designed to mimic a smart weather monitoring system that measures temperature, pressure, and humidity, logs the data, and handles emergency events — all using a real-time operating system (RTOS) architecture.

⚠️ **Note:** This project does **not use real sensors**. All sensor readings are generated using **static arrays** in the firmware to simulate real-world behavior. This approach is used to test task synchronization, inter-process communication, and cloud packet handling in a controlled embedded RTOS environment.

---

## 🚀 Project Summary

This project demonstrates:

- 📊 Simulated temperature, pressure, and humidity sensor data
- 🔁 Periodic cloud update simulation every 10 seconds
- 📬 Parsing and responding to mock process codes (like “P01”, “P05”) via queue
- 🚨 Emergency handling via external GPIO interrupt (EXTI)
- 🧵 RTOS task management with semaphores, mutexes, and queues
- 🔍 Stack monitoring using **SEGGER SystemView**
- 🖥️ UART CLI input: `'L'` for Live mode, `'S'` for Silent mode

---

## 🛠️ Technologies & Concepts Used

- **MCU**: STM32F103C8T6 (Blackpill)
- **RTOS**: FreeRTOS (via STM32CubeMX)
- **IDE**: STM32CubeIDE
- **Debugger**: ST-Link v2
- **Task Prioritization**: Emergency task has highest priority
- **Synchronization Primitives**:
  - `SemaphoreHandle_t` for task sync
  - `QueueHandle_t` for response code processing
  - `Mutex` for UART access
- **Interrupt Handling**: EXTI-based emergency input
- **UART CLI**: Interrupt-based reception and command processing
- **SEGGER SystemView**:
  - Integrated for runtime diagnostics
  - Logs stack usage and heartbeat from all critical tasks

---

## 📁 Project Folder Highlights

