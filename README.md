# Reactive Environment Controller

The **Reactive Environment Controller** is an efficient microcontroller firmware project developed for **Infineon PSoC boards** using **ModusToolbox within Visual Studio Code**. The system achieves non-blocking concurrency by coordinating ADC sampling, PWM generation, and UART command parsing within a lean, interrupt-driven superloop architecture.

## Features

At the core of the firmware is a responsive, non-blocking control loop that coordinates multiple peripherals simultaneously:
* **Analog Sampling:** Continuously samples an analog sensor at a fixed rate via interrupts, capturing real-time environmental data without halting the CPU.
* **PWM Output Drive:** Dynamically adjusts a Pulse Width Modulation output based on live sensor readings to regulate ambient conditions instantly.
* **UART Command Parsing:** Accepts asynchronous configuration commands over serial communication, using robust circular buffering to process incoming data streams without missing bytes or dropping packets.
* **Non-Blocking Architecture:** Juggles four distinct peripherals concurrently within a lean interrupt-driven superloop, preventing latency bottlenecks and showcasing efficient embedded resource management.

## Tech Stack & Tools

* **Hardware:** Infineon PSoC 6 Board
* **IDE / Toolchain:** ModusToolbox integrated within Visual Studio Code (VS Code)
* **Language:** C / C++

## Getting Started

1. **Clone the Repository:**
   ```bash
   git clone [https://github.com/AnishRao2006/Reactive-Environment-Controller.git](https://github.com/AnishRao2006/Reactive-Environment-Controller.git)

2. **Open in ModusToolbox / VS Code:**
   * Launch VS Code and open the cloned project workspace (LED_pattern_player).
   * Ensure the ModusToolbox extensions and build tools are properly configured in your environment.

3. **Build and Program:**
   * Build the project using the ModusToolbox build tools or Makefile.
   * Flash the compiled firmware onto your target PSoC board. add all those features to this markdown code and give it to me.   
