# PWM-Based LED and Fan Control System

## Project Overview
This project demonstrates a microcontroller-based speed and device control system using the LPC2148 ARM7 MCU. The system adjusts LED brightness and fan speed using Pulse Width Modulation (PWM) based on user input through UART. It features real-time feedback via a 16x2 LCD display and UART messages, simulating a zone-based speed control or home automation scenario.

## Features
- **PWM Control:** Adjusts LED brightness or fan speed from 0% to 100%.
- **Zone Selection:** User can select predefined zones (School, City, Highway) to control speed/fan duty cycle.
- **LCD Display:** Displays system status, zone selection, and duty cycle.
- **UART Interface:** Provides user input and echo feedback for real-time control.
- **Embedded Development:** Direct register-level control using LPC2148 headers for PWM, UART, and GPIO.

## Hardware Requirements
- LPC2148 ARM7 Microcontroller
- LED or Fan module
- 16x2 LCD
- UART interface (e.g., USB-to-Serial)
- Jumper wires and breadboard or PCB setup
- Power supply (3.3V/5V depending on peripherals)

## Software Requirements
- IAR Embedded Workbench or Keil uVision
- LPC214x header files
- Basic understanding of embedded C programming

## Usage
1. Flash the firmware to LPC2148 MCU using IAR/Keil.
2. Connect LED/fan to P0.8 (PWM output).
3. Connect UART for serial input (P0.0 TX, P0.1 RX).
4. Power up the system; the LCD will display `System Ready`.
5. Input one of the following via UART to control PWM:
   - `S` / `s` → School Zone (30% duty)
   - `C` / `c` → City Zone (60% duty)
   - `H` / `h` → Highway Zone (90% duty)
6. Observe LED brightness or fan speed change accordingly, with LCD feedback.

## Code Structure
- `main.c` – Main control logic including UART input, LCD display, and PWM control.
- `pwm.c/h` – PWM initialization and duty cycle functions.
- `lcd.c/h` – LCD control functions (4-bit mode).
- `uart.c/h` – UART initialization and communication functions.

## Key Learning Points
- Register-level PWM configuration on LPC2148.
- Real-time control and feedback using UART and LCD.
- Integration of hardware peripherals for embedded system applications.
- Application of PWM in controlling devices like LEDs and fans.

## Author
**Semalign Markos** – Aspiring Electronic Communication Engineer  
GitHub:https://github.com/Semalign 

## License
This project is licensed under the MIT License.

