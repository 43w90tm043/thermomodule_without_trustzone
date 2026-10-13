# Thermomodule — Embedded Firmware

<img width="1080" height="1003" alt="photo_2026-10-13_18-10-04" src="https://github.com/user-attachments/assets/111e6e5c-69af-45de-a3f9-f8a10d2285f6" />



Custom embedded temperature measurement system featuring a dedicated analog front end, asynchronous peripheral drivers, and a proprietary CAN FD communication protocol.

## Overview

This project is an independently developed embedded system built around the STM32U385CG microcontroller and a custom four-layer PCB.

The firmware is written in C++20 using custom low-level drivers built directly on CMSIS. The software follows a bare-metal architecture: peripheral operations use interrupts and GPDMA where applicable, while higher-level functionality is organized around state machines.

The measurement subsystem uses a four-wire Pt1000 RTD and an ADS1262 precision ADC. A reference resistor is measured through the same ADC and used in the resistance measurement process to reduce sensitivity to excitation-current variations.

The project is under active development. The core measurement and CAN FD data path has been tested on hardware, while the external flash filesystem is still being developed.

## Key Features

* **Custom firmware architecture:** C++20 and bare-metal execution without an RTOS.
* **Low-level peripheral drivers:** Custom drivers implemented on top of CMSIS.
* **Asynchronous operation:** Interrupt-driven peripheral handling and GPDMA transfers where supported.
* **Temperature measurement:** Four-wire Pt1000 measurement using the ADS1262 ADC and a reference-resistor-based measurement approach.
* **Multi-point calibration:** Implemented 16-point calibration algorithm; measurement accuracy has not yet been experimentally verified.
* **CAN FD communication:** Custom driver and proprietary communication protocol, with hardware-tested temperature data transmission.
* **External NOR Flash:** OCTOSPI driver and supporting library implemented for the W25Q512JV.
* **Filesystem development:** Persistent storage for configuration, calibration data, measurement logs, and other application data is planned.

## Hardware Platform

The system is built on a custom four-layer PCB designed with signal integrity, power integrity, and electromagnetic compatibility in mind.

| Component                    | Purpose                                |
| ---------------------------- | -------------------------------------- |
| STM32U385CG                  | Main microcontroller                   |
| ADS1262                      | Precision measurement ADC              |
| Pt1000, four-wire connection | Temperature sensing element            |
| LT3042                       | Low-noise analog supply                |
| TPS62913                     | Buck DC-DC conversion                  |
| ISO1044                      | Galvanically isolated CAN FD interface |
| W25Q512JV                    | External NOR Flash memory              |
| UCC28782                     | Active-clamp flyback controller        |
| LMG2610                      | Power-stage switching device           |
| NCP43080                     | Synchronous rectification controller   |

### Power Architecture

The nominal 48 V input supply, with an operating input range of 38–60 V, feeds an isolated active-clamp flyback converter operating at approximately 1 MHz.

The converter generates a 9 V intermediate rail using synchronous rectification on the secondary side. Downstream power stages generate the 5 V CAN FD supply and 3.3 V digital supply. A separate LT3042-based low-noise 5 V rail supplies the analog measurement circuitry around the ADS1262.

## Firmware Architecture

The firmware uses a bare-metal architecture without an RTOS.

* **Low-level drivers:** Custom C++20 drivers built on CMSIS and the STM32 peripheral registers.
* **Asynchronous peripheral operations:** Interrupts and GPDMA are used where supported by the peripheral and operation.
* **State machines:** Higher-level functionality is organized around explicit state transitions rather than relying on a blocking execution model.
* **Communication:** A custom CAN FD driver and application-level protocol handle temperature data transmission.
* **Storage:** An OCTOSPI driver and supporting library provide access to external NOR Flash.

The architecture aims to keep hardware access separate from higher-level application logic and to support predictable, non-blocking peripheral operation.

## Measurement and Calibration

The measurement subsystem uses a four-wire Pt1000 RTD connected to the ADS1262.

A reference resistor is measured by the same ADC and participates in the resistance measurement process alongside the RTD and excitation-current source.

A 16-point calibration algorithm has been implemented. The target temperature range is **−50 °C to +500 °C**, with an intended measurement accuracy of approximately **±0.001 °C** after calibration.

This accuracy is a design target, not a verified performance specification. Experimental validation, uncertainty analysis, and characterization across the full temperature range remain necessary before the target can be claimed as achieved.

## CAN FD Communication

The firmware includes both a custom low-level CAN FD driver and a proprietary communication protocol.

**Verified functionality:** Temperature data can be transmitted over CAN FD using the custom protocol, and the communication path has been tested on physical hardware.

Protocol details, message formats, identifiers, and timing parameters can be documented separately where appropriate.

## External Flash and Filesystem

The W25Q512JV external NOR Flash is accessed through a custom OCTOSPI driver and a supporting software library.

The driver and flash-access functionality have been checked. The filesystem layer is currently under development.

Planned persistent data includes:

* Device configuration
* Calibration coefficients
* Measurement logs
* Other application data

Filesystem behavior, power-loss recovery, data integrity, and endurance characteristics will require separate verification as implementation progresses.

## Development Environment

* **Microcontroller:** STM32U385CG
* **Language:** C++20
* **IDE and toolchain:** IAR Embedded Workbench for Arm
* **Hardware access:** CMSIS and custom peripheral drivers
* **Execution model:** Bare-metal, interrupt-driven, with GPDMA where supported
* **Version control:** Git and GitHub

## Building the Firmware

1. Install a compatible version of IAR Embedded Workbench for Arm and the required STM32 device support.

2. Clone the repository:

   ```bash
   git clone https://github.com/43w90tm043/thermomodule_without_trustzone.git
   ```

3. Open `thermomodule_without_trustzone.eww` in IAR Embedded Workbench for Arm.

4. Verify the selected device, build configuration, compiler settings, and linker configuration.

5. Build the project and inspect compiler and linker diagnostics.

A compatible development environment and target hardware are required to reproduce hardware-level tests.

## Current Development Status

| Subsystem                                   | Status                                   |
| ------------------------------------------- | ---------------------------------------- |
| Custom CMSIS-based drivers                  | Implemented                              |
| Bare-metal C++20 architecture               | Implemented                              |
| Pt1000 measurement subsystem                | Implemented                              |
| 16-point calibration algorithm              | Implemented; accuracy not yet verified   |
| CAN FD driver and proprietary protocol      | Implemented and tested on hardware       |
| OCTOSPI flash driver and supporting library | Implemented; flash functionality checked |
| External NOR Flash filesystem               | In development                           |
| Full-range measurement characterization     | Pending verification                     |

## Engineering Focus

This project demonstrates practical work in:

* Bare-metal embedded software development in modern C++
* Low-level STM32 peripheral programming
* Asynchronous driver design using interrupts and DMA
* Precision analog measurement system integration
* CAN FD communication and custom protocol development
* External NOR Flash integration
* Mixed-signal hardware and power architecture
* Firmware architecture and version control

## Project Status

**Work in progress.** Features and performance targets are documented according to their current implementation and verification status.

## License

No license has been specified yet.
