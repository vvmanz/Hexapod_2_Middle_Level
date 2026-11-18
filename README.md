# Hexapod · Middle-Level Control

**STM32F446 firmware that connects hexapod actuator controllers to ROS 2 through micro-ROS, CAN, Ethernet and sensor-based odometry.**

English | [Русский](README.ru.md)

This repository implements the **intermediate control layer** of a three-level hexapod architecture. An STM32F446RE receives commands from ROS 2, distributes parameters and targets to lower-level motor controllers over CAN, and processes orientation/motion measurements from onboard sensors.

## Features

- **micro-ROS integration** through a custom UDP transport over the W5500 Ethernet controller (SPI).
- **CAN coordination** of three motor-controller bridges / six actuator channels.
- **Control messages** for target angles, PID coefficients and additional actuator parameters.
- **FreeRTOS tasks** for ROS communication and sensor/odometry processing.
- **IMU orientation estimation** using MPU6050 readings, filtering and a Madgwick update.
- **Optical-flow/distance processing** over UART DMA to estimate planar displacement and velocity.

## Architecture

```text
ROS 2 host / micro-ROS agent
          │ Ethernet / UDP (W5500)
          ▼
STM32F446RE + FreeRTOS
  ├── ros_TASKS.c         ROS messages → targets / coefficients
  ├── CAN master          commands → 3 lower-level controllers
  └── odometry.c          IMU + optical flow / range → motion estimate
          │
          ▼
STM32F103 low-level boards → actuator control
```

## Software and hardware requirements

- **MCU:** STM32F446RE (configuration: `freeRTOS_microROS_F446RET6.ioc`).
- **Firmware toolchain:** STM32CubeIDE, STM32 HAL, FreeRTOS and the micro-ROS STM32 integration present in the project.
- **Communication:** W5500 Ethernet controller via SPI, CAN interfaces, a host running a compatible micro-ROS agent.
- **Sensors:** MPU6050 and a serial motion/distance source matching the frame parser in `odometry.c`.
- **A connected low-level controller setup** for real actuator commands.

Hardware-specific pin assignments and the serial sensor protocol must be checked in code and the `.ioc` file; there is no complete wiring manual in this snapshot.

## Build and connect

1. Import the repository into STM32CubeIDE as an existing project.
2. Inspect [`freeRTOS_microROS_F446RET6.ioc`](freeRTOS_microROS_F446RET6.ioc) and confirm the MCU, SPI, UART, CAN, I²C and GPIO wiring.
3. Build the `Debug` configuration for the STM32F446RE target.
4. In [`Core/Src/transport_w5500.c`](Core/Src/transport_w5500.c), review and adapt the hard-coded **static MCU / agent addresses** for your own network.
5. Run a compatible micro-ROS agent on the ROS 2 host, then flash and start the MCU firmware.
6. Verify ROS topic traffic and CAN outputs **before** enabling motor power.

A helper Dockerfile for rebuilding the micro-ROS static library is included in [`microros_fix/`](microros_fix/). It is a dependency workaround, not a ready-to-run system installer. Agent and firmware compatibility depend on the selected ROS/micro-ROS version.

## ROS message contracts

In [`Core/Src/ros_TASKS.c`](Core/Src/ros_TASKS.c), three callbacks accept `std_msgs/msg/Int32MultiArray` payloads:

| Payload | Expected minimum length | Meaning |
|---|---:|---|
| Target angles | 6 integers | One setpoint per actuator channel |
| Control coefficients | 42 integers | Seven coefficients per channel × six channels |
| Other parameters | 27 integers | Bridge/channel configuration and limits |

These are **firmware-specific array layouts**, not generic ROS message definitions. Check the actual topic names and field ordering in the source before publishing commands.

## Repository layout

```text
Core/Src/main.c                  peripheral setup and CAN command handling
Core/Src/ros_TASKS.c             micro-ROS node, subscriptions, callbacks
Core/Src/transport_w5500.c       custom Ethernet/UDP transport
Core/Src/odometry.c              MPU6050, orientation, optical flow, odometry
Core/Src/can_*_master.c          CAN message conversion and transfer
Core/Src/freertos.c              FreeRTOS task creation
Core/Inc/                       headers, config and W5500 declarations
Middlewares/                    bundled middleware sources
microros_fix/Dockerfile         experimental micro-ROS build workaround
*.ioc                           STM32CubeMX configuration
```

## Status and limitations

- Development firmware intended for a hardware-backed hexapod; **not** a PC-only simulator.
- Network settings are currently fixed in the source; tailor them for your lab.
- The task configuration in `freertos.c` has the CAN task creation commented out in this snapshot. Review task behavior and CAN timing before deploying.
- No separate top-level licence was included; third-party middleware and ST code retain their respective licences.
