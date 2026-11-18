# Hexapod_2_Middle_Level

**The coordinator of a six-legged robot: an STM32F446RE that talks micro-ROS 2 over Ethernet to the computer, drives three motor boards over two CAN buses and computes odometry from an IMU and an optical-flow sensor.**

English | [Русский](README.ru.md)

```
 ROS 2 computer                STM32F446RE · FreeRTOS · 180 MHz                  motor boards
┌────────────────────┐        ┌──────────────────────────────────────┐          ┌────────────────┐
│ micro-ROS agent    │        │ rosTask       micro-ROS node         │          │ boards 1 and 2 │
│ 192.168.10.2:33333 │◄──────►│ CAN           RX in IRQs, TX frames  │◄─ CAN1 ─►│ board 3        │
│                    │ UDP/IP │ odometryTask  MPU6050 + optical flow │◄─ CAN2 ─►│ (Hexapod_1)    │
│                    │        │ W5500 SPI1, 192.168.10.3             │          │                │
└────────────────────┘        └──────────────────────────────────────┘          └────────────────┘
```

This is the middle layer of the Hexapod stack. Upward it speaks ROS 2: it publishes the telemetry of all six drives and
accepts target angles, settings and controller coefficients. Downward it packs those into CAN frames for the three
[motor boards](https://github.com/<your-account>/Hexapod_1_Low_Level) and unpacks their answers. On the side it estimates
the robot's pose from an IMU and an optical-flow sensor.

## Features

* **micro-ROS on FreeRTOS** (`ros_TASKS.c`): node `STM32F446RET6` with a connection state machine: wait for the agent →
  create node, publisher and subscriptions → spin; the agent is pinged periodically and after more than 10 missed pings
  everything is torn down and the node reconnects by itself.
* **Ethernet transport on a W5500** over SPI1 (`transport_w5500.c`): a custom micro-XRCE-DDS transport using one UDP
  socket and a static address; custom allocators (`custom_memory_manager.c`) keep micro-ROS inside the FreeRTOS heap.
* **Two CAN buses at 1 Mbit/s** (`can_*_master.c`): CAN1 serves boards 1 and 2, CAN2 board 3; receive in interrupt
  callbacks with separate FIFOs, error counters, and a bus-idle watchdog that restarts the exchange after 100 ms of
  silence.
* **Odometry** (`odometry.c`, 100 Hz): MPU6050 over I2C1 with a low-pass filter, a Madgwick attitude filter, standstill
  detection with automatic gyro-Z bias estimation and yaw freezing, plus an optical-flow and rangefinder module on USART1
  (circular DMA reception, frame parser) whose displacements are scaled by height and rotated into the world frame to
  give `x`, `y`, `φ` and their velocities.
* **FreeRTOS** (CMSIS-RTOS v2): `rosTask` (32 KB static stack) and `odometryTask`, both above normal priority; CAN is
  handled in interrupts, not in a task.
* **STM32CubeMX project** (`freeRTOS_microROS_F446RET6.ioc`): 180 MHz, CAN1, CAN2, CRC, I2C1, SPI1, USART1, FreeRTOS.

### ROS 2 interface

All topics use `std_msgs/Int32MultiArray`. Motors are numbered `board_slot`: 1_1, 1_2, 2_1, 2_2, 3_1, 3_2.

| topic | direction | size | content |
|---|---|---|---|
| `vector` | MCU → ROS | 18 | per motor, in order: angle, speed, current |
| `set_angles` | ROS → MCU | 6 | target angle of each motor |
| `set_other` | ROS → MCU | 27 | per board, 9 values: obstacle ×2, phase ×2, set-zero, system type ×2, speed limit ×2 |
| `set_koeffs` | ROS → MCU | 42 | per motor 7 gains: angle `kp ki kd`, speed `kp ki`, current `kp ki` |

The odometry state (`x`, `y`, `φ`, `vx`, `vy`, `ω`) is computed on the board; the `vector` topic carries only the drive
telemetry.

### CAN frames

| direction | IDs | content |
|---|---|---|
| MCU → board *n* | `0x200` / `0x300` / `0x400` | target angles |
| | `0x201` / `0x301` / `0x401` | limits and other settings |
| | `0x202`, `0x203` / `0x302`, `0x303` / `0x402`, `0x403` | controller coefficients per motor |
| board → MCU | `0x101`, `0x103`, `0x105` | board 1: currents, speeds, angles (CAN1) |
| | `0x100`, `0x102`, `0x104` | board 2: currents, speeds, angles (CAN1) |
| | `0x501`, `0x503`, `0x505` | board 3: currents, speeds, angles (CAN2) |

Each answer carries two `int32` values, one per motor of the board.

## Requirements

* STM32CubeIDE and an ST-LINK;
* an STM32F446RE board with an MPU6050 (I2C1), an optical-flow and rangefinder module (USART1), a W5500 module (SPI1)
  and two CAN transceivers;
* a computer with ROS 2 and a micro-ROS agent, on the same subnet as the MCU.

## Installation

```bash
git clone https://github.com/<your-account>/Hexapod_2_Middle_Level.git
```

1. **micro-ROS static library.** `libmicroros` is not stored in the repository. Generate it with
   [micro_ros_stm32cubemx_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils) for the Cortex-M4F target of
   this project. `microros_fix/Dockerfile` is a patch of the builder image
   (`micro_ros_static_library_builder:foxy`) that refreshes the ROS apt repository key so the build works again.
2. Open the folder in STM32CubeIDE (*File → Open Projects from File System…*), build and flash.
3. Match the micro-ROS agent's ROS 2 distribution to the one the library was built for.

## Quick start

Give the computer the address the firmware expects, start the agent, then power the MCU:

```bash
sudo ip addr add 192.168.10.2/24 dev <your-ethernet-interface>
ros2 run micro_ros_agent micro_ros_agent udp4 --port 33333

ros2 topic list                    # vector, set_angles, set_other, set_koeffs
ros2 topic echo /vector            # telemetry of the six drives: angle, speed, current
```

> `set_angles`, `set_other` and `set_koeffs` move real legs. Publish to them only with the robot lifted off the ground
> and with angles you have checked.

Addresses live in `Core/Src/transport_w5500.c`: MCU `192.168.10.3/24`, agent `192.168.10.2:33333`.

## Repository layout

```
Core/Src/ros_TASKS.c            micro-ROS task: node, topics, reconnection
Core/Src/transport_w5500.c      W5500 UDP transport for micro-XRCE-DDS
Core/Src/microros_*.c, custom_memory_manager.c   allocators and time for micro-ROS
Core/Src/can_TRANSMIT_RECEIVE_master.c   CAN receive callbacks, bus watchdog
Core/Src/can_CONVERSION_master.c         packing of angles, settings and coefficients into frames
Core/Src/odometry.c             MPU6050, Madgwick filter, optical flow, pose
Core/Src/freertos.c             task creation and priorities
Core/Src/w5500/, Core/Inc/w5500/   W5500 driver
microros_fix/                   Dockerfile for the micro-ROS library builder
freeRTOS_microROS_F446RET6.ioc  STM32CubeMX project
Drivers/, Middlewares/          ST HAL, CMSIS, FreeRTOS (their own licences)
```

## Part of the Hexapod stack

| level | repository | runs on |
|---|---|---|
| 1 — motors | [Hexapod_1_Low_Level](https://github.com/<your-account>/Hexapod_1_Low_Level) | STM32F103C8T6 × 3 |
| 2 — coordination | **Hexapod_2_Middle_Level** | STM32F446RE |
| 3 — gaits and navigation | [Hexapod_3_High_Level](https://github.com/<your-account>/Hexapod_3_High_Level) | ROS 2 / Gazebo |

## Licence

The ST HAL, CMSIS, FreeRTOS and W5500 driver code keep their original licences.
