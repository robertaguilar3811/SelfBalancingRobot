# Self Balancing Robot

> **Status:** Active development  
> This project is under active development and is being used as a platform to study
industrial-style motion control, feedback systems, and real-time architecture.
---

## Overview

A two-wheeled self-balancing robot built as a controls engineering exercise, with emphasis on deterministic control loops, state-based logic, and clean separation of real-time and supervisory tasks.

The design philosophy mirrors industrial automation: clearly defined I/O, cyclic execution, and closed-loop control driven by sensor feedback applied here to a dynamic, inherently unstable mechanical system.

---

## Mechanical Design

The frame consists of stacked 3D-printed PLA plates constrained by 3/8"-16 threaded rods, forming a rigid, modular structure that allows:

- Rapid iteration and reconfiguration
- Controlled center-of-mass adjustment
- Easy access to wiring, sensors, and drive components

---

## Hardware

| Component | Role |
|---|---|
| Teensy 4.1 | Low-level motor controller |
| Raspberry Pi 4 | Soft PLC — runs CodeSys runtime |
| MPU6050 IMU | Orientation and angular rate feedback |
| DC Gear Motors | Primary actuators |

The Teensy handles time-critical motor control while the Raspberry Pi runs a CodeSys soft PLC runtime, handling supervisory logic, configuration, and monitoring a direct implementation of a PLC + distributed I/O architecture found in industrial systems.

---

## Control Strategy

The robot stabilizes using a closed-loop feedback controller driven by tilt angle and angular velocity from the IMU. Key design principles include:

- Deterministic cyclic execution
- Tunable feedback control loop
- Clear separation of control and supervisory logic
- Defined safe startup and shutdown behavior

Currently implemented as a PID controller, with planned migration to more advanced control strategies as the platform matures.

---

## Software Structure

Control software is organized around industrial automation principles:

- Cyclic main control loop with fixed execution timing
- Explicit state machine (Init → Idle → Enable → Run → Fault)
- Deterministic sensor acquisition and actuator output
- Parameterized gains and configurable limits

Monitoring and tuning are handled outside the real-time loop to preserve control determinism.

---

## System State Machine

| State | Description |
|---|---|
| Init | Hardware and sensor validation |
| Idle | Motors disabled, system safe |
| Enable | Control loop armed, awaiting command |
| Run | Active balancing control |
| Fault | Safe shutdown on error or instability |

---

## Planned Improvements

- Port control logic concepts to PLC platforms (Structured Text / IEC 61131-3)
- Gain scheduling and advanced control strategies
- Enhanced fault diagnostics and recovery
- Industrial HMI or supervisory system integration