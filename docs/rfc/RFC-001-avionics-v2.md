# RFC-001: Avionics board v2

**Author(s):** Jozef Jabczun <jabczunjozef@gmail.com>
**Created:** 2025-06-01
**Status:** Draft
**Supersedes:** NONE
**Superseded by:** NONE

---

## 1. Summary

RFC-001 proposes version 2 of the Viikate avionics board. The new board
specification includes the following changes:

- a switch to an STM32H7 architecture,
- GPS,
- radio telemetry,
- separated power and guidance sections,
- current usage sensing,
- modular bus architecture based on CAN,
- removal of magnetometer.

This is to complement the already developed features from version 1 of the
Viikate avionics boards, including, but not limited to:

- 4 servo controls,
- 1 pyrotechnics control channel,
- battery power,
- integrated accelerometer, gyroscope, and barometer.

---

## 2. Motivation

The current Viikate avionics board (version 1) has served as a functional
prototype, but several limitations in its architecture and physical design have
become evident during testing and field use. Version 2 seeks to address these
issues to improve reliability, maintainability, and operational safety, as well
as to support future expansion.

### 1. Board size constraints

The existing avionics board occupies more physical space than desirable within
the airframe, leading to tight mechanical tolerances and increased complexity
during integration. A more compact board layout would make assembly easier,
improve internal wiring, and reduce the total system mass.

### 2. Design flaws in the original layout

Version 1 suffered from multiple small but critical hardware design issues,
including misrouted power traces, missing connections to certain
microcontroller pins, and inconsistent grounding. These errors required
workarounds during assembly and debugging, impacting reliability and
repeatability. A new iteration provides an opportunity to correct these flaws
comprehensively.

### 3. Streamlining pad operations and improving safety

Current assembly and launch preparations require multiple manual steps after
the rocket motor is installed. Some of these steps expose personnel to
unnecessary risk (e.g., connecting or testing live electrical systems near the
pad). Version 2 will simplify and consolidate these operations, reducing time
on the pad and minimizing safety hazards.

### 4. Improving recovery capability

At present, recovery operations can be difficult, especially once the rocket
lands far from the pad or beyond line of sight. Integrating GPS and radio
telemetry into the avionics board will make tracking and recovery significantly
easier, avoiding the need for separate tracking modules.

### 5. Connector reliability 

The connectors used in the current design may show mechanical weakness under
stress and vibration, leading to intermittent connections or partial
disconnections in flight. The updated design will incorporate more secure
connector types suited for the operational environment.

### 6. Data logging and analysis 

The existing logging procedure of logging directly to an SD card is fragile and
may lead to loss of data in higher performance rockets in the future. The new
design will aim to store data internally within the flight controller with a
secure soldered connection.

The new board will also include further sources of telemetry, such as GPS,
current sensing for servos (allows for stall detection and better ground
analysis), etc.

### 7. Magnetometer removal

In practice, the onboard magnetometer has proven unnecessary and may be
unreliable within the electromagnetic environment of the rocket, particularly
near high-current cabling and pyrotechnic circuits. Because its readings may be
noisy and currently contribute little to flight control, it will be removed to
simplify the design.

### 8. Transition away from the Teensy platform 

The Teensy (IMXRT1062) platform used in version 1 has become impractical for
further development due to the large size of the Teensy. The onboard IMXRT1062
microcontroller unit (MCU) is impractical for our uses as it is only packaged
in BGA (ball grid array), which complicates fabrication and rework. Migrating
to an STM32H7 microcontroller provides greater availability, better support,
and an improved development ecosystem while maintaining high performance for
real-time control tasks.

### 9. Long-term expandability 

Version 1’s layout does not lend itself well to modular upgrades or peripheral
expansion. By implementing a CAN-based modular bus, version 2 will allow easy
integration of future subsystems such as advanced telemetry links, additional
sensors, or actuator controllers. This extends the platform’s lifespan and
adaptability.

### 10. Routing issues

Version 1 has routing issues to the external expansion headers, such as the
power pins not being connected. This was caused by insufficient peer-review
before the ordering of the final design. The development process of V2 should
address this.

---

## 3. Goals


The main objectives of Viikate Avionics Board v2 are to improve system
reliability, safety, and maintainability while enabling continued development
of more capable rockets. This version will serve as a mature, flight-ready
platform that corrects shortcomings in version 1 and supports modular future
expansion.

### Primary Goals

1. Reduce physical footprint.
2. Rectify electrical design flaws.
3. Improve operational safety and ease of assembly.
4. Enhance recovery capabilities.
5. Increase connector reliability.
6. Implement reliable onboard data logging.
7. Remove non-essential or unreliable sensors.
8. Migrate to STM32H7 architecture.
9. Enable modular system expansion.

### Non-Goals
1. Complete software feature redesign (should be addressed in separate RFC).

---

## 4. Detailed Design

### 4.1 Requirements

#### 1. Mechanical & Structural

| ID | Requirement | Priority | Rationale |
|----|--------------|-----------|-----------|
| M1 | The avionics board stack MUST fit within a 50x50 mm footprint. | Critical | Ensures compatibility with 54–75 mm airframes with clearance. Allows for economic PCB assembly with JLC.  |
| M2 | The avionics system SHOULD be modularly split into at least two boards: **Power/IO** and **Guidance/Telemetry**, connected via a stacking bus. | High | Separates sensitive sensors from power noise and simplifies maintenance. |
| M3 | Boards MUST be physically and electrically keyed to prevent reverse installation. | Critical | Prevents catastrophic damage during integration. |
| M4 | The assembled avionics SHOULD tolerate **> 8 g RMS vibration** up to **2 kHz** for **30 minutes** and survive **64 g peak shock loads** without functional failure. | Critical | Matches the rocket-level vibration and shock environments. |
| M5 | The unit MUST be mechanically mountable to the airframe using at least **four M3 or equivalent fasteners** with isolation options. | High | Ensures robust mounting and isolation. |
| M6 | All connectors MUST be **locking or positive retention** type, rated for **8 g vibration**. | Critical | Prevents disconnects in flight. |
| M7 | All manually operated electrical connections apart from screw terminals MUST use dissimilar connectors | Medium | Reduces the chance of misconnections |
| M8 | All electronics switches or connectors that need to be manually operated MUST be accessible from outside the vehicle via either access panels or direct mounting on the outer skin. | High | Ease of operation and operational safety |
| M9 | All electronics switches or connectors that need to be manually operated MUST be mounted on the vehicle side opposite to the launch rail. | High | Launch rail blocks access |

---

#### 2. Electrical Power

| ID | Requirement | Priority | Rationale |
|----|--------------|-----------|-----------|
| P1 | Nominal power input MUST be **12 V DC**, with allowable range **9–18 V**. | Critical | Matches common LiPo 3S or Li-ion systems. |
| P2 | Power distribution SHOULD include **separate isolated rails** for logic, sensors, and actuators. | High | Prevents servo brownouts from corrupting sensor readings. |
| P3 | Total avionics power draw MUST NOT exceed **5 W average**, **10 W peak** under normal operation. | Medium | Supports battery sizing and electrical budget. |
| P4 | Board MUST incorporate a **reverse-polarity protection** and **over-current protection** feature. | Critical | Prevents damage due to wiring or battery errors. |
| P5 | Current-sense circuitry SHOULD measure **individual servo rails** and **total system draw**. | High | Enables fault detection and ground diagnostics. |
| P6 | System MUST maintain low EMI to not interfere with sensitive sensors and RF sections of the avionics unit | High | Enables fault detection and ground diagnostics. |

---

#### 3. Processing & Storage

| ID | Requirement | Priority | Rationale |
|----|--------------|-----------|-----------|
| C1 | Main MCU MUST be an **STM32H7** series (Cortex-M7) running >= 400 MHz. | Critical | Matches computational and I/O needs for guidance. |
| C2 | Board MUST include at least **16 MB external flash** for data logging, soldered and non-removable. | Critical | Prevents SD card ejection or data loss in high-vibe conditions. |
| C3 | Unit MUST present internal flash storage as a **USB mass storage device** for easy log retrieval. | High | Simplifies field data extraction. |
| C4 | Logging rate SHOULD allow ≥100 Hz of telemetry logging for all channels with timestamping. | High | Ensures sufficient temporal resolution for trajectory reconstruction. |

---

#### 4. Sensing & Guidance

| ID | Requirement | Priority | Rationale |
|----|--------------|-----------|-----------|
| S1 | System MUST include a **low‑G IMU** (accelerometer <= 16 g and gyroscope) for attitude control. | Critical | Fundamental sensor suite. |
| S2 | A **barometer** with 10 cm resolution up to 5 km altitude MUST be included. | High | Provides altitude reference for staging/apogee detection. |
| S3 | A **high‑G accelerometer (> 100 g)** MAY be included for high‑dynamic recording (boost). | Optional | Provides redundancy for high‑G flight. |
| S4 | A **GPS receiver with >= 10 Hz update rate** MUST be integrated. | Critical | Required for location tracking and velocity estimation. |
| S5 | All sensor clocks MUST be synchronized via **hardware timestamping** relative to system time. | High | Enables data correlation during flight analysis. |
| S6 | MUST include a pressure relief hole **3 mm in diameter or equivalent area**| High | Allows for accurate barometer readings |

---

#### **5. Telemetry & Communication**

| ID | Requirement | Priority | Rationale |
|----|--------------|-----------|-----------|
| T1 | All boards MUST communicate via **CAN-FD** bus standard up to 1 Mbps. | Critical | Deterministic, noise-resistant comms architecture. |
| T2 | Telemetry radio MUST achieve **>= 1 km range (rural)** or **>= 5 km LOS** at 10 mW nominal power. | High | Ensures reliable tracking range. |
| T3 | Radio subsystem SHOULD support **frequency diversity** or **configurable frequencies (868 MHz / 433 MHz / 2.4GHz / ...)**. | Medium | Regulatory and performance flexibility. |
| T4 | System SHOULD support **wired debugging interface** via SWD and USB. | High | Essential for maintenance and HIL testing. |

---

#### **6. Pyrotechnic & Servo Control**

| ID | Requirement | Priority | Rationale |
|----|--------------|-----------|-----------|
| ACT1 | System MUST provide **>= 2 pyro channels** with 2 A drive, 50 ms minimum hold, isolated from logic. | Critical | Main and backup deployment. |
| ACT2 | System MUST provide **>= 4 PWM servo outputs (5 V)** with independent current monitoring. | Critical | Supports thrust vectoring or fin control. |
| ACT3 | Activation of pyro channels MUST require a **mechanical arm/safe interlock** in addition to software confirmation. | Critical | Prevents accidental ignition. |
| ACT4 | System SHOULD include **ignition continuity sensing** and onboard status LEDs. | High | Supports preflight verification. |

---

#### 7. Safety & Operational

| ID | Requirement | Priority | Rationale |
|----|--------------|-----------|-----------|
| SAF1 | The system MUST include **dual-layer arming (hardware + software)**. | Critical | Prevents unintended actuation. |
| SAF2 | The system MUST provide **visual indicators (LED or OLED)** for power, armed state, and telemetry link. | High | Ground crew feedback. |
| SAF3 | The system MUST support a **preflight checklist state machine**. | Medium | Enforces safe configuration process. |
| SAF4 | The avionics SHOULD be capable of remaining in the **pad‑armed state for >=4 hours** using internal battery without recharging. | Medium | Allows for delays before launch. |
| SAF5 | The board SHOULD log **pre‑launch checks and arm status events** for post‑flight review. | Medium | Accountability and troubleshooting. |
| SAF6 | Budget SHOULD stay under 140 EUR | High | *"The current budget left for viikate is 138,09€, with 600€ reserved for 'fall rocket project'"* - Tuomas |


#### 8. Environmental performance

| ID | Requirement | Priority | Rationale |
|----|--------------|-----------|-----------|
| ENV1 | MUST be able to survive **relative humidity between 0 and 95%** without issues. | High | Prevents strange errors during flight. |
| ENV2 | SHOULD be at least **IP54 water-resistant.** | High | Prevents loss of avionics in case of landing in wet environments. |
| ENV3 | MUST survive **internal temperatures between -20 and +90 celsius**. | High | Allows for winter and summer launches. |
| ENV4 | SHOULD survive **external temperatures above -40 celsius**. | Low | Allows for high altitude launches. |


---

## 5. Alternatives Considered

TBD

---

## 6. Drawbacks

TBD

---

## 7. Impact

TBD

---

## 8. Rollout Plan

TBD

---

## 9. References

TBD
