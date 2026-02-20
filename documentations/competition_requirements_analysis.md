# Competition Rulebook Analysis — UGV Firmware Requirements

> **Scope**: Extract firmware-relevant requirements from all 5 competition rulebooks (IGVC, IRC, UGVC, URC, ERC) and map them to the existing STM32H743ZI firmware modules.

---

## 1. Competition Overview Matrix

| Feature             | IGVC 2026                    | IRC 2026                | UGVC 2025                          | URC 2025                     | ERC 2025                    |
| ------------------- | ---------------------------- | ----------------------- | ---------------------------------- | ---------------------------- | --------------------------- |
| **Vehicle Type**    | Ground (wheeled)             | Mars rover              | Ground (wheeled/tracked)           | Mars rover                   | Mars rover                  |
| **Propulsion**      | Electric                     | Electric                | Electric only                      | No air-breathing             | Not specified               |
| **Max Dimensions**  | L: 200cm, W: 150cm, H: 200cm | 1.2m×1.2m×1.2m (stowed) | L: 90–200cm, W: 60–120cm, H: 120cm | 1.2m cube (stowed)           | Per requirements appendix   |
| **Max Weight**      | 180 lbs (~82 kg)             | 50 kg (deployed)        | Not specified                      | 50 kg deployed / 70 kg total | Scored (lighter = more pts) |
| **Speed Range**     | 1–5 mph (1.6–8 km/h)         | Not specified           | 2–10 km/h                          | Not specified                | Not specified               |
| **Mech. E‑Stop**    | ✅ Required                   | ✅ Kill switch           | ✅ Required                         | ✅ Red push button            | ✅ Per requirements          |
| **Wireless E‑Stop** | ✅ Required                   | ✅ Kill switch           | ✅ ≥150 m range                     | Not explicitly req'd         | ✅ Per requirements          |
| **Autonomous Nav**  | ✅ Core task                  | ✅ RADO delivery         | ✅ Core task                        | ✅ Core mission               | ✅ Traverse task             |
| **Remote/Teleop**   | ✅ Self Drive                 | ✅ All missions          | ✅ Remote nav task                  | ✅ Most missions              | ✅ Maintenance + more        |
| **Robotic Arm**     | ❌                            | ✅ Required              | ❌                                  | ✅ Required                   | ✅ Required                  |
| **Drone Support**   | ❌                            | ❌                       | ❌                                  | ✅ Optional                   | ✅ Droning sub-task          |

---

## 2. Firmware-Critical Requirements

### 2.1 Emergency Stop System

> [!CAUTION]
> **All 5 competitions require a hardware-based E-STOP that immediately halts the vehicle.** This is a safety-critical, non-negotiable requirement.

| Requirement            | Detail                                          | Competitions         |
| ---------------------- | ----------------------------------------------- | -------------------- |
| Mechanical E‑Stop      | Physical button, readily visible & accessible   | All 5                |
| Wireless E‑Stop        | RF-based, hardware-controlled (not software)    | IGVC, IRC, UGVC, ERC |
| Minimum wireless range | 50 m (IGVC), 150 m (UGVC)                       | IGVC, UGVC           |
| Immediate full stop    | Must bring vehicle to "quick and complete stop" | All 5                |
| Power isolation (URC)  | "Cease all power draw from batteries"           | URC                  |

**Firmware Impact**:
- ✅ **Already implemented** in `safety.c` — E-STOP ISR immediately zeros all motor outputs
- ⚠️ **Enhancement needed**: Add support for wireless E-STOP receiver GPIO input (separate from mechanical)
- ⚠️ **Enhancement needed**: URC requires complete battery power isolation — firmware should signal a hardware relay/contactor to disconnect battery bus

### 2.2 Speed Control & Limits

| Competition     | Min Speed        | Max Speed      | Unit          |
| --------------- | ---------------- | -------------- | ------------- |
| IGVC            | 1 mph (1.6 km/h) | 5 mph (8 km/h) | mph           |
| UGVC            | 2 km/h           | 10 km/h        | km/h          |
| IRC / URC / ERC | —                | —              | Not specified |

**Firmware Impact**:
- ✅ **Already implemented** in `pid.c` — output clamping limits motor commands
- ⚠️ **Enhancement needed**: Configurable speed limit parameters per competition profile
- ⚠️ **Enhancement needed**: Minimum speed enforcement via CAN command or firmware constant (IGVC requires ≥ 1 mph at all times, UGVC requires ≥ 2 km/h in first 15 m)

### 2.3 Safety Indicator Lights

| Competition | Requirement                                                            |
| ----------- | ---------------------------------------------------------------------- |
| IGVC        | Yellow light ON = power; flashing = moving                             |
| UGVC        | Solid = power ON; flashing = autonomous mode                           |
| URC         | Red LED = autonomous; Blue = teleop; Flashing Green = waypoint arrival |
| ERC         | Per requirements appendix                                              |

**Firmware Impact**:
- ❌ **Not yet implemented**: Need a light/LED controller module
- New module: `lights.h` / `lights.c` — GPIO outputs for multicolor LED or addressable LED strip
- CAN message to configure light mode based on operational state (IDLE → solid, AUTONOMOUS → flash pattern, TELEOP → different pattern, WAYPOINT_ARRIVAL → flashing green)

### 2.4 Autonomous Navigation Support

| Capability                 | Competitions Requiring    |
| -------------------------- | ------------------------- |
| Waypoint navigation (GNSS) | IGVC, UGVC, URC, IRC, ERC |
| Lane following / detection | IGVC, UGVC                |
| Obstacle avoidance         | IGVC, UGVC, URC, ERC      |
| ArUco / AR tag detection   | URC, ERC                  |
| Image/person recognition   | IGVC (Self Drive), UGVC   |
| Autonomous return to start | ERC, URC                  |

**Firmware Impact**:
- The STM32H743ZI motor controller firmware provides **low-level velocity control** — the autonomous navigation stack runs on a separate compute module (e.g., Jetson, RPi)
- ✅ **Already implemented**: CAN-FD velocity command reception in `can_comm.c`
- ⚠️ **Enhancement needed**: Add CAN message IDs for:
  - Receiving GNSS waypoint status from navigation computer
  - Receiving autonomous mode enable/disable commands
  - Reporting odometry (encoder-based) back to navigation computer
  - Speed profile commands (cruise, approach, stop)

### 2.5 Operational Mode State Machine

Based on all rulebooks, the UGV needs these operational modes:

```
┌──────────┐    ┌───────────┐    ┌──────────────┐
│  STARTUP │───►│  STANDBY   │───►│  TELEOP      │
└──────────┘    └───────────┘    └──────────────┘
                     │                   │
                     │                   ▼
                     │           ┌──────────────┐
                     └──────────►│  AUTONOMOUS  │
                                └──────────────┘
                     │                   │
                     ▼                   ▼
                ┌──────────┐    ┌──────────────┐
                │  E-STOP  │◄───│   FAULT      │
                └──────────┘    └──────────────┘
```

**Firmware Impact**:
- ✅ **Partially implemented** in `safety.c` — 5-state machine (INIT, STANDBY, ACTIVE, FAULT, ESTOP)
- ⚠️ **Enhancement needed**: Split ACTIVE into TELEOP and AUTONOMOUS sub-states
- ⚠️ **Enhancement needed**: Light controller state should be driven by this mode

### 2.6 Communication Requirements

| Band                 | IGVC | IRC   | UGVC   | URC         | ERC         |
| -------------------- | ---- | ----- | ------ | ----------- | ----------- |
| 2.4 GHz              | ✅    | ✅     | ✅ Only | ✅           | ✅ WiFi only |
| 5 GHz                | —    | ✅     | ❌      | ✅           | ✅           |
| 900 MHz              | —    | —     | ❌      | ✅ Sub-bands | ❌           |
| Amateur bands        | —    | —     | —      | ✅           | ✅           |
| Antenna height limit | —    | 1.5 m | 3 m    | 3 m         | Per venue   |

**Firmware Impact** (CAN-FD internal bus):
- ✅ **Already implemented**: CAN-FD at 1 Mbps nominal / 5 Mbps data in `can_comm.c`
- The wireless communication layer is **external** to the motor controller firmware (handled by the navigation/comms computer)
- ⚠️ **Enhancement needed**: Add heartbeat monitoring for the wireless link — if the navigation computer stops sending commands (communication timeout), transition to FAULT state

### 2.7 Telemetry & Diagnostics

All competitions benefit from real-time telemetry for operator situational awareness:

| Data Point            | Source              | CAN Msg       |
| --------------------- | ------------------- | ------------- |
| Wheel speeds (4×)     | Encoders            | ✅ Implemented |
| Motor currents (4×)   | ADC                 | ✅ Implemented |
| Battery voltage       | ADC                 | ⚠️ Need to add |
| Battery current / SOC | ADC or BMS          | ⚠️ Need to add |
| MCU temperature       | Internal sensor     | ⚠️ Need to add |
| Motor temperatures    | NTC                 | ⚠️ Need to add |
| Safety state          | State machine       | ✅ Implemented |
| Fault codes           | Fault module        | ✅ Implemented |
| Odometry (distance)   | Encoder integration | ⚠️ Need to add |

**Firmware Impact**:
- ✅ **Partially implemented** in `telemetry.c` — basic wheel speed + current + state
- ⚠️ **Enhancement needed**: Expand telemetry to include battery, temperature, and odometry channels

---

## 3. Competition-Specific Requirements

### 3.1 IGVC 2026 — Specific

| Requirement                                  | Firmware Relevance                           |
| -------------------------------------------- | -------------------------------------------- |
| Payload: 20 lbs (9 kg) additional            | PID tuning must account for variable payload |
| Speed: must travel ≥ 1 mph at all times      | Minimum speed enforcement in control loop    |
| Self Drive: pedestrian detection, stop signs | Navigation computer — no firmware changes    |
| Lane following: 8 cm white lines             | Vision stack — no firmware changes           |

### 3.2 IRC 2026 — Specific

| Requirement                                          | Firmware Relevance                                       |
| ---------------------------------------------------- | -------------------------------------------------------- |
| Rover ≤ 50 kg, stowed in 1.2m cube                   | Mechanical — no firmware changes                         |
| Autonomous delivery (50% penalty for non-autonomous) | Mode switching needed in firmware                        |
| Object terrain ≤ 500 m from base                     | Range/telemetry consideration                            |
| Robotic arm control                                  | Separate arm controller, not part of drivetrain firmware |

### 3.3 UGVC 2025 — Specific

| Requirement                                                 | Firmware Relevance                           |
| ----------------------------------------------------------- | -------------------------------------------- |
| Wireless E-STOP ≥ 150 m range                               | Hardware receiver → GPIO → ISR in `safety.c` |
| Solid light = power ON, flash = autonomous                  | `lights.c` module needed                     |
| Light/laser pointer for person detection                    | GPIO output triggered via CAN command        |
| Qualification: lane detection over 15 m                     | Navigation stack                             |
| Course max time: 8 min, min speed over first 15 m in 30 sec | Speed profile control                        |

### 3.4 URC 2025 — Specific

| Requirement                                       | Firmware Relevance                         |
| ------------------------------------------------- | ------------------------------------------ |
| Red push-button E-STOP                            | Hardware — existing `safety.c` covers this |
| LED: Red=auto, Blue=teleop, FlashingGreen=arrival | Multicolor LED via `lights.c`              |
| 50 kg max deployed weight                         | Mechanical                                 |
| ArUco tag detection (4×4_50 library)              | Vision stack                               |
| Budget ≤ $22,000                                  | Administrative                             |
| Modular configs between missions                  | CAN-based reconfiguration commands         |

### 3.5 ERC 2025 — Specific

| Requirement                                                           | Firmware Relevance                        |
| --------------------------------------------------------------------- | ----------------------------------------- |
| Traverse: 4 waypoints + return, 20 min                                | Autonomous navigation + speed control     |
| Autonomy scoring: full marks only without video/GNSS feed to operator | Mode flag in telemetry                    |
| Maintenance panel: switches, knobs, plugs                             | Arm controller (separate from drivetrain) |
| Probing: collect 3 probes, time-based scoring                         | Fast traverse + arm coordination          |
| ArUco landmarks (5×5 library, IDs 51-64)                              | Vision stack                              |
| Multiple teams on Mars Yard simultaneously                            | Collision avoidance awareness             |

---

## 4. Required Firmware Modifications Summary

### High Priority (Safety-Critical)

| #   | Modification                               | Module                    | Justification                               |
| --- | ------------------------------------------ | ------------------------- | ------------------------------------------- |
| 1   | Dual E-STOP inputs (mechanical + wireless) | `safety.c`                | All competitions require ≥ 2 E-STOP methods |
| 2   | Battery power isolation relay control      | `safety.c`                | URC requires complete power disconnect      |
| 3   | Configurable speed limits (min + max)      | `pid.c`, `control_loop.c` | IGVC/UGVC have strict speed ranges          |

### Medium Priority (Functionality)

| #   | Modification                            | Module                          | Justification                                       |
| --- | --------------------------------------- | ------------------------------- | --------------------------------------------------- |
| 4   | LED/Light indicator controller          | New: `lights.c`                 | Required by IGVC, UGVC, URC                         |
| 5   | TELEOP ↔ AUTONOMOUS sub-states          | `safety.c`                      | All competitions distinguish modes                  |
| 6   | Odometry integration + CAN reporting    | `telemetry.c`, `control_loop.c` | All autonomous nav stacks need odometry             |
| 7   | Battery voltage/current telemetry       | `telemetry.c`                   | Operational monitoring across all events            |
| 8   | Competition profile selection (CAN cmd) | New: `config.c`                 | Switch speed limits, light patterns per competition |

### Low Priority (Enhancement)

| #   | Modification                             | Module        | Justification                    |
| --- | ---------------------------------------- | ------------- | -------------------------------- |
| 9   | Motor/MCU temperature monitoring         | `telemetry.c` | Thermal protection               |
| 10  | Laser pointer GPIO control               | `lights.c`    | UGVC person detection indicate   |
| 11  | Heartbeat timeout tuning per competition | `fault.c`     | Different comms latency profiles |

---

## 5. Cross-Competition Compatibility Matrix

The existing firmware architecture is **well-suited** for all 5 competitions. The core control loop, PID, CAN-FD communication, and safety state machine are universal. Key additions needed:

```mermaid
graph LR
    subgraph Existing ✅
        A[PID Controller] --> B[Motor Control]
        C[CAN-FD Comms] --> A
        D[Safety State Machine] --> B
        E[Fault Detection] --> D
        F[Telemetry TX] --> C
    end
    subgraph Additions Needed ⚠️
        G[Light Controller] --> B
        H[Odometry Integration] --> F
        I[Battery Monitoring] --> F
        J[Competition Config] --> A
        J --> G
        K[Wireless E-STOP Input] --> D
    end
    style G fill:#ffa
    style H fill:#ffa
    style I fill:#ffa
    style J fill:#ffa
    style K fill:#ffa
```

> [!IMPORTANT]
> The autonomous navigation, computer vision, SLAM, and path planning are **not** part of the motor controller firmware. They run on a separate high-level compute module that communicates with this firmware over CAN-FD. The firmware's job is strictly: receive velocity commands → execute PID → enforce safety → report telemetry.
