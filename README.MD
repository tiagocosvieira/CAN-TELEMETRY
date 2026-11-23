# 🏎️ RP2040 CAN Telemetry & Vehicle Dynamics Engine

![Language](https://img.shields.io/badge/C%2B%2B-17-00599C?style=flat&logo=cplusplus)
![Platform](https://img.shields.io/badge/MCU-RP2040%20Dual--Core-C51A4A?style=flat&logo=raspberrypi)
![Framework](https://img.shields.io/badge/Framework-PlatformIO-FF7F00?style=flat&logo=platformio)
![Protocol](https://img.shields.io/badge/Automotive-CAN%20Bus%20%2F%20OBD--II-2EA44F?style=flat)

An embedded real-time automotive telemetry and vehicle dynamics system built on the **Raspberry Pi Pico (RP2040)**. This project integrates **Embedded C++**, **Automotive Electronics (CAN Bus)**, and **Applied Calculus** into a single deterministic pipeline: acquiring raw vehicle parameters via OBD-II and computing real-time physical metrics—such as longitudinal acceleration, jerk, traveled distance, and instantaneous wheel horsepower—visualized through an 8-LED shift-register bar and logged over USB Serial.

---

## 🎯 Project Objectives

1. **Deterministic Dual-Core Processing:** Leverage the RP2040's dual ARM Cortex-M0+ cores to isolate asynchronous SPI/CAN communication (`Core 0`) from fixed-timestep numerical calculus (`Core 1` at exact $20\text{ Hz}$).
2. **Real-Time Vehicle Kinematics & Dynamics:** Estimate unmeasured physical states (acceleration, jerk, odometry, and tractive power) directly from ECU speed and RPM frames using numerical differentiation and integration.
3. **Hardware-Level Safety & Visual Feedback:** Safely interface a $5\text{V}$ CAN transceiver (`MCP2515` + `TJA1050`) with $3.3\text{V}$ RP2040 logic and drive an 8-bit `SN74HC164N` shift register as a real-time power/RPM shift light.
4. **Bench-to-Vehicle Validation:** Support hardware-in-the-loop testing via MCP2515 `LOOPBACK` mode before deploying to a live vehicle OBD-II port (`500 kbps`).

---

## 🏗️ MVP Software Architecture

To prevent SPI bus latency from introducing jitter into the time step ($\Delta t$), execution is split across both RP2040 cores using a thread-safe shared state:

```text
[Vehicle OBD-II / CAN Loopback]
       │ (SPI0 @ 10 MHz)
       ▼
┌────────────────────────────────────────────────────────┐
│ CORE 0: Acquisition & I/O Layer (Asynchronous)         │
│  • CanBusDriver::poll() -> Reads OBD-II PIDs 0x0C/0x0D │
│  • BenchSimulator       -> Virtual 0-108 km/h ramp     │
│  • SerialLogger         -> Streams CSV telemetry (USB) │
└──────────────────────────┬─────────────────────────────┘
                           │ VehicleState (Spinlock / Mutex)
                           ▼
┌────────────────────────────────────────────────────────┐
│ CORE 1: Deterministic Math Engine (Fixed 20 Hz Timer)  │
│  • Δt = 0.05s (50ms strict hardware alarm)             │
│  • EMA Filter   -> Attenuates speed quantization noise │
│  • MathEngine   -> Computes a(t), j(t), s(t), P_cv(t)  │
│  • ShiftLight   -> Bit-bangs 8-bit mask to SN74HC164N  │
└──────────────────────────┬─────────────────────────────┘
                           │ (GPIO DATA + CLOCK)
                           ▼
             [8-LED Power/Shift Bar (SN74HC164N)]
```

### Repository Structure

```text
rp2040-can-telemetry/
├── platformio.ini              # RP2040 build config & autowp-mcp2515 dependency
├── include/
│   ├── Config.h                # GPIO pinout, vehicle constants (mass, Cd, area), and Δt
│   ├── VehicleState.h          # Mutex-protected struct shared between Core 0 and Core 1
│   ├── CanBusDriver.h          # MCP2515 SPI driver (Normal & Loopback modes)
│   ├── MathEngine.h            # EMA filter, numerical calculus, and tractive power model
│   └── ShiftRegister74HC164.h  # 8-LED bar graph controller via serial-to-parallel shifting
└── src/
    ├── main.cpp                # Dual-core entry points (setup/loop & setup1/loop1)
    ├── CanBusDriver.cpp
    ├── MathEngine.cpp
    └── ShiftRegister74HC164.cpp
```

---

## 📐 Mathematical Model (`MathEngine`)

All kinematic and dynamic estimations run at a fixed sampling period of $\Delta t = 0.05\text{ s}$ ($20\text{ Hz}$).

### 1. Exponential Moving Average (EMA) Filter
Raw OBD-II speed (`PID 0x0D`) has a $1\text{ km/h}$ ($0.278\text{ m/s}$) integer resolution. Differentiation amplifies quantization steps, so the signal is pre-filtered ($\alpha = 0.35$):

$$v_f[n] = \alpha \cdot v_{\text{raw}}[n] + (1 - \alpha) \cdot v_f[n-1]$$

### 2. Longitudinal Acceleration (1st Numerical Derivative)
Computed via backward finite difference on the filtered velocity:

$$a[n] = \frac{dv}{dt} \approx \frac{v_f[n] - v_f[n-1]}{\Delta t}$$

### 3. Longitudinal Jerk (2nd Numerical Derivative)
Measures rate of change of acceleration (ride smoothness / shift shock):

$$j[n] = \frac{da}{dt} = \frac{d^2v}{dt^2} \approx \frac{a[n] - a[n-1]}{\Delta t}$$

### 4. Traveled Distance (Trapezoidal Integration + Zero-Velocity Update)
Integrated numerically using the trapezoidal rule. To eliminate drift when idling, accumulation is gated by a Zero-Velocity Update (ZUPT) threshold ($0.1\text{ m/s}$):

$$s[n] = \int_{0}^{t} v(\tau)\,d\tau \approx \begin{cases} s[n-1] + \dfrac{v_f[n] + v_f[n-1]}{2} \cdot \Delta t, & \text{if } v_f[n] > 0.1\text{ m/s} \\ s[n-1], & \text{if } v_f[n] \le 0.1\text{ m/s} \end{cases}$$

### 5. Instantaneous Tractive Power (Horsepower / CV)
Total longitudinal force accounts for inertial resistance, aerodynamic drag, and rolling resistance:

$$F_{\text{total}}[n] = m \cdot a[n] + \frac{1}{2} \rho C_d A \cdot (v_f[n])^2 + C_{rr} \cdot m \cdot g$$

$$P_{\text{cv}}[n] = \max\left(0,\; \frac{F_{\text{total}}[n] \cdot v_f[n]}{735.5}\right)$$

| Symbol | Parameter | Default Reference Value |
| :--- | :--- | :--- |
| $m$ | Vehicle curb mass + driver | $1100\text{ kg}$ |
| $\rho$ | Air density at sea level | $1.225\text{ kg/m}^3$ |
| $C_d$ | Aerodynamic drag coefficient | $0.32$ |
| $A$ | Vehicle frontal area | $2.10\text{ m}^2$ |
| $C_{rr}$ | Tire rolling resistance coefficient | $0.015$ |
| $g$ | Gravitational acceleration | $9.81\text{ m/s}^2$ |

---

## 🔌 Hardware & Pinout

### Bill of Materials (BOM)
* **MCU:** Raspberry Pi Pico (`RP2040` Dual-Core, USB Type-C)
* **CAN Interface:** `MCP2515` CAN Controller + `TJA1050` Transceiver Module ($8\text{ MHz}$ crystal)
* **Display Actuator:** `SN74HC164N` 8-Bit Parallel-Out Serial Shift Register + $8\times$ LEDs + $8\times 330\,\Omega$ resistors
* **Power & Level Shifting:** `MB-102` Breadboard Power Supply ($3.3\text{V} / 5\text{V}$) + resistive voltage dividers ($1\text{ k}\Omega / 2\text{ k}\Omega$)

### Wiring Table

| Peripheral | Module Pin | RP2040 GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **MCP2515 (SPI0)** | `SCK` | `GP18` | $3.3\text{V}$ logic |
| **MCP2515 (SPI0)** | `SI` (MOSI) | `GP19` | $3.3\text{V}$ logic |
| **MCP2515 (SPI0)** | `SO` (MISO) | `GP16` | ⚠️ **Requires $1\text{ k}\Omega + 2\text{ k}\Omega$ divider** ($5\text{V} \rightarrow 3.3\text{V}$) |
| **MCP2515 (SPI0)** | `CS` | `GP17` | Chip Select |
| **MCP2515 (SPI0)** | `INT` | `GP20` | ⚠️ **Requires $1\text{ k}\Omega + 2\text{ k}\Omega$ divider** ($5\text{V} \rightarrow 3.3\text{V}$) |
| **SN74HC164N** | `A & B` (DATA) | `GP14` | Powered at $3.3\text{V}$ |
| **SN74HC164N** | `CLK` | `GP15` | Rising-edge shift clock |
| **OBD-II Port** | `CAN-H` / `CAN-L` | Transceiver `H / L` | OBD-II Pin 6 (`CAN-H`) & Pin 14 (`CAN-L`) |

> **⚠️ Hardware Safety Notice:** The RP2040 GPIOs are **not 5V tolerant**. Because the `TJA1050` transceiver requires a $5\text{V}$ supply (`VBUS`), you must step down the `MISO` and `INT` return lines to $\le 3.3\text{V}$ before powering the microcontroller. Additionally, remove the $120\,\Omega$ termination jumper on the MCP2515 board when connecting to an active vehicle OBD-II bus.

---

## 🚀 Getting Started

### Prerequisites
* [PlatformIO Core](https://platformio.org/) or VS Code with the PlatformIO extension.

### Build & Flash
```bash
# Clone the repository
git clone [https://github.com/](https://github.com/)<your-username>/rp2040-can-telemetry.git
cd rp2040-can-telemetry

# Build firmware and upload to RP2040
# Open Serial Monitor (115200 baud) to inspect CSV telemetry
pio device monitor -b 115200
```

### Telemetry CSV Output Format
```csv
time_s,rpm,speed_ms,accel_ms2,jerk_ms3,distance_m,power_cv
0.05,850,0.00,0.00,0.00,0.00,0.00
0.10,1420,1.50,3.00,60.00,0.04,6.42

## 📋 MVP Milestones

- [x] **M1 — Hardware & Electrical Safety:** Solder headers, verify $\le 3.35\text{V}$ logic levels on SPI return lines, and wire the `SN74HC164N` 8-LED bar.
- [ ] **M2 — Deterministic Core 1 Timer:** Validate strict $50\text{ ms}$ ($\pm 0.5\text{ ms}$) execution jitter and LED shift register sweep.
- [ ] **M3 — CAN Loopback Simulation:** Inject synthetic OBD-II `0x0C` / `0x0D` frames ($0 \rightarrow 30\text{ m/s}$ in $10\text{ s}$) with zero packet loss.
- [ ] **M4 — Numerical Calculus Validation:** Verify analytical accuracy ($< 2\%$ error against $a = 3.0\text{ m/s}^2$ and $s = 150.0\text{ m}$) and map power output to the 8-LED bar.
- [ ] **M5 — Live Vehicle Test:** Capture a $60\text{ s}+$ real driving log via OBD-II (`500 kbps`) and plot Speed, Acceleration/Jerk, and Power vs. RPM curves.```

---

