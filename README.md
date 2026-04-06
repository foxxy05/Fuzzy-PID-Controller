# Fuzzy-PID Controller

A self-tuning PID controller that uses a **Fuzzy Logic system** to dynamically adjust the proportional (Kp) and derivative (Kd) gains in real time based on the current error and rate of change of error. Built initially for orientation control of a 3-wheel omni-drive robot using a BNO055 IMU, but the fuzzy engine is general-purpose and can be adapted to any PID-controlled system.

---

## Table of Contents

- [What Is This?](#what-is-this)
- [How It Works](#how-it-works)
- [Repository Structure](#repository-structure)
- [Quick Start](#quick-start)
- [Understanding the Fuzzy System](#understanding-the-fuzzy-system)
- [Test Results](#test-results)
- [Hardware Implementation](#hardware-implementation)
- [Adapting to Your System](#adapting-to-your-system)
- [Dependencies](#dependencies)
- [Roadmap](#roadmap)

---

## What Is This?

A standard PID controller uses fixed gains (Kp, Ki, Kd). This works fine for simple, well-modelled systems — but in practice, the best gains for a large error are different from the best gains near the setpoint. A **Fuzzy-PID** controller solves this by using a rule-based fuzzy system to adjust Kp and Kd on every control cycle based on:

- **Error (e):** how far the system is from the target
- **Delta Error (Δe):** how fast the error is changing

The result is a controller that is more aggressive when far from the setpoint and more gentle when approaching it — without manually scheduling gains.

---

## How It Works

```
Sensor Reading
      |
      v
  Compute Error (e) and Delta Error (Δe)
      |
      v
  Fuzzify e and Δe
  (convert real values into fuzzy membership degrees)
      |
      v
  Apply Rule Base (7×7 lookup table)
  (Mamdani MIN-MAX inference)
      |
      v
  Defuzzify → delta_Kp, delta_Kd
  (weighted centroid method)
      |
      v
  Apply gains:
    Kp_actual = Kp_base × (1 + delta_Kp)
    Kd_actual = Kd_base × (1 + delta_Kd)
      |
      v
  PID Output → Actuator
```

The fuzzy system uses **7 linguistic sets** for both inputs and outputs:

| Label | Meaning         |
|-------|----------------|
| NB    | Negative Big   |
| NM    | Negative Medium|
| NS    | Negative Small |
| ZO    | Zero           |
| PS    | Positive Small |
| PM    | Positive Medium|
| PB    | Positive Big   |

This gives a **7×7 = 49-rule** rule base for each of Kp and Kd.

---

## Repository Structure

```
Fuzzy-PID-Controller/
│
├── Test-File/
│   └── test_Case.py                  ← Main script: runs the fuzzy system, sweeps the
│                                       full input space, generates CSVs and all plots
│
├── Fuzzy-Controller-Engine/
│   └── Fuzzy_Controller_Test_2.c     ← C implementation of the fuzzy engine
│                                       (ready to port to embedded targets)
│
├── Hardware-Implementation/
│   ├── 3-Wheel-Omni-Master/
│   │   └── 3-Wheel-Omni-Master.ino  ← Arduino/STM32 firmware for 3-wheel omni robot
│   └── Fuzzy-Controller-Attempt/
│       └── Fuzzy-Controller-Attempt.ino  ← Earlier hardware integration attempt
│
├── Results/
│   ├── Test-1/                       ← Output: plots + CSV
│   ├── Test-2/
│   ├── Test-3/
│   ├── Test-4/
│   ├── Test-5/
│   └── Fuzzy-Test-1/                 ← Output from earlier analysis script
│
└── Stash/
    ├── enhanced_fuzzy_v2.py          ← Earlier analysis script (superseded)
    ├── enhanced_fuzzy_analysis_complete.py
    └── Fuzzy_Controller_Test.cpp
```

---

## Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/foxxy05/Fuzzy-PID-Controller.git
cd Fuzzy-PID-Controller
```

### 2. Install Python dependencies

```bash
pip install numpy matplotlib
```

> No additional fuzzy logic libraries needed. The fuzzy system is implemented from scratch.

### 3. Run the fuzzy surface sweep

```bash
cd Test-File
python test_Case.py
```

This will:
- Sweep the full input space: error ∈ [-180°, 180°], Δe ∈ [-1200, 1200] deg/s
- Compute delta_Kp and delta_Kd for every combination
- Save results to `fuzzy_surface_all_cases.csv`
- Generate 4 plots:
  - `delta_kp_surface.png` — 3D surface of Kp adjustment
  - `delta_kp_heatmap.png` — 2D heatmap of Kp adjustment
  - `delta_kd_surface.png` — 3D surface of Kd adjustment
  - `delta_kd_heatmap.png` — 2D heatmap of Kd adjustment

All output files are saved in the same directory you run the script from.

---

## Understanding the Fuzzy System

### Membership Functions

Both inputs use **triangular membership functions** defined by three points `[a, b, c]`:

**Error (range: -180° to +180°)**

| Set | a      | b      | c     |
|-----|--------|--------|-------|
| NB  | -180.0 | -180.0 | -90.0 |
| NM  | -180.0 | -90.0  | 0.0   |
| NS  | -90.0  | 0.0    | 90.0  |
| ZO  | -20.0  | 0.0    | 20.0  |
| PS  | 0.0    | 20.0   | 40.0  |
| PM  | 20.0   | 70.0   | 140.0 |
| PB  | 90.0   | 180.0  | 180.0 |

**Delta Error (range: -1200 to +1200 deg/s)**

| Set | a       | b       | c      |
|-----|---------|---------|--------|
| NB  | -1200.0 | -1200.0 | -680.0 |
| NM  | -950.0  | -400.0  | 0.0    |
| NS  | -240.0  | -100.0  | 0.0    |
| ZO  | -80.0   | 0.0     | 80.0   |
| PS  | 0.0     | 100.0   | 240.0  |
| PM  | 0.0     | 400.0   | 950.0  |
| PB  | 680.0   | 1200.0  | 1200.0 |

### Output Centroids

Both delta_Kp and delta_Kd share the same output singleton centroids:

```
NB=-0.20, NM=-0.15, NS=-0.075, ZO=0.0, PS=0.075, PM=0.15, PB=0.20
```

These are **fractional adjustments** — they scale the base gains, not replace them.

### Rule Bases

**Kp Rule Base** (rows = Δe sets, columns = e sets, values = output set index):

```
       NB  NM  NS  ZO  PS  PM  PB   ← error
NB  [  3,  4,  5,  6,  6,  6,  5 ]
NM  [  2,  3,  4,  6,  5,  5,  4 ]
NS  [  1,  2,  3,  5,  4,  4,  1 ]
ZO  [  2,  1,  1,  2,  2,  1,  2 ]
PS  [  5,  4,  3,  1,  3,  2,  1 ]
PM  [  6,  5,  4,  2,  3,  2,  1 ]
PB  [  5,  6,  5,  2,  2,  1,  3 ]
↑ delta_error
```

**Kd Rule Base:**

```
       NB  NM  NS  ZO  PS  PM  PB
NB  [  2,  1,  0,  0,  0,  1,  2 ]
NM  [  3,  2,  1,  0,  1,  2,  3 ]
NS  [  4,  3,  2,  1,  2,  3,  4 ]
ZO  [  5,  4,  3,  3,  3,  4,  5 ]
PS  [  6,  5,  4,  1,  2,  3,  4 ]
PM  [  6,  6,  5,  4,  4,  3,  1 ]
PB  [  6,  6,  6,  5,  4,  4,  3 ]
```

### Inference Method

- **Fuzzification:** triangular membership (shoulder MFs at boundaries)
- **Inference:** Mamdani MIN (firing strength) → MAX aggregation
- **Defuzzification:** weighted centroid (centre of gravity across singletons)

### Gain Update Formula

```
Kp_actual = Kp_base × (1 + delta_Kp)
Kd_actual = Kd_base × (1 + delta_Kd)
```

Where `delta_Kp` and `delta_Kd` ∈ [-0.20, +0.20] — so the fuzzy system adjusts gains by at most ±20% from the base values.

---

## Test Results

Each `Results/Test-N/` folder contains:

| File | Description |
|------|-------------|
| `fuzzy_surface_all_cases.csv` | Full grid sweep data (error, delta_error, delta_Kp, delta_Kd) |
| `delta_kp_surface.png` | 3D surface plot of Kp adjustment |
| `delta_kp_heatmap.png` | 2D heatmap of Kp adjustment |
| `delta_kd_surface.png` | 3D surface plot of Kd adjustment |
| `delta_kd_heatmap.png` | 2D heatmap of Kd adjustment |

Different test runs correspond to different membership function tunings and rule base iterations. Test-5 is the most recent.

---

## Hardware Implementation

The `Hardware-Implementation/` folder contains Arduino/STM32 firmware:

- **3-Wheel-Omni-Master.ino** — Full omni-wheel robot firmware. Reads orientation from a BNO055 IMU over I2C, computes error, runs the fuzzy-PID loop, drives motors via PCA9685 PWM driver.
- **Fuzzy-Controller-Attempt.ino** — An earlier standalone integration attempt of the fuzzy tuner.

The C implementation in `Fuzzy-Controller-Engine/Fuzzy_Controller_Test_2.c` is a clean, dependency-free version of the fuzzy engine that can be directly ported to any embedded target (STM32, Arduino, ESP32, etc.).

**To use the C engine in your firmware:**
1. Copy `Fuzzy_Controller_Test_2.c` into your project
2. Call `updateGains(error, delta_error, &Kp, &Kd)` on every control cycle
3. Set `Kp_Base` and `Kd_Base` inside the function to your tuned base values
4. Use the returned `Kp` and `Kd` in your PID computation

---

## Adapting to Your System

### Step 1 — Set your input ranges

In `test_Case.py`, change the sweep ranges to match your system:

```python
ERROR_MIN, ERROR_MAX, ERROR_STEP = -180.0, 180.0, 5.0       # degrees (orientation)
DE_MIN, DE_MAX, DE_STEP = -1200.0, 1200.0, 50.0              # deg/s (gyro rate)
```

### Step 2 — Rescale membership functions

The error MFs cover ±180° and delta-error MFs cover ±1200 deg/s — tuned for BNO055 orientation control. If your system has different units or ranges, rescale the `error_mf_points` and `delta_error_mf_points` arrays proportionally.

### Step 3 — Tune the rule bases

The rule bases encode the control strategy. The general intuition is:
- Large error + large Δe → large Kp (be aggressive)
- Near zero error + small Δe → small Kp (be gentle)
- Kd is higher when Δe is large (damp oscillations)

Modify `KP_Rule_Base` and `KD_Rule_Base` to reflect your system's behaviour.

### Step 4 — Set base gains

In the C engine (`Fuzzy_Controller_Test_2.c`):

```c
double Kp = 10.0;   // your Kp_base
double Kd = 5.0;    // your Kd_base
updateGains(error, delta_error, &Kp, &Kd);
```

Tune `Kp_base` and `Kd_base` as you would a regular PD controller, then let the fuzzy system handle the dynamic adjustment.

---

## Dependencies

**Python (for simulation and visualisation):**

```
numpy
matplotlib
```

Install via:
```bash
pip install numpy matplotlib
```

**C engine:** standard C only (`stdio.h`, `math.h`). No external libraries.

**Hardware firmware:** Arduino IDE or PlatformIO with STM32 / ESP32 board support.

---

## Roadmap

- [ ] Add Ki fuzzy rule base (Ki tuning is commented out in the C engine — rule base exists, pending validation)
- [ ] README plots inline (embed sample heatmaps)
- [ ] Clean hardware firmware with modular fuzzy header (`.h` / `.c` split)
- [ ] Real-time serial plotter for hardware-in-the-loop testing
- [ ] Python simulation with a plant model (step response visualisation)

---

## Author

**Atreya** — [@foxxy05](https://github.com/foxxy05)
