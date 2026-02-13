# TustinPID

A lightweight, deterministic, embedded‑friendly **discrete‑time PID controller** based on the **Tustin (bilinear) transform**.

Implements:
- Tustin (trapezoidal) integrator  
- Tustin filtered differentiator  
- Integrator freeze anti‑windup when output saturates  
- Fully deterministic state updates  
- Portable C++ (no Arduino dependency)

Designed for motor control, robotics, and real‑time embedded systems where numerical stability and predictable behavior matter.

---

## Features

- Discrete‑time PID using Tustin transform  
- Derivative filtering via Tustin low‑pass  
- Integrator freeze anti‑windup  
- Deterministic, allocation‑free  
- Portable C++ (Arduino, Teensy, ESP32, STM32, desktop)  
- Simple API: one call per loop

---

## Installation

### Arduino IDE
Copy the folder into:


Documents/Arduino/libraries/TustinPID


### PlatformIO

```ini
lib_deps =
    https://github.com/vishwam-aggarwal/TustinPID
```

Quick Start Example

```cpp

#include <TustinPID.h>

TustinPID pid;

void setup() {
    pid.setup(
        7.5827,   // Kp
        82.7956,  // Ki
        0.1552,   // Kd
        0.005,    // tau (derivative filter time constant)
        0.01      // Ts (sample time)
    );
}

void loop() {
    float reference = 1.0f;
    float measurement = readSensor();

    float error = reference - measurement;

    // Saturated control output
    float u = pid.getControl(error, -100.0f, 100.0f);

    applyActuator(u);
}

```
API Overview

Initialization

```cpp
void setup(float Kp, float Ki, float Kd, float tau, float Ts);

```

Compute control output

```cpp
float getControl(float error, float uMin, float uMax);
float getControl(float error);   // no saturation
```

State management

```cpp
void reset();
void setStates(float integrator, float derivative);
float getI() const;   // actual integrator contribution
```

## How It Works

###Integrator (I‑term)

Tustin (trapezoidal) rule:

[ I[k] = I[k-1] + \frac{Ki \cdot Ts}{2}(e[k] + e[k-1]) ]

### Derivative (D‑term)

Filtered differentiator:

[ D[k] = a D[k-1] + b (e[k] - e[k-1]) ]

where:

[ a = \frac{2\tau - Ts}{2\tau + Ts}, \quad b = \frac{2Kd}{2\tau + Ts} ]

## Anti‑Windup (Integrator Freeze)

When the output saturates:

the integrator does not update

getI() returns the frozen value

prevents runaway integral buildup

## Example: Closed‑Loop Simulation

The repository includes an example demonstrating:

a discrete plant

sinusoidal reference

PID tracking

real‑time plotting via Serial Plotter

Useful for tuning and validation.

Why Tustin?

The Tustin (bilinear) transform provides:

better numerical stability than Euler methods

accurate frequency‑domain behavior

smooth derivative filtering

predictable behavior at small sample times

Ideal for embedded control loops.

## License

MIT License.See LICENSE for details.

