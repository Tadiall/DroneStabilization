# Drone Attitude Stabilization Simulation PID

Simulation of the stabilization of one rotation axis (pitch) of a small
quadcopter, using a PID controller, to illustrate a closed loop
control system.

## Structure

```
sim/
 dynamics.py      physical model (rotational dynamics around one axis)
 controller.py    PID controller
 simulate.py      runs the simulation, generates the plots

results/              generated plots
```

## Installation

```bash
pip install -r requirements.txt
```

## Usage

```bash
python -m sim.simulate
```

Generates `results/attitude_response.png`: pitch angle response to a
step reference, with the associated control signal (motor torque).

## Physical Model

Rotation around a single axis (pitch), simplified model:

```
I * θ̈ = τ - τ_damping
```

where:

- `I` : moment of inertia around the axis (kg·m²)
- `τ` : torque applied by the controller (N·m), saturated at ±τ_max
- `τ_damping` : viscous damping proportional to the angular velocity

Numerical integration using the explicit Euler method, with a configurable
time step `dt`.

## Controller

Standard PID:

```
τ(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·(de/dt)
```

with output torque saturation and simple anti windup (the integral is
accumulated only if the control signal is not saturated).

## Default Parameters

See `sim/simulate.py` step reference from 0° to 15°, possible initial
disturbance, PID gains to be adjusted manually (`Kp`, `Ki`, `Kd`) to observe
the effect of each term on the response (overshoot, rise time, steady state
error).

## Limitations

- Single axis model only (no coupling between pitch/roll/yaw).
- No real motor model (torque is applied instantaneously, with saturation).
- Intended to illustrate closed loop control concepts, not to represent a real drone.
