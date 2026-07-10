# CubeSat AOCS Simulator

MATLAB/Simulink simulator for CubeSat Attitude and Orbit Control System.

The current model includes:

- rigid-body attitude dynamics with quaternion kinematics,
- Keplerian orbit propagation,
- Frame transformations,
- IGRF-14 magnetic field model,
- residual magnetic moment and gravity-gradient disturbance torques,
- validation tests and diagnostic plotting utilities.

## Quick Start

Run the default simulation from MATLAB:

```matlab
run_aocs_simulation
```

Validate the latest result:

```matlab
validate_aocs_results
```

Plot attitude dynamics:

```matlab
plot_attitude_results
```

Plot orbit and environment products:

```matlab
plot_orbit_environment_results
```

Export orbit/environment figures to PNG:

```matlab
plot_orbit_environment_results("", "results/orbit_plots")
```

The default result is saved to:

```text
results/aocs_simulation_results.mat
```

## Configuration

The source of truth is:

```text
config/AocsSimulationConfig.json
```

It defines the simulation time span, solver settings, UTC epoch, initial
Keplerian orbit, spacecraft inertia, initial attitude/rates, and environment
parameters. The loader validates this file and creates the normalized `AOCS`
struct used by Simulink setup and analysis:

```text
src/config/loadAocsSimulationConfig.m
```

## Simulink Setup

The setup entrypoint is:

```matlab
AOCS = setupAocsSimulation("config/AocsSimulationConfig.json");
```

`setupAocsSimulation` creates the bus objects and `Simulink.Parameter` payloads
in the MATLAB base workspace.

`applyAocsSimulationSettings` then configures the Aerospace Blockset blocks
from the validated config, including the 6DOF plant, Kepler propagator,
ECI/LLA conversion, ECI/ECEF DCM, and IGRF.

## Model Architecture

The Simulink plant is:

```text
models/aocs_plant.slx
```

Top-level structure:

```text
AOCS Simulation
├── Config constants
├── Attitude Dynamics
│   └── AOCS_StateBus
├── Orbit & Environment
│   ├── OrbitState
│   └── EnvironmentBus
└── Torque Sum
```

## Documentation

Detailed project documentation in `docs/`:

- [Frame transformations and conventions](docs/transformations.md)

## Tests

Run attitude-dynamics regression tests:

```matlab
results = runtests("tests/attitude_dynamics");
```

Run orbit and environment regression tests:

```matlab
results = runtests("tests/orbit_and_environment");
```

Convenience wrappers:

```matlab
run_attitude_dynamics_tests
run_orbit_and_environment_tests
```

## Repository Layout

```text
config/
  AocsSimulationConfig.json
docs/
  transformations.md
models/
  aocs_plant.slx
src/
  analysis/
  config/
  environment/
  simulink/
tests/
  attitude_dynamics/
  orbit_and_environment/
run_aocs_simulation.m
validate_aocs_results.m
plot_attitude_results.m
plot_orbit_environment_results.m
plot_aocs_results.m
startup.m
```

## Current Limitations

The current Orbit & Environment stage intentionally does not yet include:

- Sun products wired into `AOCS_EnvironmentBus`,
- eclipse state,
- solar radiation pressure,
- aerodynamic drag torque,
- sensor models,
- actuators,
- estimation or control algorithms.

Recommended next order: Sun products, eclipse, SRP/aero torques, sensor models,
TRIAD/MEKF, then actuators and control.
