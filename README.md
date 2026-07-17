# CubeSat AOCS Simulator

MATLAB/Simulink simulator for CubeSat Attitude and Orbit Control System.

The current model includes:

- rigid-body attitude dynamics with quaternion kinematics,
- Keplerian orbit propagation,
- Frame transformations,
- IGRF-14 magnetic field model,
- Planetary Ephemeris Sun vector from Aerospace Blockset,
- residual magnetic moment, gravity-gradient, and SRP disturbance torques,
- dual-cone eclipse modeling for shadowed solar flux,
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

It defines the simulation time span, solver settings, UTC epoch and TDB offset,
initial Keplerian orbit, spacecraft inertia, initial attitude/rates, and
environment parameters. The loader validates this file and creates the normalized
`AOCS` struct used by Simulink setup and analysis:

```text
src/config/loadAocsSimulationConfig.m
```

Scenario configs can extend the default config with small overrides:

```text
config/scenarios/srp_disabled.json
config/scenarios/no_disturbance_torques.json
config/scenarios/eclipse_disabled.json
```

Example:

```matlab
run_aocs_simulation("config/scenarios/no_disturbance_torques.json")
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
│   ├── Orbit State & Geodetic Position
│   ├── Sun Products
│   ├── Eclipse Model
│   ├── Environment Bus Assembly
│   └── AOCS_EnvironmentBus
└── Torque Sum
```

## Documentation

Detailed project documentation in `docs/`:

- [Frame transformations and conventions](docs/transformations.md)
- [Sun, eclipse, and SRP modeling](docs/sun_environment_modeling.md)

## Tests

Run attitude-dynamics regression tests:

```matlab
results = runtests("tests/attitude_dynamics");
```

Run orbit and environment regression tests:

```matlab
results = runtests("tests/orbit_and_environment");
```

## Repository Layout

```text
config/
  AocsSimulationConfig.json
  scenarios/
docs/
  transformations.md
  sun_environment_modeling.md
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
```
