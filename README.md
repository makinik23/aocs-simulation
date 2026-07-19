# CubeSat AOCS Simulator
CubeSat Attitude and Orbit Control System simulation environment in MATLAB/Simulink. It ties
together flight dynamics simulation and GNC algorithms.

## Core Capabilities

- Configurable orbit and environment pipeline.
- Scenario-driven configuration for repeatable mission cases and disturbance studies.
- Model validation against real flight data: Sentinel-1A POD for ECI/ECEF transformations and Swarm A MAG/VirES for geomagnetic field output.
- Post-processing utilities for simulation diagnostics.

## Run

```matlab
run_aocs_simulation
plot_attitude_results
plot_orbit_environment_results
```

Optional local dynamics sanity check:

```matlab
validate_aocs_results
```

Scenario examples:

```matlab
run_aocs_simulation("config/scenarios/high_precision.json")
run_aocs_simulation("config/scenarios/no_disturbance_torques.json")
```

## Configuration

The main config is `config/AocsSimulationConfig.json`, composed from:

```text
config/simulation.json
config/spacecraft_geometry.json
config/orbit_environment.json
config/dynamics.json
```

Scenarios in `config/scenarios/` override only what changes between experiments.
For example, `high_precision.json` switches the Orbit Propagator from unperturbed to numerical
high precision.

## Tests

Model validation is based on dedicated Simulink harnesses and real flight data. The
ECI/ECEF transformation harness is checked against Sentinel-1A precise orbit
products with an independent ERFA/SOFA reference.
The geomagnetic environment harness is checked against Swarm A MAG Level-1B data
from VirES, including the onboard magnetic-field measurements and VirES IGRF
reference.

```matlab
runtests("tests/transformations")
runtests("tests/orbit_and_environment/SwarmMagneticValidationTest.m")
```

Validation data and download/reference-generation scripts live in `validation/`.
Harness models live in `tests/harnesses/`.

More detail:
[Frame transformations](docs/transformations.md) and
[Sun, eclipse, and SRP modeling](docs/sun_environment_modeling.md).
