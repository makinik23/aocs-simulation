# CubeSat AOCS Simulator
CubeSat Attitude and Orbit Control System simulation environment in MATLAB/Simulink. It ties
together spacecraft attitude dynamics, celestial mechanics, orbit environmental models, and
disturbance torques into one configurable system-level simulation.

```text
spacecraft:  3U CubeSat, 0.10 x 0.10 x 0.30 m
mass:        3.71565 kg
inertia:     diag([0.0409 0.0403 0.0073]) kg*m^2
```

## Core Capabilities

- Rigid-body CubeSat attitude dynamics with quaternion state propagation.
- Configurable orbital environment, from Kepler baseline to high-precision EGM2008.
- Magnetic field, Sun vector, eclipse, SRP, gravity-gradient, and residual magnetic moment models.
- Scenario-based configuration for repeatable mission and disturbance studies.
- Validation and plotting tools for attitude, orbit, and environment products.

## Run

```matlab
run_aocs_simulation
validate_aocs_results
plot_attitude_results
plot_orbit_environment_results
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
For example, `high_precision.json` switches the Orbit Propagator to numerical
high precision with EGM2008 spherical harmonics, degree 2159, EOP corrections,
and ICRF output.

## Tests

Validation is moving to dedicated Simulink Test harnesses. Current harness-backed
Swarm magnetic validation:

```matlab
runtests("tests/orbit_and_environment/SwarmMagneticValidationTest.m")
```

Harness models live in `tests/harnesses/`.

More detail:
[Frame transformations](docs/transformations.md) and
[Sun, eclipse, and SRP modeling](docs/sun_environment_modeling.md).
