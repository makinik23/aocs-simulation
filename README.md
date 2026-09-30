# AOCS Simulator
Attitude and Orbit Control System simulation environment in MATLAB/Simulink. It ties
together flight dynamics simulation and GNC algorithms for a 3U CubeSat.

## Flight dynamics

- High precision orbit propagator:
    - EGM2008 gravity model.
    - IGRF14 magnetic field model.
    - Sun and Moon third-body gravity.
    - Aerodynamic drag based on DTM2020 and Sentman free-molecular flow.
    - Lumped constant-area SRP with Earth-Moon dual-cone eclipse shadowing.
- Rotational dynamics:
    - I * omega_dot = M_total - omega × (I * omega)
    - q_dot = 0.5 * Omega(omega) * q

    External torques include gravity gradient, residual magnetic moment, SRP
    and aerodynamics.
- Configuration scenarios for repeatable mission cases and disturbance studies.

## Sensors and GNC

- Gyroscope, magnetometer, coarse Sun sensors and GNSS produce typed measurements.
  Drivers retain the latest received report and acquisition status for each sensor.
- An onboard SGP4 propagator, independent of GNSS, supplies orbit information for
  Sun and magnetic reference vectors.
- TRIAD initializes attitude from those references and sensor observations.
  A six-state MEKF then estimates attitude and gyroscope bias using gyro prediction
  and gated magnetic/Sun-vector updates. GNSS navigation filtering is future work.


## Run

Requires MATLAB R2025a, Simulink, Aerospace Blockset, Aerospace Toolbox and DSP
System Toolbox. Open the MATLAB Project or run from the repository root:

```matlab
bootstrapAocs("core")
run_aocs_tests("core")
```

For the complete plant, configure a supported C MEX compiler and GNU Fortran:

```matlab
bootstrapAocs("full")
[out, config, runDirectory] = run_aocs_simulation;
plot_attitude_results
plot_attitude_estimation_results
plot_orbit_environment_results
```

Scenario example:

```matlab
run_aocs_simulation("config/scenarios/no_disturbance_torques.json")
```

Generated files and reports remain local in `build/`, `results/` and
`test-results/`. See [getting started](docs/getting_started.md) for native
dependencies, platform support and troubleshooting.

## Configuration

The main config is `config/AocsSimulationConfig.json`, composed from:

```text
config/simulation.json
config/spacecraft_geometry.json
config/orbit_environment.json
config/dynamics.json
config/sensors.json
config/gnc.json
```

Scenarios in `config/scenarios/` override only what changes between experiments.

## DTM2020 Setup

`bootstrapAocs("full")` obtains the pinned DTM2020 source when needed and builds
the native library locally. It does not overwrite an existing modified checkout.
See [getting started](docs/getting_started.md) for compiler setup and the explicit
Apple-silicon Command Line Tools fallback.

## Tests

The core suite covers sensor and driver contracts, onboard references, TRIAD,
MEKF propagation and correction, seeded Monte Carlo trials, and infrastructure.
The full suite also includes plant and environment checks:

```matlab
run_aocs_tests("core")
run_aocs_tests("full")
```

External validation uses Sentinel-1A precise orbit products for ECI/ECEF
transformations, Swarm A magnetic-field data and optional Planet Dove 3U
**predicted** OEM states for orbit propagation. Longer Planet Dove arcs require
explicit opt-in. Missing optional cases are skipped, not passed.

Validation data and harnesses live in `validation/` and `tests/harnesses/`.
More detail:
[frame transformations](docs/transformations.md),
[Sun, eclipse and SRP modeling](docs/sun_environment_modeling.md),
[atmosphere modeling](docs/atmosphere_modeling.md) and
[sensor modeling](docs/sensors.md).
