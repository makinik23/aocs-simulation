# AOCS Simulator
Attitude and Orbit Control System simulation environment in MATLAB/Simulink. It ties
together flight dynamics simulation and GNC algorithms.

## Flight dynamics

For now, the spacecraft of interest is a simple CubeSat 3U.

- High precision orbit propagator:
    - EGM2008 gravity model.
    - IGRF14 magnetic field model.
    - Sun and Moon third body gravity.
    - Aerodynamic drag based on DTM2020 atmosphere and Sentman free-molecular flow equations.
    - A lumped constant-area SRP model with Earth–Moon dual-cone eclipse shadowing.
    - Gravity gradient and residual magnetic moment.
- Rotational dynamics:
    - I * omega_dot = M_total - omega × (I * omega)
    - q_dot = 0.5 * Omega(omega) * q
    
    where:

     - M_total = M_external + M_gravity_gradient + M_residual_magnetic + M_SRP + M_aerodynamic
- Configuration scenarios for repeatable mission cases and disturbance studies.
- Model validation against real flight data: Sentinel-1A POD for ECI/ECEF transformations and Swarm A MAG/VirES for geomagnetic field output.

## Run

```matlab
run_aocs_simulation
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
The default plant uses numerical high-precision propagation with all environment options (mentioned in Flight Dynamics Section) enabled.

## DTM2020 Setup

```bash
git clone https://github.com/swami-h2020-eu/mcm.git \
    third_party/dtm2020/upstream
git -C third_party/dtm2020/upstream checkout \
    a488a7c9d030bfbe86e88ab3d28a7ec5589b92e0
```

```matlab
addpath("tools")
buildDtm2020Native
run_aocs_simulation
```

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
runtests("tests/environment")
```

Validation data and download/reference-generation scripts live in `validation/`.
Harness models live in `tests/harnesses/`.

More detail:
[Frame transformations](docs/transformations.md) and
[Sun, eclipse, and SRP modeling](docs/sun_environment_modeling.md), plus the
[atmosphere modeling contract](docs/atmosphere_modeling.md).
