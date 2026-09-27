# AOCS Simulator

MATLAB/Simulink simulation of a 3U CubeSat: attitude/orbit dynamics, environment,
measurement models and onboard sensor acquisition. The current milestone adds an
independent onboard SGP4 orbit reference, inertial Sun/magnetic references and
TRIAD attitude initialization. A six-state MEKF is the next GNC increment.

```text
Flight Dynamics -> Sensors -> Drivers -> GNC
     truth       measurements   reports    SGP4 references + TRIAD
```

The plant includes EGM2008 spherical harmonics, Sun/Moon third-body gravity,
IGRF14, DTM2020 atmosphere, Sentman free-molecular aerodynamics, solar radiation
pressure with eclipse shadowing, gravity-gradient and residual magnetic torque.
Attitude dynamics use `I * omega_dot = M_total - omega × (I * omega)` with a
scalar-first inertial-to-body quaternion. All interfaces document units and frames.

Gyroscope, magnetometer, coarse Sun sensors and GNSS have typed measurement buses.
An event-driven driver per sensor captures each received payload, receipt timestamp
and sequence number. GNC uses an onboard SGP4 propagator, independent of GNSS, to
generate Sun and magnetic inertial reference vectors. TRIAD combines those vectors
with magnetometer and coarse Sun sensor reports to initialize `q_BI`. GNSS remains
a measurement for a future navigation filter; the MEKF subsystem is still empty.

## Start here

Baseline: **MATLAB R2025a, Simulink, Aerospace Blockset, Aerospace Toolbox**.
Open the MATLAB Project in this folder, or run from the cloned repository root:

```matlab
bootstrapAocs("core")
run_aocs_tests("core")
```

The core suite needs no Fortran compiler or downloaded DTM2020 source. For the full
plant, configure a supported C MEX compiler and GNU Fortran, then:

```matlab
bootstrapAocs("full")
run_aocs_tests("full")
[out, config, runDirectory] = run_aocs_simulation;
plot_attitude_results
plot_orbit_environment_results
```

See [getting started](docs/getting_started.md) for platform support, dependencies,
Apple-silicon CLT setup, optional datasets and troubleshooting. **Windows native
DTM2020 is not supported yet**; the Windows CI job covers the core suite.

## Repository layout

The root keeps the MATLAB Project, this README and four entry points:
`bootstrapAocs`, `run_aocs_simulation`, `run_aocs_tests` and `setupAocsPaths`.

| Folder | Contents |
|---|---|
| `models/` | Simulink plant |
| `src/` | Configuration loaders, physics, sensors and bus definitions |
| `src/analysis/` | Plotting, result validation and visualization export |
| `config/` | Parameters and simulation scenarios |
| `tools/project/`, `tools/diagnostics/` | Project lifecycle, cleanup and read-only diagnostics |
| `tools/provenance/`, `tools/native/` | Reproducibility helpers and native DTM2020 build management |
| `tools/model_builders/` | Optional visual styling; model geometry is edited directly in Simulink |
| `tests/`, `validation/` | Automated tests, harnesses and external references |
| `docs/`, `resources/project/` | Documentation and MATLAB Project metadata |
| `build/`, `results/`, `test-results/` | Ignored build/cache files and run reports |

Open the project or call `setupAocsPaths` to make the analysis and tool commands
available by name. The categorized tool folders are documented in
`tools/README.md`. Simulink cache and code generation use `build/simulink/`.

## Reproducible experiments

`config/AocsSimulationConfig.json` composes simulation, geometry, orbit/environment,
dynamics, sensor and GNC JSON files. Scenarios override only the changed values:

```matlab
run_aocs_simulation("config/scenarios/no_disturbance_torques.json")
```

Each run stores resolved configuration, MATLAB/product versions, Git state and
source hashes beside its result in a unique `results/runs/` directory. Scenario
values and block-mask overrides are scoped with `Simulink.SimulationInput`.
The configured latest-result file is retained for the plotting scripts.

## Verification

`run_aocs_tests` exports JUnit XML, JSON counts and detailed MATLAB results.
Sensor tests form one suite, `tests/sensors/SensorsTest.m`, grouped by tags:
Configuration, Contracts, Wiring, MeasurementModels, Acquisition and Integration.
Attitude-initialization tests live in `tests/gnc/AttitudeInitializationTest.m` and
cover SGP4 independence from GNSS, reference/initialization contracts, TRIAD
geometry and compiled wiring.
Other suites verify configuration infrastructure, equations, dedicated Simulink
harnesses and external references. CI defines a three-platform core matrix and a
Linux full-plant job; remote execution is separate evidence from local testing.

External checks use Sentinel POD with an independent ERFA/SOFA transformation
reference, Swarm magnetic-field data and optional Planet Dove **predicted** OEM
states. These establish bounded comparisons, not flight qualification or a
mission-wide accuracy guarantee. Optional missing/long cases are reported as
skipped, never passed.

- [Engineering rules, adapted MAB subset and requirement-to-test mapping](docs/engineering.md)
- [Local verification results and limitations](docs/verification.md)
- [Sensor and driver contracts](docs/sensors.md)
- [State estimation](docs/state_estimation.md)
- [Frames and time](docs/transformations.md)
- [Sun, eclipse and SRP](docs/sun_environment_modeling.md)
- [Atmosphere](docs/atmosphere_modeling.md)

Validation fixtures/provenance live in `validation/`; harnesses in `tests/harnesses/`.
Review dataset and third-party license terms before redistribution. No blanket
license for those external assets is implied by this repository.
