# Engineering baseline

The current baseline contains truth dynamics, four sensor models and
onboard acquisition, an independent onboard SGP4 orbit reference, inertial Sun and
magnetic references, TRIAD attitude initialization and six-state MEKF attitude/
gyro-bias estimation.

## Model/data ownership

`Flight Dynamics -> Sensors -> Drivers -> GNC`

- Flight Dynamics owns physical truth and environment products.
- Sensors own measurement physics and interface DataReady events.
- Drivers own receipt time, sequence counters and retained reports. Receipt time
  is not measurement time when transport delay is introduced. Transport currently
  has zero latency/loss; reports are not a realistic hardware driver stack.
- GNC consumes reports and decides freshness/age and validity. It has no access to
  truth through the report contract. Its SGP4 position reference is independent
  of GNSS; GNSS remains a measurement for future navigation-state estimation.
- JSON is the scenario source of truth. An `extends` path is relative to the file
  containing it. Cycles are errors; later parents and then the child take priority.
- `create*Bus.m` files are the version-controlled interface schema. Creating buses
  in MATLAB is an intentional supported design. The base workspace holds shared
  schema definitions; a second binary data dictionary would duplicate the schema.
- `setupAocsSimulation` supports interactive use. It publishes default config
  parameters but never compiles or downloads. Batch runs use
  `createAocsSimulationInput`: scenario values and mask settings are scoped to the
  simulation, so preparing a scenario does not edit the saved model or overwrite
  another scenario's base-workspace parameters.
- External mask configuration is intentional and centralized in
  `applyAocsSimulationSettings`. Calling its two-argument form is an explicit
  interactive edit; passing `SimulationInput` prepares temporary overrides.

MATLAB supports [programmatic bus definitions](https://www.mathworks.com/help/simulink/ug/create-bus-objects.html)
and [SimulationInput overrides](https://www.mathworks.com/help/simulink/slref/simulink.simulationinput.html).
A data dictionary becomes useful when interfaces are shared across separately
owned referenced models; it is not a prerequisite for professional use.

## Project MAB subset

We adapt the [MAB guidelines](https://www.mathworks.com/help/simulink/mab-modeling-guidelines.html)
to a continuous simulation plant with a discrete onboard interface. These are
project rules and checks, not a replacement for the complete licensed Model
Advisor MAB check set.

| Rule | Application and evidence |
|---|---|
| Clear hierarchy and left-to-right flow | Top-level four-layer architecture; hand-controlled layout in `styleAocsModel`, exported diagrams reviewed visually |
| Readable block names and diagrams | Meaningful subsystem/port names, black on white, no illegible subsystem previews at the reviewed architecture, Sensors and Drivers levels |
| Orthogonal signal lines | Explicit routing on the architecture and driver diagrams; physics internals are preserved |
| Explicit unused signals (MAB db_0081) | TRIAD seeds MEKF; attitude estimate and health feed control; `checkAocsModel` rejects unconnected project-owned data ports |
| No accidental dangling wiring | Unconnected input/output/line diagnostics set to `error`; model compilation checks actual wiring |
| Numerical failures visible | An enabled assertion checks every numeric `PlantStateBus` leaf for finiteness; NaN/Inf injection tests verify failure; integer sequence wrap is intentional |
| Defined interfaces | Typed buses, explicit SI units/frames in contracts; raw measurements separated from driver metadata |
| Reproducible timing | Sensor periods come from JSON; report hold and event reception are tested independently |
| Configuration separated from algorithms | Validated JSON and temporary simulation overrides; no downloaded dependencies in initialization callbacks |

Exceptions: the truth plant legitimately uses continuous states and a variable
step solver. Fixed-step scheduling, production data types, code-generation rules,
SIL/PIL/HIL and execution-time analysis belong to a future deployable controller.
Vendor library internals are outside the custom static checker. The global
`SignalInfNanChecking` setting stays `none`: Aerospace ECI/ECEF `Check deltaT`
uses an intentional `Inf` bound in an internal Constant. Our enabled
`Flight Dynamics/Numerical Checks` assertion checks every numeric truth-bus field
instead. The static check also detects a monitor out of sync with the bus schema. Visual inspection
covers the top-level, Sensors and Drivers diagrams, not every vendor block. Legacy
Aerospace attitude field names (`q_be`, `omega_b`, `DCM_be`) retain the documented
project convention: scalar-first inertial-to-body quaternion/DCM; do not infer an
Earth-fixed attitude frame from those historic field suffixes.

## Verification and traceability

### Orbital navigation

The MATLAB algorithms live under `src/gnc/navigation/orbit`; attitude
estimation lives alongside them in `src/gnc/navigation/attitude_estimation`.

The main attitude estimator uses a MATLAB Function block calling `mekfStep.m`.
Unit Delay owns its estimate, accepted gyro history and last correction time.
The migration preserves TRIAD initialization, gyro quadrature and coning,
sequential magnetic and Sun corrections, Joseph covariance and reset, NIS gates
and health thresholds. The quaternion remains scalar first and maps GCRF to body.
An invalid innovation covariance rejects the correction instead of stopping the
simulation in the former Cholesky Solver block.

Migration verification passed 97 MATLAB tests, five comparisons against saved
block-filter traces, a 35-step MATLAB/MEX comparison and a standalone C check
compiled with Clang. Evidence is saved in
`results/validation/mekf_function_migration_2026-10-11`.

`GNC/Orbit Determination & Navigation` executes after Sensor Access at 10 Hz.
The MATLAB Function block calls `orbitNavigationStep.m`; Unit Delay owns its
state and covariance. Five fixed bus contracts are defined in
`createOrbitNavigationBuses.m`, with configuration in `AOCS.NavigationConfig`.
Navigation supplies one common orbit state to LVLH guidance and the onboard
magnetic/Sun reference models. `GNC/Guidance` is a separate function-call subsystem
that consumes NavigationState directly and generates the
reference quaternion, angular rate and angular acceleration. Its enabled LVLH
core waits for a valid current navigation solution. Time validity is owned by
navigation; guidance checks only `valid` and LVLH geometry. The execution order
is Sensor Access, Navigation, Reference Vectors, TRIAD, MEKF, Guidance, Control.
The SGP4 reference path is removed from the operational loop.
Guidance restoration passed 31 architecture, sensor and PD/PID integration
tests; evidence is in `results/validation/guidance_restoration_2026-10-11`.

The six-state EKF uses GCRF position and velocity in SI units. A valid first GNSS
fix initializes state and covariance. Prediction uses central gravity and J2,
with RK4 integration of both state and transition matrix. Each navigation step
accepts at most 1 s of propagation; longer scheduling gaps preserve the old
state and report an invalid current solution. Process noise uses the short-step
white acceleration model. The Earth axis is held at its initial epoch direction.
GNSS covariance includes white noise and stationary Gauss Markov variance;
ionospheric and periodic biases are not explicitly estimated.

GNSS updates use identity measurement Jacobian and Joseph covariance correction.
NIS is computed from the prior innovation and logged as a diagnostic. It does
not gate correction, including when its scalar calculation overflows. Finite
state, valid orbital geometry, positive definite covariance and valid innovation
factorization remain numerical requirements. Reset has priority over a new fix;
duplicate, stale and future fixes cannot be reused for correction.

`navigation.valid` denotes a numerically valid state at the execution time, not
qualification against a mission accuracy budget. Health modes are 0 waiting,
1 GNSS updated, 2 coasting and 3 invalid; state sources are 0 unavailable,
1 GNSS and 2 prediction. `solution_id` advances once per committed state update
and wraps as uint32. `algorithm_ready` reports that the EKF step is implemented.
NIS is unavailable on steps without an eligible correction.

Before the first valid GNSS fix, environment reference calculations and LVLH
are disabled. The initial desired quaternion is identity and rates/accelerations
are zero; reference validity prevents controller torque. LVLH derivative memory
resets on re-enable and uses the 0.1 s GNC period rather than the old SGP4 period.
Its rate model remains the locally planar orbit approximation; acceleration is
the backward difference of that rate, not the exact perturbed-frame derivative.
Reference metadata is `orbit_update_time_s` and `orbit_solution_id`; TRIAD records
the orbit solution identifier consumed by its vector pair. Reference bus element
names come from their declared schemas.

The temporary interface uses `receive_time_s` as measurement epoch under zero
transport latency. Delayed measurement handling and the flight software state
machine remain separate integration work. Logs `OrbitNavigationStateLog`,
`OrbitNavigationHealthLog` and `OrbitNavigationBufferLog` expose the execution.
`OrbitNavigationTest` checks initialization, prediction, correction, diagnostic
NIS, reset, timing and numerical failures. `OrbitNavigationIntegrationTest`
checks PD/PID startup, shared orbit metadata, LVLH quaternion/rate geometry and first-sample acceleration
in the full plant; it belongs to the FullPlant test profile.

`run_aocs_tests("core")` needs no native compiler or downloaded DTM source.
`run_aocs_tests("full")` runs every suite after explicit native bootstrap.
`run_aocs_tests("validation")` additionally requires the optional Planet fixture.
One-orbit and one-day cases remain opt-in (`AOCS_RUN_LONG_EXTERNAL_VALIDATION=1`).
JUnit XML, JSON counts and MATLAB TestResult diagnostics are stored in a unique
`test-results/` directory. Failed setup/tests return a nonzero MATLAB batch exit.
Skipped tests remain visible and never count as passes.

| Requirement | Executable evidence | Acceptance / interpretation |
|---|---|---|
| INF-01: deterministic scenario loading | `ProjectInfrastructureTest` | Relative-path collision, cyclic and missing parents; exact expected config/error |
| INF-02: isolated experiments | `ProjectInfrastructureTest`, `PlantReadinessTest` | Preparing input leaves model/base config unchanged; actual run restores settings and produces provenance |
| INF-03: checked model structure | `ProjectInfrastructureTest/modelFollowsProjectGuidelines` | Project MAB subset has no reported violations |
| INF-04: finite truth state | `PlantNumericsTest` | Finite bus passes; runtime NaN in attitude and Inf in environment stop at the production assertion |
| SEN-01: measurement/report contracts | `SensorsTest` tags Configuration, Contracts, Wiring | Types, fields and four production acquisition interfaces |
| SEN-02: report timing | `SensorsTest` tags Acquisition, Integration | Event-only updates, independent rates, held payload/time/sequence, invalid reports and uint32 rollover |
| SEN-03: measurement behavior | `SensorsTest` tag MeasurementModels | Deterministic selected measurement cases; not comprehensive sensor metrology/statistical qualification |
| GNC-01: reference/init contracts | `AttitudeInitializationTest` tags Configuration, Contracts, Wiring | Typed SGP4/reference/TRIAD contracts; no GNSS or direct truth input to references |
| GNC-02: SGP4 and TRIAD | `AttitudeInitializationTest` tag Algorithms | Finite moving orbit, known-attitude recovery and collinearity rejection |
| ENV-01: native atmosphere | `Dtm2020NativeTest`, `Dtm2020AtmosphereHarnessTest` | CNES frozen density relative tolerance 1e-5; temperatures absolute tolerance 0.002 K |
| ENV-02: aerodynamic/SRP equations | `SentmanPanelAerodynamicsTest`, `AerodynamicsSinglePlateHarnessTest`, `SolarRadiationPressureTest` | Per-case analytical and frozen reference tolerances in test code |
| FRM-01: Earth frame transforms | `SentinelPodErfaEciEcefValidationTest` | Sentinel trajectory with independent ERFA/SOFA reference; thresholds in test code |
| ENV-03: geomagnetics | `SwarmMagneticValidationTest` | Independent reference and measured-field residuals; distinguish model mismatch from sensor error |
| ORB-01: short-arc propagation sanity | `PlanetDoveOrbitPropagationValidationTest` | 900 s; position max <=25 km, RMS <=10 km, velocity max <=25 m/s |
| DYN-01: torque-free conservation | `PlantReadinessTest`, `validate_aocs_results` | Energy and angular-momentum-norm drift below JSON limits; missing torque log is an error |

Planet OEM is a **predicted ephemeris comparison**, not independent reconstructed
truth or a measured orbit-accuracy certification. Its current thresholds are
loose regression guardrails, not demonstrated accuracy or a mission error budget.
Actual residuals must accompany the thresholds in any reported validation result.
Sentinel/Swarm reference provenance lives in their `validation/` directories.

## Evidence for a simulation

`run_aocs_simulation` returns `[out, AOCS, runDirectory]`. Each unique run directory
stores resolved configuration, `simulation.mat`, and metadata: MATLAB/product
versions, platform, Git revision and dirty status, per-file SHA-256 hashes and
native build provenance. A failed simulation records its error. The configured
results file is a convenience latest-result copy for existing plotting scripts.
Archived runs reject unsaved model edits or a same-named model from another checkout.
Hashes identify local content but are not a backup of uncommitted source: commit
reviewed source when publishing a reproducible experiment.

Default 2026 Earth orientation may use **predicted IERS data** from the installed
Aerospace dataset. Warnings are preserved. Freeze/reference the appropriate EOP
and ephemeris datasets before claiming epoch-specific precision.

## Portability and CI

Use [getting started](getting_started.md) for the supported/untested matrix.
GitHub Actions runs a three-platform core matrix and a Linux native/full job.
These definitions are not evidence of successful remote runs until CI executes.
The workflow uses official [MATLAB Actions](https://github.com/matlab-actions/setup-matlab);
private repositories require an appropriate `MLM_LICENSE_TOKEN` repository secret.

Future increments: MEKF stochastic consistency and timing qualification, GNSS-aided
orbit estimation, Windows native toolchain validation, full
MAB/Model Advisor review for the controller, hardware transport delay/loss, B-dot
detumbling and a closed-loop performance requirement with an explicit initial-rate
envelope.

`AOCSSimulation.prj` and `resources/project/` are source artifacts generated by MATLAB Project. Keep the XML metadata in version control; do not edit it by hand. The multiple-file format avoids a single shared metadata merge hotspot.

Project lifecycle helpers live in `tools/project/`; diagnostics, provenance and
native builds have separate `tools/` subfolders. The production Simulink model is
edited directly. `tools/model_builders/styleAocsModel.m` is the only optional
model utility and changes visual style without changing geometry or signal routes.
Only bootstrap, simulation, test and path-setup entry points remain as MATLAB
files at the repository root.
`setupAocsPaths` locates the checkout from its own file, so it also works before a
MATLAB Project is opened. Plotting and result checks live in `src/analysis/`.
Generated Simulink files go to `build/simulink/` through session-local
`Simulink.fileGenControl` settings; source-model layout is independent of this
filesystem organization.
