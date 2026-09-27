# Local verification snapshot

Verified on 20-21 September 2026 with MATLAB R2025a Update 1
(`25.1.0.2973910`), macOS Apple silicon (`maca64`). These are local results;
Linux/Windows GitHub Actions jobs have been configured but were not executed here.

## Automated tests

| Run | Passed | Failed | Skipped |
|---|---:|---:|---:|
| Core suite after onboard SGP4 references and TRIAD initialization | 56 | 0 | 0 |
| Earlier full suite, before the current GNC increment | 68 | 0 | 2 |
| Core suite in a separate copy without native builds or downloaded DTM2020, with spaces in its path | 50 | 0 | 0 |

The two skipped cases are the explicitly opt-in one-orbit and 24-hour Planet
comparisons. The 900-second case ran. The full suite took approximately 247 s.
Report ID: `tp013187a4_0ff7_475b_ae00_924ca139db13` under `test-results/`.
It contains `junit.xml`, `summary.json`, `results.mat` and `console.log`.
Reports are generated artifacts; rerun `run_aocs_tests` after cloning.

The current 56-test core run includes 23 sensor/driver tests and six attitude-
initialization tests. Report ID: `tp892092ee_9a66_497b_b690_a64c4e791c86`.
The new tests cover the GNC/reference/TRIAD bus contracts, finite moving SGP4
output, exact known-attitude recovery, collinearity rejection, and compiled
wiring with no GNSS or truth input to the reference-vector branch. The remaining
core tests cover infrastructure and deterministic environment equations.

A separate 2 s full-model smoke run completed after the model checker passed.
The GNC-specific harnesses verify the numerical TRIAD result independently of
that smoke run. GNSS is deliberately not used as the onboard position reference;
the reference branch propagates its own configured SGP4 state.

Final Code Analyzer pass: **0 findings** across changed/new MATLAB files;
`git diff --check` also passed. A separate 2 s run of the default scenario
completed, left the loaded model clean (`Dirty=off`), and produced archived
results. A deliberately unsaved model edit was rejected by the run archiver.
Opening the MATLAB Project from the default MATLAB path reported no startup issues.
Default smoke run ID: `tp7ca5dc46_1133_4e2e_b38d_aef23deb580e`.

The clean-copy check used a fresh MATLAB process with `restoredefaultpath`, opened
the MATLAB Project, bootstrapped core, and verified that no native build directory
was created. This demonstrates independence from the local DTM2020 checkout and
build cache on this platform, not validation of another operating system.

## Repository layout follow-up

After moving analysis functions into `src/analysis/` and introducing the first
organized `tools/` layout, the core suite passed **50/50** again.
Report ID: `tpf1f9d01a_cb33_4d77_9dd7_31dee1ed9ee5`.
`PlantReadinessTest` separately passed its 2 s archived simulation and conservation
checks; run ID: `tpa38a194c_78ef_41ca_89b9_5046d2115d53`.

Project opening was checked from `restoredefaultpath`, including registered file
existence and relocated function resolution. A separate source copy with spaces
in its path opened and bootstrapped core without a native build or DTM checkout.
Batch bootstrap also worked without opening the project. Simulation cache/code
now target `build/simulink/`; tests and the plant run left no `slprj/` or `.slxc`
files at the repository root. No model diagrams were edited for this relocation.

The first plant check after `restoredefaultpath` could not find the installed
Aerospace ephemeris support package, whose path that command removes. The plant
check passed in a fresh session with its normal installed support-package path.
Core tests and clean-copy startup do not depend on those ephemeris files.

Code Analyzer found no issues in the changed setup/entry-point code. The relocated
orbit plotting function is byte-for-byte unchanged and retains its two existing
`MSNU` notices about obsolete suppressions; these are not new relocation findings.

## Quantitative checks

| Quantity / case | Observed | Acceptance / scope |
|---|---:|---|
| Planet OEM, 900 s, position RMS | 0.224179 m | Regression guardrail <=10,000 m |
| Planet OEM, 900 s, position max | 0.470139 m | Regression guardrail <=25,000 m |
| Planet OEM, 900 s, velocity max | 0.000994226 m/s | Regression guardrail <=25 m/s |
| Sentinel ERFA transform, position max | 0.000273188 m | <=0.05 m |
| Sentinel ERFA transform, position mean | 0.0000783081 m | <=0.01 m |
| Swarm measured-field residual, median | 22.027 nT | <=250 nT |
| Swarm measured-field residual, max | 212.205 nT | <=1,200 nT |
| Torque-free 2 s run, rotational energy drift | 2.107e-13 J | <1e-8 J |
| Torque-free 2 s run, angular momentum norm drift | 5.985e-13 kg m²/s | <1e-8 kg m²/s |

Planet is a predicted OEM comparison using a conservative validation scenario,
not measured orbit truth. Small residuals for this arc do not establish a general
orbit accuracy budget. Current guardrails intentionally remain much looser than
this one observation. Sentinel checks coordinate transformations, not orbit
propagation. Measured Swarm residuals include physical/model mismatch and are not
magnetometer sensor-noise estimates.

## Numerical diagnostics and diagram review

Unconnected input, output and line diagnostics are errors. Five unused line
fragments/branches were removed, and the TRIAD result now ends in a named
Terminator until the MEKF is implemented. The architecture, Sensors and Drivers views were exported
from Simulink and visually inspected for readable labels and signal routing.
After the final drawing-only adjustment, the complete source/destination
connection list was verified unchanged and all 23 sensor tests passed again;
the model-subset check and Code Analyzer check for the layout tool also passed.

Global NaN/Inf checking cannot be used unchanged: the Aerospace ECI/ECEF block
contains an intentional infinite interval bound. An enabled assertion now checks
every numeric leaf of `PlantStateBus` at the Flight Dynamics interface. Tests
inject NaN into angular rate and Inf into magnetic field as actual signals and
confirm that the production assertion stops simulation. A finite bus passes.

Native DTM2020 was rebuilt and passed its frozen CNES density/temperature
benchmark. Local builds use an explicit CLT fallback; a supported Xcode setup is
preferred for a new machine. Compiler/source/release provenance lives in the
native build manifest and is copied into experiment metadata.

Warnings about predicted IERS data and approximate UT1 remain visible. The
installed Earth-orientation dataset is not silently refreshed or represented as
measured data for the scenario epochs. Sequence-counter wrap is intentional and
covered by acquisition tests. No full MAB certification, SIL/PIL/HIL or controller
performance claim is made by this verification.
