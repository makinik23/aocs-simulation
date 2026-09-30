# Getting started

## Requirements

Baseline: MATLAB R2025a, Simulink, Aerospace Blockset, Aerospace Toolbox and
DSP System Toolbox (native Cholesky solve in the MEKF).
The full plant also needs the Ephemeris Data for Aerospace Toolbox support
package for its selected JPL model, GNU Fortran (`gfortran` on PATH), a
MATLAB-supported C MEX compiler and Git. The full test suite additionally
requires Simulink Test for its component harnesses. `aocsDoctor` reports
installed products, native build readiness and optional Planet fixture
availability; it does not install toolboxes or provide licenses.

| Platform | Core/config/sensor path | Native full plant |
|---|---|---|
| macOS Apple silicon | Locally verified on R2025a | Locally verified with GNU Fortran and explicit CLT fallback |
| macOS Intel | Portable code; not locally executed | Build branch provided; not validated |
| Linux x86-64 | Covered by the full CI suite; not locally executed | GNU shared-library build and CI job provided; not yet validated here |
| Windows x86-64 | Portable code; no CI job | Explicitly unsupported in this increment; no silent atmosphere substitution |

This matrix deliberately distinguishes a portable implementation from tested
platform support. CI runs the full suite on Linux; MATLAB licenses/products are
required on every platform.

## After cloning

Open `AOCSSimulation.prj` in MATLAB (or call `openProject(pwd)` from the repository).
Project startup prepares paths and bus definitions. `bootstrapAocs` also prepares
the interactive default configuration. Opening the project does not load a
scenario, fetch dependencies, compile or run simulations.
The startup function lives in `tools/project/`; its location is registered in the
project metadata. Plotting and result validation live in `src/analysis/`, and
`aocsDoctor` lives in `tools/diagnostics/`. Opening the project (or calling `setupAocsPaths`
from the root) makes these functions available under their existing names.
Without the project UI, run these commands from the cloned repository root:

```matlab
bootstrapAocs("core")
run_aocs_tests("core")
```

This is the first check on a new machine and does not require Fortran, DTM2020
or the ephemeris support package. Full-plant tests run in the `full` profile.
The project does not modify your global MATLAB startup or save the MATLAB path.
Project startup, bootstrap, and the simulation/test entry points direct generated
Simulink cache and code to `build/simulink/cache/` and `build/simulink/codegen/`.
These are session settings, not saved global preferences. Existing run evidence
stays in `results/` and `test-results/`.

## Full plant on macOS/Linux

Install GNU Fortran and configure a supported C compiler with `mex -setup C`.
On Linux a package manager installation typically supplies `gfortran` and GCC;
on macOS install the supported Xcode toolchain and GNU Fortran for your architecture.
Then:

```matlab
bootstrapAocs("full")
run_aocs_tests("full")
[out, config, runDirectory] = run_aocs_simulation;
plot_attitude_results
plot_attitude_estimation_results
plot_orbit_environment_results
```

Bootstrap fetches DTM2020 only if the dependency directory is absent and checks
out commit `a488a7c9d030bfbe86e88ab3d28a7ec5589b92e0`. It rejects a different/modified
existing checkout without resetting it. Compilation runs the CNES reference
benchmark and records source hashes, compiler and MATLAB release in `build/native/`.
Source or MATLAB-release changes invalidate the build; rerun bootstrap explicitly.
Generated binaries are local, ignored by Git and rebuilt after relocating a clone.

The existing Apple-silicon development machine uses Command Line Tools without a
MATLAB-selected Xcode configuration. Its legacy custom MEX-options fallback must
now be requested explicitly:

```matlab
bootstrapAocs("full", UseCommandLineTools=true)
```

That fallback adapts MATLAB's bundled Clang template for CLT by removing its Xcode
installation-license probe. It is **not a MathWorks-supported toolchain claim** or
permission to omit toolchain license acceptance. Prefer supported Xcode for a new
machine. There is no automatic fallback after a compiler error.

## Scenarios and evidence

```matlab
[out, config, runDirectory] = run_aocs_simulation("config/scenarios/no_disturbance_torques.json");
validate_aocs_results(fullfile(runDirectory, "simulation.mat"));
```

Each run keeps its own result/configuration/provenance. Plots use the configured
latest-result copy by default. Model mask overrides apply for the duration of the
simulation. Scenario inheritance is always relative to the declaring JSON file.

For external validation, see `validation/planet/README.md` and the other validation
folders. Planet data is intentionally not bundled. `run_aocs_tests("validation")`
fails early if the required Planet fixture is missing; full mode reports optional
missing cases as skipped. Long orbit arcs require explicit opt-in.

## Troubleshooting

- Missing product: install/license the required MathWorks product; core tests still
  require Simulink, the Aerospace products used by configuration/wiring checks,
  and DSP System Toolbox for the MEKF matrix solver.
- Missing or stale native backend: run full bootstrap; do not copy another
  platform's MEX files into the checkout.
- `gfortran` absent from PATH: ensure the process launching MATLAB inherits its path.
- Predicted IERS warning: the installed dataset lacks measured EOP at that epoch.
  Keep the warning visible and use a suitably frozen dataset for precision studies.
- DTM2020 Fortran has shared state: concurrent threaded calls/multiple coefficient
  sets in the same process are not supported. Use separate MATLAB processes.

`cleanAocsArtifacts` removes known generated simulation artifacts; preserve your
`results/runs` and `test-results` when they are experiment evidence. Do not use a
blanket ignored-file deletion to prepare the project for distribution.
