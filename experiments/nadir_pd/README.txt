Long nominal PD diagnostic
==========================

Run from the repository root in MATLAB:

    addpath(fullfile(pwd, 'experiments', 'nadir_pd'));
    [out, AOCS, runDirectory, report] = run_long_pd_diagnostic();

The default duration is 15000 s. For a shorter diagnostic:

    [out, AOCS, runDirectory, report] = run_long_pd_diagnostic(500);

Purpose
-------
Inspect long-duration nadir tracking, attitude estimation, disturbance
compensation, estimator health and control saturation. This experiment
performs no gain tuning and does not establish a stability proof.

Configuration
-------------
The main aocs_plant model runs its MEKF attitude estimator, nominal PD
controller and default torque EKF disturbance observer (ObserverCfg.Enabled=true).
Initial attitude is 5 deg from the onboard SGP4 LVLH reference about the
body error axis [1; 1; 1]/sqrt(3). Initial body angular rate matches the
transformed reference rate, so the initial rate error is zero.

The nadir_pointing scenario supplies controller gains, environment,
sensor settings and spacecraft parameters. Integral action is disabled
for this PD run. PID implementation and gains remain untouched.
SimulationInput scopes configuration, logging and the disconnected
Actuators placeholder override to this simulation; the saved model is
not edited. MATLAB-side initialization uses rng(1).

Outputs
-------
Each invocation creates a unique directory under:

    experiments/nadir_pd/results/

The returned runDirectory identifies that directory. scenario.json and
configuration.mat record configuration and assumptions. Immediately after
simulation, raw_simulation.mat archives the full SimulationOutput, AOCS,
scenario, observer configuration, metadata and elapsed wall time before
analysis begins.

windows_500s.csv contains nonoverlapping 500 s diagnostic windows, including
any final partial window. diagnostics.mat stores the report and sampled
traces. diagnostics.png contains the diagnostic plots. Missing optional
signals are represented by NaN rather than assumed valid or zero.

Interpretation
--------------
True and estimated attitude errors are measured against the onboard LVLH
reference. Estimation error compares the MEKF with plant truth. Body-rate
tracking error transforms reference-frame D rates to current body-frame B
axes before subtraction. Disturbance estimates and truth use body-frame
torques in N m. Saturation uses compensated torque demand and the scenario
actuator limit; validity and control activity must be considered together.

Tracking the onboard reference does not establish orbit-reference accuracy.
MEKF valid indicates an accepted gyro prediction, not estimator convergence;
HealthUsable reflects the separate estimator health decision.

Archived data can be reanalyzed without simulation:
  eventReport = analyze_long_pd_events(runDirectory);
Runs >=1000 s also call this analysis automatically. It compares sunlight and
eclipse intervals, measurement acceptance and actual +Z pointing to -r/|r|.
Reference-rate consistency is checked from successive guidance quaternions.

The private reference-geometry helper samples every 30 s, reconstructs IGRF-14
at both orbit positions and checks that the physical-field reconstruction
matches the archived field. Its CSV and plot separate the reference defect
from control tracking error. No disturbance truth enters the controller.
