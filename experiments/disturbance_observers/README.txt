Torque EKF disturbance estimation in the main AOCS plant

Model: models/aocs_plant.slx. The PD controller may use the six-state torque
EKF when ObserverCfg.Enabled=true. Set Enabled=false for an uncompensated PD
baseline. No other observer is selected by the current configuration.

From the repository root in MATLAB:
  setupAocsPaths();
  results = runtests("tests/gnc/TorqueEkfTest.m");
  assertSuccess(results);
  [~,~,runDirectory] = run_nadir_pointing_demo(InitialError_deg=5, ...
      StopTime_s=1200,DisturbancesEnabled=true,MakePlots=false);
  [report,folder] = validateTorqueEkfEnvironment( ...
      fullfile(runDirectory,"simulation.mat"));

The model calls torqueEkf(u); all EKF calculations and interface handling
are local to src/gnc/control/torqueEkf.m. Configuration remains separate.

The estimator state is body rate omega_BI^B [rad/s] and body disturbance
torque [N*m]. Prediction uses the previous interval's applied body torque,
not commanded torque. MEKF corrected gyro rate is the measurement. Truth
is used only for scoring. The adapter has explicit state and finite-output
checks; the six-state filter covariance uses a continuous torque random walk.

The environment validation replays three rate sources on one archived plant
trajectory: true rate, true rate plus independent gyro noise, and the logged
MEKF corrected rate. It reports torque residual, pre-gate NIS, acceptance,
reset count and covariance health. These replays share plant and control
histories; they do not establish closed-loop robustness or flight timing.
This is nominal simulation validation, not flight qualification. Historical
comparison artifacts are retained for traceability under
docs/data/disturbance_observers/archive/momentum_comparison/.

Detailed validation contract
----------------------------
The three replays use the same physical trajectory and applied control:
  TrueRate: exact body rate, isolates plant/observer model mismatch.
  IndependentGyro: exact body rate plus independent Gaussian noise using
      the EKF assumed rate covariance; reproducible local random stream.
  MEKF: the actual logged bias-corrected gyro rate from AttitudeEstimate.
No disturbance truth enters the EKF. Truth is used only to score error.
This does not test a new closed-loop feedback response for each replay.
The original archived run retains its existing controller and observer.

Frames and timing:
  omega_BI^B [rad/s]; disturbance and control torques in body axes B [N*m].
  Prior applied torque at t(k-1) drives prediction to t(k), matching the
  current ideal actuator and the model's Unit Delay observer interface.
  Scoring truth = M_dist_B_Nm + configured external body torque M_ext_B.
  Active samples must have valid, initialized, timestamp-aligned MEKF rates.

EKF settings come from defaultDisturbanceObserverConfig (Enabled=true); metadata records
all values and source hashes. Reset after rejected measurements follows the
production simulation adapter. Usable output is clipped and set to zero on rejection, matching the adapter.
Raw accepted estimates are scored separately. Applied control is already limited.

Metrics (last 120 seconds by default):
  TorqueRms_microNm: usable-output vector RMS over the common active tail.
  RawAcceptedTorqueRms_microNm: raw EKF RMS conditional on acceptance.
  ZeroEstimateRms_microNm: RMS without any disturbance estimate, scored on
      the same common active tail. ImprovesOverZero is a nominal benchmark,
      not a mission requirement. Inspect acceptance before using it.
  MeanNis: all finite candidate innovations before gating; expected 3 only
      for correctly modeled independent three-dimensional measurements.
  Nis99PercentCoverage: fraction inside the central chi-square(3) 99% band.
      Diagnostic only; no independent-sample aggregate pass/fail is claimed.
  AboveGateFraction: fraction of candidate NIS values above the actual gate.
  MaxAbsInnovationLag1: largest absolute lag-one correlation across the
      three whitened innovation axes, using consecutive valid candidates.
  RateErrorRms_deg_s / RateBiasNorm_deg_s: measurement error against truth.
  AcceptedActiveFraction, AcceptedTailFraction, ResetCount, CovarianceHealthy:
      availability and numerical health.

Artifacts: metrics.csv, validation.mat (full aligned histories and replays),
metadata.json (input hash, source hashes, settings), validation.png.

Limitations: one scenario/seed, nominal inertia, current MEKF, ideal actuator;
orbital disturbances do not exactly follow the assumed torque random walk.
MEKF corrected-rate errors may be temporally correlated. NIS coverage alone
does not validate torque accuracy, nonlinear stability, or robustness.
