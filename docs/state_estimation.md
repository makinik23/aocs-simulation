# State estimation

The current estimator is a six-error-state multiplicative extended Kalman filter
(MEKF). It estimates the scalar-first quaternion `q_BI` (inertial-to-body attitude)
and gyroscope bias `b_g` in body axes [rad/s]. Its local error state is
`[delta_theta_B; delta_b_g]`.

## Filter

TRIAD provides the initial attitude when valid magnetometer and coarse Sun sensor
observations can be paired with onboard inertial reference vectors. Those
references use an onboard SGP4 orbit prediction. The
initial gyro-bias estimate is a configured prior.

At each 0.1 s GNC tick, a fresh, valid gyro report drives prediction using
`omega_hat = omega_gyro - b_g_hat`. Quaternion integration accounts for adjacent
rate samples and coning; the error covariance propagates with
`F = [-skew(omega_hat), -I; 0, 0]` and configured gyro-noise and bias-walk terms.
Missing or malformed reports do not advance the estimate.

Fresh magnetic and Sun observations are normalized and compared sequentially
with the corresponding predicted body-frame directions. Each correction requires
aligned timestamps, a valid recent reference, an innovation within the NIS gate
and a bounded attitude step. An accepted correction updates attitude
multiplicatively, feeds back the bias estimate, and applies the Joseph covariance
update followed by the error-state reset. A rejected observation leaves the
predicted state unchanged.

`AttitudeEstimateBus` carries `q_BI`, bias-corrected angular rate, gyro bias,
covariance, initialization and update time. A separate `AttitudeHealthBus`
classifies Initializing, Tracking, Degraded or Lost using covariance and the ages
of the state and last accepted vector correction. `control_usable` is true only
in Tracking; it is an onboard validity decision, not proof of truth accuracy.

## Monte Carlo regression

`MekfMonteCarloTest` uses seeded trials and keeps truth outside the filter,
using it only for scoring. Results below come from the local MATLAB R2025a core
suite; attitude p95 values exclude the first 10 s.

| Scope | Trials | Attitude error p95 | Final gyro-bias error |
|---|---:|---:|---:|
| Saved MEKF, matched/nominal/outage cases | 12 x 60 s | worst 0.717 deg | worst 1.21e-4 rad/s |
| Full Plant -> Sensors -> Drivers -> GNC | 2 x 50 s | 0.445 and 0.484 deg | 3.56e-5 and 5.04e-5 rad/s |

All trials initialized. The isolated campaign kept finite covariance, guarded
the tested sensor outages and rejected the injected magnetic outlier. Neither
campaign recorded a `control_usable` sample with truth attitude error above
5 deg. Reproduce with `runtests("tests/gnc/MekfMonteCarloTest.m")`; the runner
`runMekfMonteCarlo` also accepts explicit seeds, scenarios and `Scope="plant"`.
