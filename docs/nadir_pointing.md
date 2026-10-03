# Quaternion PD nadir pointing

Run from the project root after the usual full-plant dependency setup:

```matlab
setupAocsPaths;
[out, AOCS, runDirectory, report] = run_nadir_pointing_demo;
```

The demo starts with a 90-degree error about `[1;1;1]/sqrt(3)` relative to the
onboard LVLH frame and zero initial relative angular velocity. It simulates
600 s with the nominal sensors, TRIAD, MEKF, disturbance torques and an ideal
body-torque actuator. Truth is used only to set the initial condition and
evaluate the result; it is not available to the controller.

```matlab
% A smaller manoeuvre, or isolate nominal tracking from disturbances:
run_nadir_pointing_demo(InitialError_deg=30, DisturbancesEnabled=false);

% Replot saved evidence without another simulation:
s = load('results/aocs_nadir_pointing.mat');
report = plot_nadir_pointing_results(s.out,s.AOCS);

% Inspect the implemented blocks:
open_system('models/aocs_plant.slx');
open_system('aocs_plant/GNC/PD feedback linearisation/Controller');
```

`runDirectory` contains the immutable `simulation.mat`, resolved configuration,
provenance, `nadir_report.mat` and `nadir_pointing.png`. If the model was open
before an external edit, reload the saved model before running; do not overwrite
the new file with a stale editor copy.

## Equations and blocks

Editable mathematical annotations are stored directly in the model, in
`GNC/PD feedback linearisation`, `Error Calculation`, and `Controller`.
They document the frame conventions, implemented torque law, transport term,
ideal continuous-time Lyapunov/LaSalle argument and the limits of that argument
for the sampled, saturated, estimator-based implementation. Double-click an
equation inside an annotation to edit its LaTeX source.

`tools/model_builders/gnc/annotateNadirController.m` contains the annotation
source. It is an optional regeneration utility, not a simulation callback;
calling it replaces only annotations tagged `NadirMath:*`. The saved model
already contains the notes and needs no regeneration before use.

Scalar-first Aerospace quaternions describe passive coordinate transforms.
`C_BD = C_BI*C_DI'` maps desired-frame D components into body-frame B components.
`Error Calculation` uses `q_e = conj(q_DI) * q_BI` in the MATLAB Hamilton
multiplication convention and chooses the quaternion hemisphere `q_e(1)>=0`.
It computes `e_omega = omega_hat_BI_B - C_BD*omega_ref_DI_D`.

The user's Selector, Reshape, Gain, Proportional Acc and Rate Damping blocks
implement

```text
v = -2*Kp*q_e(2:4) - Kd*e_omega
tau_demand = omega_hat x (J*omega_hat)
           + J*(v - e_omega x (C_BD*omega_ref) + C_BD*alpha_ref)
```

`Reference Rate in Body` and `Reference Acceleration in Body` use matrix
multiplication. `Transport Term` and `Gyroscopic Compensation` are Cross Product
blocks with the operand order shown above. `Inertia Times Acceleration` and
`Angular Momentum` use the configured full inertia matrix.

In the ideal rigid-body model with exact state/reference rates and no
disturbances or saturation, the resulting rate-error dynamics are
`d(e_omega)/dt = v`. Quaternion kinematics remain nonlinear. This is quaternion
PD with compensation of the angular-rate dynamics, not full input-output
linearization of the quaternion vector.

The outer subsystem executes after MEKF on every GNC tick. `Enable Control`
requires the configuration enable, usable estimator health and valid reference
vectors. `Health Gate` outputs zero otherwise. `Ideal Actuator Torque Limit`
limits each body-axis torque. The output is held between ticks and connected to
`Flight Dynamics/Torque Sum`, alongside environmental and configured external
torques.

## Configuration and limits

`gnc.attitude_control` is optional. Defaults are disabled, `kp_s2=0.0004`,
`kd_s=0.04`, `max_torque_Nm=2e-5`. The dedicated scenario enables control and
updates SGP4 at the GNC period of 0.1 s. The acceleration gains correspond
locally to natural frequency 0.02 rad/s and critical damping before saturation.
The demo constructs the initial orientation from the configured onboard orbit;
running the scenario JSON directly instead keeps the base initial conditions.

Guidance preserves the existing axes `z=-r/|r|`, `y=-(r x v)/|r x v|`, `x=y x z`.
Its rate `[0; -|r x v|/|r|^2; 0]` is a locally planar-orbit approximation. Alpha
is a backward difference of those rate components at the SGP4 update period,
with the first derivative suppressed. It is not the exact derivative of a
precessing SGP4 LVLH frame. A later refinement should generate coherent
quaternion, angular-rate and angular-acceleration references from that frame.

The plotted orientation error is `2*acos(abs(dot(q_true,q_ref)))`, in degrees.
This tests all three axes relative to the onboard reference, not only the
nadir-axis angle and not onboard orbit accuracy. Nonzero sensor errors,
disturbances, reference approximation and discrete sampling can leave a residual
tracking error. The actuator is ideal torque with saturation: wheel momentum,
wheel-speed limits, allocation and magnetorquer underactuation are not modeled.

## Verified acquisition example

Local MATLAB R2025a runs on 2026-10-03, both from 90 degrees over 600 s:

| Modelled disturbance torques | Final true reference error | Maximum over last 60 s |
| --- | ---: | ---: |
| Enabled | 1.518 deg | 1.518 deg |
| Disabled, nominal sensors retained | 0.067 deg | 0.112 deg |

The final MEKF attitude-estimation error was approximately 0.092 deg in both
runs. These are scenario results, not global stability or hardware-performance
guarantees. The remaining disturbed-case offset is consistent with PD tracking
under uncompensated external torque.

The model compiled and nine targeted tests passed: `QuaternionPdControllerTest`
(2), `MekfConfigTest` (6), and `PlantReadinessTest` (1). The latter also checked
that default-disabled control retains the torque-free conservation behaviour.
