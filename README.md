# CubeSat AOCS Simulator

Small Attitude and Orbit Control System simulation developed in MATLAB and Simulink.

## Workflow

Run from MATLAB:

```matlab
run_aocs_simulation
validate_aocs_results
plot_aocs_results
```

Run the attitude dynamics regression tests with:

```matlab
results = runtests("tests/attitude_dynamics")
```

`run_aocs_simulation` loads the JSON configuration, creates the Simulink bus objects, configures the Aerospace Blockset `6DOF (Quaternion)` block, runs the plant model, and saves the latest result to `results/aocs_simulation_results.mat`.

## Architecture

The source of truth is `config/AocsSimulationConfig.json`.

The Simulink plant is `models/aocs_attitude_plant.slx`.

The current attitude model contract is:

- `AOCS_ConfigBus`: one flat configuration bus generated from JSON
- `AOCS_Config`: `Simulink.Parameter` containing that bus payload
- `AOCS_StateBus`: one logged attitude state bus from the plant

The plant uses Aerospace Blockset `6DOF (Quaternion)` for rigid-body attitude and quaternion dynamics.

`AOCS_StateBus` contains:

```text
euler_rad
q_be
DCM_be
omega_b
```

The JSON stores the initial attitude as scalar-first quaternion `q_BI`. The loader derives `euler_BI_0_rad` for the Aerospace Blockset mask because the `6DOF (Quaternion)` block expects initial `[roll pitch yaw]`.

For a Simulink Constant block that emits the whole config bus:

```text
Constant value: AOCS_Config
Output data type: Bus: AOCS_ConfigBus
Sample time: inf
```

Inside Simulink block parameters, access fields as `AOCS_Config.I_B`, not `AOCS_Config.Value.I_B`.

## Attitude Baseline

- Aerospace Blockset 6DOF quaternion plant
- Config-driven inertia, initial attitude, initial body rates, and external torque
- Typed config and state buses
- Torque-free motion
- Quaternion, Euler angle, DCM, and body-rate logging
- Rotational energy validation
- Angular momentum norm validation
- Regression tests for spherical inertia, principal-axis spin, intermediate-axis instability, and coning

## Next milestones

- Orbit propagation
- IGRF
- Sun ephemeris
- Sensor models
- TRIAD
- MEKF
- Reaction Wheels
- B-dot controller
