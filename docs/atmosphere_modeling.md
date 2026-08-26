# Coupled Atmosphere and Free-Molecular Aerodynamics

This document defines the implemented LEO atmosphere and aerodynamic coupling
used by the orbit/environment and attitude-disturbance pipeline.

## Atmosphere Configuration

The source of truth is `environment.atmosphere` in
`config/orbit_environment.json`:

```text
enabled                 enable atmosphere products
model                   dtm2020
mode                    operational or research
space_weather_source    nominal or file
space_weather_file      optional time-series source for file mode
rho_scale_factor        multiplicative density calibration factor
uncertainty_enabled     enable DTM2020 uncertainty products
nominal_space_weather   constant DTM2020 drivers for integration runs
```

`operational` mode runs the native DTM2020 operational driver set (`F10.7`,
81-day mean `F10.7`, and `Kp`). `research` mode is reserved for the F30/Hp60
source and a timestamped space-weather provider.

## Runtime Products

`AOCS_AtmosphereBus` publishes:

```text
rho_kg_m3                       scaled neutral mass density [kg/m^3]
rho_raw_kg_m3                   unscaled DTM2020 density [kg/m^3]
rho_uncertainty_1sigma_kg_m3    one-sigma density uncertainty [kg/m^3]
T_local_K                       local neutral temperature [K]
T_exo_K                         exospheric temperature [K]
n_O_m3, n_N2_m3, n_O2_m3       O, N2, and O2 number density [1/m^3]
n_He_m3, n_H_m3, n_N_m3        He, H, and N number density [1/m^3]
v_atm_I_m_s                     atmosphere velocity in inertial axes [m/s]
```

All species outputs are consumed by the free-molecular panel model. Partial
mass densities from Fortran are converted to number densities in SI units.
The modeled density is:

```text
rho_kg_m3 = rho_scale_factor * rho_raw_kg_m3
```

Keep `rho_scale_factor = 1.0` for nominal runs. Use scenario overrides for
sensitivity analysis, calibration, and Monte Carlo campaigns.

## Native DTM2020 Path

The online path in Simulink is:

```text
LLA, UTC context, F10.7/Kp
  -> Prepare DTM2020 Inputs
  -> dtm2020_sfun
  -> Postprocess DTM2020
  -> AOCS_AtmosphereBus
```

Relevant sources are:

```text
src/environment/+dtm2020/prepareInputs.m
src/environment/+dtm2020/prepareNativeInput.m
src/environment/+dtm2020/postprocessNativeOutput.m
src/native/dtm2020/
```

`prepareInputs` converts `epoch_utc + t_s` to decimal day of year and local
solar time, then creates the fixed-shape arrays required by the reference
Fortran routines. Simulink calls the official operational code through the
Level-2 C S-Function. `buildDtm2020Native` also creates a test MEX and checks
the official operational benchmark before the plant is run.

## Six-Panel Sentman Model

The 3U body is a rectangular prism with panel order:

```text
+X, -X, +Y, -Y, +Z, -Z
```

For `0.1 x 0.1 x 0.3 m`, the panel areas are
`[0.03 0.03 0.03 0.03 0.01 0.01] m^2`. Configuration supplies per-panel wall
temperature and energy accommodation. Panel centroids are derived relative to
the configured center of mass.

`computeSentmanPanelAerodynamics.m` evaluates O, N2, O2, He, H, and N
separately. For each species it computes the molecular speed ratio from the
DTM2020 local temperature and species mass, evaluates the Sentman diffuse
reemission pressure and shear coefficients, and sums:

```text
F_panel_B = area * sum_species(q_species * (c_tau * tau_B - c_p * normal_B))
M_aero_B  = sum_panels(cross(r_panel_B, F_panel_B))
```

The total DTM2020 density normalizes the partial-density sum to avoid loss of
mass from reference-code roundoff. The inertial force and acceleration are:

```text
F_aero_I = C_BI' * F_aero_B
a_aero_I = F_aero_I / spacecraft_mass
```

`a_aero_I` is summed with the project SRP acceleration into `a_dist_I_m_s2`,
which is connected to the `A_icrf` input of Aerospace Blockset's Numerical
(high precision) Orbit Propagator. `M_aero_B` is included in the attitude
disturbance-torque sum. This closes the translational and rotational aerodynamic
paths.

## Current Fidelity Boundary

- Operational DTM2020 is restricted to the reference altitude range,
  120-1500 km.
- Space-weather drivers are currently nominal constants. Historical or
  forecast-quality runs still need timestamped F10.7/Kp data with the required
  delays and averages.
- Atmospheric velocity includes rigid Earth co-rotation. Horizontal
  thermospheric winds are not yet modeled.
- Surface temperature and accommodation are constant per panel. There is no
  coupled thermal or material-aging model yet.
- The six-face 3U geometry is convex, so self-shadowing is unnecessary. More
  complex deployed geometry will require a mesh and visibility test.
- The configured 2026 epoch is outside measured EOP coverage shipped with
  MATLAB R2025a and uses predicted IERS values unless a current file is supplied.

Primary references include the
[SWAMI DTM2020 implementation notes](https://swami-h2020-eu.github.io/mcm/dtm2020.html),
the [DTM2020 model paper](https://doi.org/10.1051/swsc/2021032), Sentman's
*Free Molecule Flow Theory and Its Application to the Determination of
Aerodynamic Forces* (1961), and the independently validated
[ADBSat panel methodology](https://arxiv.org/abs/2104.05543).
