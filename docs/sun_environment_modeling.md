# Sun, Eclipse, and SRP Modeling

This document describes the model choices and equations used for Sun geometry,
eclipse shadowing, solar flux, and SRP torque.

General frame conventions and signal suffix rules are defined in
[transformations.md](transformations.md). The current Sun-specific model
interfaces are defined here.

## Runtime Signals

The Sun, eclipse, and SRP model produces these runtime signals:

```text
r_sun_I_m                 Earth-to-Sun vector in inertial axes [m]
sun_I_unit                spacecraft-to-Sun unit vector in inertial axes
sun_B_unit                spacecraft-to-Sun unit vector in body axes
sun_distance_m            spacecraft-to-Sun distance [m]
solar_flux_W_m2           raw solar irradiance at spacecraft distance [W/m^2]
sun_visibility            direct solar illumination fraction in [0, 1]
solar_flux_shadowed_W_m2  eclipse-shadowed solar irradiance [W/m^2]
M_srp_B_Nm                SRP torque expressed in body axes [N*m]
M_dist_B_Nm               total modeled environment disturbance torque [N*m]
```

## Configuration

The source of truth is:

```text
config/AocsSimulationConfig.json
```

The relevant config sections are:

```text
epoch
environment.sun
environment.eclipse
environment.srp
environment.disturbances
```

Scenario overrides are stored in:

```text
config/scenarios/
```

`loadAocsSimulationConfig` validates these fields and creates
`AOCS.EnvironmentConfig`. `applyAocsSimulationSettings` applies the ephemeris and
eclipse settings to the Simulink block masks.

## Sun Ephemeris

The Sun position is provided by the Aerospace Blockset `Planetary Ephemeris`
block in:

```text
models/aocs_plant.slx/Orbit & Environment/Sun Products
```

The block is configured for:

```text
center:          Earth
target:          Sun
units:           m,m/s
epoch format:    Julian date
velocity output: off
```

The default ephemeris model is `DE405` from `Ephemeris Data for Aerospace
Toolbox` support package.

## Time Input

The project epoch is stored as a UTC calendar vector:

```text
[year month day hour minute second]
```

The Planetary Ephemeris block receives a TDB Julian date. The loader derives the
initial TDB Julian date from the configured UTC epoch and the explicit
`tdb_minus_utc_s` offset:

```text
epoch_tdb_jd = calendarUtcToJulianDate(epoch.utc) + epoch.tdb_minus_utc_s / 86400
```

The Simulink input to the ephemeris block is then:

```text
T_JD = epoch_tdb_jd + t_s / 86400
```

## Sun Direction and Flux

The ephemeris block returns the Earth-to-Sun vector `r_sun_I_m`. The
spacecraft-to-Sun vector is formed from the spacecraft inertial position:

```text
r_sat_to_sun_I_m = r_sun_I_m - r_I_m
sun_distance_m   = norm(r_sat_to_sun_I_m)
sun_I_unit       = r_sat_to_sun_I_m / sun_distance_m
```

The body-frame Sun direction is obtained from the attitude DCM:

```text
sun_B_unit = DCM_be * sun_I_unit
```

Raw solar irradiance is scaled by inverse-square distance:

```text
solar_flux_W_m2 = solar_constant_W_m2 * (AU_m / sun_distance_m)^2
```

The default solar constant is `1361.0 W/m^2`.

## Eclipse

The eclipse model is the Aerospace Blockset `Eclipse Shadow Model (Dual Cone)`
inside:

```text
models/aocs_plant.slx/Orbit & Environment/Eclipse Model
```

Its spacecraft-position input is connected to `r_I_m`, the spacecraft inertial
position vector in meters.

The project maps the block `Fraction` output to the `sun_visibility` signal. The
block also provides `Region_Earth` and `Region_Moon`, but those outputs are not
currently part of `AOCS_EnvironmentBus`.

When `environment.eclipse.enabled` is false, the subsystem bypasses the eclipse
block output and forces full direct illumination.

## Shadowed Flux

The irradiance available to models that need direct sunlight is:

```text
solar_flux_shadowed_W_m2 = sun_visibility * solar_flux_W_m2
```

## SRP Torque

The current SRP model is implemented in:

```text
src/environment/computeSrpTorque.m
```

It is a constant-area lumped-coefficient model:

```text
P_srp_N_m2  = solar_flux_shadowed_W_m2 / c_m_s
F_srp_mag_N = P_srp_N_m2 * coefficient_reflectivity * area_ref_m2
F_srp_B_N   = -F_srp_mag_N * sun_B_unit
M_srp_B_Nm  = cross(center_of_pressure_B_m, F_srp_B_N)
```

The negative sign appears because `sun_B_unit` points from the spacecraft to the
Sun, while radiation pressure acts away from the Sun.


## Current Limits

The current implementation assumes:

```text
Sun position             Aerospace Blockset Planetary Ephemeris
Eclipse                  Aerospace Blockset dual-cone Earth/Moon model
SRP area                 constant reference area
Optical properties       lumped reflectivity coefficient
Center of pressure       fixed in body axes
Orbit propagation        unperturbed Keplerian
```

## Validation

The relevant tests are:

```text
tests/orbit_and_environment/OrbitEnvironmentTest.m
```

They check Sun-vector consistency, raw and shadowed flux, eclipse-disabled
bypass behavior, SRP torque calculation, disturbance-torque summation.
