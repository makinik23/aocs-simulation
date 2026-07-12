# Frame Transformations and Conventions

This document defines the coordinate-frame and transformation conventions used
by the AOCS Simulink model. The goal is a consistent engineering simulation
chain for attitude dynamics, orbit products, magnetic-field products, and later
Sun/eclipse/SRP products.

## Signal Suffixes

Vector signal names use a suffix that states which frame the components are
expressed in:

```text
_I      inertial frame used by the orbit propagator
_ECEF   Earth-centered Earth-fixed frame
_NED    local north-east-down frame
_B      spacecraft body frame
```

For example, `B_I_T` is a magnetic-field vector expressed in inertial axes, and
`B_B_T` is the same physical field expressed in body axes.

## Time Model

The JSON config defines the UTC epoch:

```text
epoch.utc = [year month day hour minute second]
```

Simulink `Clock` provides elapsed seconds from that epoch. Blocks that need an
absolute date use:

```text
UTC(t) = epoch.utc + Clock seconds
```

`applyAocsSimulationSettings` configures both the `ECI Position to LLA` and
`Direction Cosine Matrix ECI to ECEF` blocks with the same epoch and with
`deltaT = Sec`. This keeps the orbit propagator, LLA conversion, ECI/ECEF DCM,
IGRF decimal year, and future Sun/eclipse products on one time base.

## Inertial Frame `I`

`I` is the geocentric inertial frame used by the Aerospace Blockset orbit and
frame-conversion blocks. The orbit propagator is configured with:

```text
orbit.propagator.output_frame = ICRF
```

Therefore `r_I_m`, `v_I_m_s`, and `B_I_T` are Earth-centered vectors expressed
in ICRF-aligned inertial axes. In project shorthand this is the ECI frame, but
it is not TEME and it is not a TLE/SGP4 frame.

When this project says ECI, it means the ICRF/IAU-2000/2006 inertial frame used
consistently by the MATLAB Aerospace Blockset blocks in this model.

The current orbit model is Keplerian and unperturbed. Orbit dynamics are
propagated in the inertial frame; Earth rotation only enters when converting
between inertial, Earth-fixed, and local frames.

## IAU-2000/2006 Reduction

The ECI-to-ECEF and ECI-to-LLA Aerospace Blockset blocks are configured with:

```text
red = IAU-2000/2006
```

That reduction is the modern precession-nutation and Earth-orientation
reduction path exposed by the Aerospace Blockset blocks. In this model it is
used as a coherent simulation convention: the same epoch and elapsed seconds
drive all inertial-to-Earth-fixed conversions.

At this stage the model does not feed measured Earth-orientation parameters
into those blocks. In particular, it does not provide external `dUT1`, polar
motion (`xp`, `yp`), or celestial intermediate pole offsets (`dX`, `dY`). The
intent is a consistent AOCS engineering frame chain, not sub-arcsecond orbit
determination.

## Earth-Fixed Frame `ECEF`

`ECEF` is an Earth-centered, Earth-fixed rotating frame tied to the WGS84 Earth
model used by the geodetic conversions:

- origin at the Earth center,
- `+Z` along the terrestrial reference pole,
- `+X` through the equator and zero longitude,
- `+Y` completes a right-handed frame,
- components are in meters for position-like quantities.

The ECEF frame rotates with the Earth. It is used as the bridge between inertial
orbit products and geodetic/local products such as LLA, NED, and IGRF magnetic
field components.

## Geodetic LLA

LLA is geodetic latitude, longitude, and height above the WGS84 ellipsoid:

```text
latitude   [deg]
longitude  [deg], east-positive
altitude   [m]
```

LLA is produced from the inertial position `r_I_m` by the Aerospace Blockset
`ECI Position to LLA` block using the same epoch/time convention as the ECI/ECEF
DCM block.

## Local NED Frame

NED is the local tangent frame at the geodetic LLA point:

```text
N  local north
E  local east
D  local down, opposite local up along the geodetic normal
```

The IGRF block returns magnetic-field components as `XYZ (nT)` in local NED
axes. The subsystem converts them to tesla:

```text
B_NED_T = 1e-9 * IGRF_XYZ_nT
```

The magnetic-field transform chain is:

```text
B_NED_T -> B_ECEF_T -> B_I_T -> B_B_T
```

The orbit/environment tests check that the field norm is preserved across these
rotations to roundoff level.

## Body Frame `B`

The spacecraft body frame `B` is the frame of:

- inertia matrix `I_B`,
- body rates `omega_b`,
- residual magnetic dipole `m_res_B_A_m2`,
- disturbance torques `M_*_B_Nm`,
- body-frame magnetic field `B_B_T`.

The project uses column vectors. A direction cosine matrix named `C_AB` maps
components from frame `B` into frame `A`:

```text
v_A = C_AB * v_B
```

The logged `DCM_be` name comes from the Aerospace Blockset signal naming. The
project convention treats it as the inertial-to-body attitude DCM used by the
tests and environment model:

```text
v_B = DCM_be * v_I
B_B_T = DCM_be * B_I_T
r_hat_B = DCM_be * (r_I_m / norm(r_I_m))
```

This is consistent with the JSON convention:

```text
q_BI  maps inertial frame I to body frame B
C_BI  maps v_I to v_B
```

## Disturbance Torque Frames

All disturbance torques consumed by the AOCS plant are expressed in body
axes:

```text
M_dist_B_Nm
M_rmm_B_Nm
M_gg_B_Nm
```

Residual magnetic moment torque:

```text
M_rmm_B_Nm = m_res_B_A_m2 x B_B_T
```

Gravity-gradient torque:

```text
r_hat_B = DCM_be * (r_I_m / norm(r_I_m))
M_gg_B_Nm = 3 * mu / norm(r_I_m)^3 * cross(r_hat_B, I_B * r_hat_B)
```

The total modeled disturbance torque is:

```text
M_dist_B_Nm = M_rmm_B_Nm + M_gg_B_Nm
```

## Sun Vector Convention

The staged Sun model returns an approximate Earth-to-Sun vector:

```text
r_sun_I_m
```

It is expressed in J2000/ICRF-aligned inertial axes, matching the project `I`
frame convention. For LEO use, the Earth-to-Sun direction is already adequate
for many Sun-sensor and coarse SRP/eclipsing checks. When eclipse and SRP are
wired, the spacecraft-to-Sun vector should be formed explicitly as:

```text
r_sat_to_sun_I_m = r_sun_I_m - r_I_m
sun_I_unit = r_sat_to_sun_I_m / norm(r_sat_to_sun_I_m)
sun_B_unit = DCM_be * sun_I_unit
```

The staged `AOCS_EnvironmentBus` fields for Sun products are:

```text
sun_B_unit
sun_I_unit
r_sun_I_m
sun_distance_m
solar_flux_W_m2
```

`sun_distance_m` and `solar_flux_W_m2` are evaluated for the spacecraft-to-Sun
distance. In LEO this is numerically almost identical to the Earth-to-Sun
distance, but the spacecraft-specific definition is the right contract for SRP.

The default code path uses `src/environment/sunPositionLowPrecision.m` for the
geocentric Sun vector and `src/environment/computeSunProducts.m` for the
spacecraft-specific unit vectors and flux.
A future higher-fidelity path can use Aerospace Toolbox `planetEphemeris` or the
Aerospace Blockset `aerolibcelestial/Planetary Ephemeris` block, but that
requires external JPL ephemeris data from `aeroDataPackage`.

## Implementation References

The main implementation points are:

- `config/AocsSimulationConfig.json` for epoch, orbit, and environment inputs,
- `src/config/loadAocsSimulationConfig.m` for validation and normalized config,
- `src/simulink/applyAocsSimulationSettings.m` for Aerospace Blockset mask
  setup,
- `src/simulink/createAocs*Bus.m` for bus contracts,
- `src/environment/sunPositionLowPrecision.m` for the staged analytic Sun
  vector.
