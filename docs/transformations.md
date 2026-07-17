# Frame Transformations and Conventions

This document defines the coordinate frames, time convention, signal naming, and transformation chain used by the AOCS Simulink model.

## Signal Suffixes

Vector signal names include a suffix identifying the frame in which their components are expressed:

```text
_I      inertial frame used by the orbit propagator
_ECEF   Earth-centered Earth-fixed frame
_NED    local north-east-down frame
_B      spacecraft body frame
```

For example, `B_I_T` and `B_B_T` describe the same physical magnetic field. The first signal contains its components in inertial axes, while the second contains its components in spacecraft body axes.

Units are included in signal names where practical:

```text
_m       metres
_m_s     metres per second
_T       tesla
_Nm      newton metres
_A_m2    ampere square metres
```

All vectors are represented as column vectors.

## Time Model

The simulation begins at the UTC epoch defined in:

```text
config/AocsSimulationConfig.json
```

The epoch is stored as:

```text
epoch.utc = [year month day hour minute second]
```

The Simulink `Clock` block provides elapsed simulation time in seconds. Blocks that require a calendar date combine the configured epoch with the elapsed time.

The function `applyAocsSimulationSettings` configures the Aerospace Blockset frame-conversion blocks with the same epoch and uses seconds as the time increment. This ensures that the orbit position, ECI-to-ECEF transformation, geodetic coordinates, IGRF date, and Sun model all refer to the same physical instant.


## Inertial Frame `I`

`I` is the Earth-centered inertial frame used by the Aerospace Blockset orbit-propagation and frame-conversion blocks.

In Aerospace Blockset terminology, this inertial frame is referred to as ICRF (International Celestial Reference Frame.
For an Earth-centered simulation, its origin is located at the Earth's center of mass, while its axes follow the International Celestial Reference Frame orientation.
This reference frame is called GCRF (Geocentric Celestial Reference Frame). MATLAB states that this frame may be treated as the ECI frame realized at J2000 for the purposes of spacecraft modelling.

The current orbit model uses unperturbed Keplerian propagation. The spacecraft position and velocity are propagated entirely in `I`. The model therefore includes central-body gravity but does not yet include effects such as Earth oblateness, atmospheric drag, solar-radiation pressure, or third-body gravity.

## IAU-2000/2006 Reduction

The reduction defines how the inertial frame is related to the rotating terrestrial frame at a specified date and time.

It accounts for the main effects required to describe Earth orientation, including precession, nutation, Earth rotation, and polar motion. These effects are conceptually different:

- precession describes the long-term change of the Earth's rotation-axis orientation,
- nutation describes smaller periodic changes superimposed on precession,
- Earth rotation determines the daily orientation of the Earth,
- polar motion describes the movement of the rotation pole relative to the terrestrial crust.

The current model does not provide measured Earth-orientation parameters from an external IERS data source. In particular, it does not explicitly supply:

```text
deltaAT
deltaUT1
polar motion coordinates xp and yp
celestial pole corrections dX and dY
```

This matter will be addressed in the future.

## Earth-Fixed Frame `ECEF`

`ECEF` is the Earth-centered Earth-fixed frame used for quantities tied to the rotating Earth.

Its origin is at the Earth's center of mass. Its axes rotate with the Earth:

```text
+X   passes through the equator and zero longitude
+Y   passes through the equator and 90 degrees east longitude
+Z   points toward the north terrestrial pole
```

A point fixed on the Earth's surface has nearly constant ECEF coordinates, while its inertial coordinates change as the Earth rotates.

In the MATLAB IAU-2000/2006 conversion path, the fixed frame corresponds to the terrestrial reference frame used by the Aerospace Toolbox and Aerospace Blockset implementation.

The ECEF frame provides the bridge between orbital quantities and Earth-related environment models. It is used when converting the spacecraft position to geodetic coordinates and when transforming local magnetic-field components into inertial axes.

## Geodetic LLA

The spacecraft location relative to the Earth is represented by geodetic latitude, longitude, and altitude:

```text
latitude    [deg]
longitude   [deg], east-positive
altitude    [m]
```

Geodetic latitude is defined relative to the normal of the reference ellipsoid. It therefore differs slightly from geocentric latitude except at the equator and poles.

Longitude identifies rotation about the Earth-fixed `+Z` axis and is positive eastward.

Altitude is measured along the ellipsoid normal above the WGS84 reference ellipsoid.

The `ECI Position to LLA` block converts `r_I_m` directly to geodetic coordinates using the configured UTC epoch, elapsed time, reduction method, and WGS84 ellipsoid.

These coordinates provide the location and date required by the IGRF magnetic-field model.

## Local NED Frame

`NED` is a local tangent frame attached to the spacecraft geodetic location:

```text
N   tangent to the reference ellipsoid and directed toward geodetic north
E   tangent to the reference ellipsoid and directed east
D   directed downward, opposite to the outward ellipsoid normal
```

The NED frame changes as the spacecraft moves. It is therefore a local frame rather than a single global frame.

The MATLAB IGRF block returns the magnetic-field components as `XYZ` in local NED axes. In this output:

```text
X   north component
Y   east component
Z   down component
```

## Body Frame `B`

`B` is rigidly attached to the spacecraft.

Its exact axis orientation is defined by the spacecraft mechanical and CAD convention and must remain consistent with sensor mounting, actuator directions, inertia data, and torque signs.

The following quantities are expressed in body axes:

```text
I_B - inertia matrix expressed in body axes
omega_BI_B - angular velocity of body B relative to inertial frame I, expressed in body axes
m_res_B_A_m2 - residual magnetic dipole expressed in body axes
B_B_T - magnetic field expressed in body axes
M_dist_B_Nm - total disturbance torque expressed in body axes
M_rmm_B_Nm - residual magnetic moment torque expressed in body axes
M_gg_B_Nm - gravity-gradient torque expressed in body axes
```

## DCM Convention

The project uses the notation:

```text
C_AB maps vector components from frame B to frame A
```

The first subscript is the destination frame and the second is the source frame.

Examples:

```text
C_BI       inertial components to body components
C_ECEFI    inertial components to ECEF components
C_NEDECEF  ECEF components to NED components
```

The inverse transformation is obtained by transposing the DCM because a valid direction cosine matrix is orthonormal.

## Quaternion Convention

The project configuration quaternion is named:

```text
q_BI
```

It represents the attitude transformation from inertial axes to body axes and is consistent with `C_BI`.

The current runtime state bus still logs the Aerospace Blockset quaternion output as:

```text
q_be
```

Aerospace Blockset quaternion blocks use the scalar-first convention.

## Sun Vector Convention

The low-precision Sun model returns:

```text
r_sun_I_m
```

This is the approximate position of the Sun relative to the Earth, expressed in the project inertial frame.

The spacecraft Sun direction is not exactly the same as the Earth Sun direction. The environment subsystem therefore forms a spacecraft-to-Sun vector by subtracting the spacecraft inertial position from the geocentric Sun position.

The derived Sun products are:

```text
r_sun_I_m          approximate Earth-to-Sun position vector in inertial axes
sun_I_unit         spacecraft-to-Sun unit vector in inertial axes
sun_B_unit         spacecraft-to-Sun unit vector in body axes
sun_distance_m     spacecraft-to-Sun distance
solar_flux_W_m2    solar flux evaluated at that distance
```

The body-frame Sun direction is obtained using the same inertial-to-body attitude transformation as the magnetic field and nadir direction.

The default implementation uses:

```text
src/environment/sunPositionLowPrecision.m
src/environment/computeSunProducts.m
```

A higher-fidelity implementation may later use `planetEphemeris` or the Aerospace Blockset `Planetary Ephemeris` block. Those methods require the appropriate external JPL ephemeris data package.

## Main Implementation Files

The principal files related to frames, time, and environment calculations are:

```text
config/AocsSimulationConfig.json
src/config/loadAocsSimulationConfig.m
src/simulink/applyAocsSimulationSettings.m
src/simulink/createAocs*Bus.m
src/environment/sunPositionLowPrecision.m
src/environment/computeSunProducts.m
```

Any new environment or navigation subsystem should document:

- the physical meaning of each vector,
- the frame in which its components are expressed,
- the units,
- the direction of every DCM or quaternion,
- the epoch and time scale used by date-dependent calculations.
