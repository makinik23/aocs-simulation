# Attitude initialization and state estimation

## Boundary

The current onboard data path is:

```text
configured orbit elements -> onboard SGP4 -> inertial reference vectors
SensorReportBus -------------------------> TRIAD initialization
```

GNC has no `PlantStateBus` input. Plant attitude, orbit and environment products
are simulation truth used to generate sensor measurements; they are not available
to onboard algorithms. Configuration is supplied through the typed
`AOCS_GNCConfig` parameter loaded from `config/gnc.json`.

GNSS is a measurement source, not the onboard position reference. The current
reference-vector branch propagates the configured epoch orbit with SGP4. A future
navigation estimator may compare GNSS reports with that propagated state and
correct or reinitialize the onboard orbit, but TRIAD does not consume GNSS.

## Onboard reference vectors

`Onboard Reference Vectors` samples the SGP4 adapter at
`sgp4_sample_time_s`. SGP4 produces the onboard inertial position used by:

- the configured DE405 planetary ephemeris to form `sun_I_unit`;
- IGRF-14 plus the onboard ECI/ECEF/LLA transformations to form `B_I_T`.

`InitializationConfig` is split between two local Bus Selectors. `Select SGP4
Settings` feeds the typed `Sgp4ConfigBus`, while `Select Reference Settings`
provides the environment and reference-vector settings used by the remaining
blocks. The propagator subsystem therefore has only two inputs: onboard time
and SGP4 configuration. The SGP4 bus contains `epoch_utc_jd`, the six epoch
elements and `bstar`; `sgp4_sample_time_s` remains a discrete block parameter
rather than a runtime signal.

The saved model uses Simulink and Aerospace blocks for timing, coordinate
transformations, ephemeris, geomagnetics and vector normalization. The Aerospace
Toolbox SGP4 API has no equivalent block in the installed release, so its adapter
is isolated in one Interpreted MATLAB Function block. It does not read GNSS or
plant truth.

The initial SGP4 state currently comes from the configured Keplerian elements and
uses `BStar = 0`. This is a deterministic development baseline, not a claim that
the onboard orbit remains accurate indefinitely. Flight-like operation will need
a dated TLE/OMM or an estimator-driven orbit update policy.

`ReferenceVectorBus` contains:

- `sun_I_unit` - inertial Sun direction;
- `B_I_T` - inertial magnetic-field vector [T];
- `valid` - combined SGP4/Sun/magnetic-reference validity;
- SGP4 update time and sequence ID.

## TRIAD initialization

TRIAD consumes the latest magnetometer and coarse Sun sensor reports together
with the corresponding inertial references. It accepts a solution only when:

- both reports and the reference-vector bus are valid;
- magnetometer and CSS reports are no older than their configured limits;
- every input vector exceeds `minimum_vector_norm`;
- the two measured and two reference vectors are sufficiently non-collinear,
  according to `minimum_triad_cross_norm`.

The algorithm is implemented with Simulink normalization, cross-product, matrix
concatenation and matrix-product blocks. It returns scalar-first `q_BI`, `DCM_BI`,
validity, solution time and the source sequence IDs in
`AttitudeInitializationBus`. Invalid geometry produces a safe identity attitude
with `valid = 0`; consumers must never treat that identity as a measurement.

TRIAD is an instantaneous attitude initializer, not a state estimator. It does
not propagate attitude, estimate gyro bias, maintain covariance or reject
innovations statistically.

## Planned six-state MEKF

`GNC/State Estimation (MEKF)` is intentionally an empty interface placeholder.
The next increment will use a multiplicative EKF with the six-dimensional error
state

```text
delta_x = [delta_theta_BI; delta_b_g]
```

where both terms are 3-vectors. The nominal quaternion will be initialized by
TRIAD and propagated with gyro reports; magnetometer and CSS observations will
correct attitude error and gyro bias. Covariance initialization, process/measurement
noise, gating, reset injection and behavior through eclipse remain to be designed
and tested before this subsystem can be called an estimator.

## Tests

`tests/gnc/AttitudeInitializationTest.m` verifies:

- JSON loading and typed GNC/reference/initialization bus contracts;
- finite and moving SGP4 output without GNSS input;
- exact TRIAD recovery for a known rotation;
- rejection of collinear vector pairs and safe invalid outputs;
- compiled wiring with no plant-truth or GNSS path into onboard references;
- an empty MEKF placeholder and no custom MATLAB Function chart in GNC.

Run the suite with:

```matlab
results = runtests("tests/gnc/AttitudeInitializationTest.m");
assertSuccess(results);
```
