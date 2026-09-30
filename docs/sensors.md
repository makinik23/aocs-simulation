# Sensor Modeling

This document defines the current simulated sensor layer between the flight
dynamics plant and downstream GNC algorithms.

## Runtime Role

The sensor subsystem consumes truth-like products from `PlantStateBus` and
publishes sampled measurements on `SensorMeasurementBus`. A separate onboard
`Drivers` subsystem turns receipt events into reports for GNC:

```text
Flight Dynamics / PlantStateBus
  -> Sensors
  -> SensorMeasurementBus + per-sensor DataReady events
  -> Drivers
  -> per-sensor report/status mailboxes
  -> GNC / Sensor Access calls Read...()
  -> SensorReportBus + SensorReadStatusBus snapshots
```

These models are plant-side measurement generators. They represent what onboard
sensors would report to GNC; they are not onboard navigation or propagation
algorithms.

## Configuration

The source of truth is `config/sensors.json` under the top-level `sensors`
object:

```text
sensors.gyro
sensors.magnetometer
sensors.coarse_sun_sensors
sensors.gnss
```

`loadAocsSimulationConfig` validates these sections and builds one top-level
numeric Simulink parameter:

```text
AOCS_SensorConfig     Bus: SensorConfigBus
```

The top-level config bus contains:

```text
Gyro                  Bus: GyroConfigBus
Magnetometer          Bus: MagnetometerConfigBus
CoarseSunSensors      Bus: CoarseSunSensorConfigBus
GNSS                  Bus: GnssConfigBus
```

Gyro, magnetometer, and coarse sun sensors support `mode = nominal` and
`mode = failure`. GNSS additionally supports `mode = dropout`. Nominal mode
uses the sensor model. Failure mode bypasses the nominal output and publishes
the configured deterministic failure value and validity flag. GNSS dropout
publishes zero position and velocity with `valid = 0`, so consumers must always
honor the validity flag.

The common mode IDs are `1 = nominal` and `2 = failure`. GNSS uses
`1 = nominal`, `2 = dropout`, and `3 = failure`.

The sample time for each sensor is configured separately and is also used to
discretize noise densities:

```text
noise_std = noise_density / sqrt(sample_time_s)
bias_random_walk_step_std = bias_random_walk_std * sqrt(sample_time_s)
```

## Measurement Buses

`SensorMeasurementBus` contains one sub-bus per sensor:

```text
Gyro                  Bus: GyroMeasurementBus
Magnetometer          Bus: MagnetometerMeasurementBus
CoarseSunSensors      Bus: CoarseSunSensorMeasurementBus
GNSS                  Bus: GnssMeasurementBus
```

The current measurement products are:

```text
Gyro.omega_rad_s                         [3x1] body rate measurement [rad/s]
Gyro.valid                               scalar validity flag

Magnetometer.B_B_T                       [3x1] body-frame magnetic field [T]
Magnetometer.valid                       scalar validity flag

CoarseSunSensors.sun_B_unit              [3x1] body-frame Sun direction
CoarseSunSensors.irradiance_W_m2         scalar estimated irradiance [W/m^2]
CoarseSunSensors.panel_signals_W_m2      [6x1] panel irradiance readings [W/m^2]
CoarseSunSensors.valid                   scalar validity flag

GNSS.r_I_m                               [3x1] inertial position [m]
GNSS.v_I_m_s                             [3x1] inertial velocity [m/s]
GNSS.valid                               scalar navigation-fix validity flag
```

## Onboard Drivers and Receipt Events

`Sensors` models measurement physics. `Drivers` models onboard acquisition;
only drivers publish timing and sequence metadata. The current paths are:

```text
Drivers/Gyro Driver/Receive Sample
Drivers/Magnetometer Driver/Receive Sample
Drivers/Coarse Sun Sensors Driver/Receive Sample
Drivers/GNSS Driver/Receive Sample
```

Each sensor has a `Measurement Sample Hold` for its complete payload and a
periodic function-call `DataReady` output using that sensor's configured sample
period. DataReady represents a modeled interface event such as data-ready or
completed frame reception. It is not a measurement-derived freshness flag.
The source event and measurement use the same period and zero phase offset.
CSS now holds the entire measurement, not just its random-noise input.

The Drivers subsystem receives the raw measurement bus and four separate
function-call event inputs. Each event executes exactly one `Receive Sample`
subsystem. It captures the payload and receipt clock together, increments its
own counter, and holds the complete report until the next event. The driver
has no polling timer and never tests whether current time is a multiple of a
sampling period. It does not compare measurement values to infer freshness.

`SensorReportBus` contains `Gyro`, `Magnetometer`, `CoarseSunSensors`, and `GNSS`.
Each per-sensor report retains the raw measurement fields and adds:

| Field | Type / unit | Meaning |
| --- | --- | --- |
| `receive_time_s` | scalar double, s | Onboard receipt time on the simulation clock, held between receipts. |
| `sequence_id` | scalar uint32 | Counter incremented on each receipt, including invalid reports. |

The individual report buses are `GyroReportBus`, `MagnetometerReportBus`,
`CoarseSunSensorReportBus`, and `GnssReportBus`. Each report is a nonvirtual bus
sampled as a unit. The passive Drivers output is a virtual top-level bus so
sensors retain independent sample rates. GNC instead assembles a nonvirtual
snapshot at its polling rate; this does not create new physical measurements.

Before the first receipt, report fields are zero (`valid = 0`, `sequence_id = 0`).
With the default zero-offset periodic events, the first receipt occurs at time
zero and has ID 1. Resetting/restarting the simulation resets the driver state.
The counter wraps modulo 2^32. A passive diagnostic consumer can initialize its last-seen ID to
zero and compare IDs for **inequality**, not greater-than; update the last-seen
ID for every new report, and use the payload only if `valid` permits it. This
assumes fewer than 2^32 unobserved receipts and a coordinated consumer reset.
The consuming GNC API uses driver-owned `has_data` and `new_data` flags instead.
TRIAD now applies freshness checks to magnetometer and CSS reports before attitude
initialization. The onboard SGP4 position reference is independent of GNSS; the
GNSS report remains available for a future navigation filter. There is one
last-report buffer per sensor, not a sample queue; a slower consumer can detect
skipped IDs but cannot recover overwritten data.

### Current Transport Assumptions

- Reception is immediate and lossless. Hence receipt and acquisition times
  currently coincide, but `receive_time_s` explicitly means **receipt time**.
  It must not be treated as physical measurement time once delay is introduced.
- The periodic DataReady source is an interface assumption, not a hardware
  feature claimed for a selected sensor. There is no SPI/I2C/UART, frame parser,
  scheduling jitter, packet loss, or hardware clock synchronization model yet.
- Failure/disabled modes and GNSS no-fix/dropout modes still produce scheduled
  status reports. `valid = 0` does not suppress a receipt. These modes model
  invalid measurements, not a disconnected interface or a powered-off device.
- Without a DataReady event the driver holds its old report, including its
  old validity. TRIAD additionally checks magnetometer and CSS report age; no
  general transport timeout or FDIR policy is implied by the validity bit.

The saved production model contains all four sensor-driver paths. Its structure
and block positions are maintained directly in Simulink; there is no generated
model-builder copy that must be kept synchronized.

### Consuming Sensor API

All four drivers expose PascalCase Simulink Functions: `ReadGyro()`,
`ReadMagnetometer()`, `ReadCoarseSunSensors()` and `ReadGnss()`. Each returns
`[report, status]` and owns separate report/status Data Store Memory blocks.
DataReady updates the mailbox. The existing report output is a non-consuming
view for diagnostics, currently terminated at the model root.

`report` uses the respective sensor report bus. `status` uses `DriverReadStatusBus`:

| Flag | Meaning at the instant of the read |
| --- | --- |
| `has_data` | At least one report has been received since initialization. |
| `new_data` | At least one report has arrived since the previous read. |
| `overrun` | A report was overwritten while still unread. Latched until read. |

Each read returns a snapshot and clears `new_data` and `overrun`, retaining
the report and `has_data`. The acknowledgement depends on the status read in
the block dataflow. Repeated reads return held data with `new_data = false`.
Invalid and identical sensor payloads are still new receipts. Sequence wrap
to zero does not mean that the mailbox is empty. An overrun returns the latest
report with both `new_data` and `overrun` true; it does not discard that report.

`GNC/Sensor Access` calls all four methods once per GNC tick using Function
Callers. It assembles a `SensorReportBus` snapshot for TRIAD and the MEKF
placeholder, plus a `SensorReadStatusBus` snapshot for the MEKF interface.
The independent period is `AOCS_GNCConfig.sample_time_s`, default `0.1 s`.
A Function-Call Split orders Sensor Access, TRIAD and the MEKF placeholder.
Atomic Sensors execution at priority 10 precedes the GNC generator at priority
20 on coincident hits; multi-rate integration tests verify actual receipt order.

This is a single-consumer, latest-report mailbox. Additional algorithms use
the Sensor Access snapshot, not another consuming call. Telemetry may inspect
the report output. Overrun reports lost history but cannot reconstruct it;
slower or jittering GNC execution requires a FIFO before gyro integration can
assume contiguous samples. A future MEKF must explicitly handle `overrun`,
invalid reports and receipt-time gaps. Embedded multitasking will also require
an atomic snapshot/acknowledgement implementation; the current tests cover
serialized Simulink execution, not concurrent firmware.

See [Sensor Driver API and the GNC Cycle](sensor_driver_api.md) for block-by-block
explanations, scheduling, configuration, verification and a complete flow diagram.

## Gyroscope

The gyroscope uses the plant attitude state body rate:

```text
input:  AttitudeState.omega_b
output: Gyro.omega_rad_s, Gyro.valid
```

Nominal measurement model:

```text
omega_scaled = (I + diag(scale_factor)) * omega_b
omega_cal    = misalignment_matrix * omega_scaled
omega_noisy  = omega_cal + bias + noise_std_rad_s * w_noise
omega_meas   = saturate(quantize(omega_noisy))
```

The bias is a discrete random walk:

```text
bias[k + 1] = bias[k] + bias_random_walk_step_std_rad_s * w_bias[k]
```

Both noise sources draw separate random streams on each body axis. The scalar
JSON seeds define the first stream; the Simulink blocks derive distinct seeds
for the other axes.

The symmetric saturation range is `+/- range_rad_s`, and quantization uses
`resolution_rad_s`. `valid` is true only when the sensor is enabled and the
selected mode reports a valid measurement.

## Magnetometer

The magnetometer consumes the body-frame magnetic field already produced by the
environment model:

```text
input:  Environment.B_B_T
output: Magnetometer.B_B_T, Magnetometer.valid
```

Nominal measurement model:

```text
B_scaled = (I + diag(scale_factor)) * B_B_T
B_cal    = misalignment_matrix * B_scaled
B_noisy  = B_cal + bias + noise_std_T * w_noise
B_meas   = saturate(quantize(B_noisy))
```

The bias uses the same discrete random-walk pattern as the gyroscope, with
magnetometer units:

```text
bias[k + 1] = bias[k] + bias_random_walk_step_std_T * w_bias[k]
```

The symmetric saturation range is `+/- range_T`, and quantization uses
`resolution_T`.

## Coarse Sun Sensors

The coarse sun sensor array consumes Sun products from `EnvironmentBus`:

```text
inputs:
  Environment.sun_B_unit
  Environment.solar_flux_shadowed_W_m2
  Environment.sun_visibility

outputs:
  CoarseSunSensors.sun_B_unit
  CoarseSunSensors.irradiance_W_m2
  CoarseSunSensors.panel_signals_W_m2
  CoarseSunSensors.valid
```

The default panel order is:

```text
+X, -X, +Y, -Y, +Z, -Z
```

For each panel, the ideal response is:

```text
cos_incidence = panel_normal_B' * sun_B_unit
in_fov        = cos_incidence >= cos(fov_half_angle_rad)
panel_ideal   = solar_flux_shadowed_W_m2 * max(cos_incidence, 0) * in_fov
```

Per-panel scale factor, bias, white noise, saturation, and quantization are then
applied:

```text
panel_raw  = panel_ideal * (1 + scale_factor) + bias + noise_std_W_m2 * w
panel_meas = quantize(clamp(panel_raw, 0, range_W_m2))
```

The body-frame Sun direction is reconstructed from opposing panel differences:

```text
sun_est = [
    panel(+X) - panel(-X)
    panel(+Y) - panel(-Y)
    panel(+Z) - panel(-Z)
]
sun_B_unit = sun_est / norm(sun_est)
```

The scalar irradiance product is the maximum panel signal. The measurement is
valid when the sensor is enabled, the plant reports direct Sun visibility, and
the estimated irradiance exceeds `min_valid_irradiance_W_m2`.

## GNSS Receiver

The GNSS model emulates the real-time position and velocity solution produced
by one logical, low-cost, single-frequency receiver on a 3U CubeSat. It is
calibrated to the error structures observed in flight on the Astrocast
CubeSats, while preserving the existing inertial PVT interface.

The receiver consumes the truth orbit state:

```text
inputs:
  OrbitState.r_I_m
  OrbitState.v_I_m_s

outputs:
  GNSS.r_I_m
  GNSS.v_I_m_s
  GNSS.valid
```

Position and velocity are sampled at `sample_time_s`. Errors are generated in
the instantaneous radial, along-track, and cross-track frame:

```text
R = r / norm(r)
N = cross(r, v) / norm(cross(r, v))
T = cross(N, R)
C_I_RTN = [R T N]
```

The nominal RTN errors are:

```text
delta_r_RTN = [k_iono * VTEC; 0; 0]
              + A_r .* sin(omega_orbit * t + phase)
              + x_r + sigma_white_r .* w_r

delta_v_RTN = A_v .* sin(omega_orbit * t + phase)
              + x_v + sigma_white_v .* w_v
```

`x_r` and `x_v` are independent first-order Gauss-Markov processes:

```text
alpha = exp(-sample_time_s / correlation_time_s)
x[k + 1] = alpha * x[k]
           + antenna_noise_scale * sigma_stationary
             * sqrt(1 - alpha^2) .* w[k]
```

The measured PVT solution is returned in the same inertial frame as the plant:

```text
r_meas_I = quantize(r_truth_I + C_I_RTN * delta_r_RTN)
v_meas_I = quantize(v_truth_I + C_I_RTN * delta_v_RTN)
```

The default profile is intentionally named an **Astrocast-like** profile rather
than a universal GNSS accuracy specification:

| Effect | Default | Basis |
| --- | --- | --- |
| Radial ionosphere bias | `15 TECU * 0.64 m/TECU = 9.6 m` | The coefficient is the published value for a 5 degree elevation mask; `15 TECU` is a representative scenario assumption. |
| Once-per-orbit position error | `[0, 0, 2] m` RTN | Flight-data amplitude reported in the out-of-plane component. |
| Once-per-orbit velocity error | `[0, 0, 0.05] m/s` RTN | Flight-data amplitude reported in the out-of-plane component. |
| Position Gauss-Markov standard deviation | `[10, 5, 3.5] m` RTN | Rounded representative values from Astrocast position residuals. |
| Velocity Gauss-Markov standard deviation | `[0.03, 0.03, 0.03] m/s` RTN | Engineering tuning at the reported centimeter-per-second scale. |
| Gauss-Markov correlation time | `120 s` | Engineering assumption; tune when receiver-specific time series are available. |
| White-noise standard deviation | `[0.5, 0.5, 0.5] m`, `[0.005, 0.005, 0.005] m/s` | Engineering residual-noise floor. |
| Acquisition time | `30 s` | Engineering assumption for startup behavior. |

The main flight-data reference is L. Mueller et al., *Real-time navigation
solutions of low-cost off-the-shelf GNSS receivers on board the Astrocast
constellation satellites*, Advances in Space Research 73 (2024),
[doi:10.1016/j.asr.2023.10.001](https://doi.org/10.1016/j.asr.2023.10.001).
It reports a positive radial offset of roughly 6-15 m, a once-per-revolution
out-of-plane effect with amplitudes of about 2 m and 5 cm/s, and large
antenna-placement-dependent changes in noise level.

The ionosphere coefficient comes from M. Garcia-Fernandez and O. Montenbruck,
*Low Earth orbit satellite navigation errors and vertical total electron
content in single-frequency GPS tracking*, Radio Science 41 (2006),
[doi:10.1029/2005RS003420](https://doi.org/10.1029/2005RS003420). That work
derives the approximately linear relation between unmodelled VTEC and radial
single-frequency PVT error.

`antenna_noise_scale` scales both colored and white stochastic errors. It is a
compact way to exercise antenna-installation quality without pretending to
model the RF path. `dropout_probability_per_sample` introduces independent
sample dropouts after `acquisition_time_s`; its default is zero because the
Astrocast publication does not provide a receiver-independent probability.
All random streams have separate deterministic seeds.

In `dropout` mode, the output vectors are zero and `valid = 0`. In `failure`
mode, configured deterministic position, velocity, and validity values are
published; setting `failure.valid = true` supports tests of an undetected stuck
or corrupted solution.

### Future GNSS Signal Simulator

This subsystem is a navigation-solution emulator, not a constellation or RF
signal simulator. A higher-fidelity follow-up should generate pseudorange and
Doppler observations from GNSS ephemerides, satellite and receiver clock
states, signal transit time, Earth occultation, antenna visibility, propagation
delays, and measurement noise. A separate onboard receiver model would then
solve those observations into PVT. That boundary will let GNC test receiver
algorithms and geometry-dependent outages instead of prescribing aggregate PVT
errors.

A useful starting point for that later layer is C. B. Chiaradia, H. K. Kuga,
and A. F. B. A. Prado, *Onboard and Real-Time Artificial Satellite Orbit
Determination Using GPS*, Mathematical Problems in Engineering (2013),
[doi:10.1155/2013/530516](https://doi.org/10.1155/2013/530516), which formulates
single-frequency pseudorange, receiver clock, and onboard filtering models.

## Simulink Locations

Current model paths:

```text
models/aocs_plant.slx/Sensors/Gyro
models/aocs_plant.slx/Sensors/Magnetometer
models/aocs_plant.slx/Sensors/Coarse Sun Sensors
models/aocs_plant.slx/Sensors/GNSS
```

Bus definitions live in:

```text
src/simulink/buses/config/
src/simulink/buses/measurements/
```

Config loading and conversion to bus-compatible numeric structs live in:

```text
src/config/loadAocsSimulationConfig.m
src/simulink/setupAocsSimulation.m
```

## Current Fidelity Boundary

- Noise and bias are deterministic for repeatable tests through configured
  random seeds.
- Scale factor and misalignment are static calibration errors.
- Gyro and magnetometer bias random walks are independent per sensor axis.
- Coarse sun sensors use an ideal cosine panel response with configurable field
  of view; there is no albedo, Earth IR, panel self-shadowing, temperature
  dependence, or optical aging model yet.
- GNSS is currently a navigation-solution measurement model. It includes a
  fixed startup acquisition delay, aggregate radial ionosphere bias,
  once-per-orbit terms, first-order correlated errors, white noise, and
  independent sample dropouts. It does not simulate constellation visibility,
  antenna masking, dilution of precision, raw pseudorange/Doppler, receiver
  clock states, troposphere, multipath, or the RF chain. Its configured standard
  deviations are scenario assumptions, not calibration data for a selected
  flight receiver.
- All current sensor models publish scalar validity flags. Detailed health
  status words can be added later if GNC needs fault classification instead of a
  simple valid/invalid contract.

## Tests

The sensor baseline lives in `tests/sensors/SensorsTest.m`. It contains 23
independent tests, grouped in pipeline order with consistent method prefixes
and MATLAB test tags:

| Tag | Method prefix | Scope | Tests |
| --- | --- | --- | --- |
| `Configuration` | `configuration` | Default settings, setup and operating modes | 6 |
| `Contracts` | `contracts` | Config buses, raw measurements and driver reports | 3 |
| `Wiring` | `wiring` | Sensor ports, sample periods and Sensors -> Drivers -> GNC connections | 4 |
| `MeasurementModels` | `measurement` | GNSS RTN frame and inertial error mapping | 3 |
| `Acquisition` | `acquisition` | Focused gyro timing, invalid reports, delayed receipt, identical payloads and counter rollover | 4 |
| `Integration` | `integration` | All sensors at independent rates, eclipse, failures, GNSS acquisition and dropout | 3 |

`tests/sensors/SensorDriverApiTest.m` adds 23 cases: five API behaviors for each
of four drivers, two multi-rate production-chain checks and GNC period validation.
Both suites run in `core`.

The acquisition tests isolate the gyro/receiver mechanism. The integration
section checks the complete multi-sensor chain; it also covers the CSS eclipse
transition. Tests do not depend on execution order. Harness builders and
configuration-fixture helpers are local functions below the test class, so the
complete sensor test setup can be inspected in one file. No test helper is
registered as a separate test case.

```matlab
setupAocsPaths
results = runtests("tests/sensors");
assertSuccess(results);

% Optional: run only one section.
results = runtests("tests/sensors/SensorsTest.m", "Tag", "Acquisition");
assertSuccess(results);
```
