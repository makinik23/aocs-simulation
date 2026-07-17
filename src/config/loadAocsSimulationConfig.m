function AOCS = loadAocsSimulationConfig(configFile, projectRoot)
% Description:
%   Reads the JSON source of truth, validates required fields, normalizes the
%   initial quaternion, derives Aerospace Blockset Euler initial conditions,
%   validates orbit/epoch inputs, and builds resolved paths for the model and
%   results.
%
% Arguments:
%   configFile - Optional path to an AocsSimulationConfig JSON file.
%   projectRoot - Optional project root used to resolve model and results paths.
%
% Outputs:
%   AOCS - Validated configuration struct. AOCS.Config contains only the
%          numeric plant inputs exposed through AOCS_ConfigBus.

if nargin < 2 || strlength(string(projectRoot)) == 0
    projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
end

if nargin < 1 || strlength(string(configFile)) == 0
    configFile = fullfile(projectRoot, "config", "AocsSimulationConfig.json");
end

if ~isfile(configFile)
    error("AOCS:Config:MissingFile", "AOCS config file not found: %s", configFile);
end

raw = readAocsConfigFile(configFile);

schema = stringScalarField(raw, "schema", "schema");
if schema ~= "AocsSimulationConfig/v1"
    error("AOCS:Config:UnsupportedSchema", ...
        "Unsupported AOCS config schema '%s'. Expected 'AocsSimulationConfig/v1'.", char(schema));
end

mission = requireStruct(raw, "mission", "mission");
models = requireStruct(raw, "models", "models");
results = requireStruct(raw, "results", "results");
sim = requireStruct(raw, "simulation", "simulation");
epoch = requireStruct(raw, "epoch", "epoch");
orbit = requireStruct(raw, "orbit", "orbit");
propagator = requireStruct(orbit, "propagator", "orbit.propagator");
initialKeplerian = requireStruct(orbit, "initial_keplerian", "orbit.initial_keplerian");
spacecraft = requireStruct(raw, "spacecraft", "spacecraft");
massProps = requireStruct(spacecraft, "mass_properties", "spacecraft.mass_properties");
initial = requireStruct(raw, "initial_conditions", "initial_conditions");
environment = requireStruct(raw, "environment", "environment");
disturbances = requireStruct(environment, "disturbances", "environment.disturbances");
sun = requireStruct(environment, "sun", "environment.sun");
srp = requireStruct(environment, "srp", "environment.srp");
eclipse = requireStruct(environment, "eclipse", "environment.eclipse");
numerics = requireStruct(raw, "numerics", "numerics");
conventions = requireStruct(raw, "conventions", "conventions");

epochUtc = columnField(epoch, "utc", "epoch.utc", 6);
validateUtcEpoch(epochUtc, "epoch.utc");
timeSystem = stringScalarField(epoch, "time_system", "epoch.time_system");
if timeSystem ~= "UTC"
    error("AOCS:Config:UnsupportedTimeSystem", ...
        "Unsupported epoch.time_system '%s'. Expected 'UTC'.", char(timeSystem));
end
tdbMinusUtc_s = scalarField(epoch, "tdb_minus_utc_s", "epoch.tdb_minus_utc_s", false);
validateTdbMinusUtc(tdbMinusUtc_s);
epochTdbJd = calendarUtcToJulianDate(epochUtc) + tdbMinusUtc_s / 86400.0;

centralBody = stringScalarField(orbit, "central_body", "orbit.central_body");
centralBodyConstants = centralBodyConstantsFor(centralBody);

propagatorType = stringScalarField(propagator, "type", "orbit.propagator.type");
if propagatorType ~= "kepler_unperturbed"
    error("AOCS:Config:UnsupportedOrbitPropagator", ...
        "Unsupported orbit.propagator.type '%s'. Expected 'kepler_unperturbed'.", char(propagatorType));
end

outputFrame = stringScalarField(propagator, "output_frame", "orbit.propagator.output_frame");
if outputFrame ~= "ICRF"
    error("AOCS:Config:UnsupportedOrbitFrame", ...
        "Unsupported orbit.propagator.output_frame '%s'. Expected 'ICRF'.", char(outputFrame));
end

keplerian = readKeplerianElements(initialKeplerian);
validateOrbitGeometry(keplerian, centralBodyConstants.radius_m);

I_B = matrixField(massProps, "inertia_B_kg_m2", "spacecraft.mass_properties.inertia_B_kg_m2", 3, 3);
if any(any(abs(I_B - I_B.') > 1e-12))
    error("AOCS:Config:InvalidInertia", "Inertia matrix I_B must be symmetric.");
end

if any(eig(I_B) <= 0)
    error("AOCS:Config:InvalidInertia", "Inertia matrix I_B must be positive definite.");
end

q_BI = columnField(initial, "q_BI", "initial_conditions.q_BI", 4);
qNorm = norm(q_BI);
if qNorm <= 0
    error("AOCS:Config:InvalidQuaternion", "Initial quaternion q_BI must have non-zero norm.");
end
q_BI = q_BI ./ qNorm;

euler_BI_0_rad = optionalColumnField(initial, "euler_BI_0_rad", "initial_conditions.euler_BI_0_rad", 3, quaternionToEuler321(q_BI));
omega_BI_B = columnField(initial, "omega_BI_B_rad_s", "initial_conditions.omega_BI_B_rad_s", 3);
M_ext_B = columnField(environment, "external_torque_B_Nm", "environment.external_torque_B_Nm", 3);
m_res_B = columnField(environment, "residual_magnetic_dipole_B_A_m2", ...
    "environment.residual_magnetic_dipole_B_A_m2", 3);
rmmEnabled = logicalScalarField(disturbances, ...
    "residual_magnetic_moment_enabled", "environment.disturbances.residual_magnetic_moment_enabled");
gravityGradientEnabled = logicalScalarField(disturbances, ...
    "gravity_gradient_enabled", "environment.disturbances.gravity_gradient_enabled");
sunConfig = readSunConfig(sun);
srpConfig = readSrpConfig(srp);
eclipseConfig = readEclipseConfig(eclipse);

if eclipseConfig.Enabled && sunConfig.EphemerisModel ~= eclipseConfig.EphemerisModel
    error("AOCS:Config:InconsistentEphemerisModel", ...
        "environment.sun.ephemeris_model must match environment.eclipse.ephemeris_model when eclipse is enabled.");
end

AOCS = struct();
AOCS.Meta.Schema = schema;
AOCS.Meta.ProjectRoot = string(projectRoot);
AOCS.Meta.ConfigFile = string(configFile);
AOCS.Meta.MissionName = string(requireField(mission, "name", "mission.name"));
AOCS.Meta.Description = string(optionalField(mission, "description", ""));

AOCS.Model.Name = string(requireField(models, "plant", "models.plant"));
AOCS.Model.Directory = fullfile(projectRoot, string(requireField(models, "directory", "models.directory")));
AOCS.Model.File = fullfile(AOCS.Model.Directory, AOCS.Model.Name + ".slx");

AOCS.Results.Directory = fullfile(projectRoot, string(requireField(results, "directory", "results.directory")));
AOCS.Results.File = fullfile(AOCS.Results.Directory, string(requireField(results, "file", "results.file")));

AOCS.Sim.StartTime_s = scalarField(sim, "start_time_s", "simulation.start_time_s", false);
AOCS.Sim.StopTime_s = scalarField(sim, "stop_time_s", "simulation.stop_time_s", true);
AOCS.Sim.SampleTime_s = scalarField(sim, "sample_time_s", "simulation.sample_time_s", true);
AOCS.Sim.Solver = string(requireField(sim, "solver", "simulation.solver"));
AOCS.Sim.RelTol = scalarField(sim, "relative_tolerance", "simulation.relative_tolerance", true);
AOCS.Sim.AbsTol = scalarField(sim, "absolute_tolerance", "simulation.absolute_tolerance", true);

if AOCS.Sim.StopTime_s <= AOCS.Sim.StartTime_s
    error("AOCS:Config:InvalidTimeSpan", "simulation.stop_time_s must be greater than simulation.start_time_s.");
end

AOCS.Epoch.Utc = epochUtc;
AOCS.Epoch.TimeSystem = timeSystem;
AOCS.Epoch.TdbMinusUtc_s = tdbMinusUtc_s;
AOCS.Epoch.TdbJulianDate = epochTdbJd;

AOCS.Orbit.CentralBody = centralBody;
AOCS.Orbit.Propagator.Type = propagatorType;
AOCS.Orbit.Propagator.OutputFrame = outputFrame;
AOCS.Orbit.CentralBodyConstants = centralBodyConstants;
AOCS.Orbit.InitialKeplerian = keplerian;

AOCS.Spacecraft.Id = string(requireField(spacecraft, "id", "spacecraft.id"));
AOCS.Spacecraft.I_B = I_B;

AOCS.Initial.q_BI = q_BI;
AOCS.Initial.euler_BI_0_rad = euler_BI_0_rad;
AOCS.Initial.omega_BI_B = omega_BI_B;

AOCS.Environment.M_ext_B = M_ext_B;
AOCS.Environment.m_res_B = m_res_B;
AOCS.Environment.RmmEnabled = rmmEnabled;
AOCS.Environment.GravityGradientEnabled = gravityGradientEnabled;
AOCS.Environment.Sun = sunConfig;
AOCS.Environment.SRP = srpConfig;
AOCS.Environment.Eclipse = eclipseConfig;

AOCS.Numerics.QuatNormEpsilon = scalarField(numerics, "quat_norm_epsilon", "numerics.quat_norm_epsilon", true);
AOCS.Numerics.MaxAllowedEnergyDrift = scalarField(numerics, "max_allowed_energy_drift", "numerics.max_allowed_energy_drift", true);
AOCS.Numerics.MaxAllowedHnormDrift = scalarField(numerics, "max_allowed_Hnorm_drift", "numerics.max_allowed_Hnorm_drift", true);

AOCS.Convention.QuaternionOrder = string(requireField(conventions, "quaternion_order", "conventions.quaternion_order"));
AOCS.Convention.q_BI = string(requireField(conventions, "q_BI", "conventions.q_BI"));
AOCS.Convention.C_BI = string(requireField(conventions, "C_BI", "conventions.C_BI"));
AOCS.Convention.omega_BI_B = string(requireField(conventions, "omega_BI_B", "conventions.omega_BI_B"));

AOCS.Config = buildBusConfig(AOCS);
AOCS.OrbitConfig = buildOrbitBusConfig(AOCS);
AOCS.EnvironmentConfig = buildEnvironmentBusConfig(AOCS);
AOCS.Raw = raw;
end

function config = buildBusConfig(AOCS)
% Description:
%   Selects only plant-facing numeric values from the full configuration.
%
% Arguments:
%   AOCS - Validated AOCS configuration struct.
%
% Outputs:
%   config - Struct matching createAocsConfigBus element names and dimensions.

config = struct();
config.I_B = AOCS.Spacecraft.I_B;
config.euler_BI_0_rad = AOCS.Initial.euler_BI_0_rad;
config.omega_BI_B_0 = AOCS.Initial.omega_BI_B;
config.M_ext_B = AOCS.Environment.M_ext_B;
end

function config = buildOrbitBusConfig(AOCS)
% Description:
%   Selects only numeric orbit values intended for Simulink block masks and
%   orbit/environment subsystems.
%
% Arguments:
%   AOCS - Validated AOCS configuration struct.
%
% Outputs:
%   config - Struct matching createAocsOrbitConfigBus element names.

keplerian = AOCS.Orbit.InitialKeplerian;

config = struct();
config.epoch_utc = AOCS.Epoch.Utc;
config.epoch_tdb_jd = AOCS.Epoch.TdbJulianDate;
config.mu_m3_s2 = AOCS.Orbit.CentralBodyConstants.mu_m3_s2;
config.central_body_radius_m = AOCS.Orbit.CentralBodyConstants.radius_m;
config.semi_major_axis_m = keplerian.semi_major_axis_m;
config.eccentricity = keplerian.eccentricity;
config.inclination_rad = keplerian.inclination_rad;
config.raan_rad = keplerian.raan_rad;
config.argument_of_periapsis_rad = keplerian.argument_of_periapsis_rad;
config.true_anomaly_rad = keplerian.true_anomaly_rad;
end

function config = buildEnvironmentBusConfig(AOCS)
% Description:
%   Selects numeric environment/disturbance values for orbit/environment
%   subsystems while keeping the current plant-facing config intact.
%
% Arguments:
%   AOCS - Validated AOCS configuration struct.
%
% Outputs:
%   config - Struct matching createAocsEnvironmentConfigBus element names.

config = struct();
config.rmm_enabled = double(AOCS.Environment.RmmEnabled);
config.gravity_gradient_enabled = double(AOCS.Environment.GravityGradientEnabled);
config.m_res_B_A_m2 = AOCS.Environment.m_res_B;
config.solar_constant_W_m2 = AOCS.Environment.Sun.SolarConstant_W_m2;
config.eclipse_enabled = double(AOCS.Environment.Eclipse.Enabled);
config.srp_enabled = double(AOCS.Environment.SRP.Enabled);
config.srp_area_ref_m2 = AOCS.Environment.SRP.AreaRef_m2;
config.srp_coefficient_reflectivity = AOCS.Environment.SRP.CoefficientReflectivity;
config.srp_center_of_pressure_B_m = AOCS.Environment.SRP.CenterOfPressure_B_m;
end

function raw = readAocsConfigFile(configFile)
% Description:
%   Reads a JSON config and applies optional recursive overrides declared with
%   a top-level "extends" field.
%
% Arguments:
%   configFile - Path to a JSON config file.
%
% Outputs:
%   raw - Fully merged raw config struct.

configFile = string(configFile);
raw = jsondecode(fileread(configFile));

if isfield(raw, "extends")
    baseFile = string(raw.extends);
    if ~isfile(baseFile)
        baseFile = fullfile(fileparts(configFile), baseFile);
    end

    raw = rmfield(raw, "extends");
    raw = mergeConfigStructs(readAocsConfigFile(baseFile), raw);
end
end

function merged = mergeConfigStructs(base, override)
% Description:
%   Recursively merges scalar JSON structs, with override values taking
%   precedence over the base config.
%
% Arguments:
%   base - Base decoded JSON struct.
%   override - Override decoded JSON struct.
%
% Outputs:
%   merged - Merged decoded JSON struct.

merged = base;
fields = fieldnames(override);

for k = 1:numel(fields)
    name = fields{k};
    if isfield(merged, name) && isstruct(merged.(name)) && isstruct(override.(name)) ...
            && isscalar(merged.(name)) && isscalar(override.(name))
        merged.(name) = mergeConfigStructs(merged.(name), override.(name));
    else
        merged.(name) = override.(name);
    end
end
end

function config = readSunConfig(sun)
% Description:
%   Validates the project-level Sun ephemeris configuration used by the
%   Aerospace Blockset Planetary Ephemeris block.
%
% Arguments:
%   sun - JSON object from environment.sun.
%
% Outputs:
%   config - Struct containing normalized Sun model settings.

config = struct();
config.Model = enumStringField(sun, "model", "environment.sun.model", "planet_ephemeris");
config.EphemerisModel = enumStringField(sun, "ephemeris_model", ...
    "environment.sun.ephemeris_model", ["DE405", "DE421", "DE423", "DE430", "DE432t"]);
config.SolarConstant_W_m2 = scalarField(sun, "solar_constant_W_m2", ...
    "environment.sun.solar_constant_W_m2", true);
config.UseEphemerisDateRange = logicalScalarField(sun, ...
    "use_ephemeris_date_range", "environment.sun.use_ephemeris_date_range");
config.EphemerisStartUtc = columnField(sun, ...
    "ephemeris_start_utc", "environment.sun.ephemeris_start_utc", 6);
config.EphemerisEndUtc = columnField(sun, ...
    "ephemeris_end_utc", "environment.sun.ephemeris_end_utc", 6);
config.Action = enumStringField(sun, "action", "environment.sun.action", ...
    ["Error", "Warning", "None"]);

validateUtcEpoch(config.EphemerisStartUtc, "environment.sun.ephemeris_start_utc");
validateUtcEpoch(config.EphemerisEndUtc, "environment.sun.ephemeris_end_utc");

if utcSerialDay(config.EphemerisEndUtc) <= utcSerialDay(config.EphemerisStartUtc)
    error("AOCS:Config:InvalidSunEphemerisRange", ...
        "environment.sun.ephemeris_end_utc must be later than environment.sun.ephemeris_start_utc.");
end
end

function config = readSrpConfig(srp)
% Description:
%   Validates the project-level solar radiation pressure disturbance model
%   configuration.
%
% Arguments:
%   srp - JSON object from environment.srp.
%
% Outputs:
%   config - Struct containing normalized SRP settings.

config = struct();
config.Enabled = logicalScalarField(srp, "enabled", "environment.srp.enabled");
config.Model = enumStringField(srp, "model", "environment.srp.model", "flat_plate_constant_area");
config.AreaRef_m2 = scalarField(srp, "area_ref_m2", "environment.srp.area_ref_m2", true);
config.CoefficientReflectivity = scalarField(srp, ...
    "coefficient_reflectivity", "environment.srp.coefficient_reflectivity", true);
config.CenterOfPressure_B_m = columnField(srp, ...
    "center_of_pressure_B_m", "environment.srp.center_of_pressure_B_m", 3);
end

function config = readEclipseConfig(eclipse)
% Description:
%   Validates the project-level eclipse model configuration used to drive
%   Aerospace Blockset Eclipse Shadow Model mask parameters.
%
% Arguments:
%   eclipse - JSON object from environment.eclipse.
%
% Outputs:
%   config - Struct containing normalized eclipse settings.

config = struct();
config.Enabled = logicalScalarField(eclipse, "enabled", "environment.eclipse.enabled");
config.Model = enumStringField(eclipse, "model", "environment.eclipse.model", "dual_cone");
config.TimeSource = enumStringField(eclipse, "time_source", "environment.eclipse.time_source", "dialog");
config.CentralBody = enumStringField(eclipse, "central_body", "environment.eclipse.central_body", "Earth");
config.IncludeEarth = logicalScalarField(eclipse, "include_earth", "environment.eclipse.include_earth");
config.IncludeMoon = logicalScalarField(eclipse, "include_moon", "environment.eclipse.include_moon");
config.OutputShadowRegion = logicalScalarField(eclipse, ...
    "output_shadow_region", "environment.eclipse.output_shadow_region");
config.EphemerisModel = enumStringField(eclipse, "ephemeris_model", ...
    "environment.eclipse.ephemeris_model", ["DE405", "DE421", "DE423", "DE430", "DE432t"]);
config.UseEphemerisDateRange = logicalScalarField(eclipse, ...
    "use_ephemeris_date_range", "environment.eclipse.use_ephemeris_date_range");
config.EphemerisStartUtc = columnField(eclipse, ...
    "ephemeris_start_utc", "environment.eclipse.ephemeris_start_utc", 6);
config.EphemerisEndUtc = columnField(eclipse, ...
    "ephemeris_end_utc", "environment.eclipse.ephemeris_end_utc", 6);
config.Action = enumStringField(eclipse, "action", "environment.eclipse.action", ...
    ["Error", "Warning", "None"]);
config.ZeroCrossing = logicalScalarField(eclipse, ...
    "zero_crossing", "environment.eclipse.zero_crossing");

validateUtcEpoch(config.EphemerisStartUtc, "environment.eclipse.ephemeris_start_utc");
validateUtcEpoch(config.EphemerisEndUtc, "environment.eclipse.ephemeris_end_utc");

if utcSerialDay(config.EphemerisEndUtc) <= utcSerialDay(config.EphemerisStartUtc)
    error("AOCS:Config:InvalidEclipseEphemerisRange", ...
        "environment.eclipse.ephemeris_end_utc must be later than environment.eclipse.ephemeris_start_utc.");
end
end

function section = requireStruct(parent, fieldName, displayName)
% Description:
%   Combines required-field lookup with a type check for JSON object sections.
%
% Arguments:
%   parent - Struct containing the requested field.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%
% Outputs:
%   section - Struct stored in parent.(fieldName).

section = requireField(parent, fieldName, displayName);
if ~isstruct(section)
    error("AOCS:Config:InvalidField", "Config field %s must be an object.", displayName);
end
end

function value = requireField(parent, fieldName, displayName)
% Description:
%   Throws a focused config error when a required JSON field is missing.
%
% Arguments:
%   parent - Struct containing the requested field.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%
% Outputs:
%   value - Value stored in parent.(fieldName).

fieldName = char(fieldName);
if ~isstruct(parent) || ~isfield(parent, fieldName)
    error("AOCS:Config:MissingField", "Missing required config field: %s", displayName);
end
value = parent.(fieldName);
end

function value = optionalField(parent, fieldName, defaultValue)
% Description:
%   Keeps optional JSON metadata reads explicit and local.
%
% Arguments:
%   parent - Struct that may contain fieldName.
%   fieldName - Field name to read.
%   defaultValue - Value returned when fieldName is absent.
%
% Outputs:
%   value - Field value or defaultValue.

fieldName = char(fieldName);
if isstruct(parent) && isfield(parent, fieldName)
    value = parent.(fieldName);
else
    value = defaultValue;
end
end

function value = stringScalarField(parent, fieldName, displayName)
% Description:
%   Reads a required JSON string and enforces scalar, non-empty shape.
%
% Arguments:
%   parent - Struct containing the requested field.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%
% Outputs:
%   value - Scalar string value.

rawValue = requireField(parent, fieldName, displayName);
if ~(ischar(rawValue) || (isstring(rawValue) && isscalar(rawValue)))
    error("AOCS:Config:InvalidField", "Config field %s must be a scalar string.", displayName);
end

value = string(rawValue);
if strlength(value) == 0
    error("AOCS:Config:InvalidField", "Config field %s must be non-empty.", displayName);
end
end

function value = enumStringField(parent, fieldName, displayName, allowedValues)
% Description:
%   Reads a required string field and constrains it to an allowed set.
%
% Arguments:
%   parent - Struct containing the requested field.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%   allowedValues - String-compatible array of supported values.
%
% Outputs:
%   value - Valid scalar string from allowedValues.

value = stringScalarField(parent, fieldName, displayName);
allowedValues = string(allowedValues);

if ~any(value == allowedValues)
    error("AOCS:Config:InvalidField", ...
        "Unsupported config field %s '%s'. Expected one of: %s.", ...
        displayName, char(value), strjoin(allowedValues, ", "));
end
end

function value = logicalScalarField(parent, fieldName, displayName)
% Description:
%   Reads a required JSON boolean and enforces scalar logical shape.
%
% Arguments:
%   parent - Struct containing the requested field.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%
% Outputs:
%   value - Scalar logical value.

rawValue = requireField(parent, fieldName, displayName);

if islogical(rawValue) && isscalar(rawValue)
    value = rawValue;
else
    error("AOCS:Config:InvalidField", "Config field %s must be a scalar boolean.", displayName);
end
end

function value = scalarField(parent, fieldName, displayName, mustBePositive)
% Description:
%   Converts JSON numeric values to double and enforces scalar shape plus
%   optional positivity.
%
% Arguments:
%   parent - Struct containing the requested field.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%   mustBePositive - True when zero and negative values are invalid.
%
% Outputs:
%   value - Validated finite real scalar double.

value = double(requireField(parent, fieldName, displayName));
validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar'}, mfilename, displayName);
if mustBePositive && value <= 0
    error("AOCS:Config:InvalidField", "Config field %s must be positive.", displayName);
end
end

function validateTdbMinusUtc(value)
% Description:
%   Guards the manually configured UTC->TDB epoch offset against unit mistakes.
%
% Arguments:
%   value - TDB minus UTC offset [s].
%
% Outputs:
%   None.

if value < 0 || value > 200
    error("AOCS:Config:InvalidTimeOffset", ...
        "Config field epoch.tdb_minus_utc_s must be a plausible seconds offset in [0, 200].");
end
end

function constants = centralBodyConstantsFor(centralBody)
% Description:
%   Resolves central-body constants used by orbit propagation and
%   environment products.
%
% Arguments:
%   centralBody - Name from orbit.central_body.
%
% Outputs:
%   constants - Struct with numeric constants for supported bodies.

if centralBody ~= "Earth"
    error("AOCS:Config:UnsupportedCentralBody", ...
        "Unsupported orbit.central_body '%s'. Expected 'Earth'.", char(centralBody));
end

constants = struct();
constants.mu_m3_s2 = 3.986004418e14;
constants.radius_m = 6378137.0;
end

function keplerian = readKeplerianElements(initialKeplerian)
% Description:
%   Validates classical Keplerian elements for an elliptical Earth orbit.
%
% Arguments:
%   initialKeplerian - JSON object from orbit.initial_keplerian.
%
% Outputs:
%   keplerian - Struct containing finite SI/radian Keplerian elements.

keplerian = struct();
keplerian.semi_major_axis_m = scalarField(initialKeplerian, ...
    "semi_major_axis_m", "orbit.initial_keplerian.semi_major_axis_m", true);
keplerian.eccentricity = scalarField(initialKeplerian, ...
    "eccentricity", "orbit.initial_keplerian.eccentricity", false);
keplerian.inclination_rad = scalarField(initialKeplerian, ...
    "inclination_rad", "orbit.initial_keplerian.inclination_rad", false);
keplerian.raan_rad = scalarField(initialKeplerian, ...
    "raan_rad", "orbit.initial_keplerian.raan_rad", false);
keplerian.argument_of_periapsis_rad = scalarField(initialKeplerian, ...
    "argument_of_periapsis_rad", "orbit.initial_keplerian.argument_of_periapsis_rad", false);
keplerian.true_anomaly_rad = scalarField(initialKeplerian, ...
    "true_anomaly_rad", "orbit.initial_keplerian.true_anomaly_rad", false);

if keplerian.eccentricity < 0 || keplerian.eccentricity >= 1
    error("AOCS:Config:InvalidKeplerianElements", ...
        "orbit.initial_keplerian.eccentricity must satisfy 0 <= e < 1 for the initial elliptical propagator.");
end

if keplerian.inclination_rad < 0 || keplerian.inclination_rad > pi
    error("AOCS:Config:InvalidKeplerianElements", ...
        "orbit.initial_keplerian.inclination_rad must satisfy 0 <= i <= pi.");
end
end

function validateOrbitGeometry(keplerian, centralBodyRadius_m)
% Description:
%   Checks that the configured initial ellipse is above the central body.
%
% Arguments:
%   keplerian - Validated Keplerian element struct.
%   centralBodyRadius_m - Central-body reference radius [m].
%
% Outputs:
%   None.

periapsisRadius_m = keplerian.semi_major_axis_m * (1 - keplerian.eccentricity);
if periapsisRadius_m <= centralBodyRadius_m
    error("AOCS:Config:InvalidKeplerianElements", ...
        "orbit.initial_keplerian gives a periapsis radius below the central-body radius.");
end
end

function validateUtcEpoch(epochUtc, displayName)
% Description:
%   Validates a UTC epoch vector [year month day hour minute second]'.
%
% Arguments:
%   epochUtc - 6-by-1 epoch vector.
%   displayName - Human-readable field path for error messages.
%
% Outputs:
%   None.

integerParts = epochUtc(1:5);
if any(abs(integerParts - round(integerParts)) > 0)
    error("AOCS:Config:InvalidEpoch", "Config field %s must use integer year/month/day/hour/minute values.", displayName);
end

year = epochUtc(1);
month = epochUtc(2);
day = epochUtc(3);
hour = epochUtc(4);
minute = epochUtc(5);
second = epochUtc(6);

if year < 1
    error("AOCS:Config:InvalidEpoch", "Config field %s has invalid year.", displayName);
end

if month < 1 || month > 12
    error("AOCS:Config:InvalidEpoch", "Config field %s has invalid month.", displayName);
end

maxDay = daysInMonth(year, month);
if day < 1 || day > maxDay
    error("AOCS:Config:InvalidEpoch", "Config field %s has invalid day for the given month/year.", displayName);
end

if hour < 0 || hour > 23
    error("AOCS:Config:InvalidEpoch", "Config field %s has invalid hour.", displayName);
end

if minute < 0 || minute > 59
    error("AOCS:Config:InvalidEpoch", "Config field %s has invalid minute.", displayName);
end

if second < 0 || second >= 60
    error("AOCS:Config:InvalidEpoch", "Config field %s has invalid second.", displayName);
end
end

function serialDay = utcSerialDay(epochUtc)
% Description:
%   Converts a validated UTC vector to a serial day for range comparisons.
%
% Arguments:
%   epochUtc - 6-by-1 UTC vector [year month day hour minute second]'.
%
% Outputs:
%   serialDay - MATLAB serial day number.

serialDay = datenum(epochUtc(:).');
end

function jd = calendarUtcToJulianDate(epochUtc)
% Description:
%   Converts a Gregorian UTC calendar vector to Julian date using arithmetic
%   that does not depend on Aerospace Toolbox helper functions.
%
% Arguments:
%   epochUtc - 6-by-1 UTC vector [year month day hour minute second]'.
%
% Outputs:
%   jd - Julian date corresponding to the UTC calendar instant.

year = floor(double(epochUtc(1)));
month = floor(double(epochUtc(2)));
day = floor(double(epochUtc(3)));

if month <= 2
    year = year - 1;
    month = month + 12;
end

a = floor(year / 100.0);
b = 2.0 - a + floor(a / 4.0);
dayFraction = (double(epochUtc(4)) ...
    + (double(epochUtc(5)) + double(epochUtc(6)) / 60.0) / 60.0) / 24.0;

jd = floor(365.25 * (year + 4716.0)) ...
    + floor(30.6001 * (month + 1.0)) ...
    + day + dayFraction + b - 1524.5;
end

function dayCount = daysInMonth(year, month)
% Description:
%   Returns Gregorian month length without requiring toolbox helpers.
%
% Arguments:
%   year - Integer year.
%   month - Integer month number.
%
% Outputs:
%   dayCount - Number of days in the requested month.

monthLengths = [31 28 31 30 31 30 31 31 30 31 30 31];
dayCount = monthLengths(month);
if month == 2 && isLeapYear(year)
    dayCount = 29;
end
end

function result = isLeapYear(year)
% Description:
%   Evaluates Gregorian leap-year rules.
%
% Arguments:
%   year - Integer year.
%
% Outputs:
%   result - True for leap years.

result = (mod(year, 4) == 0 && mod(year, 100) ~= 0) || mod(year, 400) == 0;
end

function value = columnField(parent, fieldName, displayName, rows)
% Description:
%   Converts a JSON vector to a MATLAB column vector and checks its length.
%
% Arguments:
%   parent - Struct containing the requested field.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%   rows - Required number of rows.
%
% Outputs:
%   value - rows-by-1 finite real double vector.

value = double(requireField(parent, fieldName, displayName));
value = value(:);
validateattributes(value, {'numeric'}, {'real', 'finite', 'size', [rows 1]}, mfilename, displayName);
end

function value = optionalColumnField(parent, fieldName, displayName, rows, defaultValue)
% Description:
%   Provides optional vector fields while still enforcing the same numeric
%   contract as required vector fields.
%
% Arguments:
%   parent - Struct that may contain fieldName.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%   rows - Required number of rows.
%   defaultValue - Vector used when fieldName is absent.
%
% Outputs:
%   value - rows-by-1 finite real double vector.

fieldName = char(fieldName);
if isstruct(parent) && isfield(parent, fieldName)
    value = double(parent.(fieldName));
    value = value(:);
else
    value = defaultValue(:);
end
validateattributes(value, {'numeric'}, {'real', 'finite', 'size', [rows 1]}, mfilename, displayName);
end

function value = matrixField(parent, fieldName, displayName, rows, cols)
% Description:
%   Converts a JSON matrix to double and enforces exact matrix dimensions.
%
% Arguments:
%   parent - Struct containing the requested field.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%   rows - Required number of rows.
%   cols - Required number of columns.
%
% Outputs:
%   value - rows-by-cols finite real double matrix.

value = double(requireField(parent, fieldName, displayName));
validateattributes(value, {'numeric'}, {'real', 'finite', 'size', [rows cols]}, mfilename, displayName);
end

function euler321 = quaternionToEuler321(q)
% Description:
%   Computes the Aerospace Blockset 6DOF initial Euler orientation from the
%   JSON quaternion. Pitch is clamped before asin to avoid roundoff overflow.
%
% Arguments:
%   q - 4-by-1 scalar-first quaternion [q0 q1 q2 q3]'.
%
% Outputs:
%   euler321 - 3-by-1 [roll pitch yaw]' vector [rad].

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

roll = atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1^2 + q2^2));
pitchArgument = 2 * (q0*q2 - q3*q1);
pitchArgument = min(max(pitchArgument, -1), 1);
pitch = asin(pitchArgument);
yaw = atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2^2 + q3^2));

euler321 = [roll; pitch; yaw];
end
