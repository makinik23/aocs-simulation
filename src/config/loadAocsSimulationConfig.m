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
spacecraftGeometry = requireStruct(spacecraft, "geometry", "spacecraft.geometry");
aerodynamics = requireStruct(spacecraft, "aerodynamics", "spacecraft.aerodynamics");
massProps = requireStruct(spacecraft, "mass_properties", "spacecraft.mass_properties");
initial = requireStruct(raw, "initial_conditions", "initial_conditions");
environment = requireStruct(raw, "environment", "environment");
disturbances = requireStruct(environment, "disturbances", "environment.disturbances");
sun = requireStruct(environment, "sun", "environment.sun");
srp = requireStruct(environment, "srp", "environment.srp");
eclipse = requireStruct(environment, "eclipse", "environment.eclipse");
earthOrientation = optionalStructField(environment, "earth_orientation");
atmosphere = requireStruct(environment, "atmosphere", "environment.atmosphere");
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

propagatorConfig = readOrbitPropagatorConfig(propagator);

keplerian = readKeplerianElements(initialKeplerian);
initialOrbitState = readInitialOrbitState(orbit, keplerian);
if initialOrbitState.Type == "keplerian"
    validateOrbitGeometry(initialOrbitState.Keplerian, centralBodyConstants.radius_m);
else
    validateInitialCartesianState(initialOrbitState.Cartesian, centralBodyConstants.radius_m);
end

dimensions_m = columnField(spacecraftGeometry, "dimensions_m", "spacecraft.geometry.dimensions_m", 3);
if any(dimensions_m <= 0.0)
    error("AOCS:Config:InvalidGeometry", ...
        "spacecraft.geometry.dimensions_m entries must be positive.");
end
mass_kg = scalarField(massProps, "mass_kg", "spacecraft.mass_properties.mass_kg", true);
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
disturbancesEnabled = optionalLogicalScalarField(disturbances, ...
    "enabled", "environment.disturbances.enabled", true);
rmmEnabled = disturbancesEnabled && logicalScalarField(disturbances, ...
    "residual_magnetic_moment_enabled", "environment.disturbances.residual_magnetic_moment_enabled");
gravityGradientEnabled = disturbancesEnabled && logicalScalarField(disturbances, ...
    "gravity_gradient_enabled", "environment.disturbances.gravity_gradient_enabled");
sunConfig = readSunConfig(sun);
earthOrientationConfig = readEarthOrientationConfig(earthOrientation, epochUtc, tdbMinusUtc_s);
atmosphereConfig = readAtmosphereConfig(atmosphere);
aerodynamicsConfig = readAerodynamicsConfig(aerodynamics, dimensions_m);
aerodynamicsConfig.Enabled = disturbancesEnabled && atmosphereConfig.Enabled && ...
    aerodynamicsConfig.Enabled;
srpConfig = readSrpConfig(srp);
srpConfig.Enabled = disturbancesEnabled && srpConfig.Enabled;
eclipseConfig = readEclipseConfig(eclipse);

if ~disturbancesEnabled
    M_ext_B = zeros(3, 1);
end

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
AOCS.Orbit.Propagator = propagatorConfig;
AOCS.Orbit.CentralBodyConstants = centralBodyConstants;
AOCS.Orbit.InitialState = initialOrbitState;
AOCS.Orbit.InitialKeplerian = initialOrbitState.Keplerian;
AOCS.Orbit.InitialCartesian = initialOrbitState.Cartesian;

AOCS.Spacecraft.Id = string(requireField(spacecraft, "id", "spacecraft.id"));
AOCS.Spacecraft.Dimensions_m = dimensions_m;
AOCS.Spacecraft.Mass_kg = mass_kg;
AOCS.Spacecraft.I_B = I_B;
AOCS.Spacecraft.Aerodynamics = aerodynamicsConfig;

AOCS.Initial.q_BI = q_BI;
AOCS.Initial.euler_BI_0_rad = euler_BI_0_rad;
AOCS.Initial.omega_BI_B = omega_BI_B;

AOCS.Environment.M_ext_B = M_ext_B;
AOCS.Environment.m_res_B = m_res_B;
AOCS.Environment.DisturbancesEnabled = disturbancesEnabled;
AOCS.Environment.RmmEnabled = rmmEnabled;
AOCS.Environment.GravityGradientEnabled = gravityGradientEnabled;
AOCS.Environment.Sun = sunConfig;
AOCS.Environment.EarthOrientation = earthOrientationConfig;
AOCS.Environment.Atmosphere = atmosphereConfig;
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
config.mass_kg = AOCS.Spacecraft.Mass_kg;
config.aero_enabled = double(AOCS.Spacecraft.Aerodynamics.Enabled);
config.aero_panel_normals_B = AOCS.Spacecraft.Aerodynamics.PanelNormals_B;
config.aero_panel_areas_m2 = AOCS.Spacecraft.Aerodynamics.PanelAreas_m2;
config.aero_panel_centers_B_m = AOCS.Spacecraft.Aerodynamics.PanelCenters_B_m;
config.aero_wall_temperature_K = AOCS.Spacecraft.Aerodynamics.WallTemperature_K;
config.aero_energy_accommodation = ...
    AOCS.Spacecraft.Aerodynamics.EnergyAccommodationCoefficient;
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
cartesian = AOCS.Orbit.InitialCartesian;

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
config.r_I_m = cartesian.position_I_m;
config.v_I_m_s = cartesian.velocity_I_m_s;
end

function validateInitialCartesianState(cartesian, centralBodyRadius_m)
% Description:
%   Checks that a Cartesian initial state is finite and starts above the
%   configured central-body surface.
%
% Arguments:
%   cartesian - Struct containing position_I_m and velocity_I_m_s vectors.
%   centralBodyRadius_m - Central-body reference radius [m].
%
% Outputs:
%   None.

radius_m = norm(cartesian.position_I_m);
speed_m_s = norm(cartesian.velocity_I_m_s);

if radius_m <= centralBodyRadius_m
    error("AOCS:Config:InvalidCartesianOrbitState", ...
        "orbit.initial_state.position_I_m must place the spacecraft above the central-body surface.");
end

if speed_m_s <= 0.0
    error("AOCS:Config:InvalidCartesianOrbitState", ...
        "orbit.initial_state.velocity_I_m_s must have non-zero norm.");
end
end

function config = readOrbitPropagatorConfig(propagator)
% Description:
%   Validates orbit.propagator and maps project-level choices to Simulink
%   Orbit Propagator mask values.
%
% Arguments:
%   propagator - JSON object from orbit.propagator.
%
% Outputs:
%   config - Struct containing normalized propagator settings.

propagatorType = enumStringField(propagator, "type", "orbit.propagator.type", ...
    ["kepler_unperturbed", "numerical_high_precision", "high_precision"]);
if propagatorType == "high_precision"
    propagatorType = "numerical_high_precision";
end

outputFrame = enumStringField(propagator, "output_frame", "orbit.propagator.output_frame", "ICRF");

config = struct();
config.Type = propagatorType;
config.OutputFrame = outputFrame;

switch propagatorType
    case "kepler_unperturbed"
        config.MaskPropagator = "Kepler (unperturbed)";
        config.StateFormatParameter = "stateFormatKep";
    case "numerical_high_precision"
        config.MaskPropagator = "Numerical (high precision)";
        config.StateFormatParameter = "stateFormatNum";
        configuredGravityModel = optionalEnumStringField(propagator, "gravity_model", ...
            "orbit.propagator.gravity_model", "Spherical Harmonics", "Spherical Harmonics");
        if configuredGravityModel == "Spherical Harmonics"
            config.GravityModel = "Spherical harmonics";
        end
        config.UseEOPs = optionalLogicalScalarField(propagator, ...
            "use_eops", "orbit.propagator.use_eops", true);
        config.UseThirdBodyGravity = optionalLogicalScalarField(propagator, ...
            "use_third_body_gravity", "orbit.propagator.use_third_body_gravity", true);
        config.EarthSphericalHarmonics = optionalEnumStringField(propagator, ...
            "earth_spherical_harmonics", "orbit.propagator.earth_spherical_harmonics", ...
            "EGM2008", "EGM2008");
        config.SphericalHarmonicsDegree = optionalScalarField(propagator, ...
            "spherical_harmonics_degree", "orbit.propagator.spherical_harmonics_degree", 2159, true);
        config.EOPFile = optionalStringScalarField(propagator, ...
            "eop_file", "orbit.propagator.eop_file", "aeroiersdata.mat");
end
end

function config = readAerodynamicsConfig(aerodynamics, dimensions_m)
% Description:
%   Validates homogeneous six-panel Sentman model settings and derives the
%   rectangular 3U panel geometry in +X, -X, +Y, -Y, +Z, -Z order.

config = struct();
config.Enabled = logicalScalarField(aerodynamics, ...
    "enabled", "spacecraft.aerodynamics.enabled");
config.Model = enumStringField(aerodynamics, "model", ...
    "spacecraft.aerodynamics.model", "sentman_multispecies");
config.CenterOfMassFromGeometricCenter_B_m = columnField(aerodynamics, ...
    "center_of_mass_from_geometric_center_B_m", ...
    "spacecraft.aerodynamics.center_of_mass_from_geometric_center_B_m", 3);
config.WallTemperature_K = columnField(aerodynamics, ...
    "wall_temperature_K", "spacecraft.aerodynamics.wall_temperature_K", 6);
config.EnergyAccommodationCoefficient = columnField(aerodynamics, ...
    "energy_accommodation_coefficient", ...
    "spacecraft.aerodynamics.energy_accommodation_coefficient", 6);

if any(config.WallTemperature_K <= 0.0)
    error("AOCS:Config:InvalidAerodynamics", ...
        "spacecraft.aerodynamics.wall_temperature_K entries must be positive.");
end
if any(config.EnergyAccommodationCoefficient < 0.0) || ...
        any(config.EnergyAccommodationCoefficient > 1.0)
    error("AOCS:Config:InvalidAerodynamics", ...
        "spacecraft.aerodynamics.energy_accommodation_coefficient entries must be in [0, 1].");
end

dx = dimensions_m(1);
dy = dimensions_m(2);
dz = dimensions_m(3);
config.PanelNormals_B = [ ...
    1.0, -1.0, 0.0, 0.0, 0.0, 0.0; ...
    0.0, 0.0, 1.0, -1.0, 0.0, 0.0; ...
    0.0, 0.0, 0.0, 0.0, 1.0, -1.0];
config.PanelAreas_m2 = [dy * dz; dy * dz; dx * dz; dx * dz; dx * dy; dx * dy];
geometricCenters_B_m = [ ...
    dx / 2.0, -dx / 2.0, 0.0, 0.0, 0.0, 0.0; ...
    0.0, 0.0, dy / 2.0, -dy / 2.0, 0.0, 0.0; ...
    0.0, 0.0, 0.0, 0.0, dz / 2.0, -dz / 2.0];
config.PanelCenters_B_m = geometricCenters_B_m - ...
    repmat(config.CenterOfMassFromGeometricCenter_B_m, 1, 6);
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
config.atmosphere_enabled = double(AOCS.Environment.Atmosphere.Enabled);
config.atmosphere_model_id = atmosphereModelId(AOCS.Environment.Atmosphere.Model);
config.atmosphere_mode_id = atmosphereModeId(AOCS.Environment.Atmosphere.Mode);
config.atmosphere_space_weather_source_id = ...
    atmosphereSpaceWeatherSourceId(AOCS.Environment.Atmosphere.SpaceWeatherSource);
config.atmosphere_uncertainty_enabled = double(AOCS.Environment.Atmosphere.UncertaintyEnabled);
config.rho_scale_factor = AOCS.Environment.Atmosphere.RhoScaleFactor;
config.f10_7_sfu = AOCS.Environment.Atmosphere.NominalSpaceWeather.F10_7_sfu;
config.f10_7_81d_sfu = AOCS.Environment.Atmosphere.NominalSpaceWeather.F10_7_81d_sfu;
config.kp = AOCS.Environment.Atmosphere.NominalSpaceWeather.Kp;
config.f30_sfu = AOCS.Environment.Atmosphere.NominalSpaceWeather.F30_sfu;
config.f30_81d_sfu = AOCS.Environment.Atmosphere.NominalSpaceWeather.F30_81d_sfu;
config.hp60 = AOCS.Environment.Atmosphere.NominalSpaceWeather.Hp60;
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


function config = readEarthOrientationConfig(earthOrientation, epochUtc, tdbMinusUtc_s)
% Description:
%   Reads Earth orientation settings and resolves the EOP values used by
%   high-accuracy IAU-2000/2006 ECI/ECEF transformations.
%
% Arguments:
%   earthOrientation - Optional JSON object from environment.earth_orientation.
%   epochUtc - Validated UTC epoch vector [year month day hour minute second]'.
%   tdbMinusUtc_s - Configured TDB minus UTC offset [s].
%
% Outputs:
%   config - Struct with scalar/vector EOP values in SI/radian units.

if isempty(earthOrientation)
    earthOrientation = struct();
end

config = struct();
config.Enabled = optionalLogicalScalarField(earthOrientation, ...
    "enabled", "environment.earth_orientation.enabled", true);
config.Source = optionalStringScalarField(earthOrientation, ...
    "source", "environment.earth_orientation.source", "aeroiersdata.mat");
config.Action = optionalEnumStringField(earthOrientation, ...
    "action", "environment.earth_orientation.action", ["None", "Warning", "Error"], "Warning");
config.DeltaAT_s = optionalScalarField(earthOrientation, ...
    "delta_at_s", "environment.earth_orientation.delta_at_s", tdbMinusUtc_s - 32.184, false);

if config.DeltaAT_s < 0 || config.DeltaAT_s > 100
    error("AOCS:Config:InvalidEarthOrientation", ...
        "environment.earth_orientation.delta_at_s must be a plausible TAI-UTC offset in [0, 100] seconds.");
end

if config.Enabled
    mjd = mjuliandate(epochUtc(:).');
    source = char(config.Source);
    action = char(config.Action);
    config.DeltaUT1_s = deltaUT1(mjd, "Source", source, "Action", action);
    config.PolarMotion_rad = polarMotion(mjd, "Source", source, "Action", action);
    config.DCIP_rad = deltaCIP(mjd, "Source", source, "Action", action);
else
    config.DeltaUT1_s = 0.0;
    config.PolarMotion_rad = [0.0, 0.0];
    config.DCIP_rad = [0.0, 0.0];
end

config.DeltaUT1_s = double(config.DeltaUT1_s(1));
config.PolarMotion_rad = reshape(double(config.PolarMotion_rad(1, :)), 1, 2);
config.DCIP_rad = reshape(double(config.DCIP_rad(1, :)), 1, 2);
end

function config = readAtmosphereConfig(atmosphere)
% Description:
%   Validates the atmosphere-model contract used by the density and
%   aerodynamic disturbance pipeline.
%
% Arguments:
%   atmosphere - JSON object from environment.atmosphere.
%
% Outputs:
%   config - Struct containing normalized atmosphere settings.

nominalSpaceWeather = requireStruct(atmosphere, ...
    "nominal_space_weather", "environment.atmosphere.nominal_space_weather");

config = struct();
config.Enabled = logicalScalarField(atmosphere, "enabled", "environment.atmosphere.enabled");
config.Model = enumStringField(atmosphere, "model", "environment.atmosphere.model", "dtm2020");
config.Mode = enumStringField(atmosphere, "mode", "environment.atmosphere.mode", ...
    ["operational", "research"]);
config.SpaceWeatherSource = enumStringField(atmosphere, ...
    "space_weather_source", "environment.atmosphere.space_weather_source", ["nominal", "file"]);
config.SpaceWeatherFile = optionalNullableStringScalarField(atmosphere, ...
    "space_weather_file", "environment.atmosphere.space_weather_file", "");
config.RhoScaleFactor = scalarField(atmosphere, ...
    "rho_scale_factor", "environment.atmosphere.rho_scale_factor", true);
config.UncertaintyEnabled = logicalScalarField(atmosphere, ...
    "uncertainty_enabled", "environment.atmosphere.uncertainty_enabled");
config.NominalSpaceWeather = readDtm2020NominalSpaceWeather(nominalSpaceWeather);

if config.SpaceWeatherSource == "file" && strlength(config.SpaceWeatherFile) == 0
    error("AOCS:Config:InvalidAtmosphere", ...
        "environment.atmosphere.space_weather_file must be non-empty when space_weather_source is 'file'.");
end
end

function spaceWeather = readDtm2020NominalSpaceWeather(nominalSpaceWeather)
% Description:
%   Reads constant DTM2020 driver values used before a time-series space
%   weather data source is connected.

spaceWeather = struct();
spaceWeather.F10_7_sfu = scalarField(nominalSpaceWeather, ...
    "f10_7_sfu", "environment.atmosphere.nominal_space_weather.f10_7_sfu", true);
spaceWeather.F10_7_81d_sfu = scalarField(nominalSpaceWeather, ...
    "f10_7_81d_sfu", "environment.atmosphere.nominal_space_weather.f10_7_81d_sfu", true);
spaceWeather.Kp = scalarField(nominalSpaceWeather, ...
    "kp", "environment.atmosphere.nominal_space_weather.kp", false);
spaceWeather.F30_sfu = scalarField(nominalSpaceWeather, ...
    "f30_sfu", "environment.atmosphere.nominal_space_weather.f30_sfu", true);
spaceWeather.F30_81d_sfu = scalarField(nominalSpaceWeather, ...
    "f30_81d_sfu", "environment.atmosphere.nominal_space_weather.f30_81d_sfu", true);
spaceWeather.Hp60 = scalarField(nominalSpaceWeather, ...
    "hp60", "environment.atmosphere.nominal_space_weather.hp60", false);

validateRange(spaceWeather.F10_7_sfu, 50.0, 400.0, ...
    "environment.atmosphere.nominal_space_weather.f10_7_sfu");
validateRange(spaceWeather.F10_7_81d_sfu, 50.0, 400.0, ...
    "environment.atmosphere.nominal_space_weather.f10_7_81d_sfu");
validateRange(spaceWeather.Kp, 0.0, 9.0, ...
    "environment.atmosphere.nominal_space_weather.kp");
validateRange(spaceWeather.F30_sfu, 50.0, 400.0, ...
    "environment.atmosphere.nominal_space_weather.f30_sfu");
validateRange(spaceWeather.F30_81d_sfu, 50.0, 400.0, ...
    "environment.atmosphere.nominal_space_weather.f30_81d_sfu");
validateRange(spaceWeather.Hp60, 0.0, 9.0, ...
    "environment.atmosphere.nominal_space_weather.hp60");
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
