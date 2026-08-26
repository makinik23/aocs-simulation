function result = runPlanetDoveOrbitValidation(referenceFile, duration_s)
% Description:
%   Runs a local Planet Dove OEM validation arc of configurable duration
%   through the full AOCS Simulink plant and reports orbit residuals.
%
% Arguments:
%   referenceFile - Optional Planet Dove MAT fixture path.
%   duration_s - Optional validation duration [s]. Use Inf for full fixture.
%
% Outputs:
%   result - Struct containing the trimmed reference and residual diagnostics.

rootDirectory = setupAocsPaths();
addpath(fullfile(rootDirectory, "validation", "planet"));

if nargin < 1 || strlength(string(referenceFile)) == 0
    referenceFile = fullfile(rootDirectory, ...
        "validation", "planet", "data", "planet_dove_oem_reference.mat");
end

if nargin < 2 || isempty(duration_s)
    duration_s = estimatePlanetDoveOrbitPeriod(loadPlanetDoveReference(referenceFile));
end

ref = loadPlanetDoveReference(referenceFile);
if ~isfinite(duration_s)
    duration_s = ref.time_s(end);
end
ref = trimPlanetReference(ref, duration_s);

configFile = writePlanetDoveValidationConfig(rootDirectory, ref);
cleanupConfig = onCleanup(@() deleteIfFileExists(configFile));

out = run_aocs_simulation(configFile);
AOCS = evalin("base", "AOCS");
cleanupModel = onCleanup(@() closeModelWithoutSaving(AOCS.Model.Name)); %#ok<NASGU>

data = extractFlightVisualizationData(out, AOCS);
residuals = computeOrbitResiduals(data.Time_s, data.Orbit.r_I_m, ...
    data.Orbit.v_I_m_s, ref.time_s, ref.r_I_ref_m, ref.v_I_ref_m_s);

printPlanetDoveResidualSummary(residuals, ref);

result = struct();
result.Reference = ref;
result.Residuals = residuals;
result.Duration_s = ref.time_s(end);
end

function ref = loadPlanetDoveReference(referenceFile)
% Description:
%   Loads and normalizes the Planet Dove MAT fixture.

if ~isfile(referenceFile)
    error("AOCS:Validation:MissingPlanetFixture", ...
        "Planet Dove fixture not found: %s", char(referenceFile));
end

ref = load(referenceFile);
ref.time_s = ref.time_s(:);
ref.r_I_ref_m = double(ref.r_I_ref_m);
ref.v_I_ref_m_s = double(ref.v_I_ref_m_s);
ref.Source.RefFrame = string(ref.Source.RefFrame);
ref.Source.TimeSystem = string(ref.Source.TimeSystem);
ref.Source.ObjectId = string(ref.Source.ObjectId);
end

function ref = trimPlanetReference(ref, duration_s)
% Description:
%   Keeps fixture samples up to the requested validation duration.

keep = ref.time_s <= min(duration_s, ref.time_s(end));
ref.time_s = ref.time_s(keep);
ref.r_I_ref_m = ref.r_I_ref_m(keep, :);
ref.v_I_ref_m_s = ref.v_I_ref_m_s(keep, :);
if isfield(ref, "time_utc_iso")
    ref.time_utc_iso = ref.time_utc_iso(keep);
end

if numel(ref.time_s) < 2
    error("AOCS:Validation:InsufficientPlanetSamples", ...
        "Planet Dove validation requires at least two reference samples.");
end
end

function configFile = writePlanetDoveValidationConfig(projectRootDirectory, ref)
% Description:
%   Writes a temporary scenario configured from the first Planet reference
%   state and conservative high-precision orbit dynamics.

config = struct();
config.schema = "AocsSimulationConfig/v1";
config.extends = fullfile(projectRootDirectory, "config", "AocsSimulationConfig.json");
config.mission = struct( ...
    "name", "planet_dove_orbit_validation", ...
    "description", "Planet Dove OEM validation for the high-precision orbit propagator");
config.results = struct("file", "planet_dove_validation.mat");
config.simulation = struct( ...
    "start_time_s", 0.0, ...
    "stop_time_s", ref.time_s(end), ...
    "sample_time_s", 1.0, ...
    "solver", "ode45", ...
    "relative_tolerance", 1e-9, ...
    "absolute_tolerance", 1e-10);
config.epoch = struct( ...
    "utc", planetEpochVector(ref.window_start_utc), ...
    "time_system", "UTC", ...
    "tdb_minus_utc_s", 69.184);
config.orbit = struct();
config.orbit.propagator = struct( ...
    "type", "numerical_high_precision", ...
    "output_frame", "ICRF", ...
    "gravity_model", "Spherical Harmonics", ...
    "earth_spherical_harmonics", "EGM2008", ...
    "spherical_harmonics_degree", 120, ...
    "use_eops", true, ...
    "eop_file", "aeroiersdata.mat", ...
    "use_third_body_gravity", true);
config.orbit.initial_state = struct( ...
    "type", "cartesian", ...
    "position_I_m", ref.r_I_ref_m(1, :), ...
    "velocity_I_m_s", ref.v_I_ref_m_s(1, :));
config.environment = planetValidationEnvironmentOverrides(ref);

configFile = string(tempname) + ".json";
writeJsonFile(configFile, config);
end

function environment = planetValidationEnvironmentOverrides(ref)
% Description:
%   Builds environment overrides for deterministic conservative validation.

dateRangeStart = planetReferenceDatetime(ref.window_start_utc) - days(1);
dateRangeStop = planetReferenceDatetime(ref.window_stop_utc) + days(1);

environment = struct();
environment.disturbances = struct("enabled", false);
environment.atmosphere = struct("enabled", false);
environment.srp = struct("enabled", false);
environment.sun = struct( ...
    "use_ephemeris_date_range", true, ...
    "ephemeris_start_utc", datevec(dateRangeStart), ...
    "ephemeris_end_utc", datevec(dateRangeStop));
environment.eclipse = struct( ...
    "enabled", false, ...
    "use_ephemeris_date_range", true, ...
    "ephemeris_start_utc", datevec(dateRangeStart), ...
    "ephemeris_end_utc", datevec(dateRangeStop));
end

function period_s = estimatePlanetDoveOrbitPeriod(ref)
% Description:
%   Estimates the osculating Keplerian period from the first reference state.

mu_m3_s2 = 3.986004418e14;
r0_I_m = ref.r_I_ref_m(1, :);
v0_I_m_s = ref.v_I_ref_m_s(1, :);
specificEnergy_J_kg = 0.5 * dot(v0_I_m_s, v0_I_m_s) - mu_m3_s2 / norm(r0_I_m);
semiMajorAxis_m = -mu_m3_s2 / (2.0 * specificEnergy_J_kg);
period_s = 2.0 * pi * sqrt(semiMajorAxis_m^3 / mu_m3_s2);
end

function printPlanetDoveResidualSummary(residuals, ref)
% Description:
%   Prints compact inertial and RTN residual statistics.

summary = residuals.Summary;
fprintf("Planet Dove orbit residuals [%s, %.0f s]\n", ...
    char(ref.Source.ObjectId), ref.time_s(end));
fprintf("  position norm [m]: rms=%.6g max=%.6g final=%.6g\n", ...
    summary.Position.Norm_m.Rms, summary.Position.Norm_m.Max, ...
    summary.Position.Norm_m.Final);
fprintf("  velocity norm [m/s]: rms=%.6g max=%.6g final=%.6g\n", ...
    summary.Velocity.Norm_m_s.Rms, summary.Velocity.Norm_m_s.Max, ...
    summary.Velocity.Norm_m_s.Final);
fprintf("  RTN position RMS [m]: R=%.6g T=%.6g N=%.6g\n", ...
    summary.Position.RTN_m.Rms(1), summary.Position.RTN_m.Rms(2), ...
    summary.Position.RTN_m.Rms(3));
fprintf("  RTN position final [m]: R=%.6g T=%.6g N=%.6g\n", ...
    summary.Position.RTN_m.Final(1), summary.Position.RTN_m.Final(2), ...
    summary.Position.RTN_m.Final(3));
end

function vector = planetEpochVector(isoUtc)
% Description:
%   Converts an ISO-8601 UTC timestamp into the JSON epoch vector.

vector = datevec(planetReferenceDatetime(isoUtc));
end

function timeUtc = planetReferenceDatetime(isoUtc)
% Description:
%   Parses the Planet fixture ISO-8601 UTC timestamp.

timeUtc = datetime(string(isoUtc), ...
    "InputFormat", "yyyy-MM-dd'T'HH:mm:ss.SSS'Z'", ...
    "TimeZone", "UTC");
end

function writeJsonFile(fileName, payload)
% Description:
%   Writes a pretty-printed JSON payload.

json = jsonencode(payload, "PrettyPrint", true);
fid = fopen(fileName, "w");
cleanup = onCleanup(@() fclose(fid));
if fid < 0
    error("AOCS:Validation:CannotWriteScenario", ...
        "Could not write temporary validation scenario: %s", char(fileName));
end
fprintf(fid, "%s\n", json);
end

function deleteIfFileExists(fileName)
% Description:
%   Deletes a temporary file if it still exists.

if isfile(fileName)
    delete(fileName);
end
end

function closeModelWithoutSaving(modelName)
% Description:
%   Closes a loaded model after validation while discarding mask updates.

modelName = char(modelName);
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end
