classdef PlanetDoveOrbitPropagationValidationTest < matlab.unittest.TestCase

    properties (Constant)
        ShortArcDuration_s = 15 * 60
        FullDayDuration_s = 24 * 3600
        ShortArcMaxPositionError_m = 25e3
        ShortArcRmsPositionError_m = 10e3
        ShortArcMaxVelocityError_m_s = 25
        OneOrbitMaxPositionError_m = 50e3
        OneOrbitRmsPositionError_m = 20e3
        OneOrbitMaxVelocityError_m_s = 50
        FullDayMaxPositionError_m = 1e6
        FullDayRmsPositionError_m = 5e5
        FullDayMaxVelocityError_m_s = 500
    end

    properties
        ProjectRoot
        ReferenceFile
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            % Description:
            %   Locates the project root and Planet Dove validation fixture.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.ProjectRoot = projectRoot();
            testCase.ReferenceFile = fullfile(testCase.ProjectRoot, ...
                "validation", "planet", "data", "planet_dove_oem_reference.mat");

            setupAocsPaths(testCase.ProjectRoot, true);
            addpath(fullfile(testCase.ProjectRoot, "validation", "planet"));
        end
    end

    methods (Test)
        function cartesianInitialStateConfiguresOrbitPropagatorMask(testCase)
            % Description:
            %   Verifies that a Cartesian initial state reaches the Aerospace
            %   Blockset Orbit Propagator mask without running a full simulation.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            ref = syntheticPlanetLikeReference();
            configFile = writePlanetDoveValidationConfig(testCase.ProjectRoot, ref);
            cleanupConfig = onCleanup(@() deleteIfFileExists(configFile));

            AOCS = setupAocsSimulation(configFile);
            load_system(AOCS.Model.File);
            cleanupModel = onCleanup(@() closeModelWithoutSaving(AOCS.Model.Name)); %#ok<NASGU>
            applyAocsSimulationSettings(AOCS.Model.Name, AOCS);

            block = findPlanetValidationOrbitPropagatorBlock(AOCS.Model.Name);

            testCase.verifyEqual(AOCS.Orbit.InitialState.Type, "cartesian");
            testCase.verifyEqual(string(get_param(block, "stateFormatNum")), "ICRF state vector");
            testCase.verifyEqual(string(get_param(block, "inertialPosition")), "AOCS_OrbitConfig.r_I_m");
            testCase.verifyEqual(string(get_param(block, "inertialVelocity")), "AOCS_OrbitConfig.v_I_m_s");
        end

        function planetDoveFixtureContainsUsableShortArc(testCase)
            % Description:
            %   Verifies that the local Planet Dove OEM fixture contains a
            %   monotonic, physically plausible LEO state-vector arc.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.assumeTrue(isfile(testCase.ReferenceFile), missingFixtureDiagnostic(testCase.ReferenceFile));

            ref = loadPlanetDoveReference(testCase.ReferenceFile);

            testCase.verifyGreaterThanOrEqual(numel(ref.time_s), 2);
            testCase.verifyEqual(ref.time_s(1), 0);
            testCase.verifyTrue(all(diff(ref.time_s) > 0.0), "Planet reference times must be strictly increasing.");
            testCase.verifyTrue(any(ref.time_s >= min(testCase.ShortArcDuration_s, ref.time_s(end))), ...
                "Planet reference fixture is too short for the configured validation duration.");

            testCase.verifyTrue(any(ref.Source.RefFrame == ["EME2000", "J2000", "ICRF"]), ...
                "Planet OEM reference frame should be EME2000/J2000-compatible.");
            testCase.verifyEqual(ref.Source.TimeSystem, "UTC");

            radius_m = vecnorm(ref.r_I_ref_m, 2, 2);
            speed_m_s = vecnorm(ref.v_I_ref_m_s, 2, 2);
            testCase.verifyGreaterThan(min(radius_m), 6.3e6);
            testCase.verifyLessThan(max(radius_m), 7.5e6);
            testCase.verifyGreaterThan(min(speed_m_s), 6.5e3);
            testCase.verifyLessThan(max(speed_m_s), 8.5e3);
        end

        function highPrecisionPropagatorMatchesPlanetDoveShortArc(testCase)
            % Description:
            %   Runs the full Simulink plant from a Planet Dove Cartesian initial
            %   state and compares propagated inertial states against the OEM arc.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.assumeTrue(isfile(testCase.ReferenceFile), missingFixtureDiagnostic(testCase.ReferenceFile));

            [residuals, ref] = runPlanetDovePropagationValidation( ...
                testCase, testCase.ShortArcDuration_s, "short arc");

            verifyPlanetDoveResidualLimits(testCase, residuals, ...
                testCase.ShortArcMaxPositionError_m, ...
                testCase.ShortArcRmsPositionError_m, ...
                testCase.ShortArcMaxVelocityError_m_s, ref.time_s(end));
        end

        function highPrecisionPropagatorMatchesPlanetDoveOneOrbit(testCase)
            % Description:
            %   Runs the full Simulink plant for approximately one Planet Dove
            %   orbit when long external validation is explicitly enabled and
            %   compares propagated inertial states against the OEM arc.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.assumeTrue(runLongPlanetValidation(), ...
                "Set AOCS_RUN_LONG_EXTERNAL_VALIDATION=1 to run the one-orbit Planet Dove validation.");
            testCase.assumeTrue(isfile(testCase.ReferenceFile), missingFixtureDiagnostic(testCase.ReferenceFile));

            ref = loadPlanetDoveReference(testCase.ReferenceFile);
            requestedDuration_s = estimateReferenceOrbitPeriod(ref);
            testCase.assumeTrue(ref.time_s(end) >= requestedDuration_s, ...
                "Planet Dove fixture is too short for a one-orbit validation arc.");

            [residuals, ref] = runPlanetDovePropagationValidation( ...
                testCase, requestedDuration_s, "one orbit");

            verifyPlanetDoveResidualLimits(testCase, residuals, ...
                testCase.OneOrbitMaxPositionError_m, ...
                testCase.OneOrbitRmsPositionError_m, ...
                testCase.OneOrbitMaxVelocityError_m_s, ref.time_s(end));
        end

        function highPrecisionPropagatorMatchesPlanetDoveFullDay(testCase)
            % Description:
            %   Runs the full Simulink plant for 24 hours against a Planet Dove
            %   OEM arc when long external validation is explicitly enabled.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.assumeTrue(runLongPlanetValidation(), ...
                "Set AOCS_RUN_LONG_EXTERNAL_VALIDATION=1 to run the 24h Planet Dove validation.");
            testCase.assumeTrue(isfile(testCase.ReferenceFile), missingFixtureDiagnostic(testCase.ReferenceFile));

            ref = loadPlanetDoveReference(testCase.ReferenceFile);
            testCase.assumeTrue(ref.time_s(end) >= testCase.FullDayDuration_s, ...
                "Planet Dove fixture is too short for a 24h validation arc.");

            [residuals, ref] = runPlanetDovePropagationValidation( ...
                testCase, testCase.FullDayDuration_s, "24h");

            verifyPlanetDoveResidualLimits(testCase, residuals, ...
                testCase.FullDayMaxPositionError_m, ...
                testCase.FullDayRmsPositionError_m, ...
                testCase.FullDayMaxVelocityError_m_s, ref.time_s(end));
        end
    end
end

function [residuals, ref] = runPlanetDovePropagationValidation(testCase, requestedDuration_s, label)
% Description:
%   Runs the plant from the Planet Cartesian state and computes orbit residuals.
%
% Arguments:
%   testCase - matlab.unittest.TestCase instance.
%   requestedDuration_s - Requested validation duration [s].
%   label - Diagnostic label printed with residual statistics.
%
% Outputs:
%   residuals - Orbit residual struct.
%   ref - Trimmed Planet reference fixture.

ref = loadPlanetDoveReference(testCase.ReferenceFile);
ref = trimPlanetReference(ref, requestedDuration_s);
testCase.assumeTrue(ref.time_s(end) > 0.0, ...
    "Planet Dove fixture does not contain a usable positive-duration arc.");

configFile = writePlanetDoveValidationConfig(testCase.ProjectRoot, ref);
cleanupConfig = onCleanup(@() deleteIfFileExists(configFile));

out = run_aocs_simulation(configFile);
AOCS = evalin("base", "AOCS");
cleanupModel = onCleanup(@() closeModelWithoutSaving(AOCS.Model.Name)); %#ok<NASGU>
data = extractFlightVisualizationData(out, AOCS);
residuals = computeOrbitResiduals(data.Time_s, data.Orbit.r_I_m, ...
    data.Orbit.v_I_m_s, ref.time_s, ref.r_I_ref_m, ref.v_I_ref_m_s);

printPlanetDoveResidualSummary(residuals, ref, label);
end

function verifyPlanetDoveResidualLimits( ...
    testCase, residuals, maxPositionError_m, rmsPositionError_m, maxVelocityError_m_s, duration_s)
% Description:
%   Applies common start-state and residual limits to one validation arc.
%
% Arguments:
%   testCase - matlab.unittest.TestCase instance.
%   residuals - Orbit residual struct from computeOrbitResiduals.
%   maxPositionError_m - Maximum allowed 3D position error [m].
%   rmsPositionError_m - Maximum allowed RMS 3D position error [m].
%   maxVelocityError_m_s - Maximum allowed 3D velocity error [m/s].
%   duration_s - Actual validation duration [s].
%
% Outputs:
%   None.

summary = residuals.Summary;

testCase.verifyLessThanOrEqual(norm(residuals.PositionError_I_m(1, :)), 1.0, ...
    sprintf("The propagated %.0f s arc should start from the first Planet reference position.", duration_s));
testCase.verifyLessThanOrEqual(norm(residuals.VelocityError_I_m_s(1, :)), 1e-3, ...
    sprintf("The propagated %.0f s arc should start from the first Planet reference velocity.", duration_s));
testCase.verifyLessThanOrEqual(summary.Position.Norm_m.Max, maxPositionError_m);
testCase.verifyLessThanOrEqual(summary.Position.Norm_m.Rms, rmsPositionError_m);
testCase.verifyLessThanOrEqual(summary.Velocity.Norm_m_s.Max, maxVelocityError_m_s);
end

function printPlanetDoveResidualSummary(residuals, ref, label)
% Description:
%   Prints compact inertial and RTN residual statistics for one validation arc.
%
% Arguments:
%   residuals - Orbit residual struct from computeOrbitResiduals.
%   ref - Trimmed Planet reference fixture.
%   label - Diagnostic label for the validation arc.
%
% Outputs:
%   None.

summary = residuals.Summary;
fprintf("Planet Dove orbit residuals [%s, %s, %.0f s]\n", ...
    char(ref.Source.ObjectId), char(label), ref.time_s(end));
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

function enabled = runLongPlanetValidation()
% Description:
%   Checks whether intentionally slow external validation cases are enabled.
%
% Arguments:
%   None.
%
% Outputs:
%   enabled - True when long external validation should run.

enabled = any(string(getenv("AOCS_RUN_LONG_EXTERNAL_VALIDATION")) == ["1", "true", "TRUE", "yes", "YES"]);
end

function period_s = estimateReferenceOrbitPeriod(ref)
% Description:
%   Estimates the osculating Keplerian period from the first reference state.
%
% Arguments:
%   ref - Planet reference fixture.
%
% Outputs:
%   period_s - Estimated orbital period [s].

mu_m3_s2 = 3.986004418e14;
r0_I_m = ref.r_I_ref_m(1, :);
v0_I_m_s = ref.v_I_ref_m_s(1, :);
specificEnergy_J_kg = 0.5 * dot(v0_I_m_s, v0_I_m_s) - mu_m3_s2 / norm(r0_I_m);
semiMajorAxis_m = -mu_m3_s2 / (2.0 * specificEnergy_J_kg);
period_s = 2.0 * pi * sqrt(semiMajorAxis_m^3 / mu_m3_s2);
end

function block = findPlanetValidationOrbitPropagatorBlock(modelName)
% Description:
%   Finds the single Orbit Propagator block in the plant.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%
% Outputs:
%   block - Orbit Propagator block path.

blocks = find_system(modelName, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "BlockType", "OrbitPropagator");
if numel(blocks) ~= 1
    error("AOCS:Validation:UnexpectedOrbitPropagatorCount", ...
        "Expected one Orbit Propagator block, found %d.", numel(blocks));
end

block = blocks{1};
end

function ref = syntheticPlanetLikeReference()
% Description:
%   Creates a small Planet-shaped reference struct for configuration tests.
%
% Arguments:
%   None.
%
% Outputs:
%   ref - Synthetic reference struct accepted by the scenario writer.

ref = struct();
ref.Source = struct("ObjectId", "synthetic", "RefFrame", "EME2000", "TimeSystem", "UTC");
ref.window_start_utc = "2024-01-01T00:00:00.000Z";
ref.window_stop_utc = "2024-01-01T00:01:00.000Z";
ref.time_s = [0.0; 60.0];
ref.r_I_ref_m = [
    6871000.0, 0.0, 0.0
    6855000.0, 455000.0, 0.0
];
ref.v_I_ref_m_s = [
    0.0, 7612.6, 0.0
    -505.0, 7595.0, 0.0
];
end

function diagnostic = missingFixtureDiagnostic(referenceFile)
% Description:
%   Builds the skip diagnostic for the external Planet validation fixture.
%
% Arguments:
%   referenceFile - Expected local MAT fixture path.
%
% Outputs:
%   diagnostic - Human-readable assumption diagnostic.

diagnostic = "Planet Dove OEM fixture is missing: " + string(referenceFile) + newline + ...
    "Generate it from a local Planet HWID_oem.txt with validation/planet/generatePlanetDoveReference.m.";
end

function ref = loadPlanetDoveReference(referenceFile)
% Description:
%   Loads and normalizes the Planet Dove MAT fixture.
%
% Arguments:
%   referenceFile - Planet Dove MAT fixture path.
%
% Outputs:
%   ref - Struct with column time and N-by-3 state arrays.

ref = load(referenceFile);
ref.time_s = ref.time_s(:);
ref.r_I_ref_m = double(ref.r_I_ref_m);
ref.v_I_ref_m_s = double(ref.v_I_ref_m_s);
ref.Source.RefFrame = string(ref.Source.RefFrame);
ref.Source.TimeSystem = string(ref.Source.TimeSystem);
ref.Source.ObjectId = string(ref.Source.ObjectId);
end

function ref = trimPlanetReference(ref, maxDuration_s)
% Description:
%   Keeps only the validation arc used by the end-to-end test.
%
% Arguments:
%   ref - Planet reference fixture struct.
%   maxDuration_s - Maximum duration from the first fixture sample [s].
%
% Outputs:
%   ref - Trimmed Planet reference fixture.

keep = ref.time_s <= min(maxDuration_s, ref.time_s(end));
ref.time_s = ref.time_s(keep);
ref.r_I_ref_m = ref.r_I_ref_m(keep, :);
ref.v_I_ref_m_s = ref.v_I_ref_m_s(keep, :);
if isfield(ref, "time_utc_iso")
    ref.time_utc_iso = ref.time_utc_iso(keep);
end
end

function configFile = writePlanetDoveValidationConfig(projectRoot, ref)
% Description:
%   Writes a temporary scenario configured from the first Planet reference
%   state and conservative high-precision orbit dynamics.
%
% Arguments:
%   projectRoot - Repository root directory.
%   ref - Trimmed Planet Dove reference fixture.
%
% Outputs:
%   configFile - Path to the temporary JSON scenario file.

config = struct();
config.schema = "AocsSimulationConfig/v1";
config.extends = fullfile(projectRoot, "config", "AocsSimulationConfig.json");
config.mission = struct( ...
    "name", "planet_dove_orbit_validation", ...
    "description", "Planet Dove OEM short-arc validation for the high-precision orbit propagator");
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
%
% Arguments:
%   ref - Trimmed Planet Dove reference fixture.
%
% Outputs:
%   environment - Scenario environment override struct.

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

function vector = planetEpochVector(isoUtc)
% Description:
%   Converts an ISO-8601 UTC timestamp into the JSON epoch vector.
%
% Arguments:
%   isoUtc - Timestamp string from the Planet reference fixture.
%
% Outputs:
%   vector - 1-by-6 UTC vector [year month day hour minute second].

vector = datevec(planetReferenceDatetime(isoUtc));
end

function timeUtc = planetReferenceDatetime(isoUtc)
% Description:
%   Parses the Planet fixture ISO-8601 UTC timestamp.
%
% Arguments:
%   isoUtc - Timestamp string.
%
% Outputs:
%   timeUtc - UTC datetime.

timeUtc = datetime(string(isoUtc), ...
    "InputFormat", "yyyy-MM-dd'T'HH:mm:ss.SSS'Z'", ...
    "TimeZone", "UTC");
end

function writeJsonFile(fileName, payload)
% Description:
%   Writes a pretty-printed JSON payload.
%
% Arguments:
%   fileName - Output JSON file path.
%   payload - Struct to encode.
%
% Outputs:
%   None.

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
%
% Arguments:
%   fileName - File path.
%
% Outputs:
%   None.

if isfile(fileName)
    delete(fileName);
end
end

function closeModelWithoutSaving(modelName)
% Description:
%   Closes a loaded model after validation while discarding mask updates.
%
% Arguments:
%   modelName - Simulink model name.
%
% Outputs:
%   None.

modelName = char(modelName);
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end
