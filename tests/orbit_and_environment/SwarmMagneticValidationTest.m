classdef SwarmMagneticValidationTest < matlab.unittest.TestCase

    properties
        ProjectRoot
        DataFile
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            % Description:
            %   Locates the project root and Swarm magnetic-validation fixture.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.ProjectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.DataFile = fullfile(testCase.ProjectRoot, ...
                "validation", "swarm", "data", ...
                "swarm_a_mag_lr_20240101_0000_0015.nc");

            addpath(testCase.ProjectRoot);
            addpath(fullfile(testCase.ProjectRoot, "src", "analysis"));
            addpath(fullfile(testCase.ProjectRoot, "src", "config"));
            addpath(fullfile(testCase.ProjectRoot, "src", "simulink"));
            addpath(fullfile(testCase.ProjectRoot, "tests", "harnesses"));
        end
    end

    methods (TestClassTeardown)
        function printValidationStatistics(testCase)
            % Description:
            %   Prints a compact statistical summary after the Swarm validation
            %   test class finishes.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            if ~isfile(testCase.DataFile)
                fprintf('\nSwarm magnetic validation statistics\n');
                fprintf('  Fixture missing: %s\n', char(testCase.DataFile));
                return;
            end

            result = swarmSimulinkValidationResult(testCase.ProjectRoot, testCase.DataFile, "cachedOnly");
            if isempty(result)
                return;
            end

            printSwarmValidationStatistics(result);
        end
    end

    methods (Test)
        function swarmFixtureContainsExpectedUtcTrack(testCase)
            % Description:
            %   Verifies that the Swarm A Level-1B fixture covers the expected
            %   UTC interval and contains a physically plausible LEO track.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.assertTrue(isfile(testCase.DataFile), ...
                "Swarm magnetic-validation fixture is missing.");

            data = readSwarmMagneticFixture(testCase.DataFile);

            testCase.verifyEqual(data.SampleCount, 900);
            testCase.verifyEqual(data.TimeUtc(1), datetime(2024, 1, 1, 0, 0, 0, "TimeZone", "UTC"));
            testCase.verifyEqual(data.TimeUtc(end), datetime(2024, 1, 1, 0, 14, 59, "TimeZone", "UTC"));
            testCase.verifyEqual(seconds(diff(data.TimeUtc)), ones(data.SampleCount - 1, 1));

            testCase.verifyEqual(data.NecLabels(:).', ["N", "E", "C"]);
            testCase.verifyEqual(data.QuaternionLabels(:).', ["1", "i", "j", "k"]);
            testCase.verifyTrue(all(data.Spacecraft == "A"), ...
                "The fixture should contain only Swarm A samples.");

            testCase.verifyEqual(readVariableUnits(testCase.DataFile, "B_NEC"), "nT");
            testCase.verifyEqual(readVariableUnits(testCase.DataFile, "B_NEC_IGRF"), "nT");
            testCase.verifyEqual(readVariableUnits(testCase.DataFile, "F"), "nT");
            testCase.verifyEqual(readVariableUnits(testCase.DataFile, "Radius"), "m");

            testCase.verifyGreaterThanOrEqual(min(data.Latitude_deg), -90);
            testCase.verifyLessThanOrEqual(max(data.Latitude_deg), 90);
            testCase.verifyGreaterThanOrEqual(min(data.Longitude_deg), -180);
            testCase.verifyLessThanOrEqual(max(data.Longitude_deg), 180);
            testCase.verifyGreaterThan(min(data.Radius_m), 6.7e6);
            testCase.verifyLessThan(max(data.Radius_m), 7.1e6);

            lla = swarmGeocentricToLla(data.Latitude_deg, data.Longitude_deg, data.Radius_m);
            testCase.verifyGreaterThan(min(lla(:, 3)), 4.0e5);
            testCase.verifyLessThan(max(lla(:, 3)), 6.0e5);
        end

        function swarmVectorAndScalarFieldsAreInternallyConsistent(testCase)
            % Description:
            %   Checks units, finite values, magnetic-field magnitudes, scalar
            %   intensity consistency, and quaternion normalization.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            data = readSwarmMagneticFixture(testCase.DataFile);

            testCase.verifyTrue(all(isfinite(data.B_NEC_nT(:))), "B_NEC must be finite.");
            testCase.verifyTrue(all(isfinite(data.B_NEC_IGRF_nT(:))), "B_NEC_IGRF must be finite.");
            testCase.verifyTrue(all(isfinite(data.F_nT)), "F must be finite.");
            testCase.verifyTrue(all(isfinite(data.F_IGRF_nT)), "F_IGRF must be finite.");
            testCase.verifyTrue(all(isfinite(data.q_NEC_CRF(:))), "q_NEC_CRF must be finite.");

            measuredNorm_nT = vecnorm(data.B_NEC_nT, 2, 2);
            modelNorm_nT = vecnorm(data.B_NEC_IGRF_nT, 2, 2);
            measuredScalarError_nT = abs(data.F_nT - measuredNorm_nT);
            modelScalarError_nT = abs(data.F_IGRF_nT - modelNorm_nT);
            quaternionNormError = abs(vecnorm(data.q_NEC_CRF, 2, 2) - 1);

            testCase.verifyGreaterThan(min(data.F_nT), 15e3, ...
                sprintf("Minimum measured magnetic-field intensity is %.3f nT.", min(data.F_nT)));
            testCase.verifyLessThan(max(data.F_nT), 80e3, ...
                sprintf("Maximum measured magnetic-field intensity is %.3f nT.", max(data.F_nT)));
            testCase.verifyGreaterThan(min(data.F_IGRF_nT), 15e3, ...
                sprintf("Minimum IGRF magnetic-field intensity is %.3f nT.", min(data.F_IGRF_nT)));
            testCase.verifyLessThan(max(data.F_IGRF_nT), 80e3, ...
                sprintf("Maximum IGRF magnetic-field intensity is %.3f nT.", max(data.F_IGRF_nT)));

            testCase.verifyLessThanOrEqual(max(measuredScalarError_nT), 2.0, ...
                sprintf("Maximum |F - norm(B_NEC)| is %.3f nT.", max(measuredScalarError_nT)));
            testCase.verifyLessThanOrEqual(max(modelScalarError_nT), 1e-8, ...
                sprintf("Maximum |F_IGRF - norm(B_NEC_IGRF)| is %.3e nT.", max(modelScalarError_nT)));
            testCase.verifyLessThanOrEqual(max(quaternionNormError), 1e-8, ...
                sprintf("Maximum q_NEC_CRF norm error is %.3e.", max(quaternionNormError)));
        end

        function simulinkMagneticOutputResidualAgainstSwarmIsBounded(testCase)
            % Description:
            %   Runs the dedicated geomagnetic-field harness from the measured
            %   Swarm A trajectory and checks the Simulink magnetic-field output
            %   against real Swarm A vector magnetometer measurements.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            result = swarmSimulinkValidationResult(testCase.ProjectRoot, testCase.DataFile);
            error_nT = result.ErrorToSwarm_nT;
            error_percent = result.ErrorToSwarm_percent;

            testCase.verifyLessThanOrEqual(median(error_nT), 250.0, ...
                sprintf("Median Simulink-to-Swarm magnetic-field error is %.3f nT.", median(error_nT)));
            testCase.verifyLessThanOrEqual(mean(error_nT), 350.0, ...
                sprintf("Mean Simulink-to-Swarm magnetic-field error is %.3f nT.", mean(error_nT)));
            testCase.verifyLessThanOrEqual(max(error_nT), 1200.0, ...
                sprintf("Maximum Simulink-to-Swarm magnetic-field error is %.3f nT.", max(error_nT)));
            testCase.verifyLessThanOrEqual(median(error_percent), 0.6, ...
                sprintf("Median Simulink-to-Swarm magnetic-field error is %.3f%%.", median(error_percent)));
            testCase.verifyLessThanOrEqual(max(error_percent), 3.0, ...
                sprintf("Maximum Simulink-to-Swarm magnetic-field error is %.3f%%.", max(error_percent)));
        end

        function simulinkMagneticOutputTracksViresIgrfReference(testCase)
            % Description:
            %   Compares the harness magnetic-field output with the VirES IGRF
            %   reference in the Swarm fixture. This exercises the Simulink
            %   ECI/LLA conversion, IGRF block, and inertial-frame output without
            %   introducing orbit-propagation drift.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            result = swarmSimulinkValidationResult(testCase.ProjectRoot, testCase.DataFile);
            error_nT = result.ErrorToViresIgrf_nT;

            testCase.verifyLessThanOrEqual(median(error_nT), 1e-6, ...
                sprintf("Median Simulink-to-VirES IGRF error is %.3f nT.", median(error_nT)));
            testCase.verifyLessThanOrEqual(mean(error_nT), 1e-6, ...
                sprintf("Mean Simulink-to-VirES IGRF error is %.3f nT.", mean(error_nT)));
            testCase.verifyLessThanOrEqual(max(error_nT), 1e-4, ...
                sprintf("Maximum Simulink-to-VirES IGRF error is %.3f nT.", max(error_nT)));
            testCase.verifyLessThanOrEqual(max(result.BodyFrameError_T), 1e-15, ...
                sprintf("Maximum B_B_T/B_I_T identity-attitude error is %.3e T.", ...
                max(result.BodyFrameError_T)));
        end

    end
end

function printSwarmValidationStatistics(result)
% Description:
%   Prints Simulink-to-Swarm validation error statistics.
%
% Arguments:
%   result - Struct returned by swarmSimulinkValidationResult.
%
% Outputs:
%   None.

error_nT = result.ErrorToSwarm_nT;
error_percent = result.ErrorToSwarm_percent;

fprintf('Swarm error [nT]: min=%.3f, max=%.3f, mean=%.3f, median=%.3f, std=%.3f\n', ...
    min(error_nT), max(error_nT), mean(error_nT), median(error_nT), std(error_nT));
fprintf('Swarm error [%% of measurement]: min=%.4f, max=%.4f, mean=%.4f, median=%.4f, std=%.4f\n', ...
    min(error_percent), max(error_percent), mean(error_percent), median(error_percent), std(error_percent));
end

function result = swarmSimulinkValidationResult(projectRoot, dataFile, mode)
% Description:
%   Runs or returns the cached Simulink validation result for the Swarm fixture.
%
% Arguments:
%   projectRoot - Project root containing config/, src/, and models/.
%   dataFile - Path to the Swarm NetCDF fixture.
%   mode - Optional string; "cachedOnly" returns [] instead of running.
%
% Outputs:
%   result - Struct with Simulink magnetic-field output and error histories.

persistent cachedProjectRoot cachedDataFile cachedResult

if nargin < 3
    mode = "run";
end

projectRoot = string(projectRoot);
dataFile = string(dataFile);
mode = string(mode);

if ~isempty(cachedResult) && cachedProjectRoot == projectRoot && cachedDataFile == dataFile
    result = cachedResult;
    return;
end

if mode == "cachedOnly"
    result = [];
    return;
end

result = runSwarmSimulinkValidation(projectRoot, dataFile);
cachedProjectRoot = projectRoot;
cachedDataFile = dataFile;
cachedResult = result;
end

function result = runSwarmSimulinkValidation(projectRoot, dataFile)
% Description:
%   Configures and runs the geomagnetic-field harness from the Swarm track.

projectRoot = string(projectRoot);
data = readSwarmMagneticFixture(dataFile);
configFile = writeSwarmHarnessConfig(projectRoot, data);
AOCS = setupAocsSimulation(configFile);
earthOrientation = AOCS.Environment.EarthOrientation;
swarmR_I_m = swarmEciTrack(data, earthOrientation);
harnessName = "SwarmGeomagneticHarness";
owner = "aocs_plant/Orbit & Environment/Environment Products/Geomagnetic Field Model";
harnessFile = fullfile(projectRoot, "tests", "harnesses", harnessName + ".slx");

if ~isfile(harnessFile)
    error("AOCS:Tests:MissingSwarmHarness", ...
        "Swarm geomagnetic harness is missing: %s", char(harnessFile));
end

load_system(AOCS.Model.File);
applyAocsSimulationSettings(AOCS.Model.Name, AOCS);
sltest.harness.open(owner, harnessName);
cleanup = onCleanup(@() closeSwarmHarness(AOCS.Model.Name, harnessName));

inputDataset = swarmHarnessInputDataset(data, swarmR_I_m, AOCS);
simIn = Simulink.SimulationInput(harnessName);
simIn = simIn.setExternalInput(inputDataset);
simIn = simIn.setModelParameter( ...
    "StartTime", "0", ...
    "StopTime", num2str(data.Timestamp_s(end)), ...
    "Solver", "ode4", ...
    "FixedStep", "1", ...
    "SaveOutput", "on", ...
    "OutputSaveName", "yout", ...
    "SaveFormat", "Dataset", ...
    "SignalLogging", "on", ...
    "SignalLoggingName", "logsout");

simOut = sim(simIn);

[B_I_time_s, B_I_T, B_B_time_s, B_B_T] = harnessMagneticFieldOutputWithTime(simOut);

B_I_T = interp1(B_I_time_s, B_I_T, data.Timestamp_s, "linear");
B_B_T = interp1(B_B_time_s, B_B_T, data.Timestamp_s, "linear");

if any(isnan(B_I_T), "all") || any(isnan(B_B_T), "all")
    error("AOCS:Tests:SwarmSimInterpolationFailed", ...
        "Could not interpolate Simulink outputs to all Swarm fixture timestamps.");
end

B_NEC_sim_nT = inertialBToSwarmNec(B_I_T, swarmR_I_m, data.TimeUtc, earthOrientation);
validVectorSamples = data.Flags_B == 0;
measurementNorm_nT = vecnorm(data.B_NEC_nT(validVectorSamples, :), 2, 2);

result = struct();
result.Data = data;
result.ConfigFile = configFile;
result.HarnessName = harnessName;
result.SimTime_s = data.Timestamp_s;
result.SimulinkB_NEC_nT = B_NEC_sim_nT;
result.SimulinkB_I_T = B_I_T;
result.SimulinkB_B_T = B_B_T;
result.SwarmR_I_m = swarmR_I_m;
result.ValidVectorSamples = validVectorSamples;
result.ErrorToSwarm_nT = vecnorm( ...
    B_NEC_sim_nT(validVectorSamples, :) - data.B_NEC_nT(validVectorSamples, :), 2, 2);
result.ErrorToSwarm_percent = 100 .* result.ErrorToSwarm_nT ./ measurementNorm_nT;
result.ErrorToViresIgrf_nT = vecnorm( ...
    B_NEC_sim_nT(validVectorSamples, :) - data.B_NEC_IGRF_nT(validVectorSamples, :), 2, 2);
result.BodyFrameError_T = vecnorm(B_B_T(validVectorSamples, :) - B_I_T(validVectorSamples, :), 2, 2);
end

function configFile = writeSwarmHarnessConfig(projectRoot, data)
% Description:
%   Writes a temporary config matching the Swarm fixture epoch/span.

startUtc = datevec(data.TimeUtc(1));
config = struct();
config.extends = char(fullfile(projectRoot, "config", "AocsSimulationConfig.json"));
config.mission = struct( ...
    "name", "swarm_a_magnetic_validation", ...
    "description", "Swarm A magnetic-field validation scenario for Simulink harness output");
config.results = struct("file", "aocs_swarm_validation.mat");
config.simulation = struct( ...
    "start_time_s", 0.0, ...
    "stop_time_s", double(data.Timestamp_s(end)), ...
    "sample_time_s", 1.0, ...
    "solver", "ode4", ...
    "relative_tolerance", 1.0e-9, ...
    "absolute_tolerance", 1.0e-10);
config.epoch = struct( ...
    "utc", reshape(double(startUtc), 1, []), ...
    "time_system", "UTC", ...
    "tdb_minus_utc_s", 69.184);

try
    encoded = jsonencode(config, "PrettyPrint", true);
catch
    encoded = jsonencode(config);
end

configFile = string(tempname) + ".json";
fid = fopen(configFile, "w");
if fid < 0
    error("AOCS:Tests:CouldNotWriteSwarmConfig", ...
        "Could not write temporary Swarm validation config: %s", char(configFile));
end
cleanup = onCleanup(@() fclose(fid));
fwrite(fid, encoded, "char");
end

function inputDataset = swarmHarnessInputDataset(data, r_I_m, AOCS)
% Description:
%   Builds the flattened root-Inport dataset for SwarmGeomagneticHarness.

time_s = data.Timestamp_s(:);
sampleCount = data.SampleCount;
DCM_be = repmat(eye(3), 1, 1, sampleCount);
dyear = swarmDecimalYear(data.TimeUtc);
epochUtc = datevec(data.TimeUtc(1));
earthOrientation = AOCS.Environment.EarthOrientation;

inputDataset = Simulink.SimulationData.Dataset();
inputDataset = inputDataset.addElement(namedTimeseries("AttitudeState_euler_rad", zeros(sampleCount, 3), time_s));
inputDataset = inputDataset.addElement(namedTimeseries("AttitudeState_q_be", repmat([1, 0, 0, 0], sampleCount, 1), time_s));
inputDataset = inputDataset.addElement(namedTimeseries("AttitudeState_DCM_be", DCM_be, time_s));
inputDataset = inputDataset.addElement(namedTimeseries("AttitudeState_omega_b", zeros(sampleCount, 3), time_s));
inputDataset = inputDataset.addElement(namedTimeseries("EnvironmentContext_decimal_year", dyear, time_s));
inputDataset = inputDataset.addElement(namedTimeseries("EnvironmentContext_mu_m3_s2", repmat(3.986004418e14, 1, 1, sampleCount), time_s));
inputDataset = inputDataset.addElement(namedTimeseries("EnvironmentContext_epoch_utc", repmat(reshape(epochUtc, 6, 1), 1, 1, sampleCount), time_s));
inputDataset = inputDataset.addElement(namedTimeseries("EnvironmentContext_t_s", time_s, time_s));
inputDataset = inputDataset.addElement(namedTimeseries("EnvironmentContext_epoch_tdb_jd", ...
    repmat(AOCS.Epoch.TdbJulianDate, sampleCount, 1), time_s));
inputDataset = inputDataset.addElement(namedTimeseries("EnvironmentContext_delta_at_s", ...
    repmat(earthOrientation.DeltaAT_s, sampleCount, 1), time_s));
inputDataset = inputDataset.addElement(namedTimeseries("EnvironmentContext_delta_ut1_s", ...
    repmat(earthOrientation.DeltaUT1_s, sampleCount, 1), time_s));
inputDataset = inputDataset.addElement(namedTimeseries("EnvironmentContext_polar_motion_rad", ...
    repmat(earthOrientation.PolarMotion_rad, sampleCount, 1), time_s));
inputDataset = inputDataset.addElement(namedTimeseries("EnvironmentContext_d_cip_rad", ...
    repmat(earthOrientation.DCIP_rad, sampleCount, 1), time_s));
inputDataset = inputDataset.addElement(namedTimeseries("OrbitState_r_I_m", r_I_m, time_s));
inputDataset = inputDataset.addElement(namedTimeseries("OrbitState_v_I_m_s", swarmVelocityTrack(r_I_m, time_s), time_s));
end

function ts = namedTimeseries(name, values, time_s)
% Description:
%   Creates a named timeseries for a flattened harness root Inport.

ts = timeseries(values, time_s(:));
ts.Name = char(name);
end

function v_I_m_s = swarmVelocityTrack(r_I_m, time_s)
% Description:
%   Estimates inertial velocity samples for the OrbitState bus.

time_s = time_s(:);
v_I_m_s = zeros(size(r_I_m));
for k = 1:3
    v_I_m_s(:, k) = gradient(r_I_m(:, k), time_s);
end
end

function decimalYear = swarmDecimalYear(timeUtc)
% Description:
%   Converts UTC datetimes to decimal years for the Simulink IGRF block.

utc = datevec(timeUtc);
decimalYear = decyear(utc(:, 1), utc(:, 2), utc(:, 3), utc(:, 4), utc(:, 5), utc(:, 6));
decimalYear = decimalYear(:);
end

function closeSwarmHarness(modelName, harnessName)
% Description:
%   Closes the harness and owner model without saving test-time parameter edits.

if bdIsLoaded(harnessName)
    close_system(harnessName, 0);
end

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end

function r_I_m = swarmEciTrack(data, earthOrientation)
% Description:
%   Converts the Swarm geocentric ITRF track into the project inertial frame.

r_ECEF_m = swarmEcefTrack(data.Latitude_deg, data.Longitude_deg, data.Radius_m);
utc = datevec(data.TimeUtc);
r_I_m = zeros(data.SampleCount, 3);

for k = 1:data.SampleCount
    C_ECEF_I = highAccuracyEciToEcefDcm(utc(k, :), earthOrientation);
    r_I_m(k, :) = (C_ECEF_I.' * r_ECEF_m(k, :).').';
end
end

function C_ECEF_I = highAccuracyEciToEcefDcm(utc, earthOrientation)
% Description:
%   Applies the same IAU-2000/2006 EOP inputs used by the Simulink blocks.

C_ECEF_I = dcmeci2ecef("IAU-2000/2006", utc, ...
    earthOrientation.DeltaAT_s, ...
    earthOrientation.DeltaUT1_s, ...
    earthOrientation.PolarMotion_rad, ...
    "dCIP", earthOrientation.DCIP_rad);
end

function r_ECEF_m = swarmEcefTrack(latitude_deg, longitude_deg, radius_m)
% Description:
%   Converts geocentric latitude, longitude, and radius to ECEF position.

x_m = radius_m .* cosd(latitude_deg) .* cosd(longitude_deg);
y_m = radius_m .* cosd(latitude_deg) .* sind(longitude_deg);
z_m = radius_m .* sind(latitude_deg);
r_ECEF_m = [x_m, y_m, z_m];
end

function B_NEC_nT = inertialBToSwarmNec(B_I_T, r_I_m, timeUtc, earthOrientation)
% Description:
%   Converts harness inertial magnetic-field samples into Swarm geocentric NEC.

utc = datevec(timeUtc);
B_NEC_nT = zeros(size(B_I_T));

for k = 1:size(B_I_T, 1)
    C_ECEF_I = highAccuracyEciToEcefDcm(utc(k, :), earthOrientation);
    B_ECEF_T = C_ECEF_I * B_I_T(k, :).';
    r_ECEF_m = C_ECEF_I * r_I_m(k, :).';
    [latitudeGeocentric_deg, longitudeGeocentric_deg] = geocentricLatLonFromEcef(r_ECEF_m);
    C_NEC_ECEF = geocentricNecDcm(latitudeGeocentric_deg, longitudeGeocentric_deg);
    B_NEC_nT(k, :) = (1e9 .* (C_NEC_ECEF * B_ECEF_T)).';
end
end

function [latitude_deg, longitude_deg] = geocentricLatLonFromEcef(r_ECEF_m)
% Description:
%   Computes geocentric latitude and longitude from an ECEF position vector.

r = r_ECEF_m(:);
longitude_deg = atan2d(r(2), r(1));
latitude_deg = atan2d(r(3), hypot(r(1), r(2)));
end

function [B_I_time_s, B_I_T, B_B_time_s, B_B_T] = harnessMagneticFieldOutputWithTime(simOut)
% Description:
%   Reads B_I_T and B_B_T from the harness MagneticField bus output.

yout = simOut.get("yout");
[B_I_time_s, B_I_T] = outputVectorByAnyName(yout, ["B_I_T", "MagneticField.B_I_T", "MagneticField_B_I_T"]);
[B_B_time_s, B_B_T] = outputVectorByAnyName(yout, ["B_B_T", "MagneticField.B_B_T", "MagneticField_B_B_T"]);
end

function [time_s, data] = outputVectorByAnyName(yout, candidateNames)
% Description:
%   Finds a vector output in a Dataset, including nested bus Dataset values.

[found, time_s, data] = tryDatasetNames(yout, candidateNames);
if found
    return;
end

for k = 1:yout.numElements
    element = yout{k};
    if ~isa(element, "timeseries") && isprop(element, "Values")
        values = element.Values;
        if isa(values, "Simulink.SimulationData.Dataset")
            [found, time_s, data] = tryDatasetNames(values, candidateNames);
            if found
                return;
            end
        elseif isstruct(values)
            [found, time_s, data] = tryStructValues(values, candidateNames);
            if found
                return;
            end
        elseif isa(values, "timeseries") && isstruct(values.Data)
            [found, time_s, data] = tryStructTimeseries(values, candidateNames);
            if found
                return;
            end
        end
    elseif isa(element, "timeseries") && isstruct(element.Data)
        [found, time_s, data] = tryStructTimeseries(element, candidateNames);
        if found
            return;
        end
    end
end

error("AOCS:Tests:MissingHarnessOutput", ...
    "Could not find any harness output named one of: %s", strjoin(candidateNames, ", "));
end

function [found, time_s, data] = tryDatasetNames(dataset, candidateNames)
% Description:
%   Attempts to read a named vector timeseries from a Dataset.

found = false;
time_s = [];
data = [];

if ~isa(dataset, "Simulink.SimulationData.Dataset")
    return;
end

for k = 1:dataset.numElements
    element = dataset{k};
    elementName = "";
    if isa(element, "timeseries")
        elementName = string(element.Name);
        values = element;
    elseif isprop(element, "Name") && isprop(element, "Values")
        elementName = string(element.Name);
        values = element.Values;
    else
        continue;
    end

    if ~any(elementName == candidateNames) || ~isa(values, "timeseries")
        continue;
    end

    time_s = values.Time(:);
    data = loggedSignalMatrix(values.Data, 3, elementName);
    found = true;
    return;
end
end

function [found, time_s, data] = tryStructValues(values, candidateNames)
% Description:
%   Attempts to read a named timeseries field from a struct-valued bus output.

found = false;
time_s = [];
data = [];
fieldNames = erase(candidateNames, "MagneticField.");
fieldNames = erase(fieldNames, "MagneticField_");

for name = fieldNames(:).'
    field = char(name);
    if ~isfield(values, field) || ~isa(values.(field), "timeseries")
        continue;
    end

    ts = values.(field);
    time_s = ts.Time(:);
    data = loggedSignalMatrix(ts.Data, 3, field);
    found = true;
    return;
end
end

function [found, time_s, data] = tryStructTimeseries(values, candidateNames)
% Description:
%   Attempts to read a named field from a struct-valued bus timeseries.

found = false;
time_s = [];
data = [];
fieldNames = erase(candidateNames, "MagneticField.");
fieldNames = erase(fieldNames, "MagneticField_");

for name = fieldNames(:).'
    field = char(name);
    if ~isfield(values.Data, field)
        continue;
    end

    raw = values.Data.(field);
    time_s = values.Time(:);
    data = loggedSignalMatrix(raw, 3, field);
    found = true;
    return;
end
end

function data = readSwarmMagneticFixture(dataFile)
% Description:
%   Reads the Swarm A magnetic-validation NetCDF fixture and normalizes
%   variable shapes for MATLAB tests.
%
% Arguments:
%   dataFile - Path to the Swarm NetCDF fixture.
%
% Outputs:
%   data - Struct with UTC times, position, magnetic-field, attitude, and
%          geomagnetic-index samples.

data = struct();
data.Timestamp_s = readNetcdfVector(dataFile, "Timestamp");
data.TimeUtc = readNetcdfUtcTime(dataFile, "Timestamp");
data.SampleCount = numel(data.Timestamp_s);

data.Spacecraft = readStringVariable(dataFile, "Spacecraft");
data.NecLabels = readStringVariable(dataFile, "NEC");
data.QuaternionLabels = readStringVariable(dataFile, "quaternion");

data.Latitude_deg = readNetcdfVector(dataFile, "Latitude");
data.Longitude_deg = readNetcdfVector(dataFile, "Longitude");
data.Radius_m = readNetcdfVector(dataFile, "Radius");
data.B_NEC_nT = readNetcdfMatrix(dataFile, "B_NEC", 3);
data.B_NEC_IGRF_nT = readNetcdfMatrix(dataFile, "B_NEC_IGRF", 3);
data.F_nT = readNetcdfVector(dataFile, "F");
data.F_IGRF_nT = readNetcdfVector(dataFile, "F_IGRF");
data.q_NEC_CRF = readNetcdfMatrix(dataFile, "q_NEC_CRF", 4);
data.Flags_B = readNetcdfVector(dataFile, "Flags_B");
data.Flags_F = readNetcdfVector(dataFile, "Flags_F");
data.Flags_q = readNetcdfVector(dataFile, "Flags_q");
data.Kp = readNetcdfVector(dataFile, "Kp");
data.Dst_nT = readNetcdfVector(dataFile, "Dst");
data.SunZenithAngle_deg = readNetcdfVector(dataFile, "SunZenithAngle");

assertSampleCount(data, "Spacecraft", data.Spacecraft);
assertSampleCount(data, "Latitude", data.Latitude_deg);
assertSampleCount(data, "Longitude", data.Longitude_deg);
assertSampleCount(data, "Radius", data.Radius_m);
assertSampleCount(data, "B_NEC", data.B_NEC_nT);
assertSampleCount(data, "B_NEC_IGRF", data.B_NEC_IGRF_nT);
assertSampleCount(data, "F", data.F_nT);
assertSampleCount(data, "F_IGRF", data.F_IGRF_nT);
assertSampleCount(data, "q_NEC_CRF", data.q_NEC_CRF);
assertSampleCount(data, "Flags_B", data.Flags_B);
assertSampleCount(data, "Flags_F", data.Flags_F);
assertSampleCount(data, "Flags_q", data.Flags_q);
assertSampleCount(data, "Kp", data.Kp);
assertSampleCount(data, "Dst", data.Dst_nT);
assertSampleCount(data, "SunZenithAngle", data.SunZenithAngle_deg);
end

function values = readNetcdfVector(dataFile, variableName)
% Description:
%   Reads a NetCDF variable as an N-by-1 double vector.

values = double(ncread(dataFile, variableName));
values = values(:);
end

function matrix = readNetcdfMatrix(dataFile, variableName, expectedColumns)
% Description:
%   Reads a NetCDF variable as an N-by-M double matrix.

matrix = squeeze(double(ncread(dataFile, variableName)));

if ndims(matrix) ~= 2
    error("AOCS:Tests:UnexpectedNetcdfShape", ...
        "Variable '%s' has shape %s; expected a matrix.", ...
        char(variableName), mat2str(size(matrix)));
end

if size(matrix, 2) == expectedColumns
    return;
end

if size(matrix, 1) == expectedColumns
    matrix = matrix.';
    return;
end

error("AOCS:Tests:UnexpectedNetcdfShape", ...
    "Variable '%s' has shape %s; expected %d columns.", ...
    char(variableName), mat2str(size(matrix)), expectedColumns);
end

function values = readStringVariable(dataFile, variableName)
% Description:
%   Reads a NetCDF string or character variable as a string column vector.

raw = ncread(dataFile, variableName);

if isstring(raw)
    values = raw(:);
elseif iscell(raw)
    values = string(raw(:));
elseif ischar(raw)
    values = string(cellstr(raw));
else
    values = string(raw(:));
end

values = strip(values(:));
end

function units = readVariableUnits(dataFile, variableName)
% Description:
%   Reads the NetCDF units attribute for a variable.

units = string(ncreadatt(dataFile, variableName, "units"));
end

function timeUtc = readNetcdfUtcTime(dataFile, variableName)
% Description:
%   Reads a NetCDF time variable stored as seconds since a UTC epoch.

values_s = readNetcdfVector(dataFile, variableName);
units = string(ncreadatt(dataFile, variableName, "units"));
tokens = regexp(units, "^seconds since (\d{4}-\d{2}-\d{2}) (\d{2}:\d{2}:\d{2})$", ...
    "tokens", "once");

if isempty(tokens)
    error("AOCS:Tests:UnsupportedNetcdfTimeUnits", ...
        "Unsupported NetCDF time units for '%s': %s.", char(variableName), units);
end

epochUtc = datetime(string(tokens{1}) + " " + string(tokens{2}), ...
    "InputFormat", "yyyy-MM-dd HH:mm:ss", "TimeZone", "UTC");
timeUtc = epochUtc + seconds(values_s);
end

function assertSampleCount(data, variableName, values)
% Description:
%   Verifies that a sample-indexed variable has the fixture sample count.

if size(values, 1) ~= data.SampleCount
    error("AOCS:Tests:UnexpectedSampleCount", ...
        "Variable '%s' has %d samples; expected %d.", ...
        char(variableName), size(values, 1), data.SampleCount);
end
end

function lla = swarmGeocentricToLla(latitude_deg, longitude_deg, radius_m)
% Description:
%   Converts Swarm geocentric ITRF latitude/longitude/radius to geodetic LLA.

x_m = radius_m .* cosd(latitude_deg) .* cosd(longitude_deg);
y_m = radius_m .* cosd(latitude_deg) .* sind(longitude_deg);
z_m = radius_m .* sind(latitude_deg);
lla = ecef2lla([x_m, y_m, z_m]);
end

function [B_NEC_nT, F_nT] = computeMatlabIgrfNec(data)
% Description:
%   Computes MATLAB IGRF-14 in geodetic NED coordinates and transforms it
%   into the Swarm geocentric NEC frame.

lla = swarmGeocentricToLla(data.Latitude_deg, data.Longitude_deg, data.Radius_m);
utc = datevec(data.TimeUtc);
decimalYear = decyear(utc(:, 1), utc(:, 2), utc(:, 3), utc(:, 4), utc(:, 5), utc(:, 6));

[B_NED_nT, ~, ~, ~, ~] = igrfmagm(lla(:, 3), lla(:, 1), lla(:, 2), decimalYear, 14);

B_NEC_nT = zeros(data.SampleCount, 3);
for k = 1:data.SampleCount
    C_NED_ECEF = dcmecef2ned(lla(k, 1), lla(k, 2));
    B_ECEF_nT = C_NED_ECEF.' * B_NED_nT(k, :).';
    C_NEC_ECEF = geocentricNecDcm(data.Latitude_deg(k), data.Longitude_deg(k));
    B_NEC_nT(k, :) = (C_NEC_ECEF * B_ECEF_nT).';
end

F_nT = vecnorm(B_NEC_nT, 2, 2);
end

function C_NEC_ECEF = geocentricNecDcm(latitude_deg, longitude_deg)
% Description:
%   Builds the DCM mapping ECEF vector components into Swarm geocentric NEC.

lat_rad = deg2rad(latitude_deg);
lon_rad = deg2rad(longitude_deg);

north_ECEF = [-sin(lat_rad) * cos(lon_rad), -sin(lat_rad) * sin(lon_rad), cos(lat_rad)];
east_ECEF = [-sin(lon_rad), cos(lon_rad), 0];
center_ECEF = -[cos(lat_rad) * cos(lon_rad), cos(lat_rad) * sin(lon_rad), sin(lat_rad)];

C_NEC_ECEF = [north_ECEF; east_ECEF; center_ECEF];
end

function text = formatUtcForReport(timeUtc)
% Description:
%   Formats a UTC datetime for deterministic test-console reporting.

value = datevec(timeUtc);
text = string(sprintf('%04.0f-%02.0f-%02.0f %02.0f:%02.0f:%02.0f UTC', ...
    value(1), value(2), value(3), value(4), value(5), value(6)));
end

function printRange(label, values, units)
% Description:
%   Prints minimum and maximum for a numeric vector.

values = finiteColumn(values);
fprintf('  %-38s min %12.3f  max %12.3f %s\n', ...
    char(label), min(values), max(values), char(units));
end

function printDistribution(label, values, units)
% Description:
%   Prints median, 95th percentile, and maximum for a numeric vector.

values = finiteColumn(values);
fprintf('  %-38s median %12.3f  p95 %12.3f  max %12.3f %s\n', ...
    char(label), median(values), nearestPercentile(values, 95), max(values), char(units));
end

function values = finiteColumn(values)
% Description:
%   Returns finite numeric values as a column vector.

values = values(:);
values = values(isfinite(values));

if isempty(values)
    error('AOCS:Tests:EmptyStatisticsInput', 'Cannot print statistics for an empty array.');
end
end

function text = formatValueCounts(values)
% Description:
%   Formats unique numeric values and their counts as a compact string.

values = finiteColumn(double(values));
uniqueValues = unique(values).';
parts = strings(1, numel(uniqueValues));

for k = 1:numel(uniqueValues)
    parts(k) = sprintf('%g:%d', uniqueValues(k), nnz(values == uniqueValues(k)));
end

text = strjoin(parts, ', ');
end

function value = nearestPercentile(samples, percentile)
% Description:
%   Computes a nearest-rank percentile without requiring Statistics Toolbox.

samples = sort(samples(:));
samples = samples(isfinite(samples));

if isempty(samples)
    error("AOCS:Tests:EmptyPercentileInput", "Cannot compute percentile of an empty array.");
end

index = ceil(percentile / 100 * numel(samples));
index = min(max(index, 1), numel(samples));
value = samples(index);
end
