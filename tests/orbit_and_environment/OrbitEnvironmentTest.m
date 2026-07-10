classdef OrbitEnvironmentTest < matlab.unittest.TestCase

    properties
        ProjectRoot
        DataFile
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            % Description:
            %   Locates the project root and magnetic-field injection data.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.ProjectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.DataFile = fullfile(testCase.ProjectRoot, ...
                "tests", "orbit_and_environment", "data", "magnetic_injections.json");

            addpath(testCase.ProjectRoot);
            addpath(fullfile(testCase.ProjectRoot, "src", "analysis"));
            addpath(fullfile(testCase.ProjectRoot, "src", "config"));
            addpath(fullfile(testCase.ProjectRoot, "src", "simulink"));
        end
    end

    methods (Test)
        function igrfEciFieldMatchesInjectedReference(testCase)
            % Description:
            %   Verifies the magnetic-field frame chain against injected
            %   r_ECI/B_ECI reference samples. The implementation mirrors the
            %   Simulink subsystem: ECI->LLA, IGRF NED field, NED->ECEF, and
            %   ECEF->ECI.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            samples = loadMagneticInjections(testCase.DataFile);
            actualB_ECI_T = computeBeciFromInjections(samples);

            error_B_T = actualB_ECI_T - samples.ExpectedB_ECI_T;
            vectorError_T = vecnorm(error_B_T, 2, 2);
            referenceNorm_T = vecnorm(samples.ExpectedB_ECI_T, 2, 2);

            maxComponentError_T = max(abs(error_B_T), [], "all");
            maxVectorError_T = max(vectorError_T);
            maxRelativeVectorError = max(vectorError_T ./ referenceNorm_T);

            testCase.verifyLessThanOrEqual(maxComponentError_T, 3.0e-7, ...
                sprintf("Maximum B_ECI component error is %.3e T.", maxComponentError_T));
            testCase.verifyLessThanOrEqual(maxVectorError_T, 3.0e-7, ...
                sprintf("Maximum B_ECI vector error is %.3e T.", maxVectorError_T));
            testCase.verifyLessThanOrEqual(maxRelativeVectorError, 6.0e-3, ...
                sprintf("Maximum relative B_ECI vector error is %.3e.", maxRelativeVectorError));
        end

        function simulinkEnvironmentProductsStayPhysical(testCase)
            % Description:
            %   Runs the current Simulink plant and checks that the magnetic
            %   field and disturbance-torque products have physically plausible
            %   magnitudes and internally consistent frame transforms.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            configFile = fullfile(testCase.ProjectRoot, "config", "AocsSimulationConfig.json");
            AOCS = loadAocsSimulationConfig(configFile, testCase.ProjectRoot);
            simOut = run_aocs_simulation(configFile);
            logsout = simOut.logsout;

            B_NED_T = loggedVectorSignal(logsout, "B_NED_T");
            B_I_T = loggedVectorSignal(logsout, "B_I_T");
            B_B_T = loggedVectorSignal(logsout, "B_B_T");
            DCM_be = loggedMatrixSignal(logsout, "DCM_be");
            C_ECEF_NED = loggedMatrixSignal(logsout, "C_ECEF_NED");
            C_ECI_ECEF = loggedMatrixSignal(logsout, "C_ECI_ECEF");
            r_I_m = loggedVectorSignal(logsout, "r_I_m");

            testCase.verifyTrue(all(isfinite(B_NED_T), "all"), "B_NED_T must remain finite.");
            testCase.verifyTrue(all(isfinite(B_I_T), "all"), "B_I_T must remain finite.");
            testCase.verifyTrue(all(isfinite(B_B_T), "all"), "B_B_T must remain finite.");
            testCase.verifyTrue(all(isfinite(r_I_m), "all"), "r_I_m must remain finite.");

            B_NED_norm_T = vecnorm(B_NED_T, 2, 2);
            B_I_norm_T = vecnorm(B_I_T, 2, 2);
            B_B_norm_T = vecnorm(B_B_T, 2, 2);

            testCase.verifyGreaterThan(min(B_NED_norm_T), 15e-6, ...
                sprintf("Minimum geomagnetic field norm is %.3e T.", min(B_NED_norm_T)));
            testCase.verifyLessThan(max(B_NED_norm_T), 80e-6, ...
                sprintf("Maximum geomagnetic field norm is %.3e T.", max(B_NED_norm_T)));

            testCase.verifyLessThanOrEqual(max(abs(B_I_norm_T - B_NED_norm_T)), 1e-12, ...
                "NED->ECI rotation should preserve magnetic-field norm.");
            testCase.verifyLessThanOrEqual(max(abs(B_B_norm_T - B_I_norm_T)), 1e-12, ...
                "ECI->body rotation should preserve magnetic-field norm.");

            expectedB_B_T = transformLoggedDcm(DCM_be, B_I_T, "DCM_be", "B_I_T");
            bodyFieldError_T = vecnorm(B_B_T - expectedB_B_T, 2, 2);
            testCase.verifyLessThanOrEqual(max(bodyFieldError_T), 1e-12, ...
                sprintf("Maximum B_B_T transform error is %.3e T.", max(bodyFieldError_T)));

            testCase.verifyLessThanOrEqual(maximumOrthonormalityError(DCM_be), 1e-10, ...
                "DCM_be should remain orthonormal.");
            testCase.verifyLessThanOrEqual(maximumOrthonormalityError(C_ECEF_NED), 1e-10, ...
                "C_ECEF_NED should remain orthonormal.");
            testCase.verifyLessThanOrEqual(maximumOrthonormalityError(C_ECI_ECEF), 1e-10, ...
                "C_ECI_ECEF should remain orthonormal.");

            expectedM_rmm_B_Nm = residualMagneticTorque(AOCS.Environment.m_res_B, B_B_T);
            expectedM_gg_B_Nm = gravityGradientTorque(AOCS.Spacecraft.I_B, ...
                AOCS.Orbit.CentralBodyConstants.mu_m3_s2, r_I_m, DCM_be);
            expectedM_dist_B_Nm = expectedM_rmm_B_Nm + expectedM_gg_B_Nm;

            expectedRmmNorm_Nm = vecnorm(expectedM_rmm_B_Nm, 2, 2);
            expectedGravityGradientNorm_Nm = vecnorm(expectedM_gg_B_Nm, 2, 2);
            expectedDisturbanceNorm_Nm = vecnorm(expectedM_dist_B_Nm, 2, 2);
            torqueUpperBound_Nm = norm(AOCS.Environment.m_res_B) * max(B_B_norm_T);
            gravityGradientUpperBound_Nm = max(3 * AOCS.Orbit.CentralBodyConstants.mu_m3_s2 ./ ...
                vecnorm(r_I_m, 2, 2).^3) * norm(AOCS.Spacecraft.I_B, 2);

            testCase.verifyLessThanOrEqual(max(expectedRmmNorm_Nm), torqueUpperBound_Nm * (1 + 1e-12), ...
                "Residual magnetic torque must satisfy |m x B| <= |m||B|.");
            testCase.verifyLessThan(max(expectedRmmNorm_Nm), 1e-6, ...
                sprintf("Maximum residual magnetic torque is %.3e N*m.", max(expectedRmmNorm_Nm)));

            testCase.verifyLessThanOrEqual(max(expectedGravityGradientNorm_Nm), ...
                gravityGradientUpperBound_Nm * (1 + 1e-12), ...
                "Gravity-gradient torque must stay below the inertia-norm analytical bound.");
            testCase.verifyLessThan(max(expectedGravityGradientNorm_Nm), 1e-7, ...
                sprintf("Maximum gravity-gradient torque is %.3e N*m.", max(expectedGravityGradientNorm_Nm)));
            testCase.verifyLessThan(max(expectedDisturbanceNorm_Nm), 2e-6, ...
                sprintf("Maximum total disturbance torque is %.3e N*m.", max(expectedDisturbanceNorm_Nm)));

            if norm(AOCS.Environment.m_res_B) > 0
                testCase.verifyGreaterThan(max(expectedRmmNorm_Nm), 1e-9, ...
                    sprintf("Maximum residual magnetic torque is %.3e N*m.", max(expectedRmmNorm_Nm)));
            end

            if max(eig(AOCS.Spacecraft.I_B)) - min(eig(AOCS.Spacecraft.I_B)) > 1e-12
                testCase.verifyGreaterThan(max(expectedGravityGradientNorm_Nm), 1e-10, ...
                    sprintf("Maximum gravity-gradient torque is %.3e N*m.", max(expectedGravityGradientNorm_Nm)));
            end

            M_rmm_B_Nm = loggedVectorSignal(logsout, "M_rmm_B_Nm");
            M_gg_B_Nm = loggedVectorSignal(logsout, "M_gg_B_Nm");
            M_dist_B_Nm = loggedVectorSignal(logsout, "M_dist_B_Nm");

            rmmTorqueError_Nm = vecnorm(M_rmm_B_Nm - expectedM_rmm_B_Nm, 2, 2);
            gravityGradientTorqueError_Nm = vecnorm(M_gg_B_Nm - expectedM_gg_B_Nm, 2, 2);
            disturbanceTorqueError_Nm = vecnorm(M_dist_B_Nm - expectedM_dist_B_Nm, 2, 2);
            disturbanceSumError_Nm = vecnorm(M_dist_B_Nm - (M_rmm_B_Nm + M_gg_B_Nm), 2, 2);

            testCase.verifyLessThanOrEqual(max(rmmTorqueError_Nm), 1e-12, ...
                sprintf("Maximum M_rmm_B_Nm error is %.3e N*m.", max(rmmTorqueError_Nm)));
            testCase.verifyLessThanOrEqual(max(gravityGradientTorqueError_Nm), 1e-12, ...
                sprintf("Maximum M_gg_B_Nm error is %.3e N*m.", max(gravityGradientTorqueError_Nm)));
            testCase.verifyLessThanOrEqual(max(disturbanceTorqueError_Nm), 1e-12, ...
                sprintf("Maximum M_dist_B_Nm error is %.3e N*m.", max(disturbanceTorqueError_Nm)));
            testCase.verifyLessThanOrEqual(max(disturbanceSumError_Nm), 1e-12, ...
                sprintf("Maximum M_dist_B_Nm sum error is %.3e N*m.", max(disturbanceSumError_Nm)));
        end

        function environmentBusAssemblyMatchesBusObjectOrder(testCase)
            % Description:
            %   Verifies that the Orbit & Environment output bus is assembled
            %   in the same element order declared by AOCS_EnvironmentBus.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            configFile = fullfile(testCase.ProjectRoot, "config", "AocsSimulationConfig.json");
            AOCS = setupAocsSimulation(configFile);
            busObject = createAocsEnvironmentBus();

            load_system(AOCS.Model.File);
            cleanup = onCleanup(@() close_system(AOCS.Model.Name, 0));

            busCreatorPath = AOCS.Model.Name + "/Orbit & Environment/Environment Bus Assembly";
            actualNames = busCreatorInputSignalNames(busCreatorPath);
            expectedNames = string({busObject.Elements.Name});

            testCase.verifyEqual(actualNames, expectedNames, ...
                "Environment Bus Assembly input order must match AOCS_EnvironmentBus.");
        end
    end
end

function samples = loadMagneticInjections(dataFile)
% Description:
%   Reads the magnetic injection JSON and normalizes units/shapes.
%
% Arguments:
%   dataFile - Path to tests/orbit_and_environment/data/magnetic_injections.json.
%
% Outputs:
%   samples - Struct with UTC, r_ECI in meters, and expected B_ECI in tesla.

raw = jsondecode(fileread(dataFile));
scenario = raw.test_scenario_0;
injections = scenario.injections;
sampleCount = numel(injections);

epochStrings = strings(sampleCount, 1);
r_ECI_m = zeros(sampleCount, 3);
expectedB_ECI_T = zeros(sampleCount, 3);

for k = 1:sampleCount
    epochStrings(k) = string(injections(k).epoch);
    r_ECI_m(k, :) = 1000 .* injections(k).r_eci_given(:).';
    expectedB_ECI_T(k, :) = injections(k).expected_beci(:).';
end

samples = struct();
samples.Utc = datevec(datetime(epochStrings, ...
    "InputFormat", "dd-MMM-yyyy HH:mm:ss", ...
    "Locale", "en_US"));
samples.R_ECI_m = r_ECI_m;
samples.ExpectedB_ECI_T = expectedB_ECI_T;
end

function B_ECI_T = computeBeciFromInjections(samples)
% Description:
%   Computes the inertial magnetic field from injected ECI positions using
%   the same frame sequence as the Simulink Orbit & Environment subsystem.
%
% Arguments:
%   samples - Struct returned by loadMagneticInjections.
%
% Outputs:
%   B_ECI_T - N-by-3 inertial magnetic-field vectors [T].

utc = samples.Utc;
lla = eci2lla(samples.R_ECI_m, utc, "IAU-2000/2006");
decimalYear = decyear(utc(:, 1), utc(:, 2), utc(:, 3), utc(:, 4), utc(:, 5), utc(:, 6));

[B_NED_nT, ~, ~, ~, ~] = igrfmagm(lla(:, 3), lla(:, 1), lla(:, 2), decimalYear, 14);
B_NED_T = 1e-9 .* B_NED_nT;

sampleCount = size(samples.R_ECI_m, 1);
B_ECI_T = zeros(sampleCount, 3);

for k = 1:sampleCount
    C_NED_ECEF = dcmecef2ned(lla(k, 1), lla(k, 2));
    C_ECEF_ECI = dcmeci2ecef("IAU-2000/2006", utc(k, :));

    B_ECEF_T = C_NED_ECEF.' * B_NED_T(k, :).';
    B_ECI_T(k, :) = (C_ECEF_ECI.' * B_ECEF_T).';
end
end

function data = loggedVectorSignal(logsout, signalName)
% Description:
%   Reads a logged 3-vector signal as an N-by-3 matrix.
%
% Arguments:
%   logsout - Simulink logsout dataset.
%   signalName - Logged signal name.
%
% Outputs:
%   data - N-by-3 signal samples.

element = logsout.getElement(char(signalName));
data = loggedSignalMatrix(element.Values.Data, 3, signalName);
end

function data = loggedMatrixSignal(logsout, signalName)
% Description:
%   Reads a logged 3-by-3 DCM signal as a 3-by-3-by-N page array.
%
% Arguments:
%   logsout - Simulink logsout dataset.
%   signalName - Logged signal name.
%
% Outputs:
%   data - 3-by-3-by-N logged matrix samples.

element = logsout.getElement(char(signalName));
data = squeeze(element.Values.Data);

if ndims(data) ~= 3 || size(data, 1) ~= 3 || size(data, 2) ~= 3
    error("AOCS:Tests:UnexpectedSignalShape", ...
        "Logged signal '%s' has shape %s; expected 3-by-3-by-N.", ...
        char(signalName), mat2str(size(data)));
end
end

function names = busCreatorInputSignalNames(busCreatorPath)
% Description:
%   Reads input signal names from a Bus Creator block in port order.
%
% Arguments:
%   busCreatorPath - Full Simulink path to the Bus Creator block.
%
% Outputs:
%   names - 1-by-N string array of input signal names.

ports = get_param(busCreatorPath, "PortHandles");
names = strings(1, numel(ports.Inport));

for k = 1:numel(ports.Inport)
    line = get_param(ports.Inport(k), "Line");
    if line == -1
        error("AOCS:Tests:UnconnectedBusInput", ...
            "Bus Creator input %d is unconnected at '%s'.", k, char(busCreatorPath));
    end

    names(k) = string(get_param(line, "Name"));
end
end

function vectorsOut = transformLoggedDcm(dcmPages, vectorsIn, dcmName, vectorName)
% Description:
%   Applies a logged 3-by-3-by-N DCM page array to an N-by-3 vector signal.
%
% Arguments:
%   dcmPages - 3-by-3-by-N DCM pages.
%   vectorsIn - N-by-3 vector samples.
%   dcmName - DCM signal name for diagnostics.
%   vectorName - Vector signal name for diagnostics.
%
% Outputs:
%   vectorsOut - N-by-3 transformed vectors.

sampleCount = size(vectorsIn, 1);
if size(dcmPages, 3) ~= sampleCount
    error("AOCS:Tests:SampleCountMismatch", ...
        "Signal '%s' has %d samples but '%s' has %d samples.", ...
        char(dcmName), size(dcmPages, 3), char(vectorName), sampleCount);
end

vectorsOut = zeros(sampleCount, 3);
for k = 1:sampleCount
    vectorsOut(k, :) = (dcmPages(:, :, k) * vectorsIn(k, :).').';
end
end

function maxError = maximumOrthonormalityError(dcmPages)
% Description:
%   Computes the maximum Frobenius-norm DCM orthonormality error.
%
% Arguments:
%   dcmPages - 3-by-3-by-N DCM pages.
%
% Outputs:
%   maxError - max(norm(C*C' - eye(3), "fro")) over logged samples.

maxError = 0;
I3 = eye(3);
for k = 1:size(dcmPages, 3)
    maxError = max(maxError, norm(dcmPages(:, :, k) * dcmPages(:, :, k).' - I3, "fro"));
end
end

function M_rmm_B_Nm = residualMagneticTorque(m_res_B_A_m2, B_B_T)
% Description:
%   Computes residual magnetic moment torque in body axes.
%
% Arguments:
%   m_res_B_A_m2 - 3-by-1 residual magnetic dipole [A*m^2].
%   B_B_T - N-by-3 magnetic-field vectors expressed in body axes [T].
%
% Outputs:
%   M_rmm_B_Nm - N-by-3 torque samples [N*m].

m_res_B = repmat(m_res_B_A_m2(:).', size(B_B_T, 1), 1);
M_rmm_B_Nm = cross(m_res_B, B_B_T, 2);
end

function M_gg_B_Nm = gravityGradientTorque(I_B_kg_m2, mu_m3_s2, r_I_m, DCM_be)
% Description:
%   Computes gravity-gradient torque from logged orbit and attitude products.
%
% Arguments:
%   I_B_kg_m2 - 3-by-3 spacecraft inertia matrix expressed in body axes.
%   mu_m3_s2 - Central-body gravitational parameter.
%   r_I_m - N-by-3 inertial position samples [m].
%   DCM_be - 3-by-3-by-N DCM samples mapping inertial vectors into body axes.
%
% Outputs:
%   M_gg_B_Nm - N-by-3 gravity-gradient torque samples [N*m].

sampleCount = size(r_I_m, 1);
M_gg_B_Nm = zeros(sampleCount, 3);

if size(DCM_be, 3) ~= sampleCount
    error("AOCS:Tests:SampleCountMismatch", ...
        "Signal 'DCM_be' has %d samples but 'r_I_m' has %d samples.", ...
        size(DCM_be, 3), sampleCount);
end

for k = 1:sampleCount
    rNorm_m = norm(r_I_m(k, :));
    if rNorm_m <= 0
        continue;
    end

    rHat_I = r_I_m(k, :).' ./ rNorm_m;
    rHat_B = DCM_be(:, :, k) * rHat_I;
    M_gg_B_Nm(k, :) = (3 * mu_m3_s2 / rNorm_m^3 * ...
        cross(rHat_B, I_B_kg_m2 * rHat_B)).';
end
end
