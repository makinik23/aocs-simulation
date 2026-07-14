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
            addpath(fullfile(testCase.ProjectRoot, "src", "environment"));
            addpath(fullfile(testCase.ProjectRoot, "src", "simulink"));
        end
    end

    methods (Test)
        function lowPrecisionSunProductsStayPhysical(testCase)
            % Description:
            %   Checks standalone low-precision Sun products over one year for
            %   finite vectors, unit directions, and realistic Sun distance/flux.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            configFile = fullfile(testCase.ProjectRoot, "config", "AocsSimulationConfig.json");
            AOCS = loadAocsSimulationConfig(configFile, testCase.ProjectRoot);

            AU_m = 149597870700.0;
            t_s = linspace(0, 365.25 * 86400, 16).';
            r_I_m = [AOCS.Orbit.InitialKeplerian.semi_major_axis_m; 0; 0];
            DCM_be = eye(3);

            sunDistance_m = zeros(numel(t_s), 1);
            solarFlux_W_m2 = zeros(numel(t_s), 1);
            sunIUnitNorm = zeros(numel(t_s), 1);
            sunBUnitNorm = zeros(numel(t_s), 1);
            bodyTransformError = zeros(numel(t_s), 1);

            for k = 1:numel(t_s)
                [sun_B_unit, sun_I_unit, r_sun_I_m, sun_distance_m, solar_flux_W_m2] = ...
                    computeSunProducts(AOCS.Epoch.Utc, t_s(k), ...
                    AOCS.Environment.Sun.SolarConstant_W_m2, r_I_m, DCM_be);

                testCase.verifyTrue(all(isfinite(r_sun_I_m)), "r_sun_I_m must remain finite.");
                testCase.verifyTrue(all(isfinite(sun_I_unit)), "sun_I_unit must remain finite.");
                testCase.verifyTrue(all(isfinite(sun_B_unit)), "sun_B_unit must remain finite.");

                sunDistance_m(k) = sun_distance_m;
                solarFlux_W_m2(k) = solar_flux_W_m2;
                sunIUnitNorm(k) = norm(sun_I_unit);
                sunBUnitNorm(k) = norm(sun_B_unit);
                bodyTransformError(k) = norm(sun_B_unit - DCM_be * sun_I_unit);
            end

            testCase.verifyGreaterThan(min(sunDistance_m), 0.97 * AU_m, ...
                sprintf("Minimum spacecraft-to-Sun distance is %.3e m.", min(sunDistance_m)));
            testCase.verifyLessThan(max(sunDistance_m), 1.03 * AU_m, ...
                sprintf("Maximum spacecraft-to-Sun distance is %.3e m.", max(sunDistance_m)));
            testCase.verifyGreaterThan(min(solarFlux_W_m2), 1280, ...
                sprintf("Minimum solar flux is %.3f W/m^2.", min(solarFlux_W_m2)));
            testCase.verifyLessThan(max(solarFlux_W_m2), 1450, ...
                sprintf("Maximum solar flux is %.3f W/m^2.", max(solarFlux_W_m2)));
            testCase.verifyLessThanOrEqual(max(abs(sunIUnitNorm - 1)), 1e-12, ...
                "sun_I_unit must remain unit length.");
            testCase.verifyLessThanOrEqual(max(abs(sunBUnitNorm - 1)), 1e-12, ...
                "sun_B_unit must remain unit length.");
            testCase.verifyLessThanOrEqual(max(bodyTransformError), 1e-12, ...
                "Body-frame Sun vector must match DCM_be * sun_I_unit.");
        end

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
            sun_B_unit = loggedVectorSignal(logsout, "sun_B_unit");
            sun_I_unit = loggedVectorSignal(logsout, "sun_I_unit");
            r_sun_I_m = loggedVectorSignal(logsout, "r_sun_I_m");
            sun_distance_m = loggedScalarSignal(logsout, "sun_distance_m");
            solar_flux_W_m2 = loggedScalarSignal(logsout, "solar_flux_W_m2");
            solar_flux_shadowed_W_m2 = loggedScalarSignal(logsout, "solar_flux_shadowed_W_m2");
            sun_visibility = loggedScalarSignal(logsout, "sun_visibility");

            testCase.verifyTrue(all(isfinite(B_NED_T), "all"), "B_NED_T must remain finite.");
            testCase.verifyTrue(all(isfinite(B_I_T), "all"), "B_I_T must remain finite.");
            testCase.verifyTrue(all(isfinite(B_B_T), "all"), "B_B_T must remain finite.");
            testCase.verifyTrue(all(isfinite(r_I_m), "all"), "r_I_m must remain finite.");
            testCase.verifyTrue(all(isfinite(sun_B_unit), "all"), "sun_B_unit must remain finite.");
            testCase.verifyTrue(all(isfinite(sun_I_unit), "all"), "sun_I_unit must remain finite.");
            testCase.verifyTrue(all(isfinite(r_sun_I_m), "all"), "r_sun_I_m must remain finite.");
            testCase.verifyTrue(all(isfinite(sun_distance_m), "all"), "sun_distance_m must remain finite.");
            testCase.verifyTrue(all(isfinite(solar_flux_W_m2), "all"), "solar_flux_W_m2 must remain finite.");
            testCase.verifyTrue(all(isfinite(solar_flux_shadowed_W_m2), "all"), ...
                "solar_flux_shadowed_W_m2 must remain finite.");
            testCase.verifyTrue(all(isfinite(sun_visibility), "all"), "sun_visibility must remain finite.");

            B_NED_norm_T = vecnorm(B_NED_T, 2, 2);
            B_I_norm_T = vecnorm(B_I_T, 2, 2);
            B_B_norm_T = vecnorm(B_B_T, 2, 2);
            AU_m = 149597870700.0;
            sunIUnitNorm = vecnorm(sun_I_unit, 2, 2);
            sunBUnitNorm = vecnorm(sun_B_unit, 2, 2);

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

            expectedSun_B_unit = transformLoggedDcm(DCM_be, sun_I_unit, "DCM_be", "sun_I_unit");
            bodySunError = vecnorm(sun_B_unit - expectedSun_B_unit, 2, 2);
            expectedSun = computeExpectedSunProducts(AOCS, simOut.tout(:), r_I_m, DCM_be);
            sunIError = vecnorm(sun_I_unit - expectedSun.sun_I_unit, 2, 2);
            sunBError = vecnorm(sun_B_unit - expectedSun.sun_B_unit, 2, 2);
            sunPositionError_m = vecnorm(r_sun_I_m - expectedSun.r_sun_I_m, 2, 2);
            sunDistanceError_m = abs(sun_distance_m - expectedSun.sun_distance_m);
            solarFluxError_W_m2 = abs(solar_flux_W_m2 - expectedSun.solar_flux_W_m2);
            shadowedFluxError_W_m2 = abs(solar_flux_shadowed_W_m2 - solar_flux_W_m2 .* sun_visibility);

            testCase.verifyLessThanOrEqual(max(abs(sunIUnitNorm - 1)), 1e-12, ...
                "sun_I_unit must remain unit length.");
            testCase.verifyLessThanOrEqual(max(abs(sunBUnitNorm - 1)), 1e-12, ...
                "sun_B_unit must remain unit length.");
            testCase.verifyGreaterThan(min(sun_distance_m), 0.97 * AU_m, ...
                sprintf("Minimum spacecraft-to-Sun distance is %.3e m.", min(sun_distance_m)));
            testCase.verifyLessThan(max(sun_distance_m), 1.03 * AU_m, ...
                sprintf("Maximum spacecraft-to-Sun distance is %.3e m.", max(sun_distance_m)));
            testCase.verifyGreaterThan(min(solar_flux_W_m2), 1280, ...
                sprintf("Minimum solar flux is %.3f W/m^2.", min(solar_flux_W_m2)));
            testCase.verifyLessThan(max(solar_flux_W_m2), 1450, ...
                sprintf("Maximum solar flux is %.3f W/m^2.", max(solar_flux_W_m2)));
            testCase.verifyGreaterThanOrEqual(min(sun_visibility), -1e-12, ...
                sprintf("Minimum sun visibility is %.3e.", min(sun_visibility)));
            testCase.verifyLessThanOrEqual(max(sun_visibility), 1 + 1e-12, ...
                sprintf("Maximum sun visibility is %.3e.", max(sun_visibility)));
            testCase.verifyGreaterThanOrEqual(min(solar_flux_shadowed_W_m2), -1e-9, ...
                sprintf("Minimum shadowed solar flux is %.3e W/m^2.", min(solar_flux_shadowed_W_m2)));
            testCase.verifyLessThanOrEqual(max(solar_flux_shadowed_W_m2 - solar_flux_W_m2), 1e-9, ...
                "Shadowed solar flux must not exceed raw solar flux.");
            testCase.verifyLessThanOrEqual(max(bodySunError), 1e-12, ...
                sprintf("Maximum sun_B_unit transform error is %.3e.", max(bodySunError)));
            testCase.verifyLessThanOrEqual(max(sunIError), 1e-12, ...
                sprintf("Maximum sun_I_unit model error is %.3e.", max(sunIError)));
            testCase.verifyLessThanOrEqual(max(sunBError), 1e-12, ...
                sprintf("Maximum sun_B_unit model error is %.3e.", max(sunBError)));
            testCase.verifyLessThanOrEqual(max(sunPositionError_m), 1e-3, ...
                sprintf("Maximum r_sun_I_m model error is %.3e m.", max(sunPositionError_m)));
            testCase.verifyLessThanOrEqual(max(sunDistanceError_m), 1e-3, ...
                sprintf("Maximum sun_distance_m model error is %.3e m.", max(sunDistanceError_m)));
            testCase.verifyLessThanOrEqual(max(solarFluxError_W_m2), 1e-9, ...
                sprintf("Maximum solar_flux_W_m2 model error is %.3e W/m^2.", max(solarFluxError_W_m2)));
            testCase.verifyLessThanOrEqual(max(shadowedFluxError_W_m2), 1e-9, ...
                sprintf("Maximum solar_flux_shadowed_W_m2 product error is %.3e W/m^2.", ...
                max(shadowedFluxError_W_m2)));

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

        function eclipseShadowModelMaskFollowsConfig(testCase)
            % Description:
            %   Verifies that the Aerospace Blockset Eclipse Shadow Model
            %   mask is driven by environment.eclipse from the project config.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            configFile = fullfile(testCase.ProjectRoot, "config", "AocsSimulationConfig.json");
            AOCS = setupAocsSimulation(configFile);

            load_system(AOCS.Model.File);
            cleanup = onCleanup(@() close_system(AOCS.Model.Name, 0));
            applyAocsSimulationSettings(AOCS.Model.Name, AOCS);

            blockPath = AOCS.Model.Name + "/Orbit & Environment/Eclipse Model/Eclipse Shadow Model (Dual Cone)";
            testCase.assertEqual(string(get_param(blockPath, "BlockType")), "EclipseShadowModel", ...
                "Expected Eclipse Shadow Model block is missing from Orbit & Environment/Eclipse Model.");

            eclipse = AOCS.Environment.Eclipse;
            testCase.verifyEqual(string(get_param(blockPath, "units")), "Metric (m)");
            testCase.verifyEqual(string(get_param(blockPath, "shadowModel")), "Dual cone");
            testCase.verifyEqual(string(get_param(blockPath, "outputShadowRegion")), onOff(eclipse.OutputShadowRegion));
            testCase.verifyEqual(string(get_param(blockPath, "timeSource")), "Dialog");
            testCase.verifyEqual(string(get_param(blockPath, "startDate")), julianDateExpression(AOCS.Epoch.Utc));
            testCase.verifyEqual(string(get_param(blockPath, "centralBody")), eclipse.CentralBody);
            testCase.verifyEqual(string(get_param(blockPath, "includeEarth")), onOff(eclipse.IncludeEarth));
            testCase.verifyEqual(string(get_param(blockPath, "includeMoon")), onOff(eclipse.IncludeMoon));
            testCase.verifyEqual(string(get_param(blockPath, "customRadius")), ...
                sprintf("%.15g", AOCS.Orbit.CentralBodyConstants.radius_m));
            testCase.verifyEqual(string(get_param(blockPath, "ephemerisModel")), eclipse.EphemerisModel);
            testCase.verifyEqual(string(get_param(blockPath, "useEphemerisDateRange")), ...
                onOff(eclipse.UseEphemerisDateRange));
            testCase.verifyEqual(string(get_param(blockPath, "ephemerisStartDate")), ...
                julianDateExpression(eclipse.EphemerisStartUtc));
            testCase.verifyEqual(string(get_param(blockPath, "ephemerisEndDate")), ...
                julianDateExpression(eclipse.EphemerisEndUtc));
            testCase.verifyEqual(string(get_param(blockPath, "action")), eclipse.Action);
            testCase.verifyEqual(string(get_param(blockPath, "zeroCrossing")), onOff(eclipse.ZeroCrossing));
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

function data = loggedScalarSignal(logsout, signalName)
% Description:
%   Reads a logged scalar signal as an N-by-1 vector.
%
% Arguments:
%   logsout - Simulink logsout dataset.
%   signalName - Logged signal name.
%
% Outputs:
%   data - N-by-1 signal samples.

element = logsout.getElement(char(signalName));
data = squeeze(element.Values.Data);
data = data(:);

if ~isvector(data)
    error("AOCS:Tests:UnexpectedSignalShape", ...
        "Logged signal '%s' has shape %s; expected scalar samples.", ...
        char(signalName), mat2str(size(element.Values.Data)));
end
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

function value = onOff(flag)
% Description:
%   Converts logical values to Simulink mask on/off strings for assertions.
%
% Arguments:
%   flag - Scalar logical.
%
% Outputs:
%   value - String scalar "on" or "off".

if flag
    value = "on";
else
    value = "off";
end
end

function value = julianDateExpression(epochUtc)
% Description:
%   Formats a UTC vector as the Julian-date mask expression used by the
%   eclipse configuration script.
%
% Arguments:
%   epochUtc - 6-by-1 UTC vector [year month day hour minute second]'.
%
% Outputs:
%   value - String scalar expression.

value = string(sprintf("juliandate(%.0f, %.0f, %.0f, %.0f, %.0f, %.15g)", ...
    epochUtc(1), epochUtc(2), epochUtc(3), epochUtc(4), epochUtc(5), epochUtc(6)));
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

function expected = computeExpectedSunProducts(AOCS, t_s, r_I_m, DCM_be)
% Description:
%   Recomputes Sun products from logged orbit and attitude samples using the
%   project MATLAB implementation.
%
% Arguments:
%   AOCS - Validated simulation configuration struct.
%   t_s - N-by-1 simulation time samples [s].
%   r_I_m - N-by-3 inertial spacecraft position samples [m].
%   DCM_be - 3-by-3-by-N DCM samples mapping inertial vectors into body axes.
%
% Outputs:
%   expected - Struct containing expected Sun product sample histories.

sampleCount = size(r_I_m, 1);
if numel(t_s) ~= sampleCount
    error("AOCS:Tests:SampleCountMismatch", ...
        "Simulation time has %d samples but r_I_m has %d samples.", ...
        numel(t_s), sampleCount);
end

if size(DCM_be, 3) ~= sampleCount
    error("AOCS:Tests:SampleCountMismatch", ...
        "Signal 'DCM_be' has %d samples but r_I_m has %d samples.", ...
        size(DCM_be, 3), sampleCount);
end

expected.sun_B_unit = zeros(sampleCount, 3);
expected.sun_I_unit = zeros(sampleCount, 3);
expected.r_sun_I_m = zeros(sampleCount, 3);
expected.sun_distance_m = zeros(sampleCount, 1);
expected.solar_flux_W_m2 = zeros(sampleCount, 1);

for k = 1:sampleCount
    [sun_B_unit, sun_I_unit, r_sun_I_m, sun_distance_m, solar_flux_W_m2] = ...
        computeSunProducts(AOCS.Epoch.Utc, t_s(k), ...
        AOCS.Environment.Sun.SolarConstant_W_m2, r_I_m(k, :).', DCM_be(:, :, k));

    expected.sun_B_unit(k, :) = sun_B_unit.';
    expected.sun_I_unit(k, :) = sun_I_unit.';
    expected.r_sun_I_m(k, :) = r_sun_I_m.';
    expected.sun_distance_m(k) = sun_distance_m;
    expected.solar_flux_W_m2(k) = solar_flux_W_m2;
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
