classdef AerodynamicsSinglePlateHarnessTest < matlab.unittest.TestCase
    properties (SetAccess = private)
        ProjectRoot
        Owner
        HarnessName = "AerodynamicsSinglePlateHarness"
        HarnessFile
        ReferenceFile
        CuboidReferenceFile
    end

    methods (TestClassSetup)
        function prepareModelAndHarness(testCase)
            % Description:
            %   Locates project fixtures, configures the plant model, and opens
            %   the Sentman single-plate harness for the test class.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.ProjectRoot = projectRoot();
            testCase.Owner = "aocs_plant/Orbit & Environment/Disturbance Torques/" + ...
                "Aerodynamic Disturbance Path/Sentman Multispecies Panels";
            testCase.HarnessFile = fullfile(testCase.ProjectRoot, "tests", ...
                "harnesses", testCase.HarnessName + ".slx");
            testCase.ReferenceFile = fullfile(testCase.ProjectRoot, "validation", ...
                "aerodynamics", "data", "sentman_single_plate_reference.csv");
            testCase.CuboidReferenceFile = fullfile(testCase.ProjectRoot, ...
                "validation", "aerodynamics", "data", ...
                "sentman_3u_cuboid_reference.csv");

            setupAocsPaths(testCase.ProjectRoot, true);

            testCase.assertTrue(isfile(testCase.HarnessFile), ...
                "The single-plate aerodynamics harness is missing.");
            testCase.assertTrue(isfile(testCase.ReferenceFile), ...
                "The frozen ADBSat reference fixture is missing.");
            testCase.assertTrue(isfile(testCase.CuboidReferenceFile), ...
                "The frozen full-mesh ADBSat 3U fixture is missing.");

            AOCS = setupAocsSimulation(fullfile(testCase.ProjectRoot, ...
                "config", "AocsSimulationConfig.json"));
            load_system(AOCS.Model.File);
            applyAocsSimulationSettings(AOCS.Model.Name, AOCS);
            sltest.harness.open(testCase.Owner, testCase.HarnessName);
        end
    end

    methods (TestClassTeardown)
        function closeModelAndHarness(testCase)
            % Description:
            %   Closes the Sentman aerodynamics harness and owner plant model.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            closeHarnessAndModel(testCase.Owner, testCase.HarnessName, "aocs_plant");
        end
    end

    methods (Test)
        function fixtureMatchesIndependentAdbSatReference(testCase)
            % Description:
            %   Compares single-plate harness force, coefficients, and derived
            %   acceleration against a frozen independent ADBSat reference.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            reference = readtable(testCase.ReferenceFile, "TextType", "string");
            simulation = runReferenceCases(testCase.HarnessName, reference);

            expectedForceN = [reference.force_x_reference_N, ...
                reference.force_y_reference_N, reference.force_z_reference_N];
            forceErrorN = simulation.forceBodyN - expectedForceN;
            relativeForceError = vecnorm(forceErrorN, 2, 2) ./ ...
                max(vecnorm(expectedForceN, 2, 2), 1.0e-30);

            coefficientScale = reference.q_dynamic_reference_N_m2 .* reference.area_m2;
            cpActual = -simulation.forceBodyN(:, 1) ./ coefficientScale;
            ctauActual = simulation.forceBodyN(:, 2) ./ coefficientScale;
            coefficientError = hypot( ...
                cpActual - reference.cp_reference, ...
                ctauActual - reference.ctau_reference) ./ ...
                max(hypot(reference.cp_reference, reference.ctau_reference), 1.0e-30);

            fprintf("Single-plate ADBSat comparison: cases=%d, " + ...
                "max force relative error=%.6g, max coefficient relative error=%.6g\n", ...
                height(reference), max(relativeForceError), max(coefficientError));

            testCase.verifyLessThanOrEqual(max(relativeForceError), 1.0e-3);
            testCase.verifyLessThanOrEqual(max(coefficientError), 1.0e-3);
            testCase.verifyEqual(simulation.qDynamicNm2, ...
                reference.q_dynamic_reference_N_m2, "RelTol", 1.0e-12);
            testCase.verifyEqual(simulation.momentBodyNm, ...
                zeros(height(reference), 3), "AbsTol", 1.0e-18);
            testCase.verifyEqual(simulation.forceInertialN, ...
                simulation.forceBodyN, "RelTol", 1.0e-12, "AbsTol", 1.0e-20);
            testCase.verifyEqual(simulation.accelerationInertialMps2, ...
                simulation.forceInertialN ./ reference.mass_kg, ...
                "RelTol", 1.0e-12, "AbsTol", 1.0e-20);
        end

        function threeUCuboidMatchesFullMeshAdbSatReference(testCase)
            % Description:
            %   Verifies that the simplified 3U cuboid harness matches the
            %   frozen full-mesh ADBSat force and moment reference.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            reference = readtable(testCase.CuboidReferenceFile, ...
                "TextType", "string");
            simulation = runCuboidReferenceCases(testCase.HarnessName, ...
                reference, zeros(3, 1), eye(3));

            expectedForceN = cuboidReferenceForce(reference);
            expectedMomentNm = cuboidReferenceMoment(reference);
            relativeForceError = vectorRelativeError( ...
                simulation.forceBodyN, expectedForceN, 1.0e-30);
            momentCoefficientError = vecnorm( ...
                simulation.momentBodyNm - expectedMomentNm, 2, 2) ./ ...
                (reference.q_dynamic_reference_N_m2 .* ...
                reference.adbsat_area_reference_m2 .* ...
                reference.adbsat_length_reference_m);

            fprintf("3U full-mesh ADBSat comparison: cases=%d, " + ...
                "max force relative error=%.6g, " + ...
                "max normalized moment error=%.6g\n", ...
                height(reference), max(relativeForceError), ...
                max(momentCoefficientError));

            testCase.verifyLessThanOrEqual(max(relativeForceError), 1.0e-3);
            testCase.verifyLessThanOrEqual(max(momentCoefficientError), 1.0e-3);
            testCase.verifyEqual(simulation.qDynamicNm2, ...
                reference.q_dynamic_reference_N_m2, "RelTol", 1.0e-12);
            testCase.verifyEqual(simulation.forceInertialN, ...
                simulation.forceBodyN, "RelTol", 1.0e-12, "AbsTol", 1.0e-20);
            testCase.verifyEqual(simulation.accelerationInertialMps2, ...
                simulation.forceInertialN ./ reference.mass_kg, ...
                "RelTol", 1.0e-12, "AbsTol", 1.0e-20);
        end

        function centerOfMassOffsetProducesReferenceMomentShift(testCase)
            % Description:
            %   Checks that a center-of-mass offset shifts aerodynamic moments
            %   by the expected force cross-arm contribution.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            reference = readtable(testCase.CuboidReferenceFile, ...
                "TextType", "string");
            reference = reference(1:19:end, :);
            centerOfMassOffsetM = [0.005; -0.007; 0.010];
            simulation = runCuboidReferenceCases(testCase.HarnessName, ...
                reference, centerOfMassOffsetM, eye(3));

            expectedForceN = cuboidReferenceForce(reference);
            expectedMomentNm = cuboidReferenceMoment(reference) - cross( ...
                repmat(centerOfMassOffsetM.', height(reference), 1), ...
                expectedForceN, 2);

            testCase.verifyEqual(simulation.forceBodyN, expectedForceN, ...
                "RelTol", 1.0e-3, "AbsTol", 1.0e-15);
            testCase.verifyEqual(simulation.momentBodyNm, expectedMomentNm, ...
                "RelTol", 1.0e-3, "AbsTol", 1.0e-15);
        end

    end
end

function simulation = runReferenceCases(harnessName, reference)
% Description:
%   Runs all frozen single-plate ADBSat reference rows through the harness.
%
% Arguments:
%   harnessName - Name of the opened single-plate harness.
%   reference - Table of frozen single-plate reference cases.
%
% Outputs:
%   simulation - Struct containing harness output rows.

caseTimeS = (0:height(reference) - 1).';
inputDataset = singlePlateInputDataset(reference, caseTimeS);
simulation = runHarness(harnessName, inputDataset, caseTimeS);
end

function simulation = runCuboidReferenceCases(harnessName, reference, ...
        centerOfMassOffsetM, geometryRotation)
% Description:
%   Runs frozen 3U cuboid reference rows with optional geometry transforms.
%
% Arguments:
%   harnessName - Name of the opened single-plate harness.
%   reference - Table of frozen 3U cuboid reference cases.
%   centerOfMassOffsetM - 3-by-1 center-of-mass offset [m].
%   geometryRotation - 3-by-3 rotation applied to cuboid panel geometry.
%
% Outputs:
%   simulation - Struct containing harness output rows.

caseTimeS = (0:height(reference) - 1).';
inputDataset = cuboidInputDataset(reference, caseTimeS, ...
    centerOfMassOffsetM, geometryRotation);
simulation = runHarness(harnessName, inputDataset, caseTimeS);
end

function simulation = runHarness(harnessName, inputDataset, caseTimeS)
% Description:
%   Simulates the harness with external inputs and extracts aligned outputs.
%
% Arguments:
%   harnessName - Name of the opened single-plate harness.
%   inputDataset - Simulink external-input Dataset.
%   caseTimeS - Sample times used by reference cases [s].
%
% Outputs:
%   simulation - Struct containing force, moment, acceleration, and q-dynamic rows.

simIn = Simulink.SimulationInput(harnessName);
simIn = simIn.setExternalInput(inputDataset);
simIn = simIn.setModelParameter( ...
    "StartTime", "0", ...
    "StopTime", num2str(caseTimeS(end)), ...
    "SolverType", "Fixed-step", ...
    "Solver", "FixedStepDiscrete", ...
    "FixedStep", "1", ...
    "SaveOutput", "on", ...
    "OutputSaveName", "yout", ...
    "SaveFormat", "Dataset");

simOut = sim(simIn);
simulation.forceBodyN = harnessOutputRows(simOut, "F_aero_B_N", 3, caseTimeS);
simulation.momentBodyNm = harnessOutputRows(simOut, "M_aero_B_Nm", 3, caseTimeS);
simulation.forceInertialN = harnessOutputRows(simOut, "F_aero_I_N", 3, caseTimeS);
simulation.accelerationInertialMps2 = ...
    harnessOutputRows(simOut, "a_aero_I_m_s2", 3, caseTimeS);
simulation.qDynamicNm2 = harnessOutputRows(simOut, "q_dyn_N_m2", 1, caseTimeS);
end

function inputDataset = singlePlateInputDataset(reference, timeS)
% Description:
%   Builds external-input timeseries for single-plate ADBSat reference cases.
%
% Arguments:
%   reference - Table of frozen single-plate reference cases.
%   timeS - Sample times for each reference case [s].
%
% Outputs:
%   inputDataset - Simulink external-input Dataset for the harness.

n = height(reference);
speedMps = reference.speed_m_s;
cosine = cosd(reference.angle_deg);
cosine(abs(cosine) < 1.0e-14) = 0.0;
vFlowBodyMps = [-speedMps .* cosine, ...
    speedMps .* sind(reference.angle_deg), zeros(n, 1)];

atomicMassUnitKg = 1.66053906660e-27;
speciesMassKg = atomicMassUnitKg .* [16, 28, 32, 4, 1, 14];
massFractions = [reference.mass_fraction_O, reference.mass_fraction_N2, ...
    reference.mass_fraction_O2, reference.mass_fraction_He, ...
    reference.mass_fraction_H, reference.mass_fraction_N];
numberDensitiesM3 = reference.rho_kg_m3 .* massFractions ./ speciesMassKg;

identityDcm = repmat(eye(3), 1, 1, n);
panelNormals = zeros(3, 6, n);
panelNormals(1, 1, :) = 1.0;
panelAreasM2 = zeros(n, 6);
panelAreasM2(:, 1) = reference.area_m2;
panelCentersM = zeros(3, 6, n);
wallTemperatureK = repmat(reference.T_wall_K, 1, 6);
energyAccommodation = repmat(reference.alpha_E, 1, 6);

inputDataset = Simulink.SimulationData.Dataset();
inputDataset = addHarnessInput(inputDataset, "v_flow_B_m_s", vFlowBodyMps, timeS);
inputDataset = addHarnessInput(inputDataset, "rho_kg_m3", reference.rho_kg_m3, timeS);
inputDataset = addHarnessInput(inputDataset, "T_local_K", reference.T_atm_K, timeS);
inputDataset = addHarnessInput(inputDataset, "number_densities_m3", numberDensitiesM3, timeS);
inputDataset = addHarnessInput(inputDataset, "C_BI", identityDcm, timeS);
inputDataset = addHarnessInput(inputDataset, "mass_kg", reference.mass_kg, timeS);
inputDataset = addHarnessInput(inputDataset, "aero_enabled", ones(n, 1), timeS);
inputDataset = addHarnessInput(inputDataset, "panel_normals_B", panelNormals, timeS);
inputDataset = addHarnessInput(inputDataset, "panel_areas_m2", panelAreasM2, timeS);
inputDataset = addHarnessInput(inputDataset, "panel_centers_B_m", panelCentersM, timeS);
inputDataset = addHarnessInput(inputDataset, "wall_temperature_K", wallTemperatureK, timeS);
inputDataset = addHarnessInput(inputDataset, "energy_accommodation", energyAccommodation, timeS);
end

function inputDataset = cuboidInputDataset(reference, timeS, ...
        centerOfMassOffsetM, geometryRotation)
% Description:
%   Builds external-input timeseries for 3U cuboid ADBSat reference cases.
%
% Arguments:
%   reference - Table of frozen 3U cuboid reference cases.
%   timeS - Sample times for each reference case [s].
%   centerOfMassOffsetM - 3-by-1 center-of-mass offset [m].
%   geometryRotation - 3-by-3 rotation applied to cuboid panel geometry.
%
% Outputs:
%   inputDataset - Simulink external-input Dataset for the harness.

n = height(reference);
vFlowBodyMps = [reference.v_flow_x_m_s, reference.v_flow_y_m_s, ...
    reference.v_flow_z_m_s] * geometryRotation.';

atomicMassUnitKg = 1.66053906660e-27;
oxygenMassKg = 16.0 * atomicMassUnitKg;
numberDensitiesM3 = zeros(n, 6);
numberDensitiesM3(:, 1) = reference.rho_kg_m3 ./ oxygenMassKg;

identityDcm = repmat(eye(3), 1, 1, n);
baseNormals = [1.0, -1.0, 0.0, 0.0, 0.0, 0.0; ...
    0.0, 0.0, 1.0, -1.0, 0.0, 0.0; ...
    0.0, 0.0, 0.0, 0.0, 1.0, -1.0];
baseCentersM = [0.05, -0.05, 0.0, 0.0, 0.0, 0.0; ...
    0.0, 0.0, 0.05, -0.05, 0.0, 0.0; ...
    0.0, 0.0, 0.0, 0.0, 0.15, -0.15];
rotatedNormals = geometryRotation * baseNormals;
rotatedCentersM = geometryRotation * ...
    (baseCentersM - centerOfMassOffsetM(:));
panelNormals = repmat(rotatedNormals, 1, 1, n);
panelCentersM = repmat(rotatedCentersM, 1, 1, n);
panelAreasM2 = repmat([0.03, 0.03, 0.03, 0.03, 0.01, 0.01], n, 1);
wallTemperatureK = repmat(reference.T_wall_K, 1, 6);
energyAccommodation = repmat(reference.alpha_E, 1, 6);

inputDataset = Simulink.SimulationData.Dataset();
inputDataset = addHarnessInput(inputDataset, "v_flow_B_m_s", vFlowBodyMps, timeS);
inputDataset = addHarnessInput(inputDataset, "rho_kg_m3", reference.rho_kg_m3, timeS);
inputDataset = addHarnessInput(inputDataset, "T_local_K", reference.T_atm_K, timeS);
inputDataset = addHarnessInput(inputDataset, "number_densities_m3", numberDensitiesM3, timeS);
inputDataset = addHarnessInput(inputDataset, "C_BI", identityDcm, timeS);
inputDataset = addHarnessInput(inputDataset, "mass_kg", reference.mass_kg, timeS);
inputDataset = addHarnessInput(inputDataset, "aero_enabled", ones(n, 1), timeS);
inputDataset = addHarnessInput(inputDataset, "panel_normals_B", panelNormals, timeS);
inputDataset = addHarnessInput(inputDataset, "panel_areas_m2", panelAreasM2, timeS);
inputDataset = addHarnessInput(inputDataset, "panel_centers_B_m", panelCentersM, timeS);
inputDataset = addHarnessInput(inputDataset, "wall_temperature_K", wallTemperatureK, timeS);
inputDataset = addHarnessInput(inputDataset, "energy_accommodation", energyAccommodation, timeS);
end

function forceN = cuboidReferenceForce(reference)
% Description:
%   Extracts frozen 3U cuboid body-force reference columns.
%
% Arguments:
%   reference - Table of frozen 3U cuboid reference cases.
%
% Outputs:
%   forceN - N-by-3 reference force matrix [N].

forceN = [reference.force_x_reference_N, ...
    reference.force_y_reference_N, reference.force_z_reference_N];
end

function momentNm = cuboidReferenceMoment(reference)
% Description:
%   Extracts frozen 3U cuboid body-moment reference columns.
%
% Arguments:
%   reference - Table of frozen 3U cuboid reference cases.
%
% Outputs:
%   momentNm - N-by-3 reference moment matrix [N*m].

momentNm = [reference.moment_x_reference_Nm, ...
    reference.moment_y_reference_Nm, reference.moment_z_reference_Nm];
end

function relativeError = vectorRelativeError(actual, expected, floorValue)
% Description:
%   Computes row-wise vector relative error with a lower norm floor.
%
% Arguments:
%   actual - N-by-M actual vectors.
%   expected - N-by-M expected vectors.
%   floorValue - Minimum denominator used for near-zero expected vectors.
%
% Outputs:
%   relativeError - N-by-1 relative error vector.

relativeError = vecnorm(actual - expected, 2, 2) ./ ...
    max(vecnorm(expected, 2, 2), floorValue);
end
