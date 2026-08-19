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
            testCase.ProjectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.Owner = "aocs_plant/Orbit & Environment/Disturbance Torques/" + ...
                "Sentman Multispecies Panels";
            testCase.HarnessFile = fullfile(testCase.ProjectRoot, "tests", ...
                "harnesses", testCase.HarnessName + ".slx");
            testCase.ReferenceFile = fullfile(testCase.ProjectRoot, "validation", ...
                "aerodynamics", "data", "sentman_single_plate_reference.csv");
            testCase.CuboidReferenceFile = fullfile(testCase.ProjectRoot, ...
                "validation", "aerodynamics", "data", ...
                "sentman_3u_cuboid_reference.csv");

            addpath(testCase.ProjectRoot);
            addpath(fullfile(testCase.ProjectRoot, "src", "config"));
            addpath(fullfile(testCase.ProjectRoot, "src", "simulink"));
            addpath(fullfile(testCase.ProjectRoot, "src", "environment"));
            addpath(fullfile(testCase.ProjectRoot, "tests", "harnesses"));

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
            try
                sltest.harness.close(testCase.Owner, testCase.HarnessName);
            catch
            end
            if bdIsLoaded("aocs_plant")
                close_system("aocs_plant", 0);
            end
        end
    end

    methods (Test)
        function fixtureMatchesIndependentAdbSatReference(testCase)
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
caseTimeS = (0:height(reference) - 1).';
inputDataset = singlePlateInputDataset(reference, caseTimeS);
simulation = runHarness(harnessName, inputDataset, caseTimeS);
end

function simulation = runCuboidReferenceCases(harnessName, reference, ...
        centerOfMassOffsetM, geometryRotation)
caseTimeS = (0:height(reference) - 1).';
inputDataset = cuboidInputDataset(reference, caseTimeS, ...
    centerOfMassOffsetM, geometryRotation);
simulation = runHarness(harnessName, inputDataset, caseTimeS);
end

function simulation = runHarness(harnessName, inputDataset, caseTimeS)
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
inputDataset = addInput(inputDataset, "v_flow_B_m_s", vFlowBodyMps, timeS);
inputDataset = addInput(inputDataset, "rho_kg_m3", reference.rho_kg_m3, timeS);
inputDataset = addInput(inputDataset, "T_local_K", reference.T_atm_K, timeS);
inputDataset = addInput(inputDataset, "number_densities_m3", numberDensitiesM3, timeS);
inputDataset = addInput(inputDataset, "C_BI", identityDcm, timeS);
inputDataset = addInput(inputDataset, "mass_kg", reference.mass_kg, timeS);
inputDataset = addInput(inputDataset, "aero_enabled", ones(n, 1), timeS);
inputDataset = addInput(inputDataset, "panel_normals_B", panelNormals, timeS);
inputDataset = addInput(inputDataset, "panel_areas_m2", panelAreasM2, timeS);
inputDataset = addInput(inputDataset, "panel_centers_B_m", panelCentersM, timeS);
inputDataset = addInput(inputDataset, "wall_temperature_K", wallTemperatureK, timeS);
inputDataset = addInput(inputDataset, "energy_accommodation", energyAccommodation, timeS);
end

function inputDataset = cuboidInputDataset(reference, timeS, ...
        centerOfMassOffsetM, geometryRotation)
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
inputDataset = addInput(inputDataset, "v_flow_B_m_s", vFlowBodyMps, timeS);
inputDataset = addInput(inputDataset, "rho_kg_m3", reference.rho_kg_m3, timeS);
inputDataset = addInput(inputDataset, "T_local_K", reference.T_atm_K, timeS);
inputDataset = addInput(inputDataset, "number_densities_m3", numberDensitiesM3, timeS);
inputDataset = addInput(inputDataset, "C_BI", identityDcm, timeS);
inputDataset = addInput(inputDataset, "mass_kg", reference.mass_kg, timeS);
inputDataset = addInput(inputDataset, "aero_enabled", ones(n, 1), timeS);
inputDataset = addInput(inputDataset, "panel_normals_B", panelNormals, timeS);
inputDataset = addInput(inputDataset, "panel_areas_m2", panelAreasM2, timeS);
inputDataset = addInput(inputDataset, "panel_centers_B_m", panelCentersM, timeS);
inputDataset = addInput(inputDataset, "wall_temperature_K", wallTemperatureK, timeS);
inputDataset = addInput(inputDataset, "energy_accommodation", energyAccommodation, timeS);
end

function dataset = addInput(dataset, name, values, timeS)
signal = timeseries(values, timeS);
signal.Name = name;
dataset = dataset.addElement(signal, name);
end

function values = harnessOutputRows(simOut, signalName, width, sampleTimeS)
yout = simOut.get("yout");
for index = 1:yout.numElements
    element = yout{index};
    if outputElementMatches(element, signalName)
        signal = element.Values;
        data = squeeze(signal.Data);
        if width == 1
            data = data(:);
        elseif size(data, 1) ~= numel(signal.Time) && ...
                size(data, 2) == numel(signal.Time)
            data = data.';
        end
        data = reshape(data, [], width);
        values = interp1(signal.Time(:), data, sampleTimeS, "linear");
        return;
    end
end
error("AOCS:Tests:MissingHarnessOutput", ...
    "Could not find harness output '%s'.", signalName);
end

function tf = outputElementMatches(element, signalName)
tf = isprop(element, "Name") && string(element.Name) == string(signalName);
if tf
    return;
end
try
    blockPath = string(element.BlockPath.getBlock(1));
    tf = endsWith(blockPath, "/" + string(signalName));
catch
    tf = false;
end
end

function forceN = cuboidReferenceForce(reference)
forceN = [reference.force_x_reference_N, ...
    reference.force_y_reference_N, reference.force_z_reference_N];
end

function momentNm = cuboidReferenceMoment(reference)
momentNm = [reference.moment_x_reference_Nm, ...
    reference.moment_y_reference_Nm, reference.moment_z_reference_Nm];
end

function relativeError = vectorRelativeError(actual, expected, floorValue)
relativeError = vecnorm(actual - expected, 2, 2) ./ ...
    max(vecnorm(expected, 2, 2), floorValue);
end
