classdef AttitudeDynamicsTest < matlab.unittest.TestCase

    properties
        ProjectRoot
        BaseConfig
        WorkFolder
        ModelName
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            % Description:
            %   Adds source folders to the MATLAB path, loads the baseline JSON
            %   config, and creates a temporary folder for scenario configs.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.ProjectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            addpath(fullfile(testCase.ProjectRoot, "src", "config"));
            addpath(fullfile(testCase.ProjectRoot, "src", "simulink"));
            addpath(fullfile(testCase.ProjectRoot, "src", "analysis"));

            configFile = fullfile(testCase.ProjectRoot, "config", "AocsSimulationConfig.json");
            testCase.BaseConfig = jsondecode(fileread(configFile));
            testCase.ModelName = string(testCase.BaseConfig.models.plant);

            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            testCase.WorkFolder = fixture.Folder;
        end
    end

    methods (TestClassTeardown)
        function removeGeneratedSimulinkArtifacts(testCase)
            % Description:
            %   Discards in-memory model changes made by tests and removes common
            %   generated Simulink artifacts from the project and test folders.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            closeModelIfLoaded(testCase.ModelName);
            deleteIfPresent(fullfile(testCase.ProjectRoot, testCase.ModelName + ".slxc"));
            deleteIfPresent(fullfile(testCase.ProjectRoot, "tests", "attitude_dynamics", testCase.ModelName + ".slxc"));
            deleteIfPresent(fullfile(testCase.ProjectRoot, "slprj"));
            deleteIfPresent(fullfile(testCase.ProjectRoot, "tests", "attitude_dynamics", "slprj"));
            deleteIfPresent(fullfile(testCase.ProjectRoot, "results"));
        end
    end

    methods (Test)
        function idealSphereKeepsAngularVelocityConstant(testCase)
            % Description:
            %   For Ix = Iy = Iz and zero torque, verifies that angular velocity
            %   remains fixed while invariants and quaternion norm are conserved.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            result = testCase.runScenario( ...
                "ideal_sphere", ...
                diag([0.03 0.03 0.03]), ...
                [0.31; -0.42; 0.58], ...
                20);

            testCase.verifyTorqueFreeInvariants(result);
            testCase.verifyQuaternionNorm(result);
            testCase.verifyLessThan(maxBodyRateDeviation(result), 1e-10);
        end

        function cubesatPrincipalInertiaConservesEnergyAndMomentum(testCase)
            % Description:
            %   Uses Ix < Iy < Iz with a mixed initial rate and verifies invariant
            %   conservation plus nontrivial body-rate motion.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            result = testCase.runScenario( ...
                "cubesat_principal_inertia", ...
                diag([0.02 0.025 0.035]), ...
                [0.2; 0.1; 1.5], ...
                40);

            testCase.verifyTorqueFreeInvariants(result);
            testCase.verifyQuaternionNorm(result);
            testCase.verifyGreaterThan(maxBodyRateDeviation(result), 1e-3);
        end

        function largestPrincipalAxisSpinIsSteady(testCase)
            % Description:
            %   Spins about the largest principal inertia axis and verifies the
            %   body rate remains aligned with that axis.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            result = testCase.runScenario( ...
                "largest_principal_axis_spin", ...
                diag([0.02 0.025 0.035]), ...
                [0; 0; 1.2], ...
                20);

            testCase.verifyTorqueFreeInvariants(result);
            testCase.verifyPrincipalAxisSpin(result, 3, 1e-10);
        end

        function smallestPrincipalAxisSpinIsSteady(testCase)
            % Description:
            %   Spins about the smallest principal inertia axis and verifies the
            %   body rate remains aligned with that axis.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            result = testCase.runScenario( ...
                "smallest_principal_axis_spin", ...
                diag([0.02 0.025 0.035]), ...
                [1.2; 0; 0], ...
                20);

            testCase.verifyTorqueFreeInvariants(result);
            testCase.verifyPrincipalAxisSpin(result, 1, 1e-10);
        end

        function intermediatePrincipalAxisSpinIsUnstableWhenPerturbed(testCase)
            % Description:
            %   Starts near the intermediate inertia axis and verifies that a small
            %   transverse perturbation grows while invariants remain conserved.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            initialPerturbation = 1e-4;
            result = testCase.runScenario( ...
                "intermediate_principal_axis_spin", ...
                diag([0.02 0.025 0.035]), ...
                [initialPerturbation; 1.2; initialPerturbation], ...
                25);

            transverseRate = hypot(result.Omega(:, 1), result.Omega(:, 3));

            testCase.verifyTorqueFreeInvariants(result);
            testCase.verifyGreaterThan(max(transverseRate), 20 * transverseRate(1));
        end

        function symmetricTopInitialRateProducesConing(testCase)
            % Description:
            %   Uses Ix = Iy < Iz with transverse angular rate and verifies steady
            %   transverse magnitude with rotating phase.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            result = testCase.runScenario( ...
                "symmetric_top_coning", ...
                diag([0.02 0.02 0.035]), ...
                [0.25; 0; 1.2], ...
                25);

            transverseRate = hypot(result.Omega(:, 1), result.Omega(:, 2));
            phase = unwrap(atan2(result.Omega(:, 2), result.Omega(:, 1)));

            testCase.verifyTorqueFreeInvariants(result);
            testCase.verifyQuaternionNorm(result);
            testCase.verifyGreaterThan(mean(transverseRate), 0.1);
            testCase.verifyLessThan(max(abs(transverseRate - transverseRate(1))), 1e-7);
            testCase.verifyGreaterThan(max(phase) - min(phase), 2*pi);
        end
    end

    methods (Access = private)
        function result = runScenario(testCase, scenarioName, inertia_B, omega0_B, stopTime_s)
            % Description:
            %   Writes a scenario-specific config, configures the model, runs the
            %   plant, extracts logged state, and computes validation invariants.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   scenarioName - Name used for the temporary JSON/result files.
            %   inertia_B - 3-by-3 inertia matrix [kg*m^2].
            %   omega0_B - 3-by-1 initial body angular rate [rad/s].
            %   stopTime_s - Simulation stop time [s].
            %
            % Outputs:
            %   result - Struct with AOCS config, state logs, omega, quaternion,
            %            time, and invariant histories.

            configFile = testCase.writeScenarioConfig(scenarioName, inertia_B, omega0_B, stopTime_s);
            AOCS = setupAocsSimulation(configFile);
            disableEnvironmentDisturbancesForTorqueFreeTest();

            load_system(AOCS.Model.File);
            applyAocsSimulationSettings(AOCS.Model.Name, AOCS);

            out = sim(AOCS.Model.Name);
            state = extractAocsState(out);
            omega = loggedSignalMatrix(state.omega_b.Data, 3, "omega_b");
            q = loggedSignalMatrix(state.q_be.Data, 4, "q_be");
            invariants = computeAocsInvariants(omega, inertia_B);
            maxDisturbanceTorque = maxLoggedDisturbanceTorque(out);

            result = struct();
            result.Name = scenarioName;
            result.AOCS = AOCS;
            result.State = state;
            result.Time = state.omega_b.Time;
            result.Omega = omega;
            result.Quaternion = q;
            result.Invariants = invariants;
            result.MaxDisturbanceTorque = maxDisturbanceTorque;
        end

        function configFile = writeScenarioConfig(testCase, scenarioName, inertia_B, omega0_B, stopTime_s)
            % Description:
            %   Copies the baseline config, overrides only scenario-specific
            %   inertia/rate/time/result fields, and writes it to the temp folder.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   scenarioName - Name used for mission and result file labels.
            %   inertia_B - 3-by-3 inertia matrix [kg*m^2].
            %   omega0_B - 3-by-1 initial body angular rate [rad/s].
            %   stopTime_s - Simulation stop time [s].
            %
            % Outputs:
            %   configFile - Path to the written temporary JSON config.

            raw = testCase.BaseConfig;
            raw.mission.name = char(scenarioName);
            raw.simulation.stop_time_s = stopTime_s;
            raw.spacecraft.mass_properties.inertia_B_kg_m2 = inertia_B;
            raw.initial_conditions.q_BI = [1 0 0 0];
            raw.initial_conditions.omega_BI_B_rad_s = omega0_B(:).';
            raw.environment.external_torque_B_Nm = [0 0 0];
            raw.environment.residual_magnetic_dipole_B_A_m2 = [0 0 0];
            raw.results.directory = testCase.WorkFolder;
            raw.results.file = char(scenarioName + ".mat");

            if isfield(raw.initial_conditions, "euler_BI_0_rad")
                raw.initial_conditions = rmfield(raw.initial_conditions, "euler_BI_0_rad");
            end

            configFile = fullfile(testCase.WorkFolder, scenarioName + ".json");
            fid = fopen(configFile, "w");
            cleanup = onCleanup(@() fclose(fid));
            fwrite(fid, jsonencode(raw), "char");
        end

        function verifyTorqueFreeInvariants(testCase, result)
            % Description:
            %   Verifies relative drift of rotational energy and angular momentum
            %   norm stays within the regression tolerance.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   result - Scenario result struct from runScenario.
            %
            % Outputs:
            %   None.

            energy = result.Invariants.E_rot;
            momentum = result.Invariants.H_norm;

            testCase.verifyLessThan(result.MaxDisturbanceTorque, 1e-14, ...
                "Modeled disturbance torque is nonzero in torque-free scenario " + result.Name + ".");

            energyScale = max(abs(energy(1)), eps);
            momentumScale = max(abs(momentum(1)), eps);

            testCase.verifyLessThan(max(abs(energy - energy(1))) / energyScale, 1e-8, ...
                "Rotational energy drift is too large in " + result.Name + ".");
            testCase.verifyLessThan(max(abs(momentum - momentum(1))) / momentumScale, 1e-8, ...
                "Angular momentum norm drift is too large in " + result.Name + ".");
        end

        function verifyQuaternionNorm(testCase, result)
            % Description:
            %   Verifies the logged quaternion norm stays near one.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   result - Scenario result struct from runScenario.
            %
            % Outputs:
            %   None.

            qNorm = vecnorm(result.Quaternion, 2, 2);
            testCase.verifyLessThan(max(abs(qNorm - 1)), 1e-9, ...
                "Quaternion norm drift is too large in " + result.Name + ".");
        end

        function verifyPrincipalAxisSpin(testCase, result, axisIndex, tolerance)
            % Description:
            %   Verifies transverse body rates stay near zero and the selected
            %   principal-axis rate stays constant.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   result - Scenario result struct from runScenario.
            %   axisIndex - Principal axis index expected to contain the spin.
            %   tolerance - Absolute tolerance for transverse and drift checks.
            %
            % Outputs:
            %   None.

            omega = result.Omega;
            transverseAxes = setdiff(1:3, axisIndex);

            testCase.verifyLessThan(max(abs(omega(:, transverseAxes)), [], "all"), tolerance);
            testCase.verifyLessThan(max(abs(omega(:, axisIndex) - omega(1, axisIndex))), tolerance);
        end
    end
end

function deviation = maxBodyRateDeviation(result)
% Description:
%   Provides one scalar measure of whether body-rate motion is steady or
%   nontrivial across a test scenario.
%
% Arguments:
%   result - Scenario result struct from runScenario.
%
% Outputs:
%   deviation - Maximum Euclidean deviation of omega from omega(1,:).

deviation = max(vecnorm(result.Omega - result.Omega(1, :), 2, 2));
end

function disableEnvironmentDisturbancesForTorqueFreeTest()
% Description:
%   Zeros environment-driven torques in the base-workspace parameters while
%   leaving the top-level model architecture intact.
%
% Arguments:
%   None.
%
% Outputs:
%   None.

if evalin("base", "exist('AOCS_EnvironmentConfig', 'var')")
    AOCS_EnvironmentConfig = evalin("base", "AOCS_EnvironmentConfig");
    environmentConfig = AOCS_EnvironmentConfig.Value;
    environmentConfig.m_res_B_A_m2 = zeros(3, 1);
    if isfield(environmentConfig, "rmm_enabled")
        environmentConfig.rmm_enabled = 0;
    end
    if isfield(environmentConfig, "gravity_gradient_enabled")
        environmentConfig.gravity_gradient_enabled = 0;
    end
    if isfield(environmentConfig, "srp_enabled")
        environmentConfig.srp_enabled = 0;
    end
    AOCS_EnvironmentConfig.Value = environmentConfig;
    assignin("base", "AOCS_EnvironmentConfig", AOCS_EnvironmentConfig);
end

if evalin("base", "exist('AOCS_Config', 'var')")
    AOCS_Config = evalin("base", "AOCS_Config");
    plantConfig = AOCS_Config.Value;
    plantConfig.M_ext_B = zeros(3, 1);
    AOCS_Config.Value = plantConfig;
    assignin("base", "AOCS_Config", AOCS_Config);
end

if evalin("base", "exist('AOCS', 'var')")
    AOCS = evalin("base", "AOCS");
    if isfield(AOCS, "Config")
        AOCS.Config.M_ext_B = zeros(3, 1);
    end
    if isfield(AOCS, "EnvironmentConfig")
        AOCS.EnvironmentConfig.m_res_B_A_m2 = zeros(3, 1);
        if isfield(AOCS.EnvironmentConfig, "rmm_enabled")
            AOCS.EnvironmentConfig.rmm_enabled = 0;
        end
        if isfield(AOCS.EnvironmentConfig, "gravity_gradient_enabled")
            AOCS.EnvironmentConfig.gravity_gradient_enabled = 0;
        end
        if isfield(AOCS.EnvironmentConfig, "srp_enabled")
            AOCS.EnvironmentConfig.srp_enabled = 0;
        end
    end
    if isfield(AOCS, "Environment")
        if isfield(AOCS.Environment, "RmmEnabled")
            AOCS.Environment.RmmEnabled = false;
        end
        if isfield(AOCS.Environment, "GravityGradientEnabled")
            AOCS.Environment.GravityGradientEnabled = false;
        end
    end
    if isfield(AOCS, "Environment") && isfield(AOCS.Environment, "SRP")
        AOCS.Environment.SRP.Enabled = false;
    end
    if isfield(AOCS, "Environment") && isfield(AOCS.Environment, "M_ext_B")
        AOCS.Environment.M_ext_B = zeros(3, 1);
    end
    assignin("base", "AOCS", AOCS);
end
end

function maxTorque = maxLoggedDisturbanceTorque(out)
% Description:
%   Returns the maximum logged modeled disturbance torque, or zero when the
%   signal is absent.
%
% Arguments:
%   out - Simulink.SimulationOutput from a test scenario.
%
% Outputs:
%   maxTorque - Maximum disturbance torque norm [N*m].

maxTorque = 0;

try
    element = out.logsout.getElement("M_dist_B_Nm");
catch
    element = [];
end

if isempty(element)
    return;
end

torqueData = loggedSignalMatrix(element.Values.Data, 3, "M_dist_B_Nm");
maxTorque = max(vecnorm(torqueData, 2, 2));
end

function closeModelIfLoaded(modelName)
% Description:
%   Used by test teardown to discard runtime mask-parameter changes safely.
%
% Arguments:
%   modelName - Model name as char/string, or empty/unset value.
%
% Outputs:
%   None.

if ~(ischar(modelName) || isstring(modelName))
    return;
end

modelName = string(modelName);
if strlength(modelName) > 0 && bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end

function deleteIfPresent(path)
% Description:
%   Keeps test cleanup idempotent for generated Simulink artifacts.
%
% Arguments:
%   path - File or folder path to remove.
%
% Outputs:
%   None.

if isfolder(path)
    rmdir(path, "s");
elseif isfile(path)
    delete(path);
end
end
