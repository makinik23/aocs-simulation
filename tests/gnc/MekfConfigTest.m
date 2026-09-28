classdef MekfConfigTest < matlab.unittest.TestCase
    % Description:
    %   Verifies initialization settings, bus publication and validation.

    methods (TestClassSetup)
        function prepareEnvironment(~)
            % Description:
            %   Installs production and test paths.
            %
            % Arguments:
            %   None.
            %
            % Outputs:
            %   None.
            setupAocsPaths([], true);
        end
    end

    methods (Test)
        function defaultsConvertStandardDeviationsAndPublishOneConfig(testCase)
            % Description:
            %   Checks SI conversion, variance construction and shared config.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            [aocs, variables] = setupAocsSimulation("", false);
            expectedStd = deg2rad([5; 5; 5; 0.1; 0.1; 0.1]);
            config = variables.AOCS_GNCConfig.Value.MEKF;
            testCase.verifyEqual(config.initial_bias_B_rad_s, zeros(3, 1));
            testCase.verifyEqual(config.initial_covariance, diag(expectedStd.^2), ...
                "AbsTol", 1e-15);
            testCase.verifyEqual(aocs.GNC.MEKF.InitialAttitudeStd_rad, expectedStd(1:3));
            testCase.verifyEqual(aocs.GNC.MEKF.InitialBiasStd_rad_s, expectedStd(4:6));
            testCase.verifyGreaterThan(min(eig(config.initial_covariance)), 0);
            testCase.verifyFalse(isfield(variables, "AOCS_MEKFConfig"));
            gncBus = evalin("base", "GNCConfigBus");
            testCase.verifyEqual(fieldnames(variables.AOCS_GNCConfig.Value), ...
                {gncBus.Elements.Name}');
        end

        function contractsMatchNumericPayload(testCase)
            % Description:
            %   Checks bias and mixed-unit covariance bus definitions.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            bus = createMekfConfigBus("none");
            [~,variables] = setupAocsSimulation("", false);
            testCase.verifyEqual({bus.Elements.Name}', ...
                fieldnames(variables.AOCS_GNCConfig.Value.MEKF));
            testCase.verifyEqual(bus.Elements(1).Dimensions, [3 1]);
            testCase.verifyEqual(bus.Elements(2).Dimensions, [6 6]);
            testCase.verifyEqual(string({bus.Elements.DataType}), repmat("double",1,10));
            testCase.verifyEqual(string({bus.Elements(1:2).Unit}), ["rad/s", ""]);
            createAocsBuses;
            value = Simulink.Bus.createMATLABStruct("MekfConfigBus");
            testCase.verifySize(value.initial_bias_B_rad_s, [3 1]);
            testCase.verifySize(value.initial_covariance, [6 6]);
        end

        function overridesRemainIndependentOfSensorBiasTruth(testCase)
            % Description:
            %   Checks per-axis priors and separation from simulated bias.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            mekf = struct("initial_bias_B_rad_s", [0.001; -0.002; 0.003], ...
                "initial_attitude_std_deg", [2; 3; 4], ...
                "initial_bias_std_deg_s", [0.1; 0.2; 0.3]);
            override = struct("gnc", struct("mekf", mekf), ...
                "sensors", struct("gyro", struct("bias_initial_rad_s", [1; 2; 3])));
            aocs = loadOverride(override);
            testCase.verifyEqual(aocs.GNCConfig.MEKF.initial_bias_B_rad_s, ...
                mekf.initial_bias_B_rad_s);
            testCase.verifyEqual(aocs.GNCConfig.MEKF.initial_covariance, ...
                diag(deg2rad([2; 3; 4; 0.1; 0.2; 0.3]).^2), "AbsTol", 1e-15);
        end

        function rejectsInvalidStandardDeviations(testCase)
            % Description:
            %   Rejects nonpositive, malformed and nonnumeric uncertainty.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            for name = ["initial_attitude_std_deg", "initial_bias_std_deg_s"]
                invalid = {[0; 1; 1], [-1; 1; 1], [1; 2], "bad", [true; true; true]};
                for index = 1:numel(invalid)
                    mekf = struct();
                    mekf.(name) = invalid{index};
                    testCase.verifyError(@() loadOverride(struct("gnc", ...
                        struct("mekf", mekf))), "AOCS:Config:InvalidField");
                end
            end
        end

        function rejectsMalformedBias(testCase)
            % Description:
            %   Rejects an initial bias that cannot describe three axes.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            override = struct("gnc", struct("mekf", ...
                struct("initial_bias_B_rad_s", [1; 2])));
            testCase.verifyError(@() loadOverride(override), "AOCS:Config:InvalidField");
        end

        function validatesIndependentFilterTuning(testCase)
            % Description:
            %   Validates SI conversion and rejects invalid filter tuning.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            override.gnc.mekf.css_direction_std_deg = 3;
            override.sensors.gyro.noise_density_rad_s_sqrt_Hz = 0.1;
            aocs = loadOverride(override);
            testCase.verifyEqual(aocs.GNCConfig.MEKF.css_direction_std_rad,deg2rad(3));
            testCase.verifyEqual(aocs.GNCConfig.MEKF.gyro_noise_density_rad_s_sqrt_Hz, ...
                8.726646259971648e-5);
            for name = ["innovation_gate_squared", "reference_max_age_s", ...
                    "gyro_noise_density_rad_s_sqrt_Hz", "css_direction_std_deg"]
                for value = {0, -1, [1 2], "bad"}
                    invalid.gnc.mekf = struct(name,value{1});
                    testCase.verifyError(@() loadOverride(invalid),"AOCS:Config:InvalidField");
                end
            end
            invalid.gnc.mekf = struct("maximum_correction_deg",90);
            testCase.verifyError(@() loadOverride(invalid),"AOCS:Config:InvalidField");
        end
    end
end

function aocs = loadOverride(override)
% Description:
%   Loads a temporary inherited scenario and removes it after the check.
%
% Arguments:
%   override - Scenario fields overriding the default project configuration.
%
% Outputs:
%   aocs - Validated normalized configuration and numeric bus payloads.

override.extends = fullfile(projectRoot(), "config", "AocsSimulationConfig.json");
file = string(tempname) + ".json";
cleanup = onCleanup(@() delete(file));
writeAocsJson(file, override);
aocs = loadAocsSimulationConfig(file);
end
