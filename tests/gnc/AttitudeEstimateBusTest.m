classdef AttitudeEstimateBusTest < matlab.unittest.TestCase
    % Description:
    %   Verifies the estimator output schema without requiring a MEKF model.

    methods (TestClassSetup)
        function prepareEnvironment(~)
            % Description:
            %   Installs the project paths for production bus factories.
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
        function contractsDescribeEstimateAndErrorCovariance(testCase)
            % Description:
            %   Checks field order, dimensions, types and physical units.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            bus = createAttitudeEstimateBus("none");
            elements = bus.Elements;
            testCase.verifyEqual(string({elements.Name}), ...
                ["q_BI", "omega_BI_B_rad_s", "initialized", ...
                "gyro_bias_B_rad_s", "P_error", "valid", "update_time_s"]);
            dimensions = {[4 1], [3 1], 1, [3 1], [6 6], 1, 1};
            for index = 1:numel(elements)
                testCase.verifyEqual(elements(index).Dimensions, dimensions{index});
            end
            testCase.verifyEqual(string({elements.DataType}), ...
                ["double", "double", "boolean", "double", "double", "boolean", "double"]);
            testCase.verifyEqual(string({elements.Unit}), ...
                ["1", "rad/s", "1", "rad/s", "", "1", "s"]);
        end

        function registrationCreatesTypedEstimateStructure(testCase)
            % Description:
            %   Checks central registration and MATLAB structure generation.
            %   Zero defaults describe an invalid, uninitialized estimate.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            createAocsBuses;
            bus = evalin("base", "AttitudeEstimateBus");
            testCase.verifyClass(bus, "Simulink.Bus");
            estimate = Simulink.Bus.createMATLABStruct("AttitudeEstimateBus");
            testCase.verifySize(estimate.q_BI, [4 1]);
            testCase.verifySize(estimate.omega_BI_B_rad_s, [3 1]);
            testCase.verifySize(estimate.gyro_bias_B_rad_s, [3 1]);
            testCase.verifySize(estimate.P_error, [6 6]);
            testCase.verifyClass(estimate.initialized, "logical");
            testCase.verifyClass(estimate.valid, "logical");
            testCase.verifyFalse(estimate.initialized);
            testCase.verifyFalse(estimate.valid);
            testCase.verifyEqual(estimate.update_time_s, 0);
        end
    end
end
