classdef Dtm2020InputsTest < matlab.unittest.TestCase

    methods (TestClassSetup)
        function addProjectPaths(~)
            % Description:
            %   Adds source and test helper paths required by DTM2020 input tests.
            %
            % Arguments:
            %   None.
            %
            % Outputs:
            %   None.

            setupAocsPaths(projectRoot(), true);
        end
    end

    methods (Test)
        function modelTimeHandlesLeapDay(testCase)
            % Description:
            %   Verifies DTM2020 model-time conversion across a leap-day boundary.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            time = dtm2020.modelTime([2024; 2; 28; 23; 59; 59], 2.0, 0.0);

            testCase.verifyEqual(time.year, 2024.0);
            testCase.verifyEqual(floor(time.day_of_year), 60.0);
            testCase.verifyEqual(time.seconds_of_day, 1.0, "AbsTol", 1e-12);
        end

        function modelTimeHandlesYearRollover(testCase)
            % Description:
            %   Verifies DTM2020 model-time conversion across a year rollover.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            time = dtm2020.modelTime([2025; 12; 31; 23; 59; 59], 2.0, 0.0);

            testCase.verifyEqual(time.year, 2026.0);
            testCase.verifyEqual(time.day_of_year, 1.0 + 1.0 / 86400.0, ...
                "AbsTol", 1e-12);
            testCase.verifyEqual(time.seconds_of_day, 1.0, "AbsTol", 1e-12);
        end

        function localSolarTimeUsesEastPositiveLongitude(testCase)
            % Description:
            %   Checks local solar time sign convention for east-positive longitude.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            time = dtm2020.modelTime([2026; 1; 1; 23; 0; 0], 0.0, pi / 6.0);

            expected = 2.0 * pi / 24.0;
            testCase.verifyEqual(time.local_solar_time_rad, expected, "AbsTol", 1e-12);
        end

        function operationalInputsMatchDtm3Contract(testCase)
            % Description:
            %   Verifies operational-mode input packing against the DTM3-style
            %   DTM2020 native contract.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            config = nominalConfig(1.0);
            inputs = dtm2020.prepareInputs([45.0; 30.0; 500e3], config, nominalContext());

            testCase.verifyEqual(inputs.f, [150.0; 0.0]);
            testCase.verifyEqual(inputs.fbar, [140.0; 0.0]);
            testCase.verifyEqual(inputs.akp, [2.0; 0.0; 2.0; 0.0]);
            testCase.verifyEqual(inputs.ap60, zeros(10, 1));
            testCase.verifyEqual(inputs.altitude_km, 500.0);
            testCase.verifyEqual(inputs.latitude_rad, pi / 4.0, "AbsTol", 1e-12);
        end

        function researchInputsMatchDtm5Shapes(testCase)
            % Description:
            %   Verifies research-mode input vector shapes and space-weather
            %   parameter placement for DTM2020.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            config = nominalConfig(2.0);
            inputs = dtm2020.prepareInputs([0.0; 0.0; 400e3], config, nominalContext());

            testCase.verifySize(inputs.f, [2 1]);
            testCase.verifySize(inputs.fbar, [2 1]);
            testCase.verifySize(inputs.ap60, [10 1]);
            testCase.verifyEqual(inputs.akp, zeros(4, 1));
            testCase.verifyEqual(inputs.ap60, 7.0 * ones(10, 1), "AbsTol", 1e-12);
            testCase.verifyGreaterThan(inputs.f(1), 0.0);
            testCase.verifyGreaterThan(inputs.fbar(1), 0.0);
        end

        function nativeOperationalVectorMatchesPreparedInputs(testCase)
            % Description:
            %   Checks that the native input vector is derived from prepared
            %   operational DTM2020 inputs without reordering.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            config = nominalConfig(1.0);
            context = nominalContext();
            lla = [45.0; 30.0; 500e3];
            inputs = dtm2020.prepareInputs(lla, config, context);
            nativeInput = dtm2020.prepareNativeInput(lla, config, context);

            expected = [inputs.day_of_year; inputs.f(1); inputs.fbar(1); ...
                inputs.akp(1); inputs.akp(3); inputs.altitude_km; ...
                inputs.local_solar_time_rad; inputs.latitude_rad; ...
                inputs.longitude_rad];
            testCase.verifyEqual(nativeInput, expected);
        end
    end
end

function config = nominalConfig(modeId)
% Description:
%   Builds a representative DTM2020 environment configuration struct.
%
% Arguments:
%   modeId - Numeric atmosphere mode identifier.
%
% Outputs:
%   config - Struct containing space-weather and mode fields.

config = struct();
config.atmosphere_mode_id = modeId;
config.f10_7_sfu = 150.0;
config.f10_7_81d_sfu = 140.0;
config.kp = 2.0;
config.f30_sfu = 100.0;
config.f30_81d_sfu = 95.0;
config.hp60 = 2.0;
end

function context = nominalContext()
% Description:
%   Builds a representative DTM2020 environment context struct.
%
% Arguments:
%   None.
%
% Outputs:
%   context - Struct containing epoch and elapsed simulation time.

context = struct();
context.epoch_utc = [2026.0; 1.0; 1.0; 0.0; 0.0; 0.0];
context.t_s = 0.0;
end
