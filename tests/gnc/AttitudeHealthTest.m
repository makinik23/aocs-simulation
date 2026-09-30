classdef AttitudeHealthTest < matlab.unittest.TestCase
    % Description:
    %   Exercises the saved block-based MEKF health monitor and its contracts.

    methods (TestClassSetup)
        function prepareEnvironment(~)
            % Description:
            %   Makes production bus definitions and the isolated harness available.
            %
            % Arguments:
            %   None.
            %
            % Outputs:
            %   None.
            setupAocsPaths([], true);
            setupAocsSimulation('', true);
        end
    end

    methods (Test)
        function configurationAndBusContracts(testCase)
            % Description:
            %   Checks ordered SI thresholds and the separate typed status bus.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            cfg = evalin('base', 'AOCS_GNCConfig');
            health = cfg.Value.AttitudeHealth;
            bus = createAttitudeHealthConfigBus('none');
            testCase.verifyEqual(fieldnames(health), {bus.Elements.Name}');
            testCase.verifyEqual(health.degraded_attitude_std_rad, deg2rad(2), ...
                'AbsTol', 1e-15);
            testCase.verifyGreaterThan(health.lost_attitude_std_rad, ...
                health.degraded_attitude_std_rad);
            statusBus = createAttitudeHealthBus('none');
            testCase.verifyEqual(string({statusBus.Elements.Name}), ...
                ["mode_id", "control_usable", "attitude_std_max_rad", ...
                "state_age_s", "correction_age_s", "vector_update_accepted"]);
            testCase.verifyEqual(string({statusBus.Elements(1:2).DataType}), ...
                ["uint8", "boolean"]);
        end

        function rejectsUnorderedHealthLimits(testCase)
            % Description:
            %   Rejects a lost limit no greater than its degraded counterpart.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            override.gnc.attitude_health.lost_state_age_s = 0.1;
            testCase.verifyError(@() loadOverride(override), ...
                'AOCS:Config:InvalidField');
        end

        function uninitializedAndTrackingStates(testCase)
            % Description:
            %   Distinguishes missing TRIAD initialization from usable tracking.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            f = fixture(1.0);
            f.triadValid = false(numel(f.t), 1);
            s = simulateMekfFixture(f);
            testCase.verifyEqual(s.healthMode, zeros(numel(f.t), 1, 'uint8'));
            testCase.verifyFalse(any(s.controlUsable));

            f = fixture(1.0);
            s = simulateMekfFixture(f);
            testCase.verifyEqual(s.healthMode(end), uint8(1));
            testCase.verifyTrue(s.controlUsable(end));
            testCase.verifyLessThan(s.attitudeStd(end), deg2rad(2));

            f.triadValid = f.t >= 0.5;
            s = simulateMekfFixture(f);
            testCase.verifyEqual(s.healthMode(at(f.t, 0.4)), uint8(0));
            testCase.verifyEqual(s.correctionAge(at(f.t, 0.5)), 0, 'AbsTol', 1e-12);
            testCase.verifyEqual(s.healthMode(end), uint8(1));
        end

        function covarianceLimitsDegradeAndLose(testCase)
            % Description:
            %   Classifies large covariance without using truth attitude error.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            f = fixture(0.5);
            f.P(1:3, 1:3) = eye(3) * deg2rad(5)^2;
            s = simulateMekfFixture(f);
            testCase.verifyEqual(s.healthMode(end), uint8(2));
            testCase.verifyFalse(s.controlUsable(end));

            f.P(1:3, 1:3) = eye(3) * deg2rad(20)^2;
            s = simulateMekfFixture(f);
            testCase.verifyEqual(s.healthMode(end), uint8(3));
            testCase.verifyFalse(s.controlUsable(end));

            f.P(1:3, 1:3) = diag([-1e-4, 1e-4, 1e-4]);
            s = simulateMekfFixture(f);
            testCase.verifyEqual(s.healthMode(end), uint8(3));
            testCase.verifyFalse(s.controlUsable(end));
        end

        function staleGyroStateDegradesLosesAndRecovers(testCase)
            % Description:
            %   Checks state-age priority and recovery after fresh gyro data.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            f = fixture(1.3);
            f.fresh = f.t <= 0.2 | f.t >= 1.1;
            f.health.degraded_state_age_s = 0.2;
            f.health.lost_state_age_s = 0.6;
            s = simulateMekfFixture(f);
            testCase.verifyEqual(s.healthMode(at(f.t, 0.5)), uint8(2));
            testCase.verifyEqual(s.healthMode(at(f.t, 1.0)), uint8(3));
            testCase.verifyEqual(s.healthMode(end), uint8(1));
            testCase.verifyTrue(s.controlUsable(end));
        end

        function missingVectorCorrectionsDegradeLoseAndRecover(testCase)
            % Description:
            %   Measures age since accepted vector correction, not sensor polling.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            f = fixture(1.3);
            f.health.degraded_correction_age_s = 0.3;
            f.health.lost_correction_age_s = 0.7;
            f.mag = repmat([3e-5 0 0], numel(f.t), 1);
            f.magI = f.mag;
            f.magFresh = f.t >= 1.1;
            s = simulateMekfFixture(f);
            testCase.verifyEqual(s.healthMode(at(f.t, 0.5)), uint8(2));
            testCase.verifyEqual(s.healthMode(at(f.t, 0.9)), uint8(3));
            testCase.verifyTrue(any(s.vectorAccepted(f.t >= 1.1)));
            testCase.verifyEqual(s.healthMode(end), uint8(1));
            testCase.verifyLessThan(s.correctionAge(end), 0.3);
        end
    end
end

function f = fixture(stopTime_s)
% Description:
%   Creates a static, low-uncertainty MEKF fixture with sampled gyro reports.
%
% Arguments:
%   stopTime_s - End of the sampled scenario [s].
%
% Outputs:
%   f - Isolated MEKF harness inputs.

f.t = (0:0.1:stopTime_s)';
f.q = [1; 0; 0; 0];
f.rate = zeros(numel(f.t), 3);
f.bias = zeros(3, 1);
f.P = diag([repmat(deg2rad(0.5)^2, 3, 1); ...
    repmat(deg2rad(0.1)^2, 3, 1)]);
end

function index = at(time_s, target_s)
% Description:
%   Finds one sampled fixture time without relying on floating-point equality.
%
% Arguments:
%   time_s - Fixture sample timestamps [s].
%   target_s - Requested time [s].
%
% Outputs:
%   index - Index of the nearest sample.

[~, index] = min(abs(time_s - target_s));
end

function aocs = loadOverride(override)
% Description:
%   Applies one temporary scenario override for configuration rejection tests.
%
% Arguments:
%   override - JSON-compatible partial configuration.
%
% Outputs:
%   aocs - Validated scenario when the override is accepted.

root = setupAocsPaths();
override.extends = fullfile(root, 'config', 'AocsSimulationConfig.json');
file = string(tempname) + '.json';
cleanup = onCleanup(@() delete(file));
fid = fopen(file, 'w');
if fid < 0
    error('AOCS:Test:FixtureWrite', 'Cannot create temporary configuration.');
end
fileHandleCleanup = onCleanup(@() fclose(fid));
fwrite(fid, jsonencode(override), 'char');
clear fileHandleCleanup
aocs = loadAocsSimulationConfig(file, root);
end
