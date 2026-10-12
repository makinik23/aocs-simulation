classdef OrbitNavigationTest < matlab.unittest.TestCase
    % Verify orbital EKF initialization, prediction, correction and diagnostics.

    methods (TestClassSetup)
        function addAlgorithmPath(testCase)
            root = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, "src", "gnc", "navigation", "orbit"), ...
                IncludingSubfolders=true));
        end
    end

    methods (Test)
        function waitsForValidFix(testCase)
            cfg = fixtureConfig();

            measurement = fixtureMeasurement();
            measurement.valid = false;

            [next, nav, health] = orbitNavigationStep( ...
                cfg.initial_state, measurement, true, 30, false, cfg);

            testCase.verifyEqual(next, cfg.initial_state);
            testCase.verifyFalse(nav.valid);
            testCase.verifyEqual(health.mode, uint8(0));
            testCase.verifyTrue(health.algorithm_ready);
            testCase.verifyFalse(health.nis_available);
        end

        function initializesWithoutCorrectingTheSameFix(testCase)
            cfg = fixtureConfig();
            measurement = fixtureMeasurement();

            [next, nav, health] = orbitNavigationStep( ...
                cfg.initial_state, measurement, true, 30, false, cfg);

            testCase.verifyTrue(next.initialized);
            testCase.verifyEqual(next.x_hat, measurement.z_gnss, AbsTol=1e-10);
            testCase.verifyEqual(next.P, cfg.gnss_covariance_RTN, AbsTol=1e-12);
            testCase.verifyEqual(next.epoch_s, 30, AbsTol=1e-12);
            testCase.verifyEqual(next.last_processed_sequence_id, uint32(31));
            testCase.verifyEqual(next.solution_id, uint32(1));
            testCase.verifyTrue(nav.valid);
            testCase.verifyEqual(nav.covariance, next.P, AbsTol=1e-12);
            testCase.verifyEqual(nav.state_source, uint8(1));
            testCase.verifyEqual(health.mode, uint8(1));
            testCase.verifyEqual(health.gnss_age_s, 0, AbsTol=1e-12);
            testCase.verifyFalse(health.nis_available);
        end

        function rejectsDegenerateInitialOrbit(testCase)
            cfg = fixtureConfig();
        
            measurement = fixtureMeasurement();
            measurement.z_gnss(4:6) = [100; 0; 0];
      
            [next, nav, ~] = orbitNavigationStep( ...
                cfg.initial_state, measurement, true, 30, false, cfg);

            testCase.verifyFalse(next.initialized);
            testCase.verifyFalse(nav.valid);
        end

        function rejectsNonfiniteFix(testCase)
            cfg = fixtureConfig();

            measurement = fixtureMeasurement();
            measurement.z_gnss(1) = NaN;

            [next, nav, health] = orbitNavigationStep( ...
                cfg.initial_state, measurement, true, 30, false, cfg);

            testCase.verifyEqual(next, cfg.initial_state);
            testCase.verifyFalse(nav.valid);
            testCase.verifyFalse(health.nis_available);
        end

        function predictsWithoutGnss(testCase)
            cfg = fixtureConfig();

            previous = initializedState(cfg);

            measurement = fixtureMeasurement();
            measurement.valid = false;

            [next, nav, health] = orbitNavigationStep( ...
                previous, measurement, false, 30.1, false, cfg);

            distance = norm(next.x_hat(1:3) - previous.x_hat(1:3));
            testCase.verifyGreaterThan(distance, 749);
            testCase.verifyLessThan(distance, 751);
            testCase.verifyEqual(next.epoch_s, 30.1, AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(next.P - previous.P, 'fro'), 1e-8);
            testCase.verifyEqual(next.P, next.P.', AbsTol=1e-12);
            testCase.verifyGreaterThan(min(eig(next.P)), 0);
            testCase.verifyEqual(next.last_processed_sequence_id, uint32(31));
            testCase.verifyEqual(next.last_accepted_gnss_time_s, 30, AbsTol=1e-12);
            testCase.verifyEqual(next.solution_id, uint32(2));
            testCase.verifyTrue(nav.valid);
            testCase.verifyEqual(nav.state_source, uint8(2));
            testCase.verifyEqual(health.mode, uint8(2));
            testCase.verifyEqual(health.gnss_age_s, 0.1, AbsTol=1e-12);
            testCase.verifyFalse(health.nis_available);
        end

        function correctsAndReportsPriorInnovationNis(testCase)
            cfg = fixtureConfig();

            previous = initializedState(cfg);
            measurement = fixtureMeasurement();

            innovation = [2; 0; 0; 0; 0.2; 0];

            measurement.z_gnss = previous.x_hat + innovation;
            measurement.sequence_id = uint32(32);
            [next, nav, health] = orbitNavigationStep( ...
                previous, measurement, true, 30, false, cfg);

            testCase.verifyEqual(next.x_hat, previous.x_hat + innovation/2, AbsTol=1e-10);
            testCase.verifyEqual(next.P, previous.P/2, AbsTol=1e-12);
            testCase.verifyEqual(next.last_processed_sequence_id, uint32(32));
            testCase.verifyEqual(next.solution_id, uint32(2));
            testCase.verifyTrue(nav.valid);
            testCase.verifyTrue(health.nis_available);
            testCase.verifyEqual(health.nis, 2.5, AbsTol=1e-9);
        end

        function highNisDoesNotRejectCorrection(testCase)
            cfg = fixtureConfig();

            previous = initializedState(cfg);

            measurement = fixtureMeasurement();
            measurement.z_gnss(1) = previous.x_hat(1) + 1000;
            measurement.sequence_id = uint32(32);

            [next, nav, health] = orbitNavigationStep( ...
                previous, measurement, true, 30, false, cfg);

            testCase.verifyTrue(health.nis_available);
            testCase.verifyGreaterThan(health.nis, 1e5);
            testCase.verifyEqual(next.x_hat(1), previous.x_hat(1) + 500, AbsTol=1e-8);
            testCase.verifyEqual(next.P, previous.P/2, AbsTol=1e-12);
            testCase.verifyEqual(next.last_processed_sequence_id, uint32(32));
            testCase.verifyTrue(nav.valid);
        end

        function unavailableNisDoesNotBlockNumericallyValidCorrection(testCase)
            cfg = fixtureConfig();
            cfg.gnss_covariance_RTN = 1e-310 * eye(6);

            previous = initializedState(cfg);

            measurement = fixtureMeasurement();
            measurement.z_gnss(1) = previous.x_hat(1) + 1e4;
            measurement.sequence_id = uint32(32);

            [next, nav, health] = orbitNavigationStep( ...
                previous, measurement, true, 30, false, cfg);

            testCase.verifyFalse(health.nis_available);
            testCase.verifyEqual(next.x_hat(1), previous.x_hat(1) + 5000, AbsTol=1e-7);
            testCase.verifyTrue(nav.valid);
            testCase.verifyEqual(next.solution_id, uint32(2));
        end

        function duplicateFixDoesNotChangeStateOrCovariance(testCase)
            cfg = fixtureConfig();

            previous = initializedState(cfg);

            measurement = fixtureMeasurement();
            measurement.z_gnss(1) = previous.x_hat(1) + 1000;

            [next, nav, health] = orbitNavigationStep( ...
                previous, measurement, true, 30, false, cfg);

            testCase.verifyEqual(next, previous);
            testCase.verifyTrue(nav.valid);
            testCase.verifyFalse(health.nis_available);
        end

        function staleFixDoesNotCorrectCurrentState(testCase)
            cfg = fixtureConfig();

            previous = initializedState(cfg);

            measurement = fixtureMeasurement();
            measurement.measurement_time_s = 29;
            measurement.sequence_id = uint32(32);

            [next, nav, health] = orbitNavigationStep( ...
                previous, measurement, true, 30, false, cfg);

            testCase.verifyEqual(next, previous);
            testCase.verifyTrue(nav.valid);
            testCase.verifyFalse(health.nis_available);
        end

        function futureFixDoesNotInitialize(testCase)
            cfg = fixtureConfig();

            measurement = fixtureMeasurement();
            measurement.measurement_time_s = 31;

            [next, nav, health] = orbitNavigationStep( ...
                cfg.initial_state, measurement, true, 30, false, cfg);

            testCase.verifyEqual(next, cfg.initial_state);
            testCase.verifyFalse(nav.valid);
            testCase.verifyFalse(health.nis_available);
        end

        function invalidMeasurementCovariancePreservesPrior(testCase)
            cfg = fixtureConfig();
            previous = initializedState(cfg);
            cfg.gnss_covariance_RTN(1, 1) = -1;

            measurement = fixtureMeasurement();
            measurement.sequence_id = uint32(32);

            [next, nav, health] = orbitNavigationStep( ...
                previous, measurement, true, 30, false, cfg);

            testCase.verifyEqual(next.x_hat, previous.x_hat, AbsTol=1e-10);
            testCase.verifyEqual(next.P, previous.P, AbsTol=1e-12);
            testCase.verifyEqual(next.solution_id, previous.solution_id);
            testCase.verifyEqual(next.last_processed_sequence_id, uint32(32));
            testCase.verifyTrue(nav.valid);
            testCase.verifyFalse(health.nis_available);
        end

        function longGapDoesNotAdvanceEpochWithoutPrediction(testCase)
            cfg = fixtureConfig();

            previous = initializedState(cfg);

            measurement = fixtureMeasurement();
            measurement.valid = false;

            [next, nav, health] = orbitNavigationStep( ...
                previous, measurement, false, 32, false, cfg);

            testCase.verifyEqual(next, previous);
            testCase.verifyFalse(nav.valid);
            testCase.verifyEqual(health.mode, uint8(3));
        end

        function corruptPriorIsInvalid(testCase)
            cfg = fixtureConfig();

            previous = initializedState(cfg);
            previous.P(1, 1) = -1;

            measurement = fixtureMeasurement();

            [next, nav, health] = orbitNavigationStep( ...
                previous, measurement, false, 30, false, cfg);

            testCase.verifyEqual(next, previous);
            testCase.verifyFalse(nav.valid);
            testCase.verifyEqual(health.mode, uint8(3));
        end

        function resetHasPriorityOverNewFix(testCase)
            cfg = fixtureConfig();
            
            previous = initializedState(cfg);

            measurement = fixtureMeasurement();
            measurement.sequence_id = uint32(32);

            [next, nav, health] = orbitNavigationStep( ...
                previous, measurement, true, 30, true, cfg);

            testCase.verifyEqual(next, cfg.initial_state);
            testCase.verifyFalse(nav.valid);
            testCase.verifyEqual(health.gnss_age_s, -1, AbsTol=1e-12);
            testCase.verifyFalse(health.nis_available);
            testCase.verifyTrue(health.algorithm_ready);
        end

        function solutionIdWrapsWithoutSaturation(testCase)
            cfg = fixtureConfig();

            previous = initializedState(cfg);
            previous.solution_id = intmax('uint32');

            measurement = fixtureMeasurement();
            measurement.valid = false;

            [next, nav, ~] = orbitNavigationStep( ...
                previous, measurement, false, 30.1, false, cfg);

            testCase.verifyEqual(next.solution_id, uint32(0));
            testCase.verifyTrue(nav.valid);
            testCase.verifyEqual(nav.solution_id, next.solution_id);
        end
    end
end

function cfg = fixtureConfig()
%FIXTURECONFIG Create deterministic navigation parameters.

aocs.GNC.SampleTime_s = 0.1;

aocs.Orbit.CentralBodyConstants.mu_m3_s2 = 3.986004418e14;
aocs.Orbit.CentralBodyConstants.radius_m = 6378137;

aocs.Epoch.Utc = [2026; 1; 1; 0; 0; 0];
aocs.Environment.EarthOrientation = struct('DeltaAT_s', 37, ...
    'DeltaUT1_s', 0, 'PolarMotion_rad', [0 0], 'DCIP_rad', [0 0]);

aocs.Sensors.GNSS.AntennaNoiseScale = 1;
aocs.Sensors.GNSS.WhiteNoise.PositionStd_RTN_m = [1; 2; 3];
aocs.Sensors.GNSS.WhiteNoise.VelocityStd_RTN_m_s = [0.1; 0.2; 0.3];
aocs.Sensors.GNSS.GaussMarkov.PositionStd_RTN_m = zeros(3, 1);
aocs.Sensors.GNSS.GaussMarkov.VelocityStd_RTN_m_s = zeros(3, 1);

cfg = defaultOrbitNavigationConfig(aocs);

end

function state = initializedState(cfg)
%INITIALIZEDSTATE Create a valid prior at the measurement epoch.

measurement = fixtureMeasurement();

state = cfg.initial_state;
state.x_hat = measurement.z_gnss;
state.P = cfg.gnss_covariance_RTN;
state.epoch_s = measurement.measurement_time_s;
state.initialized = true;
state.last_accepted_gnss_time_s = measurement.measurement_time_s;
state.last_processed_sequence_id = measurement.sequence_id;
state.solution_id = uint32(1);

end

function measurement = fixtureMeasurement()
%FIXTUREMEASUREMENT Create a valid GNSS fix.

measurement = struct('z_gnss', [7e6; 0; 0; 0; 7500; 0], ...
    'measurement_time_s', 30, 'sequence_id', uint32(31), 'valid', true);

end
