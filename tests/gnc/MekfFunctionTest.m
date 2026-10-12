classdef MekfFunctionTest < matlab.unittest.TestCase
	% Checks the standalone MEKF state, rejection and gyro history contracts.

	properties (Access = private)
		Config
	end

	properties (TestParameter)
		rejectedGyro = struct( ...
			'repeatedReceipt', struct('field', 'receive_time_s', 'value', 0), ...
			'backwardReceipt', struct('field', 'receive_time_s', 'value', -1), ...
			'nonfiniteReceipt', struct('field', 'receive_time_s', 'value', Inf), ...
			'nonfiniteRate', struct('field', 'omega_rad_s', 'value', [NaN; 0; 0]), ...
			'invalidReport', struct('field', 'valid', 'value', false), ...
			'nanValidity', struct('field', 'valid', 'value', NaN))
	end

	methods (TestClassSetup)
		function prepareEnvironment(testCase)
			% Loads the production paths, buses and numeric configuration.

			setupAocsPaths([], true);
			[~, variables] = setupAocsSimulation("", false);
			testCase.Config = variables.AOCS_GNCConfig.Value;
		end
	end

	methods (Test)
		function initializationExcludesPredictionAndIgnoresLaterTriad(testCase)
			% Captures the first TRIAD solution without predicting on its tick.

			cfg = testCase.Config;
			[state, reports, status, initialization, references] = fixture(cfg);
			initialization.valid = true;
			initialization.q_BI = rotationQuaternion([0.2; -0.1; 0.3]);
			reports.Gyro.omega_rad_s = [0.1; 0.2; -0.3];

			[first, health, diagnostics] = mekfStep(state, reports, status, ...
				initialization, references, cfg.sample_time_s, cfg);
			initialization.q_BI = rotationQuaternion([-0.6; 0.4; 0.1]);
			initialization.solution_time_s = 123;
			status.Gyro.new_data = false;
			[second, ~, repeated] = mekfStep(first, reports, status, ...
				initialization, references, 2 * cfg.sample_time_s, cfg);

			testCase.verifyTrue(first.estimate.initialized);
			testCase.verifyFalse(first.estimate.valid);
			testCase.verifyTrue(diagnostics.initialized_this_step);
			testCase.verifyFalse(diagnostics.prediction_accepted);
			testCase.verifyFalse(first.have_previous_rate);
			testCase.verifyEqual(first.estimate.q_BI, ...
				rotationQuaternion([0.2; -0.1; 0.3]), 'AbsTol', 1e-15);
			testCase.verifyEqual(first.estimate.update_time_s, 0, 'AbsTol', 1e-15);
			testCase.verifyEqual(first.estimate.P_error, ...
				cfg.MEKF.initial_covariance, 'AbsTol', 1e-15);
			testCase.verifyEqual(first.last_correction_time_s, ...
				cfg.sample_time_s, 'AbsTol', 1e-15);
			testCase.verifyEqual(health.correction_age_s, 0, 'AbsTol', 1e-15);
			testCase.verifyEqual(second, first, 'AbsTol', 1e-15);
			testCase.verifyFalse(repeated.initialized_this_step);
		end

		function rejectedGyroHoldsStateAndHistory(testCase, rejectedGyro)
			% Rejects malformed or nonforward gyro receipts without changing history.

			cfg = testCase.Config;
			[state, reports, status, initialization, references] = fixture(cfg);
			state.estimate.initialized = true;
			state.estimate.valid = true;
			state.estimate.q_BI = rotationQuaternion([0.1; 0.2; 0.3]);
			state.estimate.omega_BI_B_rad_s = [0.2; 0.1; -0.4];
			state.have_previous_rate = true;
			state.have_two_rates = true;
			state.previous_gap_adjacent = true;
			state.previous_rate_B_rad_s = [0.2; -0.3; 0.1];
			state.older_rate_B_rad_s = [-0.1; 0.4; 0.2];
			reports.Gyro.(rejectedGyro.field) = rejectedGyro.value;

			[next, ~, diagnostics] = mekfStep(state, reports, status, ...
				initialization, references, cfg.sample_time_s, cfg);

			expected = state;
			expected.estimate.valid = false;

			testCase.verifyEqual(next, expected, 'AbsTol', 1e-15);
			testCase.verifyFalse(diagnostics.prediction_accepted);
			testCase.verifyFalse(diagnostics.mag_accepted);
			testCase.verifyFalse(diagnostics.sun_accepted);
		end

		function singularInnovationHoldsPredictionAndAllowsRecovery(testCase)
			% Holds a singular vector correction and accepts the next usable one.

			cfg = testCase.Config;
			cfg.MEKF.gyro_noise_density_rad_s_sqrt_Hz = 0;
			cfg.MEKF.bias_random_walk_std_rad_s_sqrt_s = 0;
			cfg.MEKF.magnetometer_direction_std_rad = 0;
			[state, reports, status, initialization, references] = fixture(cfg);
			state.estimate.initialized = true;
			state.estimate.P_error = zeros(6);
			reports.Magnetometer.B_B_T = [1; 0; 0];
			reports.Magnetometer.valid = true;
			reports.Magnetometer.receive_time_s = cfg.sample_time_s;
			references.B_I_T = [1; 0; 0];
			references.valid = true;
			references.orbit_update_time_s = cfg.sample_time_s;
			status.Magnetometer.has_data = true;
			status.Magnetometer.new_data = true;
			disabled = status;
			disabled.Magnetometer.new_data = false;

			[rejected, ~, diagnostics] = mekfStep(state, reports, status, ...
				initialization, references, cfg.sample_time_s, cfg);
			[predicted, ~, ~] = mekfStep(state, reports, disabled, ...
				initialization, references, cfg.sample_time_s, cfg);
			cfg.MEKF.magnetometer_direction_std_rad = 0.01;
			reports.Gyro.receive_time_s = 2 * cfg.sample_time_s;
			reports.Magnetometer.receive_time_s = 2 * cfg.sample_time_s;
			references.orbit_update_time_s = 2 * cfg.sample_time_s;
			[recovered, ~, recovery] = mekfStep(rejected, reports, status, ...
				initialization, references, 2 * cfg.sample_time_s, cfg);

			testCase.verifyEqual(rejected, predicted, 'AbsTol', 1e-15);
			testCase.verifyTrue(diagnostics.prediction_accepted);
			testCase.verifyFalse(diagnostics.mag_accepted);
			testCase.verifyEqual(diagnostics.mag_nis, 0, 'AbsTol', 1e-15);
			testCase.verifyTrue(recovery.mag_accepted);
			testCase.verifyTrue(recovered.estimate.valid);
			testCase.verifyTrue(all(isfinite(recovered.estimate.P_error(:))));
			testCase.verifyEqual(recovered.estimate.q_BI, [1; 0; 0; 0], ...
				'AbsTol', 1e-15);
		end

		function coningHistoryFreezesAndRestartsAfterGap(testCase)
			% Checks adjacent quadrature, frozen history and midpoint restart.

			cfg = testCase.Config;
			[state, reports, status, initialization, references] = fixture(cfg);
			state.estimate.initialized = true;
			state.estimate.q_BI = rotationQuaternion([0.4; -0.1; 0.2]);
			dt = cfg.sample_time_s;
			w1 = [0.2; 0.1; 1.5];
			w2 = [0.4; -0.2; 1.2];
			w3 = [-0.1; 0.3; 1.4];
			wGap = [0.1; -0.2; 1.1];
			wRestart = [0.3; 0.2; 1.3];
			reports.Gyro.omega_rad_s = w1 + state.estimate.gyro_bias_B_rad_s;

			[first, ~, ~] = mekfStep(state, reports, status, ...
				initialization, references, dt, cfg);
			reports.Gyro.omega_rad_s = w2 + first.estimate.gyro_bias_B_rad_s;
			reports.Gyro.receive_time_s = 2 * dt;
			[second, ~, ~] = mekfStep(first, reports, status, ...
				initialization, references, 2 * dt, cfg);
			reports.Gyro.omega_rad_s = w3 + second.estimate.gyro_bias_B_rad_s;
			reports.Gyro.receive_time_s = 3 * dt;
			[third, ~, ~] = mekfStep(second, reports, status, ...
				initialization, references, 3 * dt, cfg);
			status.Gyro.new_data = false;
			reports.Gyro.omega_rad_s = [5; -3; 2];
			reports.Gyro.receive_time_s = 4 * dt;
			[frozen, ~, rejection] = mekfStep(third, reports, status, ...
				initialization, references, 4 * dt, cfg);
			status.Gyro.new_data = true;
			reports.Gyro.omega_rad_s = wGap + frozen.estimate.gyro_bias_B_rad_s;
			reports.Gyro.receive_time_s = 5 * dt;
			[gap, ~, ~] = mekfStep(frozen, reports, status, ...
				initialization, references, 5 * dt, cfg);
			reports.Gyro.omega_rad_s = wRestart + gap.estimate.gyro_bias_B_rad_s;
			reports.Gyro.receive_time_s = 6 * dt;
			[restarted, ~, ~] = mekfStep(gap, reports, status, ...
				initialization, references, 6 * dt, cfg);

			q1 = quatmultiply(state.estimate.q_BI.', rotationQuaternion(dt * w1).').';
			theta2 = dt * (0.5 * (w1 + w2) + dt / 12 * cross(w1, w2));
			q2 = quatmultiply(q1.', rotationQuaternion(theta2).').';
			theta3 = dt * ((5 * w3 + 8 * w2 - w1) / 12 ...
				+ dt / 12 * cross(w2, w3));
			q3 = quatmultiply(q2.', rotationQuaternion(theta3).').';
			qGap = quatmultiply(q3.', rotationQuaternion(2 * dt * wGap).').';
			thetaRestart = dt * (0.5 * (wGap + wRestart) ...
				+ dt / 12 * cross(wGap, wRestart));
			qRestart = quatmultiply(qGap.', rotationQuaternion(thetaRestart).').';
			expectedFrozen = third;
			expectedFrozen.estimate.valid = false;

			testCase.verifyEqual(third.estimate.q_BI, q3, 'AbsTol', 2e-12);
			testCase.verifyEqual(frozen, expectedFrozen, 'AbsTol', 1e-15);
			testCase.verifyFalse(rejection.prediction_accepted);
			testCase.verifyEqual(gap.estimate.q_BI, qGap, 'AbsTol', 2e-12);
			testCase.verifyFalse(gap.previous_gap_adjacent);
			testCase.verifyEqual(gap.older_rate_B_rad_s, w3, 'AbsTol', 1e-15);
			testCase.verifyEqual(restarted.estimate.q_BI, qRestart, 'AbsTol', 2e-12);
			testCase.verifyEqual(restarted.previous_rate_B_rad_s, ...
				wRestart, 'AbsTol', 1e-15);
			testCase.verifyTrue(restarted.previous_gap_adjacent);
		end
	end
end

function [state, reports, status, initialization, references] = fixture(cfg)
	% Creates typed reports with only the gyro marked fresh and usable.

	state = defaultMekfState(cfg.MEKF);
	reports = Simulink.Bus.createMATLABStruct('SensorReportBus');
	status = Simulink.Bus.createMATLABStruct('SensorReadStatusBus');
	initialization = Simulink.Bus.createMATLABStruct('AttitudeInitializationBus');
	references = Simulink.Bus.createMATLABStruct('ReferenceVectorBus');
	reports.Gyro.omega_rad_s = cfg.MEKF.initial_bias_B_rad_s;
	reports.Gyro.receive_time_s = cfg.sample_time_s;
	reports.Gyro.valid = true;
	status.Gyro.has_data = true;
	status.Gyro.new_data = true;
end

function q = rotationQuaternion(rotationVector)
	% Forms an independent axis-angle quaternion for nonzero test rotations.

	angle = norm(rotationVector);
	q = [cos(angle / 2); sin(angle / 2) * rotationVector / angle];
end
