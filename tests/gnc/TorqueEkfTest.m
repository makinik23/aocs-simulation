classdef TorqueEkfTest < matlab.unittest.TestCase
	% TorqueEkfTest Check standalone disturbance-torque EKF convergence and NIS.
	% All rates are omega_BI^B [rad/s]; torques are body-frame vectors [N*m].
	% Synthetic unbiased gyro samples are independent of estimator truth.
	% Run: results = runtests("tests/gnc/TorqueEkfTest.m");

	methods (TestClassSetup)
		function addFilterPath(testCase)
			root = fileparts(fileparts(fileparts(mfilename("fullpath"))));
			testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
				fullfile(root, "src", "gnc", "control")));
		end
	end

	methods (Test)
		function disabledAdapterClearsState(testCase)
			config = defaultDisturbanceObserverConfig();
			config.Enabled = false;
			input = ones(50, 1);

			output = runPackedAdapter(input, eye(3), 0.1, config);

			testCase.verifyEqual(output, zeros(47, 1), AbsTol=1e-15);
		end

		function packedParametersMatchAdapter(testCase)
			config = defaultDisturbanceObserverConfig();
			inertia = diag([0.01, 0.012, 0.015]);
			dt_s = 0.1;
			input = [[0.01; -0.02; 0.03]; [1; -2; 3]*1e-6; 1; zeros(43, 1)];
			parameters = [inertia(:); dt_s; config.Enabled; config.TorqueRW_Nm2_s; ...
				config.RateSigma_rad_s; config.InitialTorqueSigma_Nm; ...
				config.NISLimit; config.MaxEstimate_Nm];

			output = torqueEkf([input; parameters]);
			resetCovariance = diag([repmat(config.RateSigma_rad_s^2, 3, 1); ...
				repmat(config.InitialTorqueSigma_Nm^2, 3, 1)]);
			expected = [input(1:3); zeros(3, 1); resetCovariance(:); 1; zeros(3, 1); 1];

			testCase.verifyEqual(output, expected, AbsTol=1e-15);
			testCase.verifyEqual(output(1:3), input(1:3), AbsTol=1e-15);
			testCase.verifyEqual(output(47), 1, AbsTol=1e-15);
		end

		function inactiveAdapterClearsState(testCase)
			config = defaultDisturbanceObserverConfig();
			input = [ones(6, 1); 0; ones(43, 1)];

			output = runPackedAdapter(input, eye(3), 0.1, config);

			testCase.verifyEqual(output, zeros(47, 1), AbsTol=1e-15);
		end

		function recoversCorruptedAdapterState(testCase)
			config = defaultDisturbanceObserverConfig();
			input = [zeros(6, 1); 1; NaN(43, 1)];

			output = runPackedAdapter(input, eye(3), 0.1, config);

			testCase.verifyTrue(all(isfinite(output)));
			testCase.verifyEqual(output(47), 1, AbsTol=1e-15);
		end

		function dynamicsOraclesRemainConsistent(testCase)
			verifyDynamicsOracles(testCase);
		end

		function dropoutPredictsAndBadInertiaHolds(testCase)
			config = createConfiguration();
			state = [0.01; -0.02; 0.03; 1e-7; -2e-7; 3e-7];
			invalidConfig = config;
			invalidConfig.J = -config.J;

			[~, predicted, covariance, dropoutValid] = advanceFilter( ...
				state, config.P0, NaN(3, 1), config, Inf, false);
			[badTorque, held, heldCovariance, badValid] = advanceFilter( ...
				state, config.P0, zeros(3, 1), invalidConfig, Inf, true);

			testCase.verifyFalse(dropoutValid);
			testCase.verifyTrue(all(isfinite(predicted)));
			testCase.verifyTrue(all(isfinite(covariance), "all"));
			testCase.verifyGreaterThan(norm(predicted-state), 1e-10);
			testCase.verifyFalse(badValid);
			testCase.verifyEqual(badTorque, zeros(3, 1), AbsTol=1e-15);
			testCase.verifyEqual(held, state, AbsTol=1e-15);
			testCase.verifyEqual(heldCovariance, config.P0, AbsTol=1e-15);
		end

		function convergesToConstantBodyTorque(testCase)
			config = createConfiguration();
			stream = RandStream("mt19937ar", Seed=42);

			metrics = simulateConstantTorque(config, stream);

			% Last 60 s: vector RMS < 0.1 microN*m and > 5-fold improvement.
			testCase.verifyTrue(metrics.AllAccepted);
			testCase.verifyTrue(metrics.CovarianceHealthy);
			testCase.verifyTrue(metrics.StateFinite);
			testCase.verifyLessThan(metrics.TorqueRms_Nm, 1e-7);
			testCase.verifyLessThan(metrics.TorqueRms_Nm, metrics.InitialError_Nm/5);
			fprintf("Torque EKF: tail RMS %.4f microN*m (initial %.4f).\n", ...
				1e6*metrics.TorqueRms_Nm, 1e6*metrics.InitialError_Nm);
		end

		function nisMatchesThreeMeasurements(testCase)
			config = createConfiguration();
			stream = RandStream("mt19937ar", Seed=73);
			sampleCount = 1000;
			falseAlarmProbability = 0.001;

			metrics = simulateMatchedInnovations(config, stream, sampleCount);

			% Under the matched linear-Gaussian model, sum(NIS) ~ chi2(3*N).
			probabilities = [falseAlarmProbability/2, 1-falseAlarmProbability/2];
			meanLimits = 2*gammaincinv(probabilities, 3*sampleCount/2)/sampleCount;

			testCase.verifyTrue(metrics.AllAccepted);
			testCase.verifyTrue(metrics.CovarianceHealthy);
			testCase.verifyTrue(all(isfinite(metrics.Nis)));
			testCase.verifyGreaterThanOrEqual(min(metrics.Nis), 0);
			testCase.verifyGreaterThan(mean(metrics.Nis), meanLimits(1));
			testCase.verifyLessThan(mean(metrics.Nis), meanLimits(2));
			fprintf("Torque EKF: mean NIS %.4f; 99.9%% limits [%.4f, %.4f], expected 3.\n", ...
				mean(metrics.Nis), meanLimits(1), meanLimits(2));
		end

		function rejectsMeasurementAboveNisGate(testCase)
			config = createConfiguration();
			state = zeros(6, 1);
			measurement = [0.1; -0.2; 0.3]; % Deliberate gyro outlier [rad/s].
			nisLimit = 25;

			[predictedTorque, predictedState, predictedCovariance] = ...
				advanceFilter(state, config.P0, measurement, config, Inf, false);
			[actualTorque, actualState, actualCovariance, isValid] = ...
				advanceFilter(state, config.P0, measurement, config, nisLimit, true);
			nis = computeNis(measurement, predictedState, predictedCovariance, config.R);

			testCase.verifyGreaterThan(nis, nisLimit);
			testCase.verifyFalse(isValid);
			testCase.verifyEqual(actualTorque, predictedTorque, AbsTol=1e-15);
			testCase.verifyEqual(actualState, predictedState, AbsTol=1e-15);
			testCase.verifyEqual(actualCovariance, predictedCovariance, AbsTol=1e-15);
		end
	end
end

function output = runPackedAdapter(input, inertia, dt_s, config)
	% runPackedAdapter Assemble the documented numeric Simulink input port.
	parameters = [inertia(:); dt_s; config.Enabled; config.TorqueRW_Nm2_s; ...
		config.RateSigma_rad_s; config.InitialTorqueSigma_Nm; ...
		config.NISLimit; config.MaxEstimate_Nm];
	output = torqueEkf([input; parameters]);
end

function verifyDynamicsOracles(testCase)
	% verifyDynamicsOracles Preserve the former standalone mathematical checks.
	inertia = diag([0.01, 0.012, 0.015]);
	dt_s = 0.05;
	rate = [0.12; -0.08; 0.03];
	torque = [1e-5; -2e-5; 0.5e-5];
	state = [rate; 2e-5; 3e-6; -1e-5];
	skew = @(a) [0, -a(3), a(2); a(3), 0, -a(1); -a(2), a(1), 0];
	dynamics = @(x) [inertia\(torque+x(4:6)-cross(x(1:3), inertia*x(1:3))); zeros(3, 1)];
	jacobian = [inertia\(skew(inertia*rate)-skew(rate)*inertia), inertia\eye(3); zeros(3, 6)];
	numericJacobian = zeros(6);
	step = 1e-7;
	for column = 1:6
		delta = zeros(6, 1);
		delta(column) = step;
		numericJacobian(:, column) = (dynamics(state+delta)-dynamics(state-delta))/(2*step);
	end

	testCase.verifyLessThan(norm(jacobian-numericJacobian, "fro")/norm(jacobian, "fro"), 1e-8);
	measurementMatrix = [eye(3), zeros(3)];
	testCase.verifyEqual(rank([measurementMatrix; measurementMatrix*jacobian]), 6);

	spectralDensity = 1e-10;
	axisJacobian = [0, 1/inertia(1, 1); 0, 0];
	axisNoise = [0, 0; 0, spectralDensity];
	vanLoan = expm([axisJacobian, axisNoise; zeros(2), -axisJacobian.']*dt_s);
	actualNoise = vanLoan(1:2, 3:4)*vanLoan(1:2, 1:2).';
	expectedNoise = spectralDensity*[dt_s^3/(3*inertia(1, 1)^2), dt_s^2/(2*inertia(1, 1)); ...
		dt_s^2/(2*inertia(1, 1)), dt_s];

	testCase.verifyEqual(actualNoise, expectedNoise, AbsTol=1e-20);
end

function config = createConfiguration()
	% createConfiguration Define a small synthetic rigid body and gyro tuning.
	config = struct( ...
		"J", diag([0.01, 0.012, 0.015]), ... % [kg*m^2]
		"Dt_s", 0.5, ...
		"AppliedTorque_Nm", [1; -0.5; 0.25]*1e-6, ...
		"TorqueSpectralDensity", 1e-16*eye(3), ... % [(N*m)^2/s]
		"R", deg2rad(0.02)^2*eye(3), ... % Gyro covariance [(rad/s)^2].
		"P0", diag([repmat(deg2rad(0.02)^2, 3, 1); repmat((2e-6)^2, 3, 1)]));
end

function metrics = simulateConstantTorque(config, stream)
	% simulateConstantTorque Integrate independent nonlinear Euler-equation truth.
	% Constant body torque approximates disturbances over a short interval;
	% it does not reproduce attitude-dependent orbital environmental torques.
	time_s = (0:config.Dt_s:600).';
	trueTorque_Nm = [0.2; -0.3; 0.4]*1e-6;
	initialRate = [0.01; -0.008; 0.006];
	options = odeset(RelTol=1e-10, AbsTol=1e-12);
	[~, trueRate] = ode45(@(~, rate) config.J\(config.AppliedTorque_Nm + ...
		trueTorque_Nm - cross(rate, config.J*rate)), time_s, initialRate, options);
	measurement = trueRate.' + chol(config.R, "lower")*randn(stream, 3, numel(time_s));
	output = callFilter(zeros(6, 1), config.P0, measurement(:, 1), config, Inf, true);
	state = output(1:6);
	covariance = reshape(output(7:42), 6, 6);
	torqueError = zeros(3, numel(time_s));
	torqueError(:, 1) = state(4:6) - trueTorque_Nm;
	allAccepted = true;
	covarianceHealthy = true;
	stateFinite = true;

	for sample = 2:numel(time_s)
		[torque, state, covariance, isValid] = advanceFilter( ...
			state, covariance, measurement(:, sample), config, Inf, true);
		torqueError(:, sample) = torque - trueTorque_Nm;
		allAccepted = allAccepted && isValid;
		covarianceHealthy = covarianceHealthy && isHealthyCovariance(covariance);
		stateFinite = stateFinite && all(isfinite(state));
	end

	tailError = torqueError(:, time_s >= 540);
	metrics = struct("TorqueRms_Nm", sqrt(mean(sum(tailError.^2, 1))), ...
		"InitialError_Nm", norm(torqueError(:, 1)), "AllAccepted", allAccepted, ...
		"CovarianceHealthy", covarianceHealthy, "StateFinite", stateFinite);
end

function metrics = simulateMatchedInnovations(config, stream, sampleCount)
	% simulateMatchedInnovations Use an exactly matched linear-Gaussian plant.
	% Spherical inertia removes omega x J*omega exactly. Sample the full Qd
	% jointly, including rate/torque cross covariance. No gating or burn-in.
	inertia = 0.01; % [kg*m^2]
	config.J = inertia*eye(3);
	config.AppliedTorque_Nm = zeros(3, 1);
	dt = config.Dt_s;
	spectralDensity = config.TorqueSpectralDensity(1, 1);
	transition = [eye(3), (dt/inertia)*eye(3); zeros(3), eye(3)];
	processCovariance = spectralDensity*[ ...
		(dt^3/(3*inertia^2))*eye(3), (dt^2/(2*inertia))*eye(3); ...
		(dt^2/(2*inertia))*eye(3), dt*eye(3)];
	processFactor = chol(processCovariance, "lower");
	measurementFactor = chol(config.R, "lower");
	state = zeros(6, 1);
	covariance = config.P0;
	truth = state + chol(covariance, "lower")*randn(stream, 6, 1);
	nis = zeros(1, sampleCount);
	allAccepted = true;
	covarianceHealthy = true;

	for sample = 1:sampleCount
		truth = transition*truth + processFactor*randn(stream, 6, 1);
		measurement = truth(1:3) + measurementFactor*randn(stream, 3, 1);
		% Both calls use the same posterior: prediction is not advanced twice.
		[~, prediction, predictedCovariance] = advanceFilter( ...
			state, covariance, measurement, config, Inf, false);
		nis(sample) = computeNis(measurement, prediction, predictedCovariance, config.R);
		[~, state, covariance, isValid] = advanceFilter( ...
			state, covariance, measurement, config, Inf, true);
		allAccepted = allAccepted && isValid;
		covarianceHealthy = covarianceHealthy && isHealthyCovariance(covariance);
	end

	metrics = struct("Nis", nis, "AllAccepted", allAccepted, ...
		"CovarianceHealthy", covarianceHealthy);
end

function [torque, stateNext, covarianceNext, isValid] = advanceFilter( ...
		state, covariance, measurement, config, nisLimit, measurementValid)
	% advanceFilter Exercise the same public interface as the Simulink model.
	if ~measurementValid
		measurement = NaN(3, 1);
	end
	output = callFilter(state, covariance, measurement, config, nisLimit, false);
	torque = output(44:46);
	stateNext = output(1:6);
	covarianceNext = reshape(output(7:42), 6, 6);
	isValid = logical(output(47));
end

function output = callFilter(state, covariance, measurement, config, nisLimit, reset)
	% callFilter Pack a controlled fixture; Inf disables compensation clipping.
	parameters = [config.J(:); config.Dt_s; 1; config.TorqueSpectralDensity(1, 1); ...
		sqrt(config.R(1, 1)); sqrt(config.P0(4, 4)); nisLimit; Inf];
	retained = [state; covariance(:); double(~reset)];
	output = torqueEkf([measurement; config.AppliedTorque_Nm; 1; retained; parameters]);
end

function nis = computeNis(measurement, prediction, covariance, rateCovariance)
	% computeNis Compute nu'*S^(-1)*nu from the public prior, without inv().
	innovation = measurement - prediction(1:3);
	innovationCovariance = covariance(1:3, 1:3) + rateCovariance;
	nis = innovation.'*(innovationCovariance\innovation);
end

function isHealthy = isHealthyCovariance(covariance)
	% isHealthyCovariance Check finite, symmetric, positive definite covariance.
	[~, flag] = chol(covariance);
	isHealthy = all(isfinite(covariance), "all") && flag == 0 && ...
		norm(covariance-covariance.', "fro") <= 1e-12*norm(covariance, "fro");
end
