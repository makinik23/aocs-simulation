function [report, outputDirectory] = validateTorqueEkfEnvironment(simulationFile, options)
% validateTorqueEkfEnvironment Replay the torque EKF on an archived full plant.
%   [REPORT,FOLDER] = validateTorqueEkfEnvironment(SIMULATIONFILE) compares
%   exact body rates, independent noisy rates, and the MEKF corrected gyro.
%   SIMULATIONFILE must contain out and AOCS from run_nadir_pointing_demo.
%   Disturbance truth is used only for scoring, never as an EKF input.
%   All rates are omega_BI^B [rad/s], torques are in body axes B [N*m].
%   NIS is computed before gating, using the public prediction-only call.
%   Its chi-square coverage is diagnostic: the environment is not a matched
%   random walk, and MEKF-derived measurement errors may be correlated.
arguments
	simulationFile (1,1) string {mustBeFile}
	options.TailDuration_s (1,1) double {mustBeFinite, mustBePositive} = 120
	options.NoiseSeed (1,1) double {mustBeInteger, mustBeNonnegative} = 73
end

saved = load(simulationFile, "out", "AOCS");
config = defaultDisturbanceObserverConfig();
history = readHistory(saved.out, saved.AOCS);
stream = RandStream("mt19937ar", Seed=options.NoiseSeed);
noisyRate = history.TrueRate + config.RateSigma_rad_s*randn(stream, size(history.TrueRate));
labels = ["TrueRate", "IndependentGyro", "MEKF"];
measurements = {history.TrueRate, noisyRate, history.MekfRate};
traces = cell(1, numel(labels));
rows = cell(1, numel(labels));

for index = 1:numel(labels)
	traces{index} = replayFilter(history, measurements{index}, config);
	rows{index} = scoreReplay(history, traces{index}, measurements{index}, ...
		labels(index), options.TailDuration_s);
end

report = struct2table([rows{:}]);
outputDirectory = string(tempname(fileparts(simulationFile)));
mkdir(outputDirectory);
metadata = struct("SimulationFile", simulationFile, ...
	"SimulationSHA256", aocsFileHash(simulationFile), ...
	"FilterSHA256", aocsFileHash(which("torqueEkf")), ...
	"AnalysisSHA256", aocsFileHash(mfilename("fullpath") + ".m"), ...
	"Config", config, "Options", options, ...
	"NisInterpretation", "Pre-gate diagnostic; chi-square independence is not assumed.", ...
	"Scope", "Same archived trajectory and applied torque; offline EKF replay, no feedback changes.");
save(fullfile(outputDirectory, "validation.mat"), "report", "traces", "history", "metadata");
writetable(report, fullfile(outputDirectory, "metrics.csv"));
writeAocsJson(fullfile(outputDirectory, "metadata.json"), metadata);
plotReplay(history, traces, labels, outputDirectory);
disp(report);
fprintf("Torque EKF environment validation: %s\n", outputDirectory);
end

function history = readHistory(out, AOCS)
% readHistory Align logs to current GNC ticks and verify measurement freshness.
	estimate = out.logsout.get("AttitudeEstimate").Values;
	time_s = estimate.omega_BI_B_rad_s.Time(:);
	dt = AOCS.GNC.SampleTime_s;
	assert(all(abs(diff(time_s)-dt) < 1e-8), ...
		"AOCS:TorqueValidation:Timing", "Expected uniform GNC timestamps.");
	trueRate = readRows(out.logsout.get("omega_b").Values, time_s, 3, "linear");
	mekfRate = readRows(estimate.omega_BI_B_rad_s, time_s, 3, "previous");
	active = readRows(out.logsout.get("control_active").Values, time_s, 1, "previous") > 0.5;
	initialized = readRows(estimate.initialized, time_s, 1, "previous") > 0.5;
	valid = readRows(estimate.valid, time_s, 1, "previous") > 0.5;
	updateTime_s = readRows(estimate.update_time_s, time_s, 1, "previous");
	fresh = abs(updateTime_s-time_s) < 1e-8;
	assert(all(initialized(active) & valid(active) & fresh(active)), ...
		"AOCS:TorqueValidation:StaleRate", "Active control has invalid or stale MEKF rates.");
	history = struct("Time_s", time_s, "Dt_s", dt, "J", AOCS.Spacecraft.I_B, ...
		"TrueRate", trueRate, "MekfRate", mekfRate, "Active", active, ...
		"TorqueTruth_Nm", readRows(out.logsout.get("M_dist_B_Nm").Values, time_s, 3, "linear") ...
			+ AOCS.Environment.M_ext_B.', ...
		"AppliedTorque_Nm", readRows(out.logsout.get("tau_control_B_Nm").Values, time_s, 3, "previous"));
	assert(any(active), "AOCS:TorqueValidation:Inactive", "No active control samples.");
end

function trace = replayFilter(history, measurement, config)
% replayFilter Match adapter reset policy and use completed-interval torque.
	n = numel(history.Time_s);
	state = zeros(6, 1);
	resetCovariance = diag([repmat(config.RateSigma_rad_s^2, 3, 1); ...
		repmat(config.InitialTorqueSigma_Nm^2, 3, 1)]);
	covariance = resetCovariance;
	needsReset = true;
	trace = struct("Torque_Nm", zeros(n, 3), "UsableTorque_Nm", zeros(n, 3), ...
		"Nis", NaN(n, 1), "NisLimit", config.NISLimit, ...
		"WhitenedInnovation", NaN(n, 3), "Accepted", false(n, 1), ...
		"Reset", false(n, 1), "CovarianceHealthy", true(n, 1));
	R = config.RateSigma_rad_s^2*eye(3);
	parameters = [history.J(:); history.Dt_s; config.Enabled; config.TorqueRW_Nm2_s; ...
		config.RateSigma_rad_s; config.InitialTorqueSigma_Nm; ...
		config.NISLimit; config.MaxEstimate_Nm];

	for sample = 1:n
		if ~history.Active(sample)
			needsReset = true;
			continue
		end
		previousSample = max(1, sample-1);
		appliedTorque = history.AppliedTorque_Nm(previousSample, :).';
		rate = measurement(sample, :).';
		retained = [state; covariance(:); double(~needsReset)];
		input = [rate; appliedTorque; 1; retained; parameters];
		if ~needsReset
			predictionInput = input;
			predictionInput(1:3) = NaN; % Missing measurement requests prediction only.
			predictionOutput = torqueEkf(predictionInput);
			prediction = predictionOutput(1:6);
			predictedCovariance = reshape(predictionOutput(7:42), 6, 6);
			% With positive torque random walk, every predicted torque variance
			% must grow. An unchanged prior means the prediction failed.
			assert(all(diag(predictedCovariance(4:6, 4:6)) > diag(covariance(4:6, 4:6))), ...
				"AOCS:TorqueValidation:Prediction", "EKF prediction did not advance covariance.");
			innovation = rate - prediction(1:3);
			S = predictedCovariance(1:3, 1:3) + R;
			whitened = chol(S, "lower")\innovation;
			trace.Nis(sample) = dot(whitened, whitened);
			trace.WhitenedInnovation(sample, :) = whitened.';
		end
		trace.Reset(sample) = needsReset;
		output = torqueEkf(input);
		state = output(1:6);
		covariance = reshape(output(7:42), 6, 6);
		isValid = logical(output(47));
		trace.Torque_Nm(sample, :) = state(4:6).';
		trace.Accepted(sample) = isValid;
		trace.UsableTorque_Nm(sample, :) = output(44:46).';
		[~, flag] = chol(covariance);
		trace.CovarianceHealthy(sample) = flag == 0 && all(isfinite(covariance), "all") && ...
			norm(covariance-covariance.', "fro") <= 1e-12*norm(covariance, "fro");
		assert(all(isfinite(state)), "AOCS:TorqueValidation:Nonfinite", "EKF state is nonfinite.");
		needsReset = ~isValid;
	end
end

function row = scoreReplay(history, trace, measurement, label, tailDuration_s)
% scoreReplay Report torque accuracy, pre-gate NIS, and innovation correlation.
	tail = history.Time_s >= history.Time_s(end)-tailDuration_s & history.Active;
	validTail = tail & trace.Accepted;
	nisTail = tail & isfinite(trace.Nis);
	assert(nnz(validTail) >= 10 && nnz(nisTail) >= 10, ...
		"AOCS:TorqueValidation:Samples", "Insufficient valid tail samples.");
	nis = trace.Nis(nisTail);
	coverageLimits = 2*gammaincinv([0.005, 0.995], 3/2);
	% Common active tail includes zero output on rejection and clipping.
	torqueError = trace.UsableTorque_Nm(tail, :) - history.TorqueTruth_Nm(tail, :);
	errorRms = sqrt(mean(sum(torqueError.^2, 2)));
	rawError = trace.Torque_Nm(validTail, :) - history.TorqueTruth_Nm(validTail, :);
	rawErrorRms = sqrt(mean(sum(rawError.^2, 2)));
	zeroEstimateRms = sqrt(mean(sum(history.TorqueTruth_Nm(tail, :).^2, 2)));
	rateError = measurement(tail, :) - history.TrueRate(tail, :);
	innovation = trace.WhitenedInnovation;
	pairs = find(nisTail(1:end-1) & nisTail(2:end));
	lagOne = zeros(1, 3);
	for axisIndex = 1:3
		pair = [innovation(pairs, axisIndex), innovation(pairs+1, axisIndex)];
		pair = pair - mean(pair, 1);
		lagOne(axisIndex) = sum(prod(pair, 2))/sqrt(prod(sum(pair.^2, 1)));
	end
	row = struct("Input", label, "TailDuration_s", tailDuration_s, ...
		"TorqueRms_microNm", 1e6*errorRms, "ZeroEstimateRms_microNm", 1e6*zeroEstimateRms, ...
		"RawAcceptedTorqueRms_microNm", 1e6*rawErrorRms, ...
		"ImprovesOverZero", errorRms < zeroEstimateRms, ...
		"RateErrorRms_deg_s", rad2deg(sqrt(mean(sum(rateError.^2, 2)))), ...
		"RateBiasNorm_deg_s", rad2deg(norm(mean(rateError, 1))), ...
		"MeanNis", mean(nis), ...
		"Nis99PercentCoverage", mean(nis >= coverageLimits(1) & nis <= coverageLimits(2)), ...
		"AboveGateFraction", mean(nis > trace.NisLimit), ...
		"MaxAbsInnovationLag1", max(abs(lagOne)), ...
		"AcceptedActiveFraction", mean(trace.Accepted(history.Active)), ...
		"AcceptedTailFraction", mean(trace.Accepted(tail)), ...
		"ResetCount", nnz(trace.Reset), ...
		"CovarianceHealthy", all(trace.CovarianceHealthy(history.Active)));
end

function rows = readRows(signal, time_s, width, method)
% readRows Resample a logged signal without extrapolation or duplicate times.
	if signal.IsTimeFirst
		values = reshape(signal.Data, numel(signal.Time), width);
	else
		values = reshape(signal.Data, width, numel(signal.Time)).';
	end
	[times, indices] = unique(signal.Time, "last");
	rows = interp1(times, double(values(indices, :)), time_s, method);
	assert(all(isfinite(rows), "all"), ...
		"AOCS:TorqueValidation:Log", "Nonfinite or unaligned logged data.");
end

function plotReplay(history, traces, labels, outputDirectory)
% plotReplay Save torque and pre-gate NIS evidence without leaving a figure.
	fig = figure(Visible="off", Color="w", Position=[100, 100, 1100, 900], ...
		DefaultAxesColor="w", DefaultAxesXColor=[0.15, 0.15, 0.15], ...
		DefaultAxesYColor=[0.15, 0.15, 0.15], DefaultTextColor=[0.15, 0.15, 0.15], ...
		DefaultAxesColorOrder=[0, 0.45, 0.74; 0.85, 0.33, 0.10; 0.49, 0.18, 0.56]);
	cleanup = onCleanup(@() close(fig));
	tiledlayout(4, 1, TileSpacing="compact");
	axisLabels = ["x", "y", "z"];
	for axisIndex = 1:3
		nexttile();
		plot(history.Time_s, 1e6*history.TorqueTruth_Nm(:, axisIndex), "k", ...
			LineWidth=1.2, DisplayName="Environment truth");
		hold("on");
		for index = 1:numel(labels)
			plot(history.Time_s, 1e6*traces{index}.UsableTorque_Nm(:, axisIndex), DisplayName=labels(index));
		end
		grid("on");
		ylabel(sprintf("Torque %s_B [microN m]", axisLabels(axisIndex)));
		legend(Location="best", Color="w", TextColor=[0.15, 0.15, 0.15]);
	end
	nexttile();
	hold("on");
	windowSamples = max(1, round(10/history.Dt_s));
	for index = 1:numel(labels)
		plot(history.Time_s, movmean(traces{index}.Nis, windowSamples, "omitnan"), ...
			DisplayName=labels(index));
	end
	yline(3, "k--", DisplayName="Matched-model mean");
	title(sprintf("Pre-gate NIS: 10 s rolling mean; rejection gate %.1f", traces{1}.NisLimit));
	grid("on");
	xlabel("Time [s]");
	ylabel("Pre-gate NIS");
	legend(Location="best", Color="w", TextColor=[0.15, 0.15, 0.15]);
	exportgraphics(fig, fullfile(outputDirectory, "validation.png"), Resolution=160);
end
