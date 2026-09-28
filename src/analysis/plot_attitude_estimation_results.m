function [figures, diagnostics] = plot_attitude_estimation_results(resultsFile, exportDirectory)
% Description:
%   Compares the MEKF attitude, body rate, and gyro-bias estimates with plant
%   truth. Quaternion signs are aligned before plotting; angular error is
%   sign-invariant. A missing gyro-bias truth log is tolerated for old runs.
%
% Arguments:
%   resultsFile - Optional MAT-file containing SimulationOutput variable out.
%   exportDirectory - Optional directory for PNG copies of both figures.
%
% Outputs:
%   figures - Handles to the generated MATLAB figures.
%   diagnostics - Time-aligned error histories and initialization status.

projectRoot = setupAocsPaths();
if nargin < 1 || strlength(string(resultsFile)) == 0
    AOCS = loadAocsSimulationConfig( ...
        fullfile(projectRoot, "config", "AocsSimulationConfig.json"), projectRoot);
    resultsFile = AOCS.Results.File;
end
if nargin < 2
    exportDirectory = "";
end

loaded = load(resultsFile, "out");
if ~isfield(loaded, "out")
    error("AOCS:Analysis:MissingOutput", ...
        "Results file '%s' does not contain variable out.", string(resultsFile));
end
out = loaded.out;
estimateElement = out.logsout.get("AttitudeEstimate");
if isempty(estimateElement)
    error("AOCS:Analysis:MissingEstimate", ...
        "Simulation output does not contain logged AttitudeEstimate.");
end
estimate = estimateElement.Values;
truth = extractAocsState(out);

t_s = estimate.q_BI.Time(:);
qEstimate = loggedSignalMatrix(estimate.q_BI.Data, 4, "AttitudeEstimate.q_BI");
qTruth = resampleVector(truth.q_be, t_s, 4, "q_be");
omegaEstimate = loggedSignalMatrix(estimate.omega_BI_B_rad_s.Data, 3, ...
    "AttitudeEstimate.omega_BI_B_rad_s");
omegaTruth = resampleVector(truth.omega_b, t_s, 3, "omega_b");
biasEstimate = loggedSignalMatrix(estimate.gyro_bias_B_rad_s.Data, 3, ...
    "AttitudeEstimate.gyro_bias_B_rad_s");
initialized = logical(estimate.initialized.Data(:));

qEstimate = qEstimate ./ vecnorm(qEstimate, 2, 2);
qTruth = qTruth ./ vecnorm(qTruth, 2, 2);
oppositeSign = sum(qEstimate .* qTruth, 2) < 0;
qEstimate(oppositeSign, :) = -qEstimate(oppositeSign, :);

angleError_deg = 2 * acosd(min(1, abs(sum(qEstimate .* qTruth, 2))));
omegaError_rad_s = omegaEstimate - omegaTruth;
angleError_deg(~initialized) = NaN;
omegaError_rad_s(~initialized, :) = NaN;

biasTruth = [];
try
    biasLog = out.get("GyroTrueBias");
catch
    % Older runs did not log the plant gyro bias.
    biasLog = [];
end
if isa(biasLog, "timeseries")
    biasTruth = resampleVector(biasLog, t_s, 3, "GyroTrueBias");
end
biasError_rad_s = NaN(size(biasEstimate));
if ~isempty(biasTruth)
    biasError_rad_s = biasEstimate - biasTruth;
    biasError_rad_s(~initialized, :) = NaN;
end

diagnostics = struct( ...
    "Time_s", t_s, ...
    "Initialized", initialized, ...
    "AttitudeError_deg", angleError_deg, ...
    "OmegaError_B_rad_s", omegaError_rad_s, ...
    "GyroBiasError_B_rad_s", biasError_rad_s, ...
    "HasGyroBiasTruth", ~isempty(biasTruth));

figures = gobjects(2, 1);
figures(1) = plotAttitude(t_s, qEstimate, qTruth, angleError_deg, omegaError_rad_s);
figures(2) = plotRatesAndBias(t_s, omegaEstimate, omegaTruth, biasEstimate, biasTruth);
for k = 1:numel(figures)
    styleFigure(figures(k));
end
exportFigures(figures, exportDirectory);
end

function data = resampleVector(signal, queryTime_s, width, name)
% Description:
%   Samples one logged vector history at estimator timestamps.

samples = loggedSignalMatrix(signal.Data, width, name);
data = interp1(signal.Time(:), samples, queryTime_s, "linear", NaN);
end

function fig = plotAttitude(t_s, qEstimate, qTruth, angleError_deg, omegaError_rad_s)
% Description:
%   Shows quaternion component agreement and compact convergence metrics.

fig = figure("Name", "MEKF - attitude estimate and truth", "Color", "w");
layout = tiledlayout(fig, 3, 2, "TileSpacing", "compact", "Padding", "compact");
heading = title(layout, "Attitude estimate vs plant truth");
heading.Color = [0.16 0.20 0.24];
for axisIndex = 1:4
    nexttile
    plot(t_s, qTruth(:, axisIndex), "Color", [0.16 0.24 0.31], "LineWidth", 1.4)
    hold on
    plot(t_s, qEstimate(:, axisIndex), "--", "Color", [0.05 0.54 0.67], "LineWidth", 1.2)
    grid on
    xlabel("Time [s]")
    ylabel(sprintf("q_%d [-]", axisIndex - 1))
    title(sprintf("Quaternion component %d", axisIndex - 1))
    legend("Plant truth", "MEKF", "Location", "best")
    styleAxes(gca)
end
nexttile
plot(t_s, angleError_deg, "Color", [0.78 0.28 0.18], "LineWidth", 1.3)
grid on
xlabel("Time [s]")
ylabel("Angle [deg]")
title("Quaternion geodesic error")
styleAxes(gca)

nexttile
semilogy(t_s, max(rad2deg(vecnorm(omegaError_rad_s, 2, 2)), 1e-6), ...
    "Color", [0.45 0.32 0.66], "LineWidth", 1.3)
grid on
xlabel("Time [s]")
ylabel("Rate error [deg/s]")
title("Body-rate error norm (log scale)")
styleAxes(gca)
end

function fig = plotRatesAndBias(t_s, omegaEstimate, omegaTruth, biasEstimate, biasTruth)
% Description:
%   Compares body-axis rate and gyro-bias components with plant truth.

fig = figure("Name", "MEKF - body rate and gyro bias", "Color", "w");
layout = tiledlayout(fig, 3, 2, "TileSpacing", "compact", "Padding", "compact");
heading = title(layout, "Body rate and gyro bias");
heading.Color = [0.16 0.20 0.24];
axisNames = ["X", "Y", "Z"];
for axisIndex = 1:3
    nexttile
    plot(t_s, rad2deg(omegaTruth(:, axisIndex)), ...
        "Color", [0.16 0.24 0.31], "LineWidth", 1.4)
    hold on
    plot(t_s, rad2deg(omegaEstimate(:, axisIndex)), "--", ...
        "Color", [0.05 0.54 0.67], "LineWidth", 1.2)
    grid on
    xlabel("Time [s]")
    ylabel("Rate [deg/s]")
    title("Body rate " + axisNames(axisIndex))
    legend("Plant truth", "MEKF", "Location", "best")
    styleAxes(gca)

    nexttile
    if ~isempty(biasTruth)
        plot(t_s, biasTruth(:, axisIndex) * 1e3, ...
            "Color", [0.16 0.24 0.31], "LineWidth", 1.4)
        hold on
    end
    plot(t_s, biasEstimate(:, axisIndex) * 1e3, "--", ...
        "Color", [0.78 0.28 0.18], "LineWidth", 1.2)
    grid on
    xlabel("Time [s]")
    ylabel("Bias [mrad/s]")
    title("Gyro bias " + axisNames(axisIndex))
    if isempty(biasTruth)
        legend("MEKF (truth unavailable)", "Location", "best")
    else
        legend("Plant truth", "MEKF", "Location", "best")
    end
    styleAxes(gca)
end
end

function exportFigures(figures, exportDirectory)
% Description:
%   Writes optional PNG copies alongside MATLAB figure handles.

exportDirectory = string(exportDirectory);
if strlength(exportDirectory) == 0
    return
end
if ~isfolder(exportDirectory)
    mkdir(exportDirectory);
end
names = ["attitude_estimate", "body_rate_and_gyro_bias"];
for k = 1:numel(figures)
    exportgraphics(figures(k), fullfile(exportDirectory, names(k) + ".png"), ...
        "Resolution", 180);
end
end

function styleAxes(ax)
% Description:
%   Applies the same restrained axes styling as other analysis figures.

ax.FontName = "Helvetica";
ax.FontSize = 10;
ax.LineWidth = 0.8;
ax.GridAlpha = 0.22;
end

function styleFigure(fig)
% Description:
%   Keeps exported plots readable with either MATLAB desktop theme.

ink = [0.16 0.20 0.24];
set(findall(fig, "Type", "Axes"), "Color", "w", "XColor", ink, "YColor", ink);
set(findall(fig, "Type", "Legend"), "Color", "w", "TextColor", ink, ...
    "EdgeColor", [0.75 0.78 0.80]);
set(findall(fig, "Type", "Text"), "Color", ink);
end
