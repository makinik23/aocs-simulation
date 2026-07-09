function figures = plot_aocs_results(resultsFile)
% Description:
%   Loads the latest simulation result and plots body rates, quaternion,
%   Euler angles, rotational energy, and angular momentum norm.
%
% Arguments:
%   resultsFile - Optional MAT-file containing simulation output variable out.
%
% Outputs:
%   figures - Handles to the generated MATLAB figures.

projectRoot = fileparts(mfilename("fullpath"));
addpath(fullfile(projectRoot, "src", "config"));
addpath(fullfile(projectRoot, "src", "analysis"));

AOCS = loadAocsSimulationConfig(fullfile(projectRoot, "config", "AocsSimulationConfig.json"), projectRoot);

if nargin < 1 || strlength(string(resultsFile)) == 0
    resultsFile = AOCS.Results.File;
end

load(resultsFile, "out");

state = extractAocsState(out);
disp("Using AOCS state bus: " + state.Source)

t = state.omega_b.Time;
omegaData = loggedSignalMatrix(state.omega_b.Data, 3, "omega_b");
invariants = computeAocsInvariants(omegaData, AOCS.Spacecraft.I_B);

figures = gobjects(5, 1);

figures(1) = plotSeries(t, omegaData, "Time [s]", "\omega_b [rad/s]", ...
    "Angular velocity", ["p", "q", "r"]);

tq = state.q_be.Time;
qData = loggedSignalMatrix(state.q_be.Data, 4, "q_be");

figures(2) = plotSeries(tq, qData, "Time [s]", "q_{be} [-]", ...
    "Quaternion", ["q_0", "q_1", "q_2", "q_3"]);

te = state.euler_rad.Time;
eulerData = loggedSignalMatrix(state.euler_rad.Data, 3, "euler_rad");

figures(3) = plotSeries(te, eulerData, "Time [s]", "Euler angles [rad]", ...
    "Euler attitude", ["\phi", "\theta", "\psi"]);

figures(4) = plotSeries(t, invariants.E_rot, "Time [s]", ...
    "Rotational energy [J]", "Rotational energy", strings(0));

figures(5) = plotSeries(t, invariants.H_norm, "Time [s]", ...
    "|H| [Nms]", "Angular momentum norm", strings(0));
end

function fig = plotSeries(t, y, xLabelText, yLabelText, titleText, legendText)
% Description:
%   Wraps the repeated figure, plot, grid, label, title, and optional legend
%   calls used by plot_aocs_results.
%
% Arguments:
%   t - Time vector.
%   y - Data vector or matrix to plot against t.
%   xLabelText - X-axis label text.
%   yLabelText - Y-axis label text.
%   titleText - Figure title text.
%   legendText - Legend entries, or empty string array for no legend.
%
% Outputs:
%   fig - Handle to the generated MATLAB figure.

fig = figure;
plot(t, y)
grid on
xlabel(xLabelText)
ylabel(yLabelText)
title(titleText)

if ~isempty(legendText)
    legend(legendText)
end
end
