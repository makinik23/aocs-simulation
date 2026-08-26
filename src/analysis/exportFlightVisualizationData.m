function data = exportFlightVisualizationData(resultsSource, outputFile, sampleStep, AOCS)
% Description:
%   Writes flight visualization data extracted from a saved or in-memory
%   simulation result to JSON.
%
% Arguments:
%   resultsSource - Optional MAT-file path or Simulink.SimulationOutput.
%   outputFile - Optional JSON output path.
%   sampleStep - Optional positive integer export stride.
%   AOCS - Optional validated config for in-memory SimulationOutput.
%
% Outputs:
%   data - The exported struct after optional downsampling.

if nargin < 1
    resultsSource = "";
end

if nargin < 2 || strlength(string(outputFile)) == 0
    outputFile = defaultOutputFile(resultsSource);
end

if nargin < 3 || isempty(sampleStep)
    sampleStep = 1;
end

if nargin < 4
    data = extractFlightVisualizationData(resultsSource);
else
    data = extractFlightVisualizationData(resultsSource, AOCS);
end

sampleStep = validateSampleStep(sampleStep);
if sampleStep > 1
    data = selectFlightVisualizationSamples(data, sampleStep);
end

outputFile = string(outputFile);
outputDirectory = fileparts(outputFile);
if strlength(outputDirectory) > 0 && ~isfolder(outputDirectory)
    mkdir(outputDirectory);
end

json = jsonencode(data);
fileId = fopen(outputFile, "w");
if fileId < 0
    error("AOCS:Analysis:ExportFailed", "Could not open '%s' for writing.", char(outputFile));
end

cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, "%s", json);
clear cleanup

fprintf("Flight visualization data exported to %s\n", outputFile);
end

function outputFile = defaultOutputFile(resultsSource)
% Description:
%   Places the JSON next to a MAT-file source or under the default results dir.

if ischar(resultsSource) || isstring(resultsSource)
    resultsSource = string(resultsSource);
    if strlength(resultsSource) > 0
        [directory, name] = fileparts(resultsSource);
        outputFile = fullfile(directory, name + "_flight_visualization.json");
        return;
    end
end

projectRoot = setupAocsPaths();
AOCS = loadAocsSimulationConfig(fullfile(projectRoot, "config", "AocsSimulationConfig.json"), projectRoot);
outputFile = fullfile(AOCS.Results.Directory, "flight_visualization.json");
end

function sampleStep = validateSampleStep(sampleStep)
% Description:
%   Ensures the export stride is a positive integer.

sampleStep = double(sampleStep);
if ~isscalar(sampleStep) || ~isfinite(sampleStep) || sampleStep < 1 || sampleStep ~= round(sampleStep)
    error("AOCS:Analysis:InvalidSampleStep", "sampleStep must be a positive integer.");
end
end

function data = selectFlightVisualizationSamples(data, sampleStep)
% Description:
%   Keeps every sampleStep-th sample and always preserves the final sample.

originalSamples = numel(data.Time_s);
idx = 1:sampleStep:originalSamples;
if idx(end) ~= originalSamples
    idx(end + 1) = originalSamples;
end

data.Time_s = data.Time_s(idx);
data.Time_min = data.Time_min(idx);
data.Orbit = selectStructRows(data.Orbit, idx, originalSamples);
data.Attitude = selectStructRows(data.Attitude, idx, originalSamples);
data.Environment = selectStructRows(data.Environment, idx, originalSamples);
data.Environment.Summary = computeDisturbanceSummary(data.Environment.B_NED_T, ...
    data.Environment.B_I_T, data.Environment.B_B_T, data.Environment.M_rmm_B_Nm, ...
    data.Environment.M_gg_B_Nm, data.Environment.M_srp_B_Nm, data.Environment.M_dist_B_Nm, ...
    data.Environment.F_srp_B_N, data.Environment.F_srp_I_N, data.Environment.a_srp_I_m_s2, ...
    data.Environment.a_aero_I_m_s2, data.Environment.a_dist_I_m_s2);

data.Meta.OriginalSampleCount = originalSamples;
data.Meta.SampleCount = numel(idx);
data.Meta.SampleStep = sampleStep;
end

function value = selectStructRows(value, idx, originalSamples)
% Description:
%   Recursively downsamples numeric fields with sample-aligned first dimension.

if isstruct(value)
    fields = fieldnames(value);
    for k = 1:numel(fields)
        field = fields{k};
        value.(field) = selectStructRows(value.(field), idx, originalSamples);
    end
    return;
end

if isnumeric(value) && size(value, 1) == originalSamples
    value = value(idx, :);
end
end
