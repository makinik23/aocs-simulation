function data = extractFlightVisualizationData(resultsSource, AOCS)
% Description:
%   Extracts a simulation result into a plot/viewer-friendly flight data
%   struct with orbit, attitude, LLA, magnetic-field, and disturbance products.
%
% Arguments:
%   resultsSource - Optional MAT-file path or Simulink.SimulationOutput.
%   AOCS - Optional validated configuration struct when resultsSource is an
%          in-memory SimulationOutput.
%
% Outputs:
%   data - Struct containing time-aligned products for visualization/export.

rootDirectory = setupAocsPaths();
defaultAOCS = [];

if nargin < 1 || isMissingResultsSource(resultsSource)
    defaultAOCS = defaultAocsConfig(rootDirectory, defaultAOCS);
    resultsSource = defaultAOCS.Results.File;
end

if nargin < 2
    AOCS = [];
end

[out, AOCS, sourceName] = resolveResultsSource(resultsSource, AOCS, rootDirectory, defaultAOCS);

logsout = out.logsout;
t_s = out.tout(:);

r_I_m = loggedVector(logsout, "r_I_m", 3);
v_I_m_s = loggedVector(logsout, "v_I_m_s", 3);
B_NED_T = loggedVector(logsout, "B_NED_T", 3);
B_I_T = loggedVector(logsout, "B_I_T", 3);
B_B_T = loggedVector(logsout, "B_B_T", 3);
zeroTorque_Nm = zeros(numel(t_s), 3);
zeroForce_N = zeros(numel(t_s), 3);
zeroAcceleration_m_s2 = zeros(numel(t_s), 3);
zeroScalar = zeros(numel(t_s), 1);
defaultedSignals = strings(0, 1);
[M_rmm_B_Nm, usedDefault] = loggedVector(logsout, "M_rmm_B_Nm", 3, zeroTorque_Nm);
defaultedSignals = appendDefaultedSignal(defaultedSignals, "M_rmm_B_Nm", usedDefault);
[M_gg_B_Nm, usedDefault] = loggedVector(logsout, "M_gg_B_Nm", 3, zeroTorque_Nm);
defaultedSignals = appendDefaultedSignal(defaultedSignals, "M_gg_B_Nm", usedDefault);
[M_srp_B_Nm, usedDefault] = loggedVector(logsout, "M_srp_B_Nm", 3, zeroTorque_Nm);
defaultedSignals = appendDefaultedSignal(defaultedSignals, "M_srp_B_Nm", usedDefault);
[M_dist_B_Nm, usedDefault] = loggedVector(logsout, "M_dist_B_Nm", 3, zeroTorque_Nm);
defaultedSignals = appendDefaultedSignal(defaultedSignals, "M_dist_B_Nm", usedDefault);
[F_srp_B_N, usedDefault] = loggedVector(logsout, "F_srp_B_N", 3, zeroForce_N);
defaultedSignals = appendDefaultedSignal(defaultedSignals, "F_srp_B_N", usedDefault);
[F_srp_I_N, usedDefault] = loggedVector(logsout, "F_srp_I_N", 3, zeroForce_N);
defaultedSignals = appendDefaultedSignal(defaultedSignals, "F_srp_I_N", usedDefault);
[P_srp_N_m2, usedDefault] = loggedVector(logsout, "P_srp_N_m2", 1, zeroScalar);
defaultedSignals = appendDefaultedSignal(defaultedSignals, "P_srp_N_m2", usedDefault);
[a_srp_I_m_s2, usedDefault] = loggedVector(logsout, "a_srp_I_m_s2", 3, zeroAcceleration_m_s2);
defaultedSignals = appendDefaultedSignal(defaultedSignals, "a_srp_I_m_s2", usedDefault);
[a_aero_I_m_s2, usedDefault] = loggedVector(logsout, "a_aero_I_m_s2", 3, zeroAcceleration_m_s2);
defaultedSignals = appendDefaultedSignal(defaultedSignals, "a_aero_I_m_s2", usedDefault);
[a_dist_I_m_s2, usedDefault] = loggedVector(logsout, "a_dist_I_m_s2", 3, zeroAcceleration_m_s2);
defaultedSignals = appendDefaultedSignal(defaultedSignals, "a_dist_I_m_s2", usedDefault);

state = extractAocsState(out);
orbitDiagnostics = computeOrbitDiagnostics(r_I_m, v_I_m_s, AOCS);
lla = computeLlaProducts(r_I_m, t_s, AOCS);
disturbanceSummary = computeDisturbanceSummary(B_NED_T, B_I_T, B_B_T, ...
    M_rmm_B_Nm, M_gg_B_Nm, M_srp_B_Nm, M_dist_B_Nm, ...
    F_srp_B_N, F_srp_I_N, a_srp_I_m_s2, a_aero_I_m_s2, a_dist_I_m_s2);

data = struct();
data.Meta = visualizationMeta(AOCS, sourceName, numel(t_s));
data.Meta.DefaultedSignals = defaultedSignals;
data.Time_s = t_s;
data.Time_min = t_s ./ 60;

data.Orbit = struct();
data.Orbit.r_I_m = r_I_m;
data.Orbit.v_I_m_s = v_I_m_s;
data.Orbit.Diagnostics = orbitDiagnostics;
data.Orbit.LLA = lla;

data.Attitude = struct();
data.Attitude.euler_rad = loggedSignalMatrix(state.euler_rad.Data, 3, "euler_rad");
data.Attitude.q_be = loggedSignalMatrix(state.q_be.Data, 4, "q_be");
data.Attitude.DCM_be = loggedDcmRows(state.DCM_be.Data);
data.Attitude.omega_b_rad_s = loggedSignalMatrix(state.omega_b.Data, 3, "omega_b");

data.Environment = struct();
data.Environment.B_NED_T = B_NED_T;
data.Environment.B_I_T = B_I_T;
data.Environment.B_B_T = B_B_T;
data.Environment.M_rmm_B_Nm = M_rmm_B_Nm;
data.Environment.M_gg_B_Nm = M_gg_B_Nm;
data.Environment.M_srp_B_Nm = M_srp_B_Nm;
data.Environment.M_dist_B_Nm = M_dist_B_Nm;
data.Environment.F_srp_B_N = F_srp_B_N;
data.Environment.F_srp_I_N = F_srp_I_N;
data.Environment.P_srp_N_m2 = P_srp_N_m2;
data.Environment.a_srp_I_m_s2 = a_srp_I_m_s2;
data.Environment.a_aero_I_m_s2 = a_aero_I_m_s2;
data.Environment.a_dist_I_m_s2 = a_dist_I_m_s2;
data.Environment.Summary = disturbanceSummary;
end

function tf = isMissingResultsSource(resultsSource)
% Description:
%   Checks only textual sources for emptiness so SimulationOutput inputs work.

tf = (ischar(resultsSource) || isstring(resultsSource)) && strlength(string(resultsSource)) == 0;
end

function signals = appendDefaultedSignal(signals, signalName, usedDefault)
% Description:
%   Records optional logged signals replaced by zero-valued defaults.

if usedDefault
    signals(end + 1, 1) = signalName;
end
end

function [out, AOCS, sourceName] = resolveResultsSource(resultsSource, AOCS, rootDirectory, defaultAOCS)
% Description:
%   Normalizes a MAT-file path or SimulationOutput into an output/config pair.

if ischar(resultsSource) || isstring(resultsSource)
    sourceName = string(resultsSource);
    loaded = load(sourceName);

    if ~isfield(loaded, "out")
        error("AOCS:Analysis:MissingSimulationOutput", ...
            "Results file '%s' does not contain variable 'out'.", char(sourceName));
    end

    out = loaded.out;
    if isfield(loaded, "AOCS")
        AOCS = loaded.AOCS;
    elseif isempty(AOCS)
        AOCS = defaultAocsConfig(rootDirectory, defaultAOCS);
    end

    return;
end

if isa(resultsSource, "Simulink.SimulationOutput")
    if isempty(AOCS)
        AOCS = defaultAocsConfig(rootDirectory, defaultAOCS);
    end

    out = resultsSource;
    sourceName = "<SimulationOutput>";
    return;
end

error("AOCS:Analysis:UnsupportedResultsSource", ...
    "resultsSource must be a MAT-file path or Simulink.SimulationOutput.");
end

function AOCS = defaultAocsConfig(rootDirectory, AOCS)
% Description:
%   Loads the default config only when a caller/source did not provide one.

if ~isempty(AOCS)
    return;
end

AOCS = loadAocsSimulationConfig(fullfile(rootDirectory, "config", "AocsSimulationConfig.json"), rootDirectory);
end

function meta = visualizationMeta(AOCS, sourceName, sampleCount)
% Description:
%   Builds export metadata without making plotting code depend on config shape.

meta = struct();
meta.Source = sourceName;
meta.SampleCount = sampleCount;
meta.TimeUnit = "s";
meta.PositionFrame = "ICRF";
meta.AttitudeConvention = "q_be maps inertial frame I to body frame B";

if isfield(AOCS, "Mission") && isfield(AOCS.Mission, "Name")
    meta.MissionName = AOCS.Mission.Name;
end

if isfield(AOCS, "Epoch") && isfield(AOCS.Epoch, "Utc")
    meta.EpochUtc = AOCS.Epoch.Utc(:).';
end

if isfield(AOCS, "Environment") && isfield(AOCS.Environment, "EarthOrientation")
    meta.EarthOrientation = AOCS.Environment.EarthOrientation;
end
end

function data = loggedDcmRows(rawData)
% Description:
%   Converts a logged 3-by-3-by-N or N-by-9 DCM signal into N-by-9 rows.

rawData = squeeze(rawData);

if ndims(rawData) == 3 && size(rawData, 1) == 3 && size(rawData, 2) == 3
    data = reshape(permute(rawData, [3 1 2]), [], 9);
    return;
end

if ndims(rawData) == 3 && size(rawData, 2) == 3 && size(rawData, 3) == 3
    data = reshape(rawData, [], 9);
    return;
end

data = loggedSignalMatrix(rawData, 9, "DCM_be");
end
