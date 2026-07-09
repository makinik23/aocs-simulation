function state = extractAocsState(out)
% Description:
%   Reads the current model contract: four named state signals logged in
%   out.logsout. The function fails fast if the dataset or a required signal
%   is missing.
%
% Arguments:
%   out - Simulink.SimulationOutput from a plant simulation.
%
% Outputs:
%   state - Struct containing Source plus euler_rad, q_be, DCM_be, omega_b
%           timeseries fields.

logs = simulationDataset(out, "logsout");

state = struct();
state.Source = "logsout";
state.euler_rad = loggedTimeseries(logs, "euler_rad");
state.q_be = loggedTimeseries(logs, "q_be");
state.DCM_be = loggedTimeseries(logs, "DCM_be");
state.omega_b = loggedTimeseries(logs, "omega_b");
end

function logs = simulationDataset(out, name)
% Description:
%   Wraps SimulationOutput.get and gives a focused error when the expected
%   dataset is absent or has the wrong type.
%
% Arguments:
%   out - Simulink.SimulationOutput object.
%   name - Dataset name to retrieve.
%
% Outputs:
%   logs - Simulink.SimulationData.Dataset with the requested name.

try
    logs = out.get(char(name));
catch
    logs = [];
end

if ~isa(logs, "Simulink.SimulationData.Dataset")
    error("AOCS:Analysis:MissingDataset", ...
        "Simulation output does not contain a '%s' Dataset.", char(name));
end
end

function values = loggedTimeseries(logs, signalName)
% Description:
%   Retrieves one logged signal and reports available names if it is missing.
%
% Arguments:
%   logs - Simulink.SimulationData.Dataset.
%   signalName - Name of the logged signal to retrieve.
%
% Outputs:
%   values - Timeseries stored in the requested dataset element.

element = logs.get(char(signalName));
if isempty(element)
    error("AOCS:Analysis:MissingSignal", ...
        "Missing logged signal '%s'. Available signals: %s", ...
        char(signalName), char(strjoin(loggedSignalNames(logs), ", ")));
end

values = element.Values;
end

function names = loggedSignalNames(logs)
% Description:
%   Builds the signal list used in missing-signal diagnostics.
%
% Arguments:
%   logs - Simulink.SimulationData.Dataset.
%
% Outputs:
%   names - String array of dataset element names.

names = strings(1, logs.numElements);
for k = 1:logs.numElements
    names(k) = string(logs{k}.Name);
end
end
