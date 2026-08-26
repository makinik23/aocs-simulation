function [data, usedDefault] = loggedVector(logsout, signalName, width, defaultData)
% Description:
%   Reads a logged vector signal and returns an N-by-width matrix.
%
% Arguments:
%   logsout - Simulink logsout dataset.
%   signalName - Logged signal name.
%   width - Expected vector width.
%   defaultData - Optional default returned when the signal is absent.
%
% Outputs:
%   data - N-by-width numeric matrix.
%   usedDefault - True when defaultData was returned.

if hasLoggedSignal(logsout, signalName)
    try
        element = logsout.get(char(signalName));
    catch
        element = [];
    end
else
    element = [];
end

usedDefault = false;
if isempty(element)
    if nargin >= 4
        data = defaultData;
        usedDefault = true;
        return;
    end

    error("AOCS:Analysis:MissingSignal", "Missing logged signal '%s'.", char(signalName));
end

data = loggedSignalMatrix(element.Values.Data, width, signalName);
end

function tf = hasLoggedSignal(logsout, signalName)
% Description:
%   Checks dataset element names without triggering Simulink's not-found warning.

tf = false;
try
    for k = 1:logsout.numElements
        if string(logsout{k}.Name) == string(signalName)
            tf = true;
            return;
        end
    end
catch
end
end
