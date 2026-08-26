function values = harnessOutputRows(simOut, signalName, width, sampleTime_s)
% Description:
%   Reads and interpolates a logged vector harness output at requested samples.
%
% Arguments:
%   simOut - Simulink.SimulationOutput returned by a harness run.
%   signalName - Expected output signal name.
%   width - Expected vector width.
%   sampleTime_s - Sample times for interpolation [s].
%
% Outputs:
%   values - N-by-width output matrix at sampleTime_s.

yout = simOut.get("yout");
for index = 1:yout.numElements
    element = yout{index};
    if signalElementMatches(element, signalName)
        signal = element.Values;
        data = loggedSignalMatrix(signal.Data, width, signalName);
        values = interp1(signal.Time(:), data, sampleTime_s(:), "linear");
        return;
    end
end

error("AOCS:Tests:MissingHarnessOutput", ...
    "Could not find harness output '%s'.", char(signalName));
end

function tf = signalElementMatches(element, signalName)
% Description:
%   Checks whether a Dataset element corresponds to a requested harness signal.
%
% Arguments:
%   element - Simulink Dataset element.
%   signalName - Expected signal name.
%
% Outputs:
%   tf - True when the element name or block path matches signalName.

tf = isprop(element, "Name") && string(element.Name) == string(signalName);
if tf
    return;
end

try
    blockPath = string(element.BlockPath.getBlock(1));
    tf = endsWith(blockPath, "/" + string(signalName));
catch
    tf = false;
end
end
