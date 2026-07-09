function data = loggedSignalMatrix(rawData, expectedColumns, signalName)
% Description:
%   Squeezes Simulink logged vector data and accepts either N-by-M or M-by-N
%   orientation. Throws a descriptive error for unexpected shapes.
%
% Arguments:
%   rawData - Raw timeseries Data array from Simulink logging.
%   expectedColumns - Expected signal width after reshaping.
%   signalName - Signal name used in shape error messages.
%
% Outputs:
%   data - N-by-expectedColumns numeric matrix.

data = squeeze(rawData);
if isvector(data)
    data = data(:);
end

if size(data, 2) == expectedColumns
    return;
end

if size(data, 1) == expectedColumns
    data = data.';
    return;
end

error("AOCS:Analysis:UnexpectedSignalShape", ...
    "Logged signal '%s' has shape %s; expected N-by-%d or %d-by-N.", ...
    char(string(signalName)), mat2str(size(data)), expectedColumns, expectedColumns);
end
