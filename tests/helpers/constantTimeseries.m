function signal = constantTimeseries(value, time_s)
% Description:
%   Creates a timeseries that holds one scalar/vector/matrix value at each sample.
%
% Arguments:
%   value - Constant value to repeat across samples.
%   time_s - Sample times [s].
%
% Outputs:
%   signal - MATLAB timeseries containing repeated values.

sampleCount = numel(time_s);

if isscalar(value)
    data = repmat(value, sampleCount, 1);
elseif isrow(value)
    data = repmat(value, sampleCount, 1);
else
    data = repmat(value, 1, 1, sampleCount);
end

signal = timeseries(data, time_s(:));
end
