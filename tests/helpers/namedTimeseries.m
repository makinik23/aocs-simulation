function ts = namedTimeseries(name, values, time_s)
% Description:
%   Creates a timeseries with a stable Simulink signal name.
%
% Arguments:
%   name - Signal name assigned to the timeseries.
%   values - Signal samples.
%   time_s - Sample times [s].
%
% Outputs:
%   ts - MATLAB timeseries with Name set to name.

ts = timeseries(values, time_s(:));
ts.Name = char(name);
end
