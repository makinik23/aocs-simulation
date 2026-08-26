function bus = constantBusTimeseries(config, time_s)
% Description:
%   Wraps every field of a config struct in a constant timeseries.
%
% Arguments:
%   config - Struct whose fields should be converted to timeseries.
%   time_s - Sample times [s].
%
% Outputs:
%   bus - Struct with the same fields as config, each containing a timeseries.

bus = struct();
names = fieldnames(config);

for k = 1:numel(names)
    name = names{k};
    bus.(name) = constantTimeseries(config.(name), time_s);
end
end
