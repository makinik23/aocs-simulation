function dataset = addHarnessInput(dataset, name, values, time_s)
% Description:
%   Adds a named timeseries element to a Simulink input Dataset.
%
% Arguments:
%   dataset - Simulink external-input Dataset to extend.
%   name - Signal name for the new input element.
%   values - Signal samples.
%   time_s - Sample times [s].
%
% Outputs:
%   dataset - Dataset with the named timeseries element added.

dataset = dataset.addElement(namedTimeseries(name, values, time_s), name);
end
