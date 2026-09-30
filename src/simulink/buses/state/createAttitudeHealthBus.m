function AttitudeHealthBus = createAttitudeHealthBus(targetWorkspace)
% Description:
%   Defines the estimator health result separately from AttitudeEstimateBus.
%   Modes: 0 Initializing, 1 Tracking, 2 Degraded, 3 Lost.
%
% Arguments:
%   targetWorkspace - "base" publishes the bus; otherwise returns it only.
%
% Outputs:
%   AttitudeHealthBus - Typed health status and quantitative diagnostics.

if nargin < 1
    targetWorkspace = "base";
end

names = ["mode_id", "control_usable", "attitude_std_max_rad", ...
    "state_age_s", "correction_age_s", "vector_update_accepted"];
types = ["uint8", "boolean", "double", "double", "double", "boolean"];
units = ["1", "1", "rad", "s", "s", "1"];
descriptions = ["0 Initializing, 1 Tracking, 2 Degraded, 3 Lost", ...
    "True only in Tracking mode", ...
    "Largest marginal attitude standard deviation from P", ...
    "Age of represented MEKF state", ...
    "Time since latest accepted magnetic or Sun correction", ...
    "At least one vector correction accepted on this GNC tick"];
elements(1, numel(names)) = Simulink.BusElement;
for index = 1:numel(names)
    elements(index).Name = names(index);
    elements(index).DataType = types(index);
    elements(index).Unit = units(index);
    elements(index).Description = descriptions(index);
end

AttitudeHealthBus = Simulink.Bus;
AttitudeHealthBus.Description = "Onboard MEKF health and control-use decision";
AttitudeHealthBus.Elements = elements;
if string(targetWorkspace) == "base"
    assignin("base", "AttitudeHealthBus", AttitudeHealthBus);
end
end
