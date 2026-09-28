function AttitudeHealthConfigBus = createAttitudeHealthConfigBus(targetWorkspace)
% Description:
%   Defines estimator-health thresholds in SI units for onboard GNC.
%
% Arguments:
%   targetWorkspace - "base" publishes the bus; otherwise returns it only.
%
% Outputs:
%   AttitudeHealthConfigBus - Typed health threshold bus.

if nargin < 1
    targetWorkspace = "base";
end

names = ["degraded_attitude_std_rad", "lost_attitude_std_rad", ...
    "degraded_state_age_s", "lost_state_age_s", ...
    "degraded_correction_age_s", "lost_correction_age_s"];
units = ["rad", "rad", "s", "s", "s", "s"];
descriptions = ["Maximum marginal attitude uncertainty for tracking", ...
    "Attitude uncertainty requiring lost mode", ...
    "State age requiring degraded mode", ...
    "State age requiring lost mode", ...
    "Time since a vector correction requiring degraded mode", ...
    "Time since a vector correction requiring lost mode"];
elements(1, numel(names)) = Simulink.BusElement;
for index = 1:numel(names)
    elements(index).Name = names(index);
    elements(index).DataType = "double";
    elements(index).Unit = units(index);
    elements(index).Description = descriptions(index);
end

AttitudeHealthConfigBus = Simulink.Bus;
AttitudeHealthConfigBus.Description = "Thresholds for attitude-estimator health classification";
AttitudeHealthConfigBus.Elements = elements;
if string(targetWorkspace) == "base"
    assignin("base", "AttitudeHealthConfigBus", AttitudeHealthConfigBus);
end
end
