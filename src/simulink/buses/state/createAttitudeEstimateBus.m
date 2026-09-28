function AttitudeEstimateBus = createAttitudeEstimateBus(targetWorkspace)
% Description:
%   Defines the MEKF attitude estimate and six-state error covariance.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   AttitudeEstimateBus - Simulink.Bus object for the estimator output.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("q_BI", [4 1], "double", "1", ...
    "Scalar-first estimated quaternion mapping inertial vectors to body axes");
elems(2) = busElement("omega_BI_B_rad_s", [3 1], "double", "rad/s", ...
    "Body angular rate relative to inertial axes, expressed in body axes; bias-corrected gyro");
elems(3) = busElement("initialized", 1, "boolean", "1", ...
    "Estimator initialization has completed");
elems(4) = busElement("gyro_bias_B_rad_s", [3 1], "double", "rad/s", ...
    "Estimated gyroscope bias expressed in body axes");
elems(5) = busElement("P_error", [6 6], "double", "", ...
    "Error covariance ordered [delta_theta; delta_bias]; blocks have mixed units");
elems(6) = busElement("valid", 1, "boolean", "1", ...
    "Accepted gyro prediction on this invocation; not a convergence or vector-update flag");
elems(7) = busElement("update_time_s", 1, "double", "s", ...
    "Onboard time represented by the estimate, not necessarily the latest GNC tick");

AttitudeEstimateBus = Simulink.Bus;
AttitudeEstimateBus.Description = ...
    "MEKF attitude estimate; covariance describes attitude error and gyro bias error";
AttitudeEstimateBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AttitudeEstimateBus", AttitudeEstimateBus);
end
end

function elem = busElement(name, dimensions, dataType, unit, description)
% Description:
%   Creates one explicitly typed and documented estimate field.
%
% Arguments:
%   name - Field name.
%   dimensions - Scalar or matrix dimensions.
%   dataType - Simulink data type.
%   unit - Physical unit, or empty for mixed-unit covariance.
%   description - Field meaning.
%
% Outputs:
%   elem - Configured Simulink.BusElement.

elem = Simulink.BusElement;
elem.Name = name;
elem.Dimensions = dimensions;
elem.DataType = dataType;
elem.Unit = unit;
elem.Description = description;
end
