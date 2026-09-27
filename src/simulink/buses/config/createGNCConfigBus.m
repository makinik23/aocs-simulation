function GNCConfigBus = createGNCConfigBus(targetWorkspace)
% Description:
%   Defines the top-level onboard GNC configuration contract.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   GNCConfigBus - Simulink.Bus object containing GNC subsystem settings.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

createAttitudeInitializationConfigBus(targetWorkspace);
attitudeInitialization = Simulink.BusElement;
attitudeInitialization.Name = "AttitudeInitialization";
attitudeInitialization.DataType = "Bus: AttitudeInitializationConfigBus";
attitudeInitialization.Description = ...
    "Reference-vector and TRIAD initialization configuration";

GNCConfigBus = Simulink.Bus;
GNCConfigBus.Description = "Top-level onboard GNC configuration";
GNCConfigBus.Elements = attitudeInitialization;

if targetWorkspace == "base"
    assignin("base", "GNCConfigBus", GNCConfigBus);
end
end
