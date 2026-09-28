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
createMekfConfigBus(targetWorkspace);
createAttitudeHealthConfigBus(targetWorkspace);
sampleTime = Simulink.BusElement;
sampleTime.Name = "sample_time_s";
sampleTime.Unit = "s";
sampleTime.Description = "Period of the ordered GNC processing cycle";
attitudeInitialization = Simulink.BusElement;
attitudeInitialization.Name = "AttitudeInitialization";
attitudeInitialization.DataType = "Bus: AttitudeInitializationConfigBus";
attitudeInitialization.Description = ...
    "Reference-vector and TRIAD initialization configuration";
mekf = Simulink.BusElement;
mekf.Name = "MEKF";
mekf.DataType = "Bus: MekfConfigBus";
mekf.Description = "MEKF initialization prior";
health = Simulink.BusElement;
health.Name = "AttitudeHealth";
health.DataType = "Bus: AttitudeHealthConfigBus";
health.Description = "Estimator-health classification thresholds";

GNCConfigBus = Simulink.Bus;
GNCConfigBus.Description = "Top-level onboard GNC configuration";
GNCConfigBus.Elements = [sampleTime; attitudeInitialization; mekf; health];

if targetWorkspace == "base"
    assignin("base", "GNCConfigBus", GNCConfigBus);
end
end
