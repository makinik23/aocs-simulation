function SensorConfigBus = createSensorConfigBus(targetWorkspace)
% Description:
%   Defines the top-level sensor configuration contract.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   SensorConfigBus - Simulink.Bus object for sensor configuration.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = Simulink.BusElement;
elems(1).Name = "Gyro";
elems(1).Dimensions = 1;
elems(1).DataType = "Bus: GyroConfigBus";
elems(1).Description = "Gyroscope sensor configuration";

elems(2) = Simulink.BusElement;
elems(2).Name = "Magnetometer";
elems(2).Dimensions = 1;
elems(2).DataType = "Bus: MagnetometerConfigBus";
elems(2).Description = "Magnetometer sensor configuration";

SensorConfigBus = Simulink.Bus;
SensorConfigBus.Description = "Sensor configuration bus generated from config/sensors.json";
SensorConfigBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "SensorConfigBus", SensorConfigBus);
end
end
