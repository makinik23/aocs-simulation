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

elems(3) = Simulink.BusElement;
elems(3).Name = "CoarseSunSensors";
elems(3).Dimensions = 1;
elems(3).DataType = "Bus: CoarseSunSensorConfigBus";
elems(3).Description = "Coarse sun sensor array configuration";

elems(4) = Simulink.BusElement;
elems(4).Name = "GNSS";
elems(4).Dimensions = 1;
elems(4).DataType = "Bus: GnssConfigBus";
elems(4).Description = "GNSS receiver configuration";

SensorConfigBus = Simulink.Bus;
SensorConfigBus.Description = "Sensor configuration bus generated from config/sensors.json";
SensorConfigBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "SensorConfigBus", SensorConfigBus);
end
end
