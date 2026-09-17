function SensorMeasurementBus = createSensorMeasurementBus(targetWorkspace)
% Description:
%   Defines the top-level sensor measurement contract consumed by GNC.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   SensorMeasurementBus - Simulink.Bus object for sensor measurements.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = Simulink.BusElement;
elems(1).Name = "Gyro";
elems(1).Dimensions = 1;
elems(1).DataType = "Bus: GyroMeasurementBus";
elems(1).Description = "Gyroscope measurement products";

elems(2) = Simulink.BusElement;
elems(2).Name = "Magnetometer";
elems(2).Dimensions = 1;
elems(2).DataType = "Bus: MagnetometerMeasurementBus";
elems(2).Description = "Magnetometer measurement products";

elems(3) = Simulink.BusElement;
elems(3).Name = "CoarseSunSensors";
elems(3).Dimensions = 1;
elems(3).DataType = "Bus: CoarseSunSensorMeasurementBus";
elems(3).Description = "Coarse sun sensor measurement products";

elems(4) = Simulink.BusElement;
elems(4).Name = "GNSS";
elems(4).Dimensions = 1;
elems(4).DataType = "Bus: GnssMeasurementBus";
elems(4).Description = "GNSS receiver measurement products";

SensorMeasurementBus = Simulink.Bus;
SensorMeasurementBus.Description = "Top-level sensor measurement output bus";
SensorMeasurementBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "SensorMeasurementBus", SensorMeasurementBus);
end
end
