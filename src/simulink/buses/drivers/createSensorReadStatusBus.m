function SensorReadStatusBus = createSensorReadStatusBus(targetWorkspace)
%CREATESENSORREADSTATUSBUS Define the per-sensor status snapshot read by GNC.
% Description:
%   Groups independent driver read-status flags for all four sensors.
%
% Arguments:
%   targetWorkspace - "base" publishes bus definitions; otherwise returns only.
%
% Outputs:
%   SensorReadStatusBus - Bus with one DriverReadStatusBus per sensor.

if nargin < 1
    targetWorkspace = "base";
end
createDriverReadStatusBus(targetWorkspace);
names = ["Gyro", "Magnetometer", "CoarseSunSensors", "GNSS"];
elements = repmat(Simulink.BusElement, 1, numel(names));
for index = 1:numel(names)
    elements(index).Name = names(index);
    elements(index).DataType = 'Bus: DriverReadStatusBus';
end
SensorReadStatusBus = Simulink.Bus;
SensorReadStatusBus.Elements = elements;
SensorReadStatusBus.Description = 'Driver status snapshots from one GNC polling cycle';
if string(targetWorkspace) == "base"
    assignin('base', 'SensorReadStatusBus', SensorReadStatusBus);
end
end
