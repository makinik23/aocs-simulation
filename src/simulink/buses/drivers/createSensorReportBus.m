function SensorReportBus = createSensorReportBus(targetWorkspace)
%CREATESENSORREPORTBUS Define driver reports, separate from raw measurements.
% Each report retains the sensor fields and adds receiver-owned metadata.
% sequence_id is uint32, increments per receipt, and wraps modulo 2^32.
% Compare sequence IDs for inequality, not ordering. Before receipt all fields
% are zero, including valid and sequence_id. receive_time_s uses model time.
if nargin < 1
    targetWorkspace = "base";
end
names = ["Gyro", "Magnetometer", "CoarseSunSensors", "GNSS"];
rawFactories = {@createGyroMeasurementBus, @createMagnetometerMeasurementBus, ...
    @createCoarseSunSensorMeasurementBus, @createGnssMeasurementBus};
reportNames = ["GyroReportBus", "MagnetometerReportBus", ...
    "CoarseSunSensorReportBus", "GnssReportBus"];
elements = repmat(Simulink.BusElement, 1, numel(names));
for index = 1:numel(names)
    raw = rawFactories{index}(targetWorkspace);
    timestamp = Simulink.BusElement;
    timestamp.Name = 'receive_time_s';
    timestamp.Unit = 's';
    timestamp.Description = 'Driver receipt time on the simulation clock; held until the next receipt';
    sequence = Simulink.BusElement;
    sequence.Name = 'sequence_id';
    sequence.DataType = 'uint32';
    sequence.Description = 'Receipt counter, modulo 2^32; compare with last processed ID for inequality';
    report = Simulink.Bus;
    report.Elements = [raw.Elements(:); timestamp; sequence];
    report.Description = names(index) + " driver report";
    if string(targetWorkspace) == "base"
        assignin('base', reportNames(index), report);
    end
    elements(index) = Simulink.BusElement;
    elements(index).Name = names(index);
    elements(index).DataType = "Bus: " + reportNames(index);
end
SensorReportBus = Simulink.Bus;
SensorReportBus.Elements = elements;
SensorReportBus.Description = 'Onboard driver reports consumed by GNC';
if string(targetWorkspace) == "base"
    assignin('base', 'SensorReportBus', SensorReportBus);
end
end
