function DriverReadStatusBus = createDriverReadStatusBus(targetWorkspace)
%CREATEDRIVERREADSTATUSBUS Define independent acquisition status flags.
% Description:
%   Separates receipt availability, unread data and overwritten data from
%   sensor validity. An overrun does not invalidate the latest report.
%
% Arguments:
%   targetWorkspace - "base" publishes the bus; other values return it only.
%
% Outputs:
%   DriverReadStatusBus - Bus with three scalar boolean flags.

if nargin < 1
    targetWorkspace = "base";
end
names = ["has_data", "new_data", "overrun"];
descriptions = ["At least one receipt since initialization", ...
    "At least one receipt since the previous consuming read", ...
    "An unread report was overwritten since the previous consuming read"];
elements = repmat(Simulink.BusElement, 1, numel(names));
for index = 1:numel(names)
    elements(index).Name = names(index);
    elements(index).DataType = 'boolean';
    elements(index).Description = descriptions(index);
end
DriverReadStatusBus = Simulink.Bus;
DriverReadStatusBus.Elements = elements;
DriverReadStatusBus.Description = 'Status snapshot returned by a driver Read method';
if string(targetWorkspace) == "base"
    assignin('base', 'DriverReadStatusBus', DriverReadStatusBus);
end
end
