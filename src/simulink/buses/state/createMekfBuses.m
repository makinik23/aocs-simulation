function createMekfBuses(targetWorkspace)
%CREATEMEKFBUSES Define the MEKF state and diagnostics buses.

if nargin < 1
    targetWorkspace = "base";
end

stateNames = ["estimate", "previous_rate_B_rad_s", "older_rate_B_rad_s", ...
    "have_previous_rate", "have_two_rates", "previous_gap_adjacent", ...
    "last_correction_time_s"];
stateTypes = ["Bus: AttitudeEstimateBus", "double", "double", ...
    "boolean", "boolean", "boolean", "double"];
stateDimensions = {[1 1], [3 1], [3 1], 1, 1, 1, 1};
stateElements(1, numel(stateNames)) = Simulink.BusElement;

for index = 1:numel(stateNames)
    element = Simulink.BusElement;
    element.Name = stateNames(index);
    element.DataType = stateTypes(index);
    element.Dimensions = stateDimensions{index};
    element.Description = stateNames(index);
    stateElements(index) = element;
end

stateBus = Simulink.Bus;
stateBus.Elements = stateElements;
stateBus.Description = "MEKF state";

diagNames = ["initialized_this_step", "prediction_accepted", ...
    "mag_accepted", "sun_accepted", "mag_nis", "sun_nis"];
diagTypes = ["boolean", "boolean", "boolean", "boolean", "double", "double"];
diagElements(1, numel(diagNames)) = Simulink.BusElement;

for index = 1:numel(diagNames)
    element = Simulink.BusElement;
    element.Name = diagNames(index);
    element.DataType = diagTypes(index);
    element.Description = diagNames(index);
    diagElements(index) = element;
end

diagnosticsBus = Simulink.Bus;
diagnosticsBus.Elements = diagElements;
diagnosticsBus.Description = "MEKF diagnostics";

if string(targetWorkspace) == "base"
    assignin("base", "MekfStateBus", stateBus);
    assignin("base", "MekfDiagnosticsBus", diagnosticsBus);
end

end
