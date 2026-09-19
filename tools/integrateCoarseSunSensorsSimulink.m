function integrateCoarseSunSensorsSimulink(modelFile)
% Description:
%   Completes the coarse sun sensor subsystem and publishes CSS outputs through
%   SensorMeasurementBus.
%
% Arguments:
%   modelFile - Optional path to the AOCS plant model.
%
% Outputs:
%   None.

projectRoot = fileparts(fileparts(mfilename("fullpath")));
if nargin < 1
    modelFile = fullfile(projectRoot, "models", "aocs_plant.slx");
end

addpath(projectRoot);
setupAocsPaths(projectRoot);
setupAocsSimulation(fullfile(projectRoot, "config", "AocsSimulationConfig.json"));

[~, modelName] = fileparts(modelFile);
load_system(modelFile);
cleanup = onCleanup(@() closeIfLoaded(modelName));

replaceCoarseSunSensorsSubsystem(modelName);
publishSensorMeasurements(modelName);

set_param(modelName, "SimulationCommand", "update");
save_system(modelName, modelFile);
end

function replaceCoarseSunSensorsSubsystem(modelName)
% Description:
%   Replaces the placeholder CSS subsystem with a six-panel measurement model.

sensors = modelName + "/Sensors";
css = sensors + "/Coarse Sun Sensors";
position = [130 382 355 478];

deleteBlockIfExists(css);
add_block("simulink/Ports & Subsystems/Subsystem", css, ...
    "Position", position);

buildCoarseSunSensorsSubsystem(css);
connectEnvironmentToCss(sensors);
end

function buildCoarseSunSensorsSubsystem(parent)
% Description:
%   Builds the CSS subsystem internals.

deleteBlockIfExists(parent + "/In1");
deleteBlockIfExists(parent + "/Out1");

add_block("simulink/Ports & Subsystems/In1", parent + "/Environment", ...
    "Port", "1", "Position", [-95 98 -65 112]);
add_block("simulink/Signal Routing/Bus Selector", ...
    parent + "/Select Sun Environment", ...
    "OutputSignals", "sun_B_unit,solar_flux_shadowed_W_m2,sun_visibility", ...
    "Position", [35 72 40 148]);
add_block("simulink/Sources/Constant", parent + "/CssConfig", ...
    "Value", "AOCS_SensorConfig.CoarseSunSensors", ...
    "OutDataTypeStr", "Bus: CoarseSunSensorConfigBus", ...
    "Position", [35 215 65 245]);
add_block("simulink/Sources/Random Number", parent + "/W", ...
    "Mean", "zeros(6,1)", ...
    "Variance", "ones(6,1)", ...
    "Seed", "AOCS_SensorConfig.CoarseSunSensors.noise_seed", ...
    "SampleTime", "AOCS_SensorConfig.CoarseSunSensors.sample_time_s", ...
    "Position", [35 295 65 325]);
add_block("simulink/User-Defined Functions/MATLAB Function", ...
    parent + "/Coarse Sun Sensor Model", ...
    "Position", [195 80 485 325]);
setChartScript(parent + "/Coarse Sun Sensor Model", cssModelScript());
add_block("simulink/Signal Routing/Bus Creator", ...
    parent + "/CSS Measurement Bus Assembly", ...
    "Inputs", "4", ...
    "OutDataTypeStr", "Bus: CoarseSunSensorMeasurementBus", ...
    "NonVirtualBus", "off", ...
    "Position", [560 115 565 255]);
add_block("simulink/Ports & Subsystems/Out1", ...
    parent + "/CoarseSunSensors", ...
    "Port", "1", ...
    "Position", [655 178 685 192]);

addNamedLine(parent, "Environment/1", "Select Sun Environment/1", ...
    "Environment");
addNamedLine(parent, "Select Sun Environment/1", ...
    "Coarse Sun Sensor Model/1", "sun_B_unit");
addNamedLine(parent, "Select Sun Environment/2", ...
    "Coarse Sun Sensor Model/2", "solar_flux_shadowed_W_m2");
addNamedLine(parent, "Select Sun Environment/3", ...
    "Coarse Sun Sensor Model/3", "sun_visibility");
addNamedLine(parent, "CssConfig/1", ...
    "Coarse Sun Sensor Model/4", "css");
addNamedLine(parent, "W/1", ...
    "Coarse Sun Sensor Model/5", "noise_unit");
addNamedLine(parent, "Coarse Sun Sensor Model/1", ...
    "CSS Measurement Bus Assembly/1", "sun_B_unit");
addNamedLine(parent, "Coarse Sun Sensor Model/2", ...
    "CSS Measurement Bus Assembly/2", "irradiance_W_m2");
addNamedLine(parent, "Coarse Sun Sensor Model/3", ...
    "CSS Measurement Bus Assembly/3", "panel_signals_W_m2");
addNamedLine(parent, "Coarse Sun Sensor Model/4", ...
    "CSS Measurement Bus Assembly/4", "valid");
addNamedLine(parent, "CSS Measurement Bus Assembly/1", ...
    "CoarseSunSensors/1", "CoarseSunSensors");
end

function connectEnvironmentToCss(parent)
% Description:
%   Connects EnvironmentBus from PlantStateBus to the CSS subsystem.

selector = firstBlockByType(parent, "BusSelector");
set_param(selector, "OutputSignals", "AttitudeState,Environment");
addNamedLine(parent, blockPortSpec(selector, 2), ...
    "Coarse Sun Sensors/1", "Environment");
end

function publishSensorMeasurements(modelName)
% Description:
%   Adds CSS measurements to the top-level SensorMeasurementBus.

parent = modelName + "/Sensors";
assembly = parent + "/Sensor Measurement Bus Assembly";

if getSimulinkBlockHandle(assembly) <= 0
    error("AOCS:Sensors:MissingBlock", ...
        "Missing sensor measurement bus assembly: %s.", assembly);
end

set_param(assembly, "Inputs", "3", ...
    "OutDataTypeStr", "Bus: SensorMeasurementBus");
addNamedLine(parent, "Coarse Sun Sensors/1", ...
    "Sensor Measurement Bus Assembly/3", "CoarseSunSensors");
end

function script = cssModelScript()
% Description:
%   Returns the MATLAB Function script used by the CSS measurement model.

script = strjoin([ ...
    "function [sun_B_unit_meas, irradiance_W_m2, panel_signals_W_m2, valid] = coarseSunSensors(sun_B_unit, solar_flux_shadowed_W_m2, sun_visibility, css, noise_unit)";
    "%#codegen";
    "panel_cosines = css.panel_normals_B.' * normalizeVector(sun_B_unit, [1.0; 0.0; 0.0]);";
    "visible = panel_cosines >= css.fov_cos;";
    "ideal_panel_signals_W_m2 = solar_flux_shadowed_W_m2 .* max(panel_cosines, 0.0) .* double(visible);";
    "panel_signals_W_m2 = (1.0 + css.scale_factor) .* ideal_panel_signals_W_m2 + css.bias_W_m2 + css.noise_std_W_m2 .* noise_unit;";
    "panel_signals_W_m2 = min(max(panel_signals_W_m2, 0.0), css.range_W_m2);";
    "if css.resolution_W_m2 > 0.0";
    "    panel_signals_W_m2 = round(panel_signals_W_m2 ./ css.resolution_W_m2) .* css.resolution_W_m2;";
    "end";
    "";
    "sun_raw_B = [";
    "    panel_signals_W_m2(1) - panel_signals_W_m2(2);";
    "    panel_signals_W_m2(3) - panel_signals_W_m2(4);";
    "    panel_signals_W_m2(5) - panel_signals_W_m2(6)];";
    "sun_B_unit_meas = normalizeVector(sun_raw_B, [1.0; 0.0; 0.0]);";
    "irradiance_W_m2 = max(panel_signals_W_m2);";
    "valid = double(css.enabled > 0.5 && sun_visibility > 0.5 && irradiance_W_m2 >= css.min_valid_irradiance_W_m2);";
    "";
    "if css.mode_id == 2.0";
    "    panel_signals_W_m2 = css.failure_panel_signals_W_m2;";
    "    sun_B_unit_meas = normalizeVector(css.failure_sun_B_unit, [1.0; 0.0; 0.0]);";
    "    irradiance_W_m2 = css.failure_irradiance_W_m2;";
    "    valid = double(css.failure_valid > 0.5);";
    "end";
    "";
    "if css.enabled <= 0.5";
    "    panel_signals_W_m2 = zeros(6, 1);";
    "    sun_B_unit_meas = [1.0; 0.0; 0.0];";
    "    irradiance_W_m2 = 0.0;";
    "    valid = 0.0;";
    "end";
    "end";
    "";
    "function unitVector = normalizeVector(vector, defaultVector)";
    "normVector = norm(vector);";
    "if normVector <= 1.0e-12";
    "    unitVector = defaultVector;";
    "else";
    "    unitVector = vector ./ normVector;";
    "end";
    "end"], newline);
end

function block = firstBlockByType(parent, blockType)
% Description:
%   Returns the first direct child block with the requested BlockType.

matches = find_system(parent, "SearchDepth", 1, "BlockType", blockType);
if isempty(matches)
    error("AOCS:Sensors:MissingBlock", ...
        "No %s block found under %s.", blockType, parent);
end
block = string(matches{1});
end

function spec = blockPortSpec(block, portNumber)
% Description:
%   Builds a block/port string relative to the block parent for add_line.

[~, name] = fileparts(char(block));
spec = string(name) + "/" + string(portNumber);
end

function addNamedLine(parent, sourcePort, destinationPort, signalName)
% Description:
%   Adds a line and assigns an explicit signal name when possible.

deleteDestinationLine(parent, destinationPort);
line = add_line(parent, sourcePort, destinationPort, "autorouting", "on");
if strlength(string(signalName)) == 0
    return;
end

try
    set_param(line, "Name", char(signalName));
catch exception
    if ~contains(exception.message, "Bus Selector")
        rethrow(exception);
    end
end
end

function deleteDestinationLine(parent, destinationPort)
% Description:
%   Deletes any existing line attached to a destination port string.

parts = split(string(destinationPort), "/");
if numel(parts) < 2
    return;
end

inputIndex = str2double(parts(end));
if isnan(inputIndex)
    return;
end

block = parent + "/" + strjoin(parts(1:end - 1), "/");
deleteInputLine(block, inputIndex);
end

function deleteInputLine(block, inputIndex)
% Description:
%   Deletes the line attached to a specific block input port, if present.

if getSimulinkBlockHandle(block) <= 0
    return;
end

ports = get_param(block, "PortHandles");
if numel(ports.Inport) < inputIndex
    return;
end

line = get_param(ports.Inport(inputIndex), "Line");
if line > 0
    delete_line(line);
end
end

function setChartScript(block, script)
% Description:
%   Replaces the script of a MATLAB Function block.

root = sfroot();
blockName = string(get_param(block, "Name"));
chart = root.find("-isa", "Stateflow.EMChart", "Path", char(block), ...
    "Name", char(blockName));
if isempty(chart)
    error("AOCS:Sensors:MissingChart", ...
        "MATLAB Function chart not found: %s", block);
end
chart.Script = script;
end

function deleteBlockIfExists(block)
% Description:
%   Deletes a block when it exists.

if getSimulinkBlockHandle(block) > 0
    delete_block(block);
end
end

function closeIfLoaded(modelName)
% Description:
%   Closes a loaded model without saving additional changes.

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end
