function integrateMagnetometerSensorSimulink(modelFile)
% Description:
%   Completes the magnetometer sensor path and publishes gyro plus
%   magnetometer outputs through SensorMeasurementBus.
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

configureGyroMeasurementOutput(modelName);
replaceMagnetometerSubsystem(modelName);
publishSensorMeasurements(modelName);
connectSensorsToGnc(modelName);

set_param(modelName, "SimulationCommand", "update");
save_system(modelName, modelFile);
end

function configureGyroMeasurementOutput(modelName)
% Description:
%   Wraps the existing gyro vector output in GyroMeasurementBus.

parent = modelName + "/Sensors/Gyro";
outport = ensureOutport(parent, ["Gyro", "omega"], "Gyro", 1, ...
    [2180 208 2210 222]);
deleteInputLine(outport, 1);

deleteBlockIfExists(parent + "/Gyro Measurement Bus Assembly");
deleteBlockIfExists(parent + "/Failure Valid");
deleteBlockIfExists(parent + "/Nominal Valid");
deleteBlockIfExists(parent + "/Select Gyro Valid");
deleteBlockIfExists(parent + "/Gyro Valid Product");

add_block("simulink/Signal Routing/Bus Creator", ...
    parent + "/Gyro Measurement Bus Assembly", ...
    "Inputs", "2", ...
    "OutDataTypeStr", "Bus: GyroMeasurementBus", ...
    "NonVirtualBus", "off", ...
    "Position", [1985 190 1990 250]);
add_block("simulink/Sources/Constant", parent + "/Failure Valid", ...
    "Value", "AOCS_SensorConfig.Gyro.failure_valid", ...
    "Position", [1545 330 1575 360]);
add_block("simulink/Sources/Constant", parent + "/Nominal Valid", ...
    "Value", "1", ...
    "Position", [1545 380 1575 410]);
add_block("simulink/Signal Routing/Switch", parent + "/Select Gyro Valid", ...
    "Criteria", "u2 > Threshold", ...
    "Threshold", "0", ...
    "Position", [1660 328 1710 412]);
add_block("simulink/Math Operations/Product", parent + "/Gyro Valid Product", ...
    "Inputs", "**", ...
    "Position", [1840 335 1885 385]);

failureMode = compareBlockByXPosition(parent, 1500, "left");
addNamedLine(parent, "Failure Valid/1", "Select Gyro Valid/1", "");
addNamedLine(parent, blockPortSpec(failureMode, 1), "Select Gyro Valid/2", "");
addNamedLine(parent, "Nominal Valid/1", "Select Gyro Valid/3", "");
addNamedLine(parent, "Gyro Enabled/1", "Gyro Valid Product/1", "");
addNamedLine(parent, "Select Gyro Valid/1", "Gyro Valid Product/2", "valid");
addNamedLine(parent, "Switch/1", "Gyro Measurement Bus Assembly/1", "omega_rad_s");
addNamedLine(parent, "Gyro Valid Product/1", ...
    "Gyro Measurement Bus Assembly/2", "valid");
addNamedLine(parent, "Gyro Measurement Bus Assembly/1", "Gyro/1", "Gyro");
end

function replaceMagnetometerSubsystem(modelName)
% Description:
%   Replaces the placeholder magnetometer with a calibrated/noisy sensor
%   model adapted from the gyro path.

sensors = modelName + "/Sensors";
magnetometer = sensors + "/Magnetometer";
position = [130 182 355 278];

deleteBlockIfExists(magnetometer);
add_block(sensors + "/Gyro", magnetometer, "Position", position);

configureMagnetometerBlocks(magnetometer);
connectEnvironmentToMagnetometer(sensors);
end

function configureMagnetometerBlocks(parent)
% Description:
%   Renames and parameterizes copied gyro blocks for magnetometer units.

renameBlockIfExists(parent + "/AttitudeState", "Environment");
renameBlockIfExists(parent + "/GyroConfig", "MagnetometerConfig");
renameBlockIfExists(parent + "/Gyro Calibration", "Magnetometer Calibration");
renameBlockIfExists(parent + "/Gyro Enabled", "Magnetometer Enabled");
renameBlockIfExists(parent + "/Select Gyro Mode", "Select Magnetometer Mode");
renameBlockIfExists(parent + "/Gyro Measurement Bus Assembly", ...
    "Magnetometer Measurement Bus Assembly");
renameBlockIfExists(parent + "/Select Gyro Valid", "Select Magnetometer Valid");
renameBlockIfExists(parent + "/Gyro Valid Product", "Magnetometer Valid Product");
renameBlockIfExists(parent + "/Gyro", "Magnetometer");
renameBlockIfExists(parent + "/Bias/bias_rad_s", "bias_T");
deleteBlockIfExists(parent + "/Scope");

selector = firstBlockByType(parent, "BusSelector");
set_param(selector, ...
    "Name", "Select Magnetic Field", ...
    "OutputSignals", "B_B_T");

set_param(firstBlockByType(parent, "ZeroOrderHold"), ...
    "SampleTime", "AOCS_SensorConfig.Magnetometer.sample_time_s");
set_param(parent + "/MagnetometerConfig", ...
    "Value", "AOCS_SensorConfig.Magnetometer", ...
    "OutDataTypeStr", "Bus: MagnetometerConfigBus");
set_param(parent + "/Magnetometer Enabled", ...
    "Value", "AOCS_SensorConfig.Magnetometer.enabled");
set_param(parent + "/Mode ID", ...
    "Value", "AOCS_SensorConfig.Magnetometer.mode_id");
set_param(parent + "/Failure Output", ...
    "Value", "AOCS_SensorConfig.Magnetometer.failure_output_T");
set_param(parent + "/Failure Valid", ...
    "Value", "AOCS_SensorConfig.Magnetometer.failure_valid");
set_param(parent + "/Saturation", ...
    "UpperLimit", "AOCS_SensorConfig.Magnetometer.range_T", ...
    "LowerLimit", "-AOCS_SensorConfig.Magnetometer.range_T");
set_param(parent + "/Quantizer", ...
    "QuantizationInterval", "AOCS_SensorConfig.Magnetometer.resolution_T");
set_param(parent + "/Gain", ...
    "Gain", "AOCS_SensorConfig.Magnetometer.noise_std_T");
set_param(parent + "/W", ...
    "Seed", "AOCS_SensorConfig.Magnetometer.noise_seed", ...
    "SampleTime", "AOCS_SensorConfig.Magnetometer.sample_time_s");
set_param(parent + "/Bias/Random Number", ...
    "Seed", "AOCS_SensorConfig.Magnetometer.bias_seed", ...
    "SampleTime", "AOCS_SensorConfig.Magnetometer.sample_time_s");
set_param(parent + "/Bias/Gain", ...
    "Gain", "AOCS_SensorConfig.Magnetometer.bias_random_walk_step_std_T");
set_param(parent + "/Bias/Unit Delay", ...
    "SampleTime", "AOCS_SensorConfig.Magnetometer.sample_time_s", ...
    "InitialCondition", "AOCS_SensorConfig.Magnetometer.bias_initial_T");
set_param(parent + "/Magnetometer Measurement Bus Assembly", ...
    "OutDataTypeStr", "Bus: MagnetometerMeasurementBus");

setChartScript(parent + "/Magnetometer Calibration", ...
    magnetometerCalibrationScript());

renameOutputSignal(parent + "/Switch", "B_B_T");
renameOutputSignal(parent + "/Magnetometer Measurement Bus Assembly", ...
    "Magnetometer");
end

function connectEnvironmentToMagnetometer(parent)
% Description:
%   Connects EnvironmentBus from PlantStateBus to the magnetometer subsystem.

selector = firstBlockByType(parent, "BusSelector");
set_param(selector, "OutputSignals", "AttitudeState,Environment");
addNamedLine(parent, blockPortSpec(selector, 2), "Magnetometer/1", "Environment");
end

function publishSensorMeasurements(modelName)
% Description:
%   Assembles gyro and magnetometer measurement buses into SensorMeasurementBus.

parent = modelName + "/Sensors";
outport = ensureOutport(parent, ["SensorMeasurements", "Out1"], ...
    "SensorMeasurements", 1, [555 128 585 142]);
assembly = parent + "/Sensor Measurement Bus Assembly";

deleteBlockIfExists(assembly);
deleteInputLine(outport, 1);

add_block("simulink/Signal Routing/Bus Creator", assembly, ...
    "Inputs", "2", ...
    "OutDataTypeStr", "Bus: SensorMeasurementBus", ...
    "NonVirtualBus", "off", ...
    "Position", [425 75 430 205]);

addNamedLine(parent, "Gyro/1", "Sensor Measurement Bus Assembly/1", "Gyro");
addNamedLine(parent, "Magnetometer/1", ...
    "Sensor Measurement Bus Assembly/2", "Magnetometer");
addNamedLine(parent, "Sensor Measurement Bus Assembly/1", ...
    "SensorMeasurements/1", "SensorMeasurementBus");
end

function connectSensorsToGnc(modelName)
% Description:
%   Connects SensorMeasurementBus to the placeholder GNC input when present.

gnc = modelName + "/GNC";
if getSimulinkBlockHandle(gnc) <= 0
    return;
end

renameBlockIfExists(gnc + "/In1", "SensorMeasurements");
deleteInputLine(gnc, 1);
addNamedLine(modelName, "Sensors/1", "GNC/1", "SensorMeasurements");
end

function script = magnetometerCalibrationScript()
% Description:
%   Returns the MATLAB Function script used by the magnetometer calibration block.

script = strjoin([ ...
    "function B_cal_T = applyMagnetometerCalibration(B_B_T, magnetometer)";
    "%#codegen";
    "S = eye(3) + diag(magnetometer.scale_factor);";
    "B_cal_T = magnetometer.misalignment_matrix * S * B_B_T;";
    "end"], newline);
end

function block = ensureOutport(parent, candidates, newName, portNumber, position)
% Description:
%   Returns an outport block path, renaming an obsolete block when needed.

block = ensurePortBlock(parent, candidates, newName, portNumber, position, ...
    "simulink/Ports & Subsystems/Out1");
end

function block = ensurePortBlock(parent, candidates, newName, portNumber, ...
        position, libraryBlock)
% Description:
%   Creates or renames a port block while preserving existing connections.

newBlock = parent + "/" + newName;
if getSimulinkBlockHandle(newBlock) > 0
    block = newBlock;
elseif getSimulinkBlockHandle(parent + "/" + candidates(1)) > 0
    set_param(parent + "/" + candidates(1), "Name", newName);
    block = newBlock;
else
    existing = firstExistingBlock(parent + "/" + candidates);
    if strlength(existing) > 0
        set_param(existing, "Name", newName);
        block = newBlock;
    else
        add_block(libraryBlock, newBlock, "Position", position);
        block = newBlock;
    end
end

set_param(block, "Port", num2str(portNumber));
set_param(block, "Position", position);
end

function block = firstExistingBlock(candidates)
% Description:
%   Returns the first existing block path, or "" when none exists.

block = "";
for index = 1:numel(candidates)
    if getSimulinkBlockHandle(candidates(index)) > 0
        block = candidates(index);
        return;
    end
end
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

function block = compareBlockByXPosition(parent, splitX, side)
% Description:
%   Selects the direct child Compare To Constant subsystem by horizontal position.

matches = find_system(parent, "SearchDepth", 1, "Regexp", "on", ...
    "Name", "Compare.*To Constant");
if isempty(matches)
    error("AOCS:Sensors:MissingBlock", ...
        "No Compare To Constant block found under %s.", parent);
end

for index = 1:numel(matches)
    position = get_param(matches{index}, "Position");
    isLeft = position(1) < splitX;
    if (side == "left" && isLeft) || (side == "right" && ~isLeft)
        block = string(matches{index});
        return;
    end
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

function renameBlockIfExists(block, newName)
% Description:
%   Renames a block when it exists.

if getSimulinkBlockHandle(block) > 0
    set_param(block, "Name", newName);
end
end

function renameOutputSignal(block, signalName)
% Description:
%   Renames the first output line of a block when connected.

ports = get_param(block, "PortHandles");
line = get_param(ports.Outport(1), "Line");
if line > 0
    set_param(line, "Name", char(signalName));
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
