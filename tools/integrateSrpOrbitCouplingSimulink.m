function integrateSrpOrbitCouplingSimulink(modelFile)
% Description:
%   Connects the project SRP force model to the high-precision orbit propagator
%   through the shared translational disturbance acceleration input.
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
AOCS = setupAocsSimulation(fullfile(projectRoot, "config", "AocsSimulationConfig.json"));

[~, modelName] = fileparts(modelFile);
load_system(modelFile);
cleanup = onCleanup(@() closeIfLoaded(modelName));
applyAocsSimulationSettings(modelName, AOCS);

publishSrpForceProducts(modelName);
connectSrpAcceleration(modelName);
renameOrbitAccelerationPorts(modelName);
publishDisturbanceProducts(modelName);
publishEnvironmentProducts(modelName);
markLoggedSignals(modelName);

set_param(modelName, "SimulationCommand", "update");
save_system(modelName, modelFile);
end

function publishSrpForceProducts(modelName)
% Description:
%   Publishes SRP torque, body force, and pressure on AOCS_SrpBus.

parent = modelName + "/Orbit & Environment/Environment Products/Solar Radiation Pressure";
srpFunction = parent + "/MATLAB Function";
busCreator = parent + "/SRP Bus Assembly";

deleteBlockIfExists(parent + "/Terminator");
deleteBlockIfExists(parent + "/Terminator1");
for inputIndex = 1:3
    deleteInputLine(busCreator, inputIndex);
end
set_param(busCreator, "Inputs", "3");

addNamedLine(parent, "MATLAB Function/1", "SRP Bus Assembly/1", "M_srp_B_Nm");
addNamedLine(parent, "MATLAB Function/2", "SRP Bus Assembly/2", "F_srp_B_N");
addNamedLine(parent, "MATLAB Function/3", "SRP Bus Assembly/3", "P_srp_N_m2");

if getSimulinkBlockHandle(srpFunction) <= 0
    error("AOCS:SRP:MissingBlock", "Could not find SRP MATLAB Function block.");
end
end

function connectSrpAcceleration(modelName)
% Description:
%   Converts SRP body force into inertial acceleration and sums it with aero.

parent = modelName + "/Orbit & Environment/Disturbance Torques";
selectSrp = parent + "/Select SRP";
srpAcceleration = parent + "/SRP Inertial Acceleration";
accelerationSum = parent + "/Orbit Acceleration Sum";
orbitAcceleration = ensureOutport(parent, "AerodynamicAcceleration", ...
    "OrbitAcceleration", 2, [895 678 925 692]);

deleteBlockIfExists(srpAcceleration);
deleteBlockIfExists(accelerationSum);
deleteInputLine(orbitAcceleration, 1);

set_param(selectSrp, "OutputSignals", "M_srp_B_Nm,F_srp_B_N");

add_block("simulink/User-Defined Functions/MATLAB Function", srpAcceleration, ...
    "Position", [850 555 1110 670]);
setChartScript(srpAcceleration, srpAccelerationScript());

add_block("simulink/Math Operations/Sum", accelerationSum, ...
    "Inputs", "++", "Position", [1140 650 1160 700]);

addNamedLine(parent, "Select SRP/2", "SRP Inertial Acceleration/1", "F_srp_B_N");
addNamedLine(parent, "Select Attitude State/1", "SRP Inertial Acceleration/2", "DCM_be");
addNamedLine(parent, "Select Aerodynamic Config/1", "SRP Inertial Acceleration/3", "mass_kg");
addNamedLine(parent, "Sentman Multispecies Panels/4", "Orbit Acceleration Sum/1", "a_aero_I_m_s2");
addNamedLine(parent, "SRP Inertial Acceleration/2", "Orbit Acceleration Sum/2", "a_srp_I_m_s2");
addNamedLine(parent, "Orbit Acceleration Sum/1", "OrbitAcceleration/1", "a_dist_I_m_s2");
end

function renameOrbitAccelerationPorts(modelName)
% Description:
%   Renames the orbit-propagator acceleration ports from aero-only to generic.

orbitAndEnvironment = modelName + "/Orbit & Environment";
orbitAndTime = orbitAndEnvironment + "/Orbit Propagator & Time";
orbitState = orbitAndTime + "/Orbit State";
propagatorSubsystem = orbitState + "/Orbit Propagator";

ensureInport(orbitAndTime, "AerodynamicAcceleration", "OrbitAcceleration", 2, [30 120 60 134]);
ensureInport(orbitState, "AerodynamicAcceleration", "OrbitAcceleration", 1, [25 90 55 104]);
ensureInport(propagatorSubsystem, "AerodynamicAcceleration", "OrbitAcceleration", 1, [25 130 55 144]);
end

function publishDisturbanceProducts(modelName)
% Description:
%   Rebuilds AOCS_DisturbanceBus with SRP and aero acceleration products.

parent = modelName + "/Orbit & Environment/Disturbance Torques";
busCreator = parent + "/Disturbance Bus Assembly";

for inputIndex = 1:12
    deleteInputLine(busCreator, inputIndex);
end
set_param(busCreator, "Inputs", "12");

addNamedLine(parent, "Add1/1", "Disturbance Bus Assembly/1", "M_dist_B_Nm");
addNamedLine(parent, "Select SRP/1", "Disturbance Bus Assembly/2", "M_srp_B_Nm");
addNamedLine(parent, "Sentman Multispecies Panels/2", "Disturbance Bus Assembly/3", "M_aero_B_Nm");
addNamedLine(parent, "Select SRP/2", "Disturbance Bus Assembly/4", "F_srp_B_N");
addNamedLine(parent, "SRP Inertial Acceleration/1", "Disturbance Bus Assembly/5", "F_srp_I_N");
addNamedLine(parent, "SRP Inertial Acceleration/2", "Disturbance Bus Assembly/6", "a_srp_I_m_s2");
addNamedLine(parent, "Sentman Multispecies Panels/1", "Disturbance Bus Assembly/7", "F_aero_B_N");
addNamedLine(parent, "Sentman Multispecies Panels/3", "Disturbance Bus Assembly/8", "F_aero_I_N");
addNamedLine(parent, "Sentman Multispecies Panels/4", "Disturbance Bus Assembly/9", "a_aero_I_m_s2");
addNamedLine(parent, "Orbit Acceleration Sum/1", "Disturbance Bus Assembly/10", "a_dist_I_m_s2");
addNamedLine(parent, "Relative Wind/3", "Disturbance Bus Assembly/11", "q_dyn_N_m2");
addNamedLine(parent, "Relative Wind/2", "Disturbance Bus Assembly/12", "v_rel_norm_m_s");
end

function publishEnvironmentProducts(modelName)
% Description:
%   Publishes expanded disturbance products on AOCS_EnvironmentBus.

parent = modelName + "/Orbit & Environment/Environment Bus Assembly";
selector = parent + "/Select Disturbance";
busCreator = parent + "/Environment Bus Creator";

for inputIndex = [1, 12:22]
    deleteInputLine(busCreator, inputIndex);
end
set_param(selector, "OutputSignals", ...
    "M_dist_B_Nm,M_srp_B_Nm,F_srp_B_N,F_srp_I_N,a_srp_I_m_s2," + ...
    "M_aero_B_Nm,F_aero_B_N,F_aero_I_N,a_aero_I_m_s2,a_dist_I_m_s2," + ...
    "q_dyn_N_m2,v_rel_norm_m_s");
set_param(busCreator, "Inputs", "22");

addNamedLine(parent, "Select Disturbance/1", "Environment Bus Creator/1", "M_dist_B_Nm");
for outputIndex = 2:12
    addNamedLine(parent, "Select Disturbance/" + outputIndex, ...
        "Environment Bus Creator/" + (outputIndex + 10), disturbanceOutputName(outputIndex));
end
end

function markLoggedSignals(modelName)
% Description:
%   Marks SRP and translational disturbance products for logsout diagnostics.

srpFunction = modelName + "/Orbit & Environment/Environment Products/Solar Radiation Pressure/MATLAB Function";
disturbance = modelName + "/Orbit & Environment/Disturbance Torques";
srpAcceleration = firstExistingBlock([ ...
    disturbance + "/SRP Disturbance Path/SRP Inertial Acceleration", ...
    disturbance + "/SRP Inertial Acceleration"]);
sentmanPanels = firstExistingBlock([ ...
    disturbance + "/Aerodynamic Disturbance Path/Sentman Multispecies Panels", ...
    disturbance + "/Sentman Multispecies Panels"]);
accelerationSum = firstExistingBlock([ ...
    disturbance + "/Output Assembly/Orbit Acceleration Sum", ...
    disturbance + "/Orbit Acceleration Sum"]);

markSignal(srpFunction, 1);
markSignal(srpFunction, 2);
markSignal(srpFunction, 3);
markSignal(srpAcceleration, 1);
markSignal(srpAcceleration, 2);
markSignal(sentmanPanels, 1);
markSignal(sentmanPanels, 2);
markSignal(sentmanPanels, 3);
markSignal(sentmanPanels, 4);
markSignal(accelerationSum, 1);
end

function block = firstExistingBlock(candidates)
% Description:
%   Returns the first block path that exists, or the first candidate otherwise.

block = candidates(1);
for index = 1:numel(candidates)
    if getSimulinkBlockHandle(candidates(index)) > 0
        block = candidates(index);
        return;
    end
end
end

function markSignal(block, outputPort)
% Description:
%   Enables signal logging for an output port when the block exists.

if getSimulinkBlockHandle(block) > 0
    Simulink.sdi.markSignalForStreaming(block, outputPort, "on");
end
end

function signalName = disturbanceOutputName(outputIndex)
% Description:
%   Maps Select Disturbance output index to the corresponding bus element name.

names = ["M_dist_B_Nm", "M_srp_B_Nm", "F_srp_B_N", "F_srp_I_N", ...
    "a_srp_I_m_s2", "M_aero_B_Nm", "F_aero_B_N", "F_aero_I_N", ...
    "a_aero_I_m_s2", "a_dist_I_m_s2", "q_dyn_N_m2", "v_rel_norm_m_s"];
signalName = names(outputIndex);
end

function block = ensureOutport(parent, oldName, newName, portNumber, position)
% Description:
%   Returns an outport block path, renaming an obsolete block when needed.

block = ensurePortBlock(parent, oldName, newName, portNumber, position, ...
    "simulink/Ports & Subsystems/Out1");
end

function block = ensureInport(parent, oldName, newName, portNumber, position)
% Description:
%   Returns an inport block path, renaming an obsolete block when needed.

block = ensurePortBlock(parent, oldName, newName, portNumber, position, ...
    "simulink/Ports & Subsystems/In1");
end

function block = ensurePortBlock(parent, oldName, newName, portNumber, position, libraryBlock)
% Description:
%   Creates or renames a port block while preserving existing connections.

oldBlock = parent + "/" + oldName;
newBlock = parent + "/" + newName;

if getSimulinkBlockHandle(newBlock) > 0
    if getSimulinkBlockHandle(oldBlock) > 0
        deleteBlockIfExists(oldBlock);
    end
    block = newBlock;
elseif getSimulinkBlockHandle(oldBlock) > 0
    set_param(oldBlock, "Name", newName);
    block = newBlock;
else
    add_block(libraryBlock, newBlock, "Position", position);
    block = newBlock;
end

set_param(block, "Port", num2str(portNumber));
end

function setChartScript(block, script)
% Description:
%   Replaces the script of a MATLAB Function block.

root = sfroot();
blockName = string(get_param(block, "Name"));
chart = root.find("-isa", "Stateflow.EMChart", "Path", char(block), ...
    "Name", char(blockName));
if isempty(chart)
    error("AOCS:SRP:MissingChart", ...
        "MATLAB Function chart not found: %s", block);
end
chart.Script = script;
end

function line = addNamedLine(parent, sourcePort, destinationPort, signalName)
% Description:
%   Adds a line and assigns an explicit signal name for downstream bus selectors.

deleteDestinationLine(parent, destinationPort);
line = add_line(parent, sourcePort, destinationPort, "autorouting", "on");
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

function script = srpAccelerationScript()
% Description:
%   Returns the MATLAB Function script used by the Simulink SRP acceleration block.

script = strjoin([ ...
    "function [F_srp_I_N, a_srp_I_m_s2] = srpAcceleration(F_srp_B_N, C_BI, mass_kg)";
    "%#codegen";
    "[F_srp_I_N, a_srp_I_m_s2] = computeSrpAcceleration(F_srp_B_N, C_BI, mass_kg);";
    "end"], newline);
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
