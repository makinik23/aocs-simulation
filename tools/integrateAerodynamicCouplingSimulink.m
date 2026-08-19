function integrateAerodynamicCouplingSimulink(modelFile)
% Description:
%   Adds the multispecies Sentman panel model and closes the aerodynamic
%   force loop through the Numerical (high precision) orbit propagator.

projectRoot = fileparts(fileparts(mfilename("fullpath")));
if nargin < 1
    modelFile = fullfile(projectRoot, "models", "aocs_plant.slx");
end

addpath(fullfile(projectRoot, "src", "config"));
addpath(fullfile(projectRoot, "src", "environment"));
addpath(fullfile(projectRoot, "src", "simulink"));
AOCS = setupAocsSimulation(fullfile(projectRoot, "config", "AocsSimulationConfig.json"));

[~, modelName] = fileparts(modelFile);
load_system(modelFile);
cleanup = onCleanup(@() closeIfLoaded(modelName));
applyAocsSimulationSettings(modelName, AOCS);

disturbance = modelName + "/Orbit & Environment/Disturbance Torques";
addSentmanPanelModel(disturbance);
connectAtmosphereConsumers(modelName);
addOrbitAccelerationInput(modelName);
publishAerodynamicProducts(modelName);

set_param(modelName, "SimulationCommand", "update");
save_system(modelName, modelFile);
end

function connectAtmosphereConsumers(modelName)
parent = modelName + "/Orbit & Environment";
disturbance = parent + "/Disturbance Torques";
environmentBusAssembly = parent + "/Environment Bus Assembly";

deleteInputLine(disturbance, 8);
deleteInputLine(environmentBusAssembly, 5);
add_line(parent, "Environment Products/5", "Disturbance Torques/8", ...
    "autorouting", "on");
add_line(parent, "Environment Products/5", "Environment Bus Assembly/5", ...
    "autorouting", "on");
end

function addSentmanPanelModel(parent)
chart = parent + "/Sentman Multispecies Panels";
atmosphereSelector = parent + "/Select Aerodynamic Atmosphere";
configSelector = parent + "/Select Aerodynamic Config";
speciesMux = parent + "/Species Number Densities";
panelForcesTerminator = parent + "/Terminate Panel Forces";
accelerationOut = parent + "/AerodynamicAcceleration";

deleteBlockIfExists(chart);
deleteBlockIfExists(atmosphereSelector);
deleteBlockIfExists(configSelector);
deleteBlockIfExists(speciesMux);
deleteBlockIfExists(panelForcesTerminator);
deleteBlockIfExists(accelerationOut);
deleteBlockIfExists(parent + "/Terminate Flow");
deleteBlockIfExists(parent + "/Terminate Speed");
deleteBlockIfExists(parent + "/Terminate Dynamic Pressure");

add_block("simulink/Signal Routing/Bus Selector", atmosphereSelector, ...
    "OutputSignals", "rho_kg_m3,T_local_K,n_O_m3,n_N2_m3,n_O2_m3,n_He_m3,n_H_m3,n_N_m3", ...
    "Position", [250 505 365 655]);
add_block("simulink/Signal Routing/Bus Selector", configSelector, ...
    "OutputSignals", ...
    "mass_kg,aero_enabled,aero_panel_normals_B,aero_panel_areas_m2," + ...
    "aero_panel_centers_B_m,aero_wall_temperature_K,aero_energy_accommodation", ...
    "Position", [250 675 365 815]);
add_block("simulink/Signal Routing/Mux", speciesMux, ...
    "Inputs", "6", "Position", [430 555 435 665]);
add_block("simulink/User-Defined Functions/MATLAB Function", chart, ...
    "Position", [510 480 790 825]);
setChartScript(chart, sentmanScript());
add_block("simulink/Sinks/Terminator", panelForcesTerminator, ...
    "Position", [845 743 865 757]);
add_block("simulink/Ports & Subsystems/Out1", accelerationOut, ...
    "Port", "2", "Position", [895 678 925 692]);

add_line(parent, "Atmosphere/1", "Select Aerodynamic Atmosphere/1", "autorouting", "on");
add_line(parent, "AocsConfig/1", "Select Aerodynamic Config/1", "autorouting", "on");
for index = 1:6
    add_line(parent, "Select Aerodynamic Atmosphere/" + (index + 2), ...
        "Species Number Densities/" + index, "autorouting", "on");
end

add_line(parent, "Relative Wind/1", "Sentman Multispecies Panels/1", "autorouting", "on");
add_line(parent, "Select Aerodynamic Atmosphere/1", "Sentman Multispecies Panels/2", "autorouting", "on");
add_line(parent, "Select Aerodynamic Atmosphere/2", "Sentman Multispecies Panels/3", "autorouting", "on");
add_line(parent, "Species Number Densities/1", "Sentman Multispecies Panels/4", "autorouting", "on");
add_line(parent, "Select Attitude State/1", "Sentman Multispecies Panels/5", "autorouting", "on");
for index = 1:7
    add_line(parent, "Select Aerodynamic Config/" + index, ...
        "Sentman Multispecies Panels/" + (index + 5), "autorouting", "on");
end

add_line(parent, "Sentman Multispecies Panels/5", "Terminate Panel Forces/1", "autorouting", "on");
add_line(parent, "Sentman Multispecies Panels/4", "AerodynamicAcceleration/1", "autorouting", "on");

torqueSum = parent + "/Add1";
set_param(torqueSum, "Inputs", "+++");
add_line(parent, "Sentman Multispecies Panels/2", "Add1/3", "autorouting", "on");

busCreator = parent + "/Disturbance Bus Assembly";
for inputIndex = 3:8
    deleteInputLine(busCreator, inputIndex);
end
set_param(busCreator, "Inputs", "8");
add_line(parent, "Sentman Multispecies Panels/2", "Disturbance Bus Assembly/3", "autorouting", "on");
add_line(parent, "Sentman Multispecies Panels/1", "Disturbance Bus Assembly/4", "autorouting", "on");
add_line(parent, "Sentman Multispecies Panels/3", "Disturbance Bus Assembly/5", "autorouting", "on");
add_line(parent, "Sentman Multispecies Panels/4", "Disturbance Bus Assembly/6", "autorouting", "on");
add_line(parent, "Relative Wind/3", "Disturbance Bus Assembly/7", "autorouting", "on");
add_line(parent, "Relative Wind/2", "Disturbance Bus Assembly/8", "autorouting", "on");
end

function addOrbitAccelerationInput(modelName)
orbitAndEnvironment = modelName + "/Orbit & Environment";
orbitAndTime = orbitAndEnvironment + "/Orbit Propagator & Time";
orbitState = orbitAndTime + "/Orbit State";
propagatorSubsystem = orbitState + "/Orbit Propagator";

deleteBlockIfExists(orbitAndTime + "/AerodynamicAcceleration");
deleteBlockIfExists(orbitState + "/AerodynamicAcceleration");
deleteBlockIfExists(propagatorSubsystem + "/AerodynamicAcceleration");

add_block("simulink/Ports & Subsystems/In1", orbitAndTime + "/AerodynamicAcceleration", ...
    "Port", "2", "Position", [30 120 60 134]);
add_block("simulink/Ports & Subsystems/In1", orbitState + "/AerodynamicAcceleration", ...
    "Port", "1", "Position", [25 90 55 104]);
add_block("simulink/Ports & Subsystems/In1", propagatorSubsystem + "/AerodynamicAcceleration", ...
    "Port", "1", "Position", [25 130 55 144]);

propagatorBlocks = find_system(propagatorSubsystem, ...
    "LookUnderMasks", "all", "FollowLinks", "on", "BlockType", "OrbitPropagator");
if numel(propagatorBlocks) ~= 1
    error("AOCS:Aerodynamics:OrbitPropagator", ...
        "Expected one Orbit Propagator block, found %d.", numel(propagatorBlocks));
end
propagatorHandle = get_param(propagatorBlocks{1}, "Handle");
set_param(propagatorHandle, ...
    "propagator", "Numerical (high precision)", ...
    "accelIn", "on", ...
    "accelFrame", "ICRF");

add_line(propagatorSubsystem, "AerodynamicAcceleration/1", ...
    string(get_param(propagatorHandle, "Name")) + "/1", "autorouting", "on");
add_line(orbitState, "AerodynamicAcceleration/1", "Orbit Propagator/1", "autorouting", "on");
add_line(orbitAndTime, "AerodynamicAcceleration/1", "Orbit State/1", "autorouting", "on");
add_line(orbitAndEnvironment, "Disturbance Torques/2", ...
    "Orbit Propagator & Time/2", "autorouting", "on");
end

function publishAerodynamicProducts(modelName)
parent = modelName + "/Orbit & Environment/Environment Bus Assembly";
selector = parent + "/Select Disturbance";
busCreator = parent + "/Environment Bus Creator";

for inputIndex = 12:17
    deleteInputLine(busCreator, inputIndex);
end
set_param(selector, "OutputSignals", ...
    "M_dist_B_Nm,M_aero_B_Nm,F_aero_B_N,F_aero_I_N,a_aero_I_m_s2,q_dyn_N_m2,v_rel_norm_m_s");
set_param(busCreator, "Inputs", "17");
for outputIndex = 2:7
    add_line(parent, "Select Disturbance/" + outputIndex, ...
        "Environment Bus Creator/" + (outputIndex + 10), "autorouting", "on");
end
end

function setChartScript(block, script)
root = sfroot();
blockName = string(get_param(block, "Name"));
chart = root.find("-isa", "Stateflow.EMChart", "Path", char(block), ...
    "Name", char(blockName));
if isempty(chart)
    error("AOCS:Aerodynamics:MissingChart", ...
        "MATLAB Function chart not found: %s", block);
end
chart.Script = script;
end

function script = sentmanScript()
script = strjoin([ ...
    "function [F_aero_B_N, M_aero_B_Nm, F_aero_I_N, a_aero_I_m_s2, ...";
    "    panel_forces_B_N, q_dyn_N_m2] = sentmanPanels(v_flow_B_m_s, rho_kg_m3, ...";
    "    T_local_K, number_densities_m3, C_BI, mass_kg, aero_enabled, ...";
    "    panel_normals_B, panel_areas_m2, panel_centers_B_m, ...";
    "    wall_temperature_K, energy_accommodation)";
    "%#codegen";
    "[F_aero_B_N, M_aero_B_Nm, F_aero_I_N, a_aero_I_m_s2, ...";
    "    panel_forces_B_N, q_dyn_N_m2] = computeSentmanPanelAerodynamics( ...";
    "    v_flow_B_m_s, rho_kg_m3, T_local_K, number_densities_m3, C_BI, ...";
    "    mass_kg, aero_enabled, panel_normals_B, panel_areas_m2, ...";
    "    panel_centers_B_m, wall_temperature_K, energy_accommodation);";
    "end"], newline);
end

function deleteInputLine(block, inputIndex)
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
if getSimulinkBlockHandle(block) > 0
    delete_block(block);
end
end

function closeIfLoaded(modelName)
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end
