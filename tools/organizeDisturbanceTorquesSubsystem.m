function organizeDisturbanceTorquesSubsystem(modelFile)
% Description:
%   Rebuilds the Disturbance Torques subsystem into nested functional paths.
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
AOCS = setupAocsSimulation(fullfile(projectRoot, "config", ...
    "AocsSimulationConfig.json"));

[~, modelName] = fileparts(modelFile);
load_system(modelFile);
cleanup = onCleanup(@() closeIfLoaded(modelName));
applyAocsSimulationSettings(modelName, AOCS);

parent = modelName + "/Orbit & Environment/Disturbance Torques";
if isBlock(parent + "/Gravity Gradient Path")
    layoutTopLevel(parent);
    connectTopLevel(parent);
    markLoggedSignals(parent);
    set_param(modelName, "SimulationCommand", "update");
    save_system(modelName, modelFile);
    return;
end

createGravityGradientPath(parent);
createResidualMagneticPath(parent);
createAerodynamicDisturbancePath(parent);
createSrpDisturbancePath(parent);
createOutputAssembly(parent);

removeOldImplementation(parent);
layoutTopLevel(parent);
connectTopLevel(parent);
markLoggedSignals(parent);

set_param(modelName, "SimulationCommand", "update");
save_system(modelName, modelFile);
end

function createGravityGradientPath(parent)
subsystem = createEmptySubsystem(parent, "Gravity Gradient Path", ...
    [40 -130 320 10]);

addInport(subsystem, "OrbitState", 1, [20 35 50 49]);
addInport(subsystem, "AttitudeState", 2, [20 85 50 99]);
addInport(subsystem, "AocsConfig", 3, [20 135 50 149]);
addInport(subsystem, "EnvironmentContext", 4, [20 185 50 199]);
addInport(subsystem, "EnvironmentConfig", 5, [20 235 50 249]);
addOutport(subsystem, "M_gg_B_Nm", 1, [635 130 665 144]);

addBusSelector(subsystem, "Select Orbit State", "r_I_m", [120 25 125 55]);
addBusSelector(subsystem, "Select Attitude State", "DCM_be", [120 75 125 105]);
addBusSelector(subsystem, "Select AOCS Config", "I_B", [120 125 125 155]);
addBusSelector(subsystem, "Select Environment Context", "mu_m3_s2", ...
    [120 175 125 205]);
addBusSelector(subsystem, "Select Environment Config", ...
    "gravity_gradient_enabled", [120 225 125 255]);
copyBlock(parent + "/Apply GG Enable", subsystem + "/Apply GG Enable", ...
    [255 178 285 212]);
copyBlock(parent + "/Gravity Gradient Torque", ...
    subsystem + "/Gravity Gradient Torque", [380 75 580 210]);

addNamedLine(subsystem, "OrbitState/1", "Select Orbit State/1", "OrbitState");
addNamedLine(subsystem, "AttitudeState/1", "Select Attitude State/1", ...
    "AttitudeState");
addNamedLine(subsystem, "AocsConfig/1", "Select AOCS Config/1", ...
    "AocsConfig");
addNamedLine(subsystem, "EnvironmentContext/1", ...
    "Select Environment Context/1", "EnvironmentContext");
addNamedLine(subsystem, "EnvironmentConfig/1", ...
    "Select Environment Config/1", "EnvironmentConfig");

addNamedLine(subsystem, "Select Orbit State/1", ...
    "Gravity Gradient Torque/1", "r_I_m");
addNamedLine(subsystem, "Select Attitude State/1", ...
    "Gravity Gradient Torque/2", "DCM_be");
addNamedLine(subsystem, "Select AOCS Config/1", ...
    "Gravity Gradient Torque/3", "I_B");
addNamedLine(subsystem, "Select Environment Context/1", ...
    "Apply GG Enable/1", "mu_m3_s2");
addNamedLine(subsystem, "Select Environment Config/1", ...
    "Apply GG Enable/2", "gravity_gradient_enabled");
addNamedLine(subsystem, "Apply GG Enable/1", ...
    "Gravity Gradient Torque/4", "mu_m3_s2");
addNamedLine(subsystem, "Gravity Gradient Torque/1", ...
    "M_gg_B_Nm/1", "M_gg_B_Nm");
end

function createResidualMagneticPath(parent)
subsystem = createEmptySubsystem(parent, "Residual Magnetic Path", ...
    [40 50 320 190]);

addInport(subsystem, "EnvironmentConfig", 1, [20 65 50 79]);
addInport(subsystem, "MagneticField", 2, [20 145 50 159]);
addOutport(subsystem, "M_rmm_B_Nm", 1, [505 105 535 119]);

addBusSelector(subsystem, "Select Environment Config", ...
    "m_res_B_A_m2,rmm_enabled", [120 45 125 95]);
addBusSelector(subsystem, "Select Magnetic Field", "B_B_T", ...
    [120 135 125 165]);
copyBlock(parent + "/Apply RMM Enable", subsystem + "/Apply RMM Enable", ...
    [245 50 275 85]);
copyBlock(parent + "/Cross Product", subsystem + "/Cross Product", ...
    [345 80 425 145]);

addNamedLine(subsystem, "EnvironmentConfig/1", ...
    "Select Environment Config/1", "EnvironmentConfig");
addNamedLine(subsystem, "MagneticField/1", "Select Magnetic Field/1", ...
    "MagneticField");
addNamedLine(subsystem, "Select Environment Config/1", ...
    "Apply RMM Enable/1", "m_res_B_A_m2");
addNamedLine(subsystem, "Select Environment Config/2", ...
    "Apply RMM Enable/2", "rmm_enabled");
addNamedLine(subsystem, "Apply RMM Enable/1", ...
    "Cross Product/1", "m_res_B_A_m2");
addNamedLine(subsystem, "Select Magnetic Field/1", ...
    "Cross Product/2", "B_B_T");
addNamedLine(subsystem, "Cross Product/1", ...
    "M_rmm_B_Nm/1", "M_rmm_B_Nm");
end

function createAerodynamicDisturbancePath(parent)
subsystem = createEmptySubsystem(parent, "Aerodynamic Disturbance Path", ...
    [40 275 360 500]);

addInport(subsystem, "OrbitState", 1, [20 60 50 74]);
addInport(subsystem, "AttitudeState", 2, [20 120 50 134]);
addInport(subsystem, "Atmosphere", 3, [20 190 50 204]);
addInport(subsystem, "AocsConfig", 4, [20 365 50 379]);
addOutport(subsystem, "F_aero_B_N", 1, [820 310 850 324]);
addOutport(subsystem, "M_aero_B_Nm", 2, [820 345 850 359]);
addOutport(subsystem, "F_aero_I_N", 3, [820 380 850 394]);
addOutport(subsystem, "a_aero_I_m_s2", 4, [820 415 850 429]);
addOutport(subsystem, "q_dyn_N_m2", 5, [820 450 850 464]);
addOutport(subsystem, "v_rel_norm_m_s", 6, [820 485 850 499]);

copyBlock(parent + "/Relative Wind", subsystem + "/Relative Wind", ...
    [160 55 420 205]);
addBusSelector(subsystem, "Select Attitude State", "DCM_be", ...
    [155 235 160 265]);
addBusSelector(subsystem, "Select Aerodynamic Atmosphere", ...
    "rho_kg_m3,T_local_K,n_O_m3,n_N2_m3,n_O2_m3,n_He_m3,n_H_m3,n_N_m3", ...
    [160 300 290 455]);
addBusSelector(subsystem, "Select Aerodynamic Config", ...
    "mass_kg,aero_enabled,aero_panel_normals_B,aero_panel_areas_m2," + ...
    "aero_panel_centers_B_m,aero_wall_temperature_K,aero_energy_accommodation", ...
    [160 515 290 665]);
copyBlock(parent + "/Species Number Densities", ...
    subsystem + "/Species Number Densities", [360 360 365 470]);
copyBlock(parent + "/Sentman Multispecies Panels", ...
    subsystem + "/Sentman Multispecies Panels", [465 265 730 625]);
copyBlock(parent + "/Terminate Panel Forces", ...
    subsystem + "/Terminate Panel Forces", [820 565 840 579]);
add_block("simulink/Sinks/Terminator", ...
    char(subsystem + "/Terminate Sentman Dynamic Pressure"), ...
    "Position", [820 605 840 619]);

addNamedLine(subsystem, "OrbitState/1", "Relative Wind/1", "OrbitState");
addNamedLine(subsystem, "AttitudeState/1", "Relative Wind/2", ...
    "AttitudeState");
addNamedLine(subsystem, "AttitudeState/1", "Select Attitude State/1", ...
    "AttitudeState");
addNamedLine(subsystem, "Atmosphere/1", "Relative Wind/3", "Atmosphere");
addNamedLine(subsystem, "Atmosphere/1", ...
    "Select Aerodynamic Atmosphere/1", "Atmosphere");
addNamedLine(subsystem, "AocsConfig/1", "Select Aerodynamic Config/1", ...
    "AocsConfig");

addNamedLine(subsystem, "Relative Wind/1", ...
    "Sentman Multispecies Panels/1", "v_flow_B_m_s");
addNamedLine(subsystem, "Select Aerodynamic Atmosphere/1", ...
    "Sentman Multispecies Panels/2", "rho_kg_m3");
addNamedLine(subsystem, "Select Aerodynamic Atmosphere/2", ...
    "Sentman Multispecies Panels/3", "T_local_K");
for index = 1:6
    addNamedLine(subsystem, "Select Aerodynamic Atmosphere/" + (index + 2), ...
        "Species Number Densities/" + index, speciesSignalName(index));
end
addNamedLine(subsystem, "Species Number Densities/1", ...
    "Sentman Multispecies Panels/4", "number_densities_m3");
addNamedLine(subsystem, "Select Attitude State/1", ...
    "Sentman Multispecies Panels/5", "DCM_be");
for index = 1:7
    addNamedLine(subsystem, "Select Aerodynamic Config/" + index, ...
        "Sentman Multispecies Panels/" + (index + 5), ...
        aerodynamicConfigSignalName(index));
end

addNamedLine(subsystem, "Sentman Multispecies Panels/1", ...
    "F_aero_B_N/1", "F_aero_B_N");
addNamedLine(subsystem, "Sentman Multispecies Panels/2", ...
    "M_aero_B_Nm/1", "M_aero_B_Nm");
addNamedLine(subsystem, "Sentman Multispecies Panels/3", ...
    "F_aero_I_N/1", "F_aero_I_N");
addNamedLine(subsystem, "Sentman Multispecies Panels/4", ...
    "a_aero_I_m_s2/1", "a_aero_I_m_s2");
addNamedLine(subsystem, "Relative Wind/3", ...
    "q_dyn_N_m2/1", "q_dyn_N_m2");
addNamedLine(subsystem, "Relative Wind/2", ...
    "v_rel_norm_m_s/1", "v_rel_norm_m_s");
addNamedLine(subsystem, "Sentman Multispecies Panels/5", ...
    "Terminate Panel Forces/1", "panel_forces_B_N");
addNamedLine(subsystem, "Sentman Multispecies Panels/6", ...
    "Terminate Sentman Dynamic Pressure/1", "q_dyn_N_m2");
end

function createSrpDisturbancePath(parent)
subsystem = createEmptySubsystem(parent, "SRP Disturbance Path", ...
    [455 220 735 380]);

addInport(subsystem, "SRP", 1, [20 55 50 69]);
addInport(subsystem, "AttitudeState", 2, [20 135 50 149]);
addInport(subsystem, "AocsConfig", 3, [20 215 50 229]);
addOutport(subsystem, "M_srp_B_Nm", 1, [610 55 640 69]);
addOutport(subsystem, "F_srp_B_N", 2, [610 105 640 119]);
addOutport(subsystem, "F_srp_I_N", 3, [610 155 640 169]);
addOutport(subsystem, "a_srp_I_m_s2", 4, [610 205 640 219]);

addBusSelector(subsystem, "Select SRP", "M_srp_B_Nm,F_srp_B_N", ...
    [130 40 135 85]);
addBusSelector(subsystem, "Select Attitude State", "DCM_be", ...
    [130 125 135 155]);
addBusSelector(subsystem, "Select Mass", "mass_kg", [130 205 135 235]);
copyBlock(parent + "/SRP Inertial Acceleration", ...
    subsystem + "/SRP Inertial Acceleration", [310 105 520 220]);

addNamedLine(subsystem, "SRP/1", "Select SRP/1", "SRP");
addNamedLine(subsystem, "AttitudeState/1", ...
    "Select Attitude State/1", "AttitudeState");
addNamedLine(subsystem, "AocsConfig/1", "Select Mass/1", "AocsConfig");
addNamedLine(subsystem, "Select SRP/1", "M_srp_B_Nm/1", "M_srp_B_Nm");
addNamedLine(subsystem, "Select SRP/2", ...
    "SRP Inertial Acceleration/1", "F_srp_B_N");
addNamedLine(subsystem, "Select SRP/2", "F_srp_B_N/1", "F_srp_B_N");
addNamedLine(subsystem, "Select Attitude State/1", ...
    "SRP Inertial Acceleration/2", "DCM_be");
addNamedLine(subsystem, "Select Mass/1", ...
    "SRP Inertial Acceleration/3", "mass_kg");
addNamedLine(subsystem, "SRP Inertial Acceleration/1", ...
    "F_srp_I_N/1", "F_srp_I_N");
addNamedLine(subsystem, "SRP Inertial Acceleration/2", ...
    "a_srp_I_m_s2/1", "a_srp_I_m_s2");
end

function createOutputAssembly(parent)
subsystem = createEmptySubsystem(parent, "Output Assembly", [830 85 1115 380]);

inputNames = ["M_gg_B_Nm", "M_rmm_B_Nm", "M_srp_B_Nm", "M_aero_B_Nm", ...
    "F_srp_B_N", "F_srp_I_N", "a_srp_I_m_s2", "F_aero_B_N", ...
    "F_aero_I_N", "a_aero_I_m_s2", "q_dyn_N_m2", "v_rel_norm_m_s"];
for index = 1:numel(inputNames)
    addInport(subsystem, inputNames(index), index, ...
        [20 25 + (index - 1) * 35 50 39 + (index - 1) * 35]);
end
addOutport(subsystem, "Disturbance", 1, [780 195 810 209]);
addOutport(subsystem, "OrbitAcceleration", 2, [780 390 810 404]);

add_block("simulink/Math Operations/Sum", ...
    char(subsystem + "/Attitude Torque Sum"), "Inputs", "++", ...
    "Position", [155 45 180 85]);
add_block("simulink/Math Operations/Sum", ...
    char(subsystem + "/Total Torque Sum"), "Inputs", "+++", ...
    "Position", [300 90 330 150]);
add_block("simulink/Math Operations/Sum", ...
    char(subsystem + "/Orbit Acceleration Sum"), "Inputs", "++", ...
    "Position", [300 345 330 395]);
add_block("simulink/Signal Routing/Bus Creator", ...
    char(subsystem + "/Disturbance Bus Assembly"), "Inputs", "12", ...
    "OutDataTypeStr", "Bus: AOCS_DisturbanceBus", "UseBusObject", "on", ...
    "BusObject", "AOCS_DisturbanceBus", "NonVirtualBus", "off", ...
    "Position", [570 135 575 420]);

addNamedLine(subsystem, "M_gg_B_Nm/1", ...
    "Attitude Torque Sum/1", "M_gg_B_Nm");
addNamedLine(subsystem, "M_rmm_B_Nm/1", ...
    "Attitude Torque Sum/2", "M_rmm_B_Nm");
addNamedLine(subsystem, "Attitude Torque Sum/1", ...
    "Total Torque Sum/1", "M_env_B_Nm");
addNamedLine(subsystem, "M_srp_B_Nm/1", ...
    "Total Torque Sum/2", "M_srp_B_Nm");
addNamedLine(subsystem, "M_aero_B_Nm/1", ...
    "Total Torque Sum/3", "M_aero_B_Nm");
addNamedLine(subsystem, "a_srp_I_m_s2/1", ...
    "Orbit Acceleration Sum/1", "a_srp_I_m_s2");
addNamedLine(subsystem, "a_aero_I_m_s2/1", ...
    "Orbit Acceleration Sum/2", "a_aero_I_m_s2");

addNamedLine(subsystem, "Total Torque Sum/1", ...
    "Disturbance Bus Assembly/1", "M_dist_B_Nm");
addNamedLine(subsystem, "M_srp_B_Nm/1", ...
    "Disturbance Bus Assembly/2", "M_srp_B_Nm");
addNamedLine(subsystem, "M_aero_B_Nm/1", ...
    "Disturbance Bus Assembly/3", "M_aero_B_Nm");
addNamedLine(subsystem, "F_srp_B_N/1", ...
    "Disturbance Bus Assembly/4", "F_srp_B_N");
addNamedLine(subsystem, "F_srp_I_N/1", ...
    "Disturbance Bus Assembly/5", "F_srp_I_N");
addNamedLine(subsystem, "a_srp_I_m_s2/1", ...
    "Disturbance Bus Assembly/6", "a_srp_I_m_s2");
addNamedLine(subsystem, "F_aero_B_N/1", ...
    "Disturbance Bus Assembly/7", "F_aero_B_N");
addNamedLine(subsystem, "F_aero_I_N/1", ...
    "Disturbance Bus Assembly/8", "F_aero_I_N");
addNamedLine(subsystem, "a_aero_I_m_s2/1", ...
    "Disturbance Bus Assembly/9", "a_aero_I_m_s2");
addNamedLine(subsystem, "Orbit Acceleration Sum/1", ...
    "Disturbance Bus Assembly/10", "a_dist_I_m_s2");
addNamedLine(subsystem, "q_dyn_N_m2/1", ...
    "Disturbance Bus Assembly/11", "q_dyn_N_m2");
addNamedLine(subsystem, "v_rel_norm_m_s/1", ...
    "Disturbance Bus Assembly/12", "v_rel_norm_m_s");
addNamedLine(subsystem, "Disturbance Bus Assembly/1", ...
    "Disturbance/1", "Disturbance");
addNamedLine(subsystem, "Orbit Acceleration Sum/1", ...
    "OrbitAcceleration/1", "a_dist_I_m_s2");
end

function removeOldImplementation(parent)
deleteTopLevelLines(parent);
oldBlocks = ["Add", "Add1", "Apply GG Enable", "Apply RMM Enable", ...
    "Cross Product", "Disturbance Bus Assembly", "Gravity Gradient Torque", ...
    "Orbit Acceleration Sum", "Relative Wind", "SRP Inertial Acceleration", ...
    "Select AOCS Config", "Select Aerodynamic Atmosphere", ...
    "Select Aerodynamic Config", "Select Attitude State", ...
    "Select Environment Config", "Select Environment Context", ...
    "Select Magnetic Field", "Select Orbit State", "Select SRP", ...
    "Sentman Multispecies Panels", "Species Number Densities", ...
    "Terminate Panel Forces"];

for index = 1:numel(oldBlocks)
    deleteBlockIfExists(parent + "/" + oldBlocks(index));
end
end

function layoutTopLevel(parent)
setBlockPosition(parent, "OrbitState", [-315 -82 -285 -68]);
setBlockPosition(parent, "AttitudeState", [-315 -2 -285 12]);
setBlockPosition(parent, "AocsConfig", [-315 78 -285 92]);
setBlockPosition(parent, "EnvironmentConfig", [-315 158 -285 172]);
setBlockPosition(parent, "EnvironmentContext", [-315 238 -285 252]);
setBlockPosition(parent, "MagneticField", [-315 318 -285 332]);
setBlockPosition(parent, "SRP", [-315 398 -285 412]);
setBlockPosition(parent, "Atmosphere", [-315 478 -285 492]);

setBlockPosition(parent, "Gravity Gradient Path", [40 -130 320 10]);
setBlockPosition(parent, "Residual Magnetic Path", [40 80 320 220]);
setBlockPosition(parent, "Aerodynamic Disturbance Path", [40 420 360 650]);
setBlockPosition(parent, "SRP Disturbance Path", [440 235 720 395]);
setBlockPosition(parent, "Output Assembly", [830 105 1115 420]);

setBlockPosition(parent, "Disturbance", [1190 220 1220 234]);
setBlockPosition(parent, "OrbitAcceleration", [1190 345 1220 359]);
end

function connectTopLevel(parent)
deleteTopLevelLines(parent);

addNamedLine(parent, "OrbitState/1", "Gravity Gradient Path/1", "OrbitState");
addNamedLine(parent, "AttitudeState/1", "Gravity Gradient Path/2", ...
    "AttitudeState");
addNamedLine(parent, "AocsConfig/1", "Gravity Gradient Path/3", ...
    "AocsConfig");
addNamedLine(parent, "EnvironmentContext/1", "Gravity Gradient Path/4", ...
    "EnvironmentContext");
addNamedLine(parent, "EnvironmentConfig/1", "Gravity Gradient Path/5", ...
    "EnvironmentConfig");

addNamedLine(parent, "EnvironmentConfig/1", ...
    "Residual Magnetic Path/1", "EnvironmentConfig");
addNamedLine(parent, "MagneticField/1", "Residual Magnetic Path/2", ...
    "MagneticField");

addNamedLine(parent, "OrbitState/1", "Aerodynamic Disturbance Path/1", ...
    "OrbitState");
addNamedLine(parent, "AttitudeState/1", ...
    "Aerodynamic Disturbance Path/2", "AttitudeState");
addNamedLine(parent, "Atmosphere/1", "Aerodynamic Disturbance Path/3", ...
    "Atmosphere");
addNamedLine(parent, "AocsConfig/1", "Aerodynamic Disturbance Path/4", ...
    "AocsConfig");

addNamedLine(parent, "SRP/1", "SRP Disturbance Path/1", "SRP");
addNamedLine(parent, "AttitudeState/1", "SRP Disturbance Path/2", ...
    "AttitudeState");
addNamedLine(parent, "AocsConfig/1", "SRP Disturbance Path/3", ...
    "AocsConfig");

addNamedLine(parent, "Gravity Gradient Path/1", ...
    "Output Assembly/1", "M_gg_B_Nm");
addNamedLine(parent, "Residual Magnetic Path/1", ...
    "Output Assembly/2", "M_rmm_B_Nm");
addNamedLine(parent, "SRP Disturbance Path/1", ...
    "Output Assembly/3", "M_srp_B_Nm");
addNamedLine(parent, "Aerodynamic Disturbance Path/2", ...
    "Output Assembly/4", "M_aero_B_Nm");
addNamedLine(parent, "SRP Disturbance Path/2", ...
    "Output Assembly/5", "F_srp_B_N");
addNamedLine(parent, "SRP Disturbance Path/3", ...
    "Output Assembly/6", "F_srp_I_N");
addNamedLine(parent, "SRP Disturbance Path/4", ...
    "Output Assembly/7", "a_srp_I_m_s2");
addNamedLine(parent, "Aerodynamic Disturbance Path/1", ...
    "Output Assembly/8", "F_aero_B_N");
addNamedLine(parent, "Aerodynamic Disturbance Path/3", ...
    "Output Assembly/9", "F_aero_I_N");
addNamedLine(parent, "Aerodynamic Disturbance Path/4", ...
    "Output Assembly/10", "a_aero_I_m_s2");
addNamedLine(parent, "Aerodynamic Disturbance Path/5", ...
    "Output Assembly/11", "q_dyn_N_m2");
addNamedLine(parent, "Aerodynamic Disturbance Path/6", ...
    "Output Assembly/12", "v_rel_norm_m_s");

addNamedLine(parent, "Output Assembly/1", "Disturbance/1", "Disturbance");
addNamedLine(parent, "Output Assembly/2", "OrbitAcceleration/1", ...
    "a_dist_I_m_s2");
end

function markLoggedSignals(parent)
markSignal(parent + "/Gravity Gradient Path/Gravity Gradient Torque", 1);
markSignal(parent + "/Residual Magnetic Path/Cross Product", 1);
markSignal(parent + "/SRP Disturbance Path/Select SRP", 1);
markSignal(parent + "/SRP Disturbance Path/Select SRP", 2);
markSignal(parent + "/SRP Disturbance Path/SRP Inertial Acceleration", 1);
markSignal(parent + "/SRP Disturbance Path/SRP Inertial Acceleration", 2);
markSignal(parent + "/Aerodynamic Disturbance Path/Sentman Multispecies Panels", 1);
markSignal(parent + "/Aerodynamic Disturbance Path/Sentman Multispecies Panels", 2);
markSignal(parent + "/Aerodynamic Disturbance Path/Sentman Multispecies Panels", 3);
markSignal(parent + "/Aerodynamic Disturbance Path/Sentman Multispecies Panels", 4);
markSignal(parent + "/Aerodynamic Disturbance Path/Relative Wind", 2);
markSignal(parent + "/Aerodynamic Disturbance Path/Relative Wind", 3);
markSignal(parent + "/Output Assembly/Total Torque Sum", 1);
markSignal(parent + "/Output Assembly/Orbit Acceleration Sum", 1);
end

function subsystem = createEmptySubsystem(parent, name, position)
subsystem = parent + "/" + name;
deleteBlockIfExists(subsystem);
add_block("simulink/Ports & Subsystems/Subsystem", char(subsystem), ...
    "Position", position);
deleteBlockIfExists(subsystem + "/In1");
deleteBlockIfExists(subsystem + "/Out1");
deleteTopLevelLines(subsystem);
end

function addInport(parent, name, portNumber, position)
add_block("simulink/Ports & Subsystems/In1", char(parent + "/" + name), ...
    "Port", num2str(portNumber), "Position", position);
end

function addOutport(parent, name, portNumber, position)
add_block("simulink/Ports & Subsystems/Out1", char(parent + "/" + name), ...
    "Port", num2str(portNumber), "Position", position);
end

function addBusSelector(parent, name, outputSignals, position)
add_block("simulink/Signal Routing/Bus Selector", char(parent + "/" + name), ...
    "OutputSignals", char(outputSignals), "Position", position);
end

function copyBlock(source, destination, position)
if getSimulinkBlockHandle(source) <= 0
    error("AOCS:DisturbanceLayout:MissingBlock", ...
        "Could not find block to copy: %s", source);
end
deleteBlockIfExists(destination);
add_block(char(source), char(destination), "Position", position);
end

function addNamedLine(parent, sourcePort, destinationPort, signalName)
line = add_line(char(parent), char(sourcePort), char(destinationPort), ...
    "autorouting", "on");
try
    set_param(line, "Name", char(signalName));
catch exception
    if ~contains(exception.message, "Bus Selector")
        rethrow(exception);
    end
end
end

function deleteTopLevelLines(parent)
if getSimulinkBlockHandle(parent) <= 0
    return;
end
lines = find_system(parent, "FindAll", "on", "SearchDepth", 1, ...
    "Type", "line");
for index = 1:numel(lines)
    try
        delete_line(lines(index));
    catch
    end
end
end

function setBlockPosition(parent, blockName, position)
block = parent + "/" + blockName;
if isBlock(block)
    set_param(block, "Position", position);
end
end

function markSignal(block, outputPort)
if getSimulinkBlockHandle(block) > 0
    Simulink.sdi.markSignalForStreaming(block, outputPort, "on");
end
end

function signalName = speciesSignalName(index)
names = ["n_O_m3", "n_N2_m3", "n_O2_m3", "n_He_m3", "n_H_m3", "n_N_m3"];
signalName = names(index);
end

function signalName = aerodynamicConfigSignalName(index)
names = ["mass_kg", "aero_enabled", "aero_panel_normals_B", ...
    "aero_panel_areas_m2", "aero_panel_centers_B_m", ...
    "aero_wall_temperature_K", "aero_energy_accommodation"];
signalName = names(index);
end

function exists = isBlock(block)
exists = getSimulinkBlockHandle(block) > 0;
end

function deleteBlockIfExists(block)
if isBlock(block)
    delete_block(char(block));
end
end

function closeIfLoaded(modelName)
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end
