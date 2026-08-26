function integrateDtm2020Simulink(modelFile)
% Description:
%   Replaces the temporary atmosphere MATLAB Function with the native
%   operational DTM2020 S-Function and SI postprocessing pipeline.

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

parent = modelName + "/Orbit & Environment/Environment Products/" + ...
    "Atmosphere Products/Atmosphere Model";
pipeline = parent + "/Atmosphere Products";

deleteConnectedLines(pipeline);
delete_block(pipeline);
add_block("simulink/Ports & Subsystems/Subsystem", pipeline, ...
    "Position", [270 19 625 536]);
clearSubsystem(pipeline);

add_block("simulink/Ports & Subsystems/In1", pipeline + "/r_I_m", ...
    "Port", "1", "Position", [20 48 50 62]);
add_block("simulink/Ports & Subsystems/In1", pipeline + "/v_I_m_s", ...
    "Port", "2", "Position", [20 103 50 117]);
add_block("simulink/Ports & Subsystems/In1", pipeline + "/lla", ...
    "Port", "3", "Position", [20 158 50 172]);
add_block("simulink/Ports & Subsystems/In1", pipeline + "/EnvironmentConfig", ...
    "Port", "4", "Position", [20 213 50 227]);
add_block("simulink/Ports & Subsystems/In1", pipeline + "/EnvironmentContext", ...
    "Port", "5", "Position", [20 268 50 282]);

prepareBlock = pipeline + "/Prepare DTM2020 Inputs";
add_block("simulink/User-Defined Functions/MATLAB Function", prepareBlock, ...
    "Position", [100 145 280 295]);
setChartScript(prepareBlock, prepareScript());

nativeBlock = pipeline + "/DTM2020 Operational";
add_block("simulink/User-Defined Functions/S-Function", nativeBlock, ...
    "FunctionName", "dtm2020_sfun", ...
    "Parameters", "AOCS_DTM2020_CoefficientFile", ...
    "Position", [330 175 495 265]);

postprocessBlock = pipeline + "/Postprocess DTM2020";
add_block("simulink/User-Defined Functions/MATLAB Function", postprocessBlock, ...
    "Position", [545 20 790 335]);
setChartScript(postprocessBlock, postprocessScript());

add_block("simulink/Sinks/Terminator", pipeline + "/Unused Velocity", ...
    "Position", [105 102 125 118]);
add_line(pipeline, "v_I_m_s/1", "Unused Velocity/1", "autorouting", "on");
add_line(pipeline, "lla/1", "Prepare DTM2020 Inputs/1", "autorouting", "on");
add_line(pipeline, "EnvironmentConfig/1", "Prepare DTM2020 Inputs/2", "autorouting", "on");
add_line(pipeline, "EnvironmentContext/1", "Prepare DTM2020 Inputs/3", "autorouting", "on");
add_line(pipeline, "Prepare DTM2020 Inputs/1", "DTM2020 Operational/1", "autorouting", "on");
add_line(pipeline, "DTM2020 Operational/1", "Postprocess DTM2020/1", "autorouting", "on");
add_line(pipeline, "r_I_m/1", "Postprocess DTM2020/2", "autorouting", "on");
add_line(pipeline, "EnvironmentConfig/1", "Postprocess DTM2020/3", "autorouting", "on");

outputNames = ["rho_kg_m3", "rho_raw_kg_m3", ...
    "rho_uncertainty_1sigma_kg_m3", "T_local_K", "T_exo_K", ...
    "n_O_m3", "n_N2_m3", "n_O2_m3", "n_He_m3", "n_H_m3", ...
    "n_N_m3", "v_atm_I_m_s"];
for index = 1:numel(outputNames)
    y = 25 + (index - 1) * 38;
    add_block("simulink/Ports & Subsystems/Out1", ...
        pipeline + "/" + outputNames(index), ...
        "Port", string(index), "Position", [850 y 880 y + 14]);
    add_line(pipeline, "Postprocess DTM2020/" + index, ...
        outputNames(index) + "/1", "autorouting", "on");
end

add_line(parent, "Select Orbit State/1", "Atmosphere Products/1", "autorouting", "on");
add_line(parent, "Select Orbit State/2", "Atmosphere Products/2", "autorouting", "on");
add_line(parent, "lla/1", "Atmosphere Products/3", "autorouting", "on");
add_line(parent, "EnvironmentConfig/1", "Atmosphere Products/4", "autorouting", "on");
add_line(parent, "EnvironmentContext/1", "Atmosphere Products/5", "autorouting", "on");
for index = 1:12
    add_line(parent, "Atmosphere Products/" + index, ...
        "Atmosphere Bus Assembly/" + index, "autorouting", "on");
end

set_param(modelName, "SimulationCommand", "update");
save_system(modelName, modelFile);
end

function clearSubsystem(subsystem)
blocks = find_system(subsystem, "SearchDepth", 1, "Type", "Block");
blocks = blocks(~strcmp(blocks, subsystem));
for index = 1:numel(blocks)
    delete_block(blocks{index});
end
end

function deleteConnectedLines(block)
handles = get_param(block, "LineHandles");
fields = fieldnames(handles);
for field = 1:numel(fields)
    lines = handles.(fields{field});
    lines = unique(lines(lines > 0));
    for line = reshape(lines, 1, [])
        delete_line(line);
    end
end
end

function setChartScript(block, script)
root = sfroot();
blockName = string(get_param(block, "Name"));
chart = root.find("-isa", "Stateflow.EMChart", "Path", char(block), ...
    "Name", char(blockName));
if isempty(chart)
    error("AOCS:DTM2020:MissingChart", "MATLAB Function chart not found: %s", block);
end
chart.Script = script;
end

function script = prepareScript()
script = strjoin([ ...
    "function nativeInput = prepareDtmInputs(lla, environmentConfig, environmentContext)";
    "%#codegen";
    "nativeInput = dtm2020.prepareNativeInput(lla, environmentConfig, environmentContext);";
    "end"], newline);
end

function script = postprocessScript()
script = strjoin([ ...
    "function [rho_kg_m3, rho_raw_kg_m3, rho_uncertainty_1sigma_kg_m3, ...";
    "    T_local_K, T_exo_K, n_O_m3, n_N2_m3, n_O2_m3, n_He_m3, ...";
    "    n_H_m3, n_N_m3, v_atm_I_m_s] = postprocessDtm(nativeOutput, r_I_m, environmentConfig)";
    "%#codegen";
    "[rho_kg_m3, rho_raw_kg_m3, rho_uncertainty_1sigma_kg_m3, ...";
    "    T_local_K, T_exo_K, n_O_m3, n_N2_m3, n_O2_m3, n_He_m3, ...";
    "    n_H_m3, n_N_m3, v_atm_I_m_s] = dtm2020.postprocessNativeOutput( ...";
    "    nativeOutput, r_I_m, environmentConfig);";
    "end"], newline);
end

function closeIfLoaded(modelName)
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end
