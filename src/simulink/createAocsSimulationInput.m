function [simIn, AOCS] = createAocsSimulationInput(configFile)
%CREATEAOCSSIMULATIONINPUT Prepare an experiment without publishing its values.
% Bus definitions are shared schema objects; experiment values live in simIn.
if nargin < 1
    configFile = "";
end
[AOCS, variables] = setupAocsSimulation(configFile, false);
load_system(AOCS.Model.File);
loadedFile = string(java.io.File(get_param(AOCS.Model.Name, 'FileName')).getCanonicalPath());
expectedFile = string(java.io.File(char(AOCS.Model.File)).getCanonicalPath());
if loadedFile ~= expectedFile
    error("AOCS:Simulation:ModelShadowed", ...
        "A different model named %s is already loaded from %s.", AOCS.Model.Name, loadedFile);
end
simIn = Simulink.SimulationInput(AOCS.Model.Name);
names = fieldnames(variables);
for k = 1:numel(names)
    simIn = simIn.setVariable(names{k}, variables.(names{k}));
end
simIn = applyAocsSimulationSettings(AOCS.Model.Name, AOCS, simIn);
end
