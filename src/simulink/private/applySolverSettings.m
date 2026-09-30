function simIn = applySolverSettings(modelName, AOCS, simIn)
% Description:
%   Applies model stop/start time, solver, and tolerance settings.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%   AOCS - Validated configuration struct.
%
% Outputs:
%   None.

simIn = setAocsParameters(simIn, modelName, ...
    "StartTime", num2str(AOCS.Sim.StartTime_s), ...
    "StopTime", num2str(AOCS.Sim.StopTime_s), ...
    "Solver", char(AOCS.Sim.Solver), ...
    "RelTol", num2str(AOCS.Sim.RelTol), ...
    "AbsTol", num2str(AOCS.Sim.AbsTol));
end
