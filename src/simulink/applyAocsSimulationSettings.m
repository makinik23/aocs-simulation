function applyAocsSimulationSettings(modelName, AOCS)
% Description:
%   Applies solver timing/tolerance settings and points Aerospace Blockset
%   6DOF mask parameters at the fields exposed by AOCS_Config.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%   AOCS - Validated configuration struct.
%
% Outputs:
%   None.

set_param(modelName, ...
    "StartTime", num2str(AOCS.Sim.StartTime_s), ...
    "StopTime", num2str(AOCS.Sim.StopTime_s), ...
    "Solver", char(AOCS.Sim.Solver), ...
    "RelTol", num2str(AOCS.Sim.RelTol), ...
    "AbsTol", num2str(AOCS.Sim.AbsTol));

applyAerospace6DofSettings(modelName);
end

function applyAerospace6DofSettings(modelName)
% Description:
%   Finds every EOM6DOFBodyQuat block and binds inertia, initial Euler
%   attitude, and initial body rates to AOCS_Config fields.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%
% Outputs:
%   None.

blocks = find_system(modelName, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "BlockType", "EOM6DOFBodyQuat");

for k = 1:numel(blocks)
    set_param(blocks{k}, ...
        "eul_0", "AOCS_Config.euler_BI_0_rad", ...
        "pm_0", "AOCS_Config.omega_BI_B_0", ...
        "inertia", "AOCS_Config.I_B");
end
end
