function simIn = applyAerospace6DofSettings(modelName, simIn)
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
if isempty(blocks)
    error("AOCS:Simulink:MissingRequiredBlock", "Missing EOM6DOFBodyQuat block in %s.", modelName);
end

for k = 1:numel(blocks)
    simIn = setAocsParameters(simIn, blocks{k}, ...
        "eul_0", "AOCS_Config.euler_BI_0_rad", ...
        "pm_0", "AOCS_Config.omega_BI_B_0", ...
        "inertia", "AOCS_Config.I_B");
end
end
