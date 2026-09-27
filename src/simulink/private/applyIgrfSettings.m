function simIn = applyIgrfSettings(modelName, simIn)
% Description:
%   Finds IGRF blocks and configures the modern IGRF generation with a
%   decimal-year input port and no secular-variation output ports.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%
% Outputs:
%   None.

blocks = find_system(modelName, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "BlockType", "IGRF");
if isempty(blocks)
    error("AOCS:Simulink:MissingRequiredBlock", "Missing IGRF block in %s.", modelName);
end

for k = 1:numel(blocks)
    simIn = setAocsParameters(simIn, blocks{k}, ...
        "generation", "IGRF-14", ...
        "units", "Metric (MKS)", ...
        "time_in", "on", ...
        "action", "Error", ...
        "sv_out", "off");
end
end
