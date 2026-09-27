function blocks = findBlocksByBlockType(modelName, blockType)
% Description:
%   Finds built-in Aerospace Blockset blocks by BlockType, including blocks
%   that are not masked subsystems.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%   blockType - BlockType string to match.
%
% Outputs:
%   blocks - Cell array of matching block paths.

candidates = find_system(modelName, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "Type", "Block");

types = get_param(candidates, "BlockType");
blocks = candidates(string(types) == string(blockType));
end
