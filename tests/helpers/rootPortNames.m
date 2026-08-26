function names = rootPortNames(modelName, blockType)
% Description:
%   Returns root port names sorted by Simulink port number.
%
% Arguments:
%   modelName - Loaded Simulink model or harness name.
%   blockType - Root port block type, for example "Inport" or "Outport".
%
% Outputs:
%   names - String array of root port names sorted by port number.

blocks = find_system(modelName, "SearchDepth", 1, "BlockType", blockType);
ports = cellfun(@(block) str2double(get_param(block, "Port")), blocks);
[~, order] = sort(ports);
names = string(cellfun(@(block) get_param(block, "Name"), blocks, ...
    "UniformOutput", false));
names = names(order);
end
