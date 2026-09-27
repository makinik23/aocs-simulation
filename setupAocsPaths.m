function [rootDirectory, paths] = setupAocsPaths(rootDirectory, includeTests)
%SETUPAOCSPATHS Add the standard AOCS project folders to the MATLAB path.

if nargin < 1 || isempty(rootDirectory) || strlength(string(rootDirectory)) == 0
    rootDirectory = string(fileparts(mfilename("fullpath")));
end

if nargin < 2
    includeTests = false;
end

rootDirectory = string(rootDirectory);
paths = [
    rootDirectory
    fullfile(rootDirectory, "src", "analysis")
    fullfile(rootDirectory, "src", "config")
    fullfile(rootDirectory, "src", "environment")
    fullfile(rootDirectory, "src", "gnc")
    fullfile(rootDirectory, "src", "gnc", "orbit")
    fullfile(rootDirectory, "src", "sensors")
    fullfile(rootDirectory, "src", "simulink")
    fullfile(rootDirectory, "src", "simulink", "buses")
    fullfile(rootDirectory, "src", "simulink", "buses", "config")
    fullfile(rootDirectory, "src", "simulink", "buses", "environment")
    fullfile(rootDirectory, "src", "simulink", "buses", "measurements")
    fullfile(rootDirectory, "src", "simulink", "buses", "state")
    fullfile(rootDirectory, "src", "simulink", "buses", "drivers")
    fullfile(rootDirectory, "tools", "project")
    fullfile(rootDirectory, "tools", "diagnostics")
    fullfile(rootDirectory, "tools", "provenance")
    fullfile(rootDirectory, "tools", "native")
    fullfile(rootDirectory, "tools", "model_builders")
];

if includeTests
    paths = [
        paths
        fullfile(rootDirectory, "tests", "helpers")
        fullfile(rootDirectory, "tests", "harnesses")
    ];
end

for path = paths(:).'
    if isfolder(path)
        addpath(path);
    end
end
end
