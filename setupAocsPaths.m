function rootDirectory = setupAocsPaths(rootDirectory, includeTests)
%SETUPAOCSPATHS Add the standard AOCS project folders to the MATLAB path.

if nargin < 1 || strlength(string(rootDirectory)) == 0
    rootDirectory = projectRoot();
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
    fullfile(rootDirectory, "src", "simulink")
    fullfile(rootDirectory, "tools")
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
