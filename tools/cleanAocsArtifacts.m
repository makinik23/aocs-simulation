function removed = cleanAocsArtifacts(projectRoot, dryRun)
%CLEANAOCSARTIFACTS Remove generated MATLAB/Simulink runtime artifacts.
%
%   cleanAocsArtifacts() removes ignored simulation outputs and cache files
%   from the project tree while preserving build products, third-party
%   sources, and local virtual environments.

if nargin < 1 || strlength(string(projectRoot)) == 0
    projectRoot = fileparts(fileparts(mfilename("fullpath")));
end

if nargin < 2
    dryRun = false;
end

projectRoot = string(projectRoot);
addpath(projectRoot);
setupAocsPaths(projectRoot);

skipDirectories = [
    fullfile(projectRoot, "build")
    fullfile(projectRoot, "third_party")
    fullfile(projectRoot, "venv")
    fullfile(projectRoot, ".git")
];
filePatterns = [
    ".DS_Store"
    "._*"
    "*.asv"
    "*.m~"
    "*.slx.autosave"
    "*.mdl.autosave"
    "*.sldd.autosave"
    "*.slxc"
    "*.slxc.lock"
];
directoryNames = ["results", "slprj", "test-results"];

removed = strings(0, 1);

for pattern = filePatterns(:).'
    matches = dir(fullfile(projectRoot, "**", pattern));
    for k = 1:numel(matches)
        if matches(k).isdir
            continue;
        end

        path = fullfile(matches(k).folder, matches(k).name);
        if isSkipped(path, skipDirectories)
            continue;
        end

        removed(end + 1, 1) = string(path); %#ok<AGROW>
        if ~dryRun
            delete(path);
        end
    end
end

directories = collectNamedDirectories(projectRoot, directoryNames, skipDirectories);
for path = directories(:).'
    removed(end + 1, 1) = path; %#ok<AGROW>
    if ~dryRun && isfolder(path)
        rmdir(path, "s");
    end
end

if nargout == 0
    fprintf("AOCS artifact cleanup: %d paths%s.\n", ...
        numel(removed), ternary(dryRun, " would be removed", " removed"));
    clear removed
end
end

function paths = collectNamedDirectories(rootDirectory, directoryNames, skipDirectories)
paths = strings(0, 1);
entries = dir(rootDirectory);

for k = 1:numel(entries)
    entry = entries(k);
    name = string(entry.name);
    if ~entry.isdir || name == "." || name == ".."
        continue;
    end

    path = string(fullfile(entry.folder, entry.name));
    if isSkipped(path, skipDirectories)
        continue;
    end

    if any(name == directoryNames)
        paths(end + 1, 1) = path; %#ok<AGROW>
    else
        paths = [paths; collectNamedDirectories(path, directoryNames, skipDirectories)]; %#ok<AGROW>
    end
end
end

function result = isSkipped(path, skipDirectories)
path = string(path);
result = false;
for directory = skipDirectories(:).'
    directory = string(directory);
    if path == directory || startsWith(path, directory + filesep)
        result = true;
        return;
    end
end
end

function value = ternary(condition, trueValue, falseValue)
if condition
    value = trueValue;
else
    value = falseValue;
end
end
