function report = bootstrapAocs(profile, options)
%BOOTSTRAPAOCS Explicit dependency setup. Opening the project never builds code.
% core: configuration, sensors and pure MATLAB tests, no native compiler.
% full: also fetch pinned DTM2020 when absent and build/test the native backend.
arguments
    profile (1,1) string {mustBeMember(profile,["core","full"])} = "core"
    options.UseCommandLineTools (1,1) logical = false
end
root = setupAocsPaths();
configureAocsFileGeneration();
if profile == "full"
    upstream = fullfile(root, 'third_party', 'dtm2020', 'upstream');
    revision = 'a488a7c9d030bfbe86e88ab3d28a7ec5589b92e0';
    if ~isfolder(upstream)
        runChecked("git clone https://github.com/swami-h2020-eu/mcm.git " + aocsShellQuote(upstream));
        runChecked("git -C " + aocsShellQuote(upstream) + " checkout --detach " + revision);
    end
    actual = strtrim(runChecked("git -C " + aocsShellQuote(upstream) + " rev-parse HEAD"));
    dirty = strtrim(runChecked("git -C " + aocsShellQuote(upstream) + " status --porcelain --untracked-files=no"));
    if string(actual) ~= string(revision) || strlength(string(dirty)) > 0
        error("AOCS:DTM2020:DependencyRevision", ...
            "DTM2020 must be clean at %s. Existing checkout was left unchanged.", revision);
    end
    buildDtm2020Native(UseCommandLineTools=options.UseCommandLineTools);
end
setupAocsSimulation;
report = aocsDoctor();
end

function output = runChecked(command)
[status, output] = system(command);
if status ~= 0
    error("AOCS:Bootstrap:CommandFailed", "%s\n%s", command, output);
end
end
