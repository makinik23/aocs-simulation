function artifacts = requireDtm2020Native()
%REQUIREDTM2020NATIVE Use a verified build; never silently compile on simulation.
root = projectRoot();
directory = fullfile(root, "build", "native", "dtm2020", computer('arch'));
manifestFile = fullfile(directory, 'build-manifest.mat');
gateway = fullfile(directory, "dtm2020_mex." + mexext);
sfun = fullfile(directory, "dtm2020_sfun." + mexext);
if ~isfile(manifestFile) || ~isfile(gateway) || ~isfile(sfun)
    error("AOCS:DTM2020:BuildRequired", ...
        'Native DTM2020 is not bootstrapped. Run bootstrapAocs("full") first.');
end
manifest = load(manifestFile, 'signature', 'artifacts');
if ~isequal(manifest.signature, dtm2020BuildSignature())
    error("AOCS:DTM2020:StaleBuild", ...
        'DTM2020 sources or MATLAB release changed. Run bootstrapAocs("full").');
end
if ~isfile(manifest.artifacts.NativeLibrary)
    error("AOCS:DTM2020:BuildRequired", 'Native library is missing. Run bootstrapAocs("full").');
end
addpath(directory);
artifacts = manifest.artifacts;
end
