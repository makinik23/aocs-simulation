function metadata = aocsRunProvenance(root)
%AOCSRUNPROVENANCE Identify environment and exact working-tree input content.
metadata = struct("StartedUTC", string(datetime('now', TimeZone='UTC')), ...
    "MATLAB", version, "Release", version('-release'), ...
    "Architecture", computer('arch'), "Products", ver);
[status, revision] = system("git -C " + aocsShellQuote(root) + " rev-parse HEAD");
if status == 0
    metadata.GitRevision = string(strtrim(revision));
    [status, changes] = system("git -C " + aocsShellQuote(root) + " status --porcelain");
    metadata.GitStatusAvailable = status == 0;
    metadata.GitStatus = string(changes);
else
    metadata.GitRevision = "unavailable";
    metadata.GitStatusAvailable = false;
    metadata.GitStatus = "unavailable";
end
% Include untracked sources: a commit id alone cannot identify a dirty run.
folders = ["src", "config", "models", "tools", "tests", "validation"];
files = strings(0,1);
for folder = folders
    listing = dir(fullfile(root, folder, '**', '*'));
    listing = listing(~[listing.isdir]);
    for k = 1:numel(listing)
        [~,~,extension] = fileparts(listing(k).name);
        if any(string(extension) == [".m", ".c", ".h", ".f90", ".json", ".slx", ".py", ".mat", ".csv", ".cdf", ".nc", ".EOF"])
            files(end+1,1) = string(fullfile(listing(k).folder, listing(k).name)); %#ok<AGROW>
        end
    end
end
listing = dir(fullfile(root, '*.m'));
files = [files; string(fullfile({listing.folder}, {listing.name})).'];
files = sort(files);
hashes = strings(size(files));
for k = 1:numel(files)
    hashes(k) = aocsFileHash(files(k));
end
metadata.SourceFiles = erase(files, string(root) + filesep);
metadata.SourceSHA256 = hashes;
manifest = fullfile(root, 'build', 'native', 'dtm2020', computer('arch'), 'build-manifest.json');
if isfile(manifest)
    metadata.NativeBuild = jsondecode(fileread(manifest));
end
end
