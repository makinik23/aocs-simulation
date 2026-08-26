function raw = readAocsConfigFile(configFile)
%READAOCSCONFIGFILE Read JSON config and apply recursive extends overrides.

configFile = string(configFile);
raw = jsondecode(fileread(configFile));

if isfield(raw, "extends")
    baseFiles = configExtendsList(raw.extends, configFile);
    raw = rmfield(raw, "extends");

    merged = struct();
    for k = 1:numel(baseFiles)
        merged = mergeConfigStructs(merged, readAocsConfigFile(baseFiles(k)));
    end

    raw = mergeConfigStructs(merged, raw);
end
end

function files = configExtendsList(extendsValue, configFile)
if ischar(extendsValue) || (isstring(extendsValue) && isscalar(extendsValue))
    files = string(extendsValue);
elseif isstring(extendsValue)
    files = extendsValue(:);
elseif iscell(extendsValue)
    files = strings(numel(extendsValue), 1);
    for k = 1:numel(extendsValue)
        item = extendsValue{k};
        if ~(ischar(item) || (isstring(item) && isscalar(item)))
            error("AOCS:Config:InvalidExtends", ...
                "Config extends entries must be scalar strings: %s", char(configFile));
        end
        files(k) = string(item);
    end
else
    error("AOCS:Config:InvalidExtends", ...
        "Config extends must be a scalar string or string array: %s", char(configFile));
end

for k = 1:numel(files)
    if ~isfile(files(k))
        files(k) = fullfile(fileparts(configFile), files(k));
    end

    if ~isfile(files(k))
        error("AOCS:Config:MissingFile", ...
            "AOCS extended config file not found: %s", files(k));
    end
end
end

function merged = mergeConfigStructs(base, override)
merged = base;
fields = fieldnames(override);

for k = 1:numel(fields)
    name = fields{k};
    if isfield(merged, name) && isstruct(merged.(name)) && isstruct(override.(name)) ...
            && isscalar(merged.(name)) && isscalar(override.(name))
        merged.(name) = mergeConfigStructs(merged.(name), override.(name));
    else
        merged.(name) = override.(name);
    end
end
end
