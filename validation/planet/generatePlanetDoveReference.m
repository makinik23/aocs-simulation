function fixture = generatePlanetDoveReference(oemFile, outputFile, maxDuration_s)
% Description:
%   Converts a Planet Labs CCSDS OEM file into a compact MATLAB fixture for
%   CubeSat orbit-propagator validation.
%
% Arguments:
%   oemFile - Path to a Planet Labs HWID_oem.txt file.
%   outputFile - Optional output MAT-file path.
%   maxDuration_s - Optional maximum fixture duration from the first sample [s].
%
% Outputs:
%   fixture - Struct containing reference metadata and inertial state samples.

if nargin < 2
    outputFile = "";
end

if nargin < 3 || isempty(maxDuration_s)
    maxDuration_s = 15 * 60;
end

oemFile = string(oemFile);
if strlength(oemFile) == 0 || ~isfile(oemFile)
    error("AOCS:Validation:MissingPlanetOem", ...
        "Planet OEM file not found: %s", char(oemFile));
end

[timeUtc, r_I_km, v_I_km_s, metadata] = readPlanetOemStateVectors(oemFile);
time_s = seconds(timeUtc - timeUtc(1));

if isfinite(maxDuration_s) && maxDuration_s > 0.0
    keep = time_s <= maxDuration_s;
    timeUtc = timeUtc(keep);
    time_s = time_s(keep);
    r_I_km = r_I_km(keep, :);
    v_I_km_s = v_I_km_s(keep, :);
end

if numel(time_s) < 2
    error("AOCS:Validation:InsufficientPlanetSamples", ...
        "Planet OEM fixture needs at least two state samples.");
end

fixture = struct();
fixture.Source = struct();
fixture.Source.File = oemFile;
fixture.Source.Originator = metadataValue(metadata, "originator", "PLANET/USA");
fixture.Source.ObjectName = metadataValue(metadata, "object_name", "");
fixture.Source.ObjectId = metadataValue(metadata, "object_id", "");
fixture.Source.CenterName = metadataValue(metadata, "center_name", "Earth");
fixture.Source.RefFrame = metadataValue(metadata, "ref_frame", "EME2000");
fixture.Source.TimeSystem = metadataValue(metadata, "time_system", "UTC");
fixture.Source.License = "CC BY-NC 4.0";
fixture.Source.Url = "https://ephemerides.planet-labs.com/";
fixture.Source.CreationDateUtc = metadataValue(metadata, "creation_date", "");

fixture.window_start_utc = char(planetDatetimeToIso(timeUtc(1)));
fixture.window_stop_utc = char(planetDatetimeToIso(timeUtc(end)));
fixture.time_utc_iso = planetDatetimeToIso(timeUtc);
fixture.time_s = time_s(:);
fixture.r_I_ref_m = r_I_km .* 1000.0;
fixture.v_I_ref_m_s = v_I_km_s .* 1000.0;

if strlength(string(outputFile)) > 0
    outputFile = string(outputFile);
    outputDirectory = fileparts(outputFile);
    if strlength(outputDirectory) > 0 && ~isfolder(outputDirectory)
        mkdir(outputDirectory);
    end
    save(outputFile, "-struct", "fixture");
end
end

function [timeUtc, r_I_km, v_I_km_s, metadata] = readPlanetOemStateVectors(oemFile)
% Description:
%   Parses metadata and state-vector rows from a CCSDS OEM text file.

lines = readlines(oemFile);
timeUtc = NaT(0, 1, "TimeZone", "UTC");
r_I_km = zeros(0, 3);
v_I_km_s = zeros(0, 3);
metadata = struct();

for k = 1:numel(lines)
    line = strtrim(lines(k));
    if strlength(line) == 0
        continue;
    end

    if startsWith(line, "COVARIANCE_START")
        break;
    end

    metadataMatch = regexp(line, "^([A-Z0-9_]+)\s*=\s*(.*)$", "tokens", "once");
    if ~isempty(metadataMatch)
        key = matlab.lang.makeValidName(lower(metadataMatch{1}));
        metadata.(key) = string(strtrim(metadataMatch{2}));
        continue;
    end

    if isempty(regexp(line, "^\d{4}-\d{2}-\d{2}T", "once"))
        continue;
    end

    parts = split(line);
    if numel(parts) < 7
        continue;
    end

    timeUtc(end + 1, 1) = parsePlanetUtc(parts(1)); %#ok<AGROW>
    numericValues = str2double(parts(2:7));
    if any(~isfinite(numericValues))
        error("AOCS:Validation:InvalidPlanetOemRow", ...
            "Invalid numeric state-vector row in %s at line %d.", char(oemFile), k);
    end
    r_I_km(end + 1, :) = numericValues(1:3).'; %#ok<AGROW>
    v_I_km_s(end + 1, :) = numericValues(4:6).'; %#ok<AGROW>
end

if isempty(timeUtc)
    error("AOCS:Validation:EmptyPlanetOem", ...
        "No state-vector rows found in Planet OEM file: %s", char(oemFile));
end

[timeUtc, order] = sort(timeUtc);
r_I_km = r_I_km(order, :);
v_I_km_s = v_I_km_s(order, :);
end

function value = metadataValue(metadata, fieldName, defaultValue)
% Description:
%   Reads a parsed metadata field or returns a default string.

fieldName = char(fieldName);
if isfield(metadata, fieldName)
    value = metadata.(fieldName);
else
    value = string(defaultValue);
end
end

function value = parsePlanetUtc(text)
% Description:
%   Parses CCSDS OEM UTC timestamps with optional millisecond fields.

text = string(text);
formats = [
    "yyyy-MM-dd'T'HH:mm:ss.SSS'Z'"
    "yyyy-MM-dd'T'HH:mm:ss'Z'"
];

for format = formats(:).'
    try
        value = datetime(text, "InputFormat", format, "TimeZone", "UTC");
        return;
    catch
    end
end

error("AOCS:Validation:InvalidPlanetUtc", ...
    "Could not parse Planet OEM UTC timestamp: %s", char(text));
end

function text = planetDatetimeToIso(timeUtc)
% Description:
%   Formats UTC datetimes as ISO-8601 Zulu strings.

timeUtc.Format = "yyyy-MM-dd'T'HH:mm:ss.SSS'Z'";
text = string(timeUtc);
end
