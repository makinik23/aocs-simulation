function artifacts = buildDtm2020Native()
% Description:
%   Builds the official DTM2020 operational Fortran source behind a C ABI,
%   a MATLAB MEX test gateway, and a Simulink Level-2 C S-Function.

projectRoot = fileparts(fileparts(mfilename("fullpath")));
upstreamRoot = fullfile(projectRoot, "third_party", "dtm2020", "upstream");
nativeRoot = fullfile(projectRoot, "src", "native", "dtm2020");
buildRoot = fullfile(projectRoot, "build", "native", "dtm2020", computer("arch"));

fortranModel = fullfile(upstreamRoot, "src", "libswamif", ...
    "dtm2020_F107_Kp-subr_MCM.f90");
fortranSigma = fullfile(upstreamRoot, "src", "libswamif", ...
    "dtm2020_sigma_function.f90");
fortranBridge = fullfile(nativeRoot, "dtm2020_bridge.f90");
coefficientFile = fullfile(upstreamRoot, "data", "DTM_2020_F107_Kp.dat");

requiredFiles = [string(fortranModel), string(fortranSigma), ...
    string(fortranBridge), string(coefficientFile)];
for file = requiredFiles
    if ~isfile(file)
        error("AOCS:DTM2020:MissingSource", "Required DTM2020 file not found: %s", file);
    end
end
if computer("arch") ~= "maca64"
    error("AOCS:DTM2020:UnsupportedBuildHost", ...
        "This build helper currently targets Apple silicon MATLAB (maca64).");
end

if ~isfolder(buildRoot)
    mkdir(buildRoot);
end

gfortran = strtrim(runCommand("command -v gfortran"));
if strlength(gfortran) == 0
    error("AOCS:DTM2020:MissingCompiler", "gfortran was not found on PATH.");
end

nativeLibrary = fullfile(buildRoot, "libaocs_dtm2020.dylib");
compileCommand = strjoin([ ...
    shellQuote(gfortran), "-O3", "-fPIC", "-dynamiclib", ...
    "-mmacosx-version-min=12.0", ...
    "-J", shellQuote(buildRoot), ...
    shellQuote(fortranModel), shellQuote(fortranSigma), shellQuote(fortranBridge), ...
    "-Wl,-install_name," + shellQuote(nativeLibrary), ...
    "-o", shellQuote(nativeLibrary)], " ");
runCommand(compileCommand);

mexOptions = createCommandLineToolsMexOptions(buildRoot);
commonArguments = {"-f", mexOptions, "-R2018a", ...
    "-I" + nativeRoot, nativeLibrary, "-outdir", buildRoot};
mex(commonArguments{:}, "-output", "dtm2020_mex", ...
    fullfile(nativeRoot, "dtm2020_mex.c"));
mex(commonArguments{:}, "-output", "dtm2020_sfun", ...
    fullfile(nativeRoot, "dtm2020_sfun.c"));

addpath(buildRoot);
clear dtm2020_mex;
[rho, ~, temperature, temperatureExospheric] = dtm2020_mex( ...
    char(coefficientFile), 180.0, [80.0; 0.0], [80.0; 0.0], ...
    [3.0; 0.0; 3.0; 0.0], 300.0, 3.1415, 0.0, 0.0);
assert(abs(rho - 0.94719e-14) <= 1e-19, "DTM2020 density benchmark failed.");
assert(abs(temperature - 843.243) <= 2e-3, "DTM2020 temperature benchmark failed.");
assert(abs(temperatureExospheric - 844.099) <= 2e-3, ...
    "DTM2020 exospheric-temperature benchmark failed.");

artifacts = struct( ...
    "BuildDirectory", buildRoot, ...
    "NativeLibrary", nativeLibrary, ...
    "MexGateway", fullfile(buildRoot, "dtm2020_mex." + mexext), ...
    "SFunction", fullfile(buildRoot, "dtm2020_sfun." + mexext), ...
    "CoefficientFile", coefficientFile);
end

function optionsFile = createCommandLineToolsMexOptions(buildRoot)
templateFile = fullfile(matlabroot, "bin", "maca64", "mexopts", "clang_maca64.xml");
xml = fileread(templateFile);
pattern = "(?s)\s*<!-- User needs to agree to license.*?</XCODE_AGREED_VERSION>";
xml = regexprep(xml, pattern, "");
optionsFile = fullfile(buildRoot, "clang_clt_maca64.xml");
fileId = fopen(optionsFile, "w");
if fileId < 0
    error("AOCS:DTM2020:BuildIO", "Unable to write MEX options file: %s", optionsFile);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, "%s", xml);
end

function output = runCommand(command)
[status, output] = system(command);
if status ~= 0
    error("AOCS:DTM2020:BuildCommandFailed", ...
        "Command failed with status %d:\n%s\n%s", status, command, output);
end
end

function value = shellQuote(value)
value = string(value);
if contains(value, "'")
    error("AOCS:DTM2020:UnsupportedPath", ...
        "Native build paths must not contain apostrophes: %s", value);
end
value = "'" + value + "'";
end
