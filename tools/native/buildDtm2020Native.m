function artifacts = buildDtm2020Native(options)
% Description:
%   Builds the official DTM2020 operational Fortran source behind a C ABI,
%   a MATLAB MEX test gateway, and a Simulink Level-2 C S-Function.

arguments
    options.UseCommandLineTools (1,1) logical = false
end

rootDirectory = projectRoot();
upstreamRoot = fullfile(rootDirectory, "third_party", "dtm2020", "upstream");
nativeRoot = fullfile(rootDirectory, "src", "native", "dtm2020");
buildRoot = fullfile(rootDirectory, "build", "native", "dtm2020", computer("arch"));

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
if ispc
    error("AOCS:DTM2020:UnsupportedBuildHost", ...
        'Windows supports bootstrapAocs("core"). Native Windows DTM2020 remains unvalidated; use Linux for full-plant runs.');
end

if ~isfolder(buildRoot)
    mkdir(buildRoot);
end

gfortran = strtrim(runCommand("command -v gfortran"));
if strlength(gfortran) == 0
    error("AOCS:DTM2020:MissingCompiler", "gfortran was not found on PATH.");
end

% A failed rebuild must never leave an old manifest advertising a valid backend.
for manifestName = ["build-manifest.mat", "build-manifest.json"]
    manifestFile = fullfile(buildRoot, manifestName);
    if isfile(manifestFile)
        delete(manifestFile);
    end
end
% Unload gateways before replacing a loaded library. Do not run during sim.
clear dtm2020_mex dtm2020_sfun;
if ismac
    libraryName = "libaocs_dtm2020.dylib";
    sharedFlags = "-dynamiclib -Wl,-install_name,@rpath/" + libraryName;
    loaderFlags = "LDFLAGS=$LDFLAGS -Wl,-rpath,@loader_path";
else
    libraryName = "libaocs_dtm2020.so";
    sharedFlags = "-shared -Wl,-soname," + libraryName;
    loaderFlags = "LDFLAGS=$LDFLAGS -Wl,-rpath,'$$ORIGIN'";
end
nativeLibrary = fullfile(buildRoot, libraryName);
compileCommand = strjoin([ ...
    shellQuote(gfortran), "-O3", "-fPIC", sharedFlags, ...
    "-J", shellQuote(buildRoot), ...
    shellQuote(fortranModel), shellQuote(fortranSigma), shellQuote(fortranBridge), ...
    "-o", shellQuote(nativeLibrary)], " ");
runCommand(compileCommand);

commonArguments = {"-R2018a", "-I" + nativeRoot, nativeLibrary, ...
    loaderFlags, "-outdir", buildRoot};
if options.UseCommandLineTools
    if ~ismac || computer("arch") ~= "maca64"
        error("AOCS:DTM2020:CLTHost", "The explicit CLT fallback is only for Apple silicon.");
    end
    commonArguments = [{"-f", createCommandLineToolsMexOptions(buildRoot)}, commonArguments];
end
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
signature = dtm2020BuildSignature();
save(fullfile(buildRoot, 'build-manifest.mat'), 'signature', 'artifacts');
manifest = struct("Signature", signature, "Compiler", gfortran, ...
    "CompilerVersion", strtrim(runCommand(shellQuote(gfortran) + " --version")), ...
    "UsedCommandLineToolsFallback", options.UseCommandLineTools);
writeAocsJson(fullfile(buildRoot, 'build-manifest.json'), manifest);
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
