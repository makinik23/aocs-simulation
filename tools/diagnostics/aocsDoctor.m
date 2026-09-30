function report = aocsDoctor()
%AOCSDOCTOR Read-only environment inventory; false readiness is not a pass.
root = setupAocsPaths();
installed = ver;
products = string({installed.Name});
required = ["MATLAB", "Simulink", "Aerospace Blockset", "Aerospace Toolbox", ...
    "DSP System Toolbox"];
report = struct("MATLAB", version, "Release", version('-release'), ...
    "Architecture", computer('arch'), "Root", root, ...
    "RequiredProducts", required, "MissingProducts", required(~ismember(required, products)));
report.CoreReady = isempty(report.MissingProducts);
report.NativeReady = false;
try
    requireDtm2020Native();
    report.NativeReady = true;
    report.NativeDiagnostic = "ready";
catch problem
    report.NativeDiagnostic = string(problem.message);
end
report.PlanetFixtureAvailable = isfile(fullfile(root, 'validation', 'planet', ...
    'data', 'planet_dove_oem_reference.mat'));
disp(report);
end
