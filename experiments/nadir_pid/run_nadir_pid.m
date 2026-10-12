close(findall(groot,'Type','figure','Tag','NadirGuidance'));
% Run this file directly from the MATLAB Editor or with run(fullPath).
[model, root, Ts, aocs, Ad, P, rho, relativeResidual] = loadPidExperiment;

report = rotatingReferenceTrials(model, aocs, P, root, true);
disp(report);
