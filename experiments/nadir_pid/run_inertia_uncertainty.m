close(findall(groot,'Type','figure','Tag','NadirInertia'));
% Run this file directly from the MATLAB Editor or with run(fullPath).
[model, root, Ts, aocs, Ad, P, rho, relativeResidual] = loadPidExperiment;

report = rotatingReferenceTrials(model, aocs, P, root, false);
disp(report);
