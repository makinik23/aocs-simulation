close(findall(groot,'Type','figure','Tag','NadirConstant'));
% Run this file directly from the MATLAB Editor or with run(fullPath).
[model, root, Ts, aocs, Ad, P, rho, relativeResidual] = loadPidExperiment;

report = constantTorqueTrial(model, aocs, P, root);
disp(report);
