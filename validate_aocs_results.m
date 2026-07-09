function report = validate_aocs_results(resultsFile)
% Description:
%   Loads the latest simulation result, extracts body rates from the logged
%   AOCS state, and checks torque-free rotational energy and angular momentum
%   norm conservation against limits from the JSON configuration.
%
% Arguments:
%   resultsFile - Optional MAT-file containing simulation output variable out.
%
% Outputs:
%   report - Struct with source name, energy drift, momentum drift, and pass
%            flags for the conservation checks.

projectRoot = fileparts(mfilename("fullpath"));
addpath(fullfile(projectRoot, "src", "config"));
addpath(fullfile(projectRoot, "src", "analysis"));

AOCS = loadAocsSimulationConfig(fullfile(projectRoot, "config", "AocsSimulationConfig.json"), projectRoot);

if nargin < 1 || strlength(string(resultsFile)) == 0
    resultsFile = AOCS.Results.File;
end

load(resultsFile, "out");

state = extractAocsState(out);
omegaData = loggedSignalMatrix(state.omega_b.Data, 3, "omega_b");
invariants = computeAocsInvariants(omegaData, AOCS.Spacecraft.I_B);

energyError = max(abs(invariants.E_rot - invariants.E_rot(1)));
momentumError = max(abs(invariants.H_norm - invariants.H_norm(1)));

fprintf("\n");
fprintf("Validation source : %s.omega_b + configured I_B\n", char(state.Source));
fprintf("Energy drift      : %.3e\n", energyError);
fprintf("Momentum drift    : %.3e\n", momentumError);
fprintf("\n");

if energyError < AOCS.Numerics.MaxAllowedEnergyDrift
    disp("Energy conserved.")
else
    warning("Energy NOT conserved.")
end

if momentumError < AOCS.Numerics.MaxAllowedHnormDrift
    disp("Angular momentum conserved.")
else
    warning("Angular momentum NOT conserved.")
end

report = struct();
report.Source = state.Source;
report.EnergyDrift = energyError;
report.MomentumDrift = momentumError;
report.EnergyConserved = energyError < AOCS.Numerics.MaxAllowedEnergyDrift;
report.MomentumConserved = momentumError < AOCS.Numerics.MaxAllowedHnormDrift;
end
