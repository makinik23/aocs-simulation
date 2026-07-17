function report = validate_aocs_results(resultsFile)
% Description:
%   Loads the latest simulation result, extracts body rates from the logged
%   AOCS state, reports rotational energy and angular momentum norm drift, and
%   applies strict conservation checks only for torque-free simulations.
%
% Arguments:
%   resultsFile - Optional MAT-file containing simulation output variable out.
%
% Outputs:
%   report - Struct with source name, energy drift, momentum drift,
%            disturbance-torque magnitude, and conservation diagnostics.

projectRoot = fileparts(mfilename("fullpath"));
addpath(fullfile(projectRoot, "src", "config"));
addpath(fullfile(projectRoot, "src", "analysis"));

defaultAOCS = loadAocsSimulationConfig(fullfile(projectRoot, "config", "AocsSimulationConfig.json"), projectRoot);

if nargin < 1 || strlength(string(resultsFile)) == 0
    resultsFile = defaultAOCS.Results.File;
end

loaded = load(resultsFile);
out = loaded.out;

if isfield(loaded, "AOCS")
    AOCS = loaded.AOCS;
else
    AOCS = defaultAOCS;
end

state = extractAocsState(out);
omegaData = loggedSignalMatrix(state.omega_b.Data, 3, "omega_b");
invariants = computeAocsInvariants(omegaData, AOCS.Spacecraft.I_B);
maxDisturbanceTorque = maxLoggedDisturbanceTorque(out, AOCS);
isTorqueFree = maxDisturbanceTorque < 1e-14;

energyError = max(abs(invariants.E_rot - invariants.E_rot(1)));
momentumError = max(abs(invariants.H_norm - invariants.H_norm(1)));

fprintf("\n");
fprintf("Validation source : %s.omega_b + configured I_B\n", char(state.Source));
fprintf("Energy drift      : %.3e\n", energyError);
fprintf("Momentum drift    : %.3e\n", momentumError);
fprintf("Max disturbance   : %.3e N*m\n", maxDisturbanceTorque);
fprintf("\n");

if isTorqueFree
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
else
    disp("Nonzero modeled disturbance torque detected; conservation is reported as a diagnostic, not a pass/fail check.")
end

report = struct();
report.Source = state.Source;
report.EnergyDrift = energyError;
report.MomentumDrift = momentumError;
report.MaxDisturbanceTorque = maxDisturbanceTorque;
report.TorqueFree = isTorqueFree;
report.EnergyConserved = isTorqueFree && energyError < AOCS.Numerics.MaxAllowedEnergyDrift;
report.MomentumConserved = isTorqueFree && momentumError < AOCS.Numerics.MaxAllowedHnormDrift;
end

function maxTorque = maxLoggedDisturbanceTorque(out, AOCS)
% Description:
%   Returns the maximum modeled body disturbance torque from logs when
%   available, falling back to the configured external torque.
%
% Arguments:
%   out - Simulink.SimulationOutput from a plant simulation.
%   AOCS - Validated configuration struct.
%
% Outputs:
%   maxTorque - Maximum body disturbance torque norm [N*m].

maxTorque = norm(AOCS.Environment.M_ext_B);

try
    logsout = out.logsout;
    element = logsout.getElement("M_dist_B_Nm");
catch
    element = [];
end

if isempty(element)
    return;
end

torqueData = loggedSignalMatrix(element.Values.Data, 3, "M_dist_B_Nm");
totalDisturbanceTorque = torqueData + AOCS.Environment.M_ext_B(:).';
maxTorque = max(vecnorm(totalDisturbanceTorque, 2, 2));
end
