function simIn = applyAocsSimulationSettings(modelName, AOCS, simIn)
% Description:
%   Applies solver timing/tolerance settings and points Aerospace Blockset
%   block mask parameters at values exposed by the validated AOCS config.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%   AOCS - Validated configuration struct.
%
% Outputs:
%   simIn - Updated simulation specification, or [] for an interactive edit.

% With no third argument, this is an explicit interactive model edit.
% With SimulationInput, all overrides are scoped to that simulation.
if nargin < 3
    simIn = [];
end
simIn = applySolverSettings(modelName, AOCS, simIn);
simIn = applyAerospace6DofSettings(modelName, simIn);
simIn = applyOrbitPropagatorSettings(modelName, AOCS, simIn);
simIn = applySunEphemerisSettings(modelName, AOCS, simIn);
simIn = applyEarthFrameSettings(modelName, AOCS, simIn);
simIn = applyIgrfSettings(modelName, simIn);
simIn = applyEclipseShadowModelSettings(modelName, AOCS, simIn);
end
