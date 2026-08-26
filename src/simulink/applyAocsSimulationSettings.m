function applyAocsSimulationSettings(modelName, AOCS)
% Description:
%   Applies solver timing/tolerance settings and points Aerospace Blockset
%   block mask parameters at values exposed by the validated AOCS config.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%   AOCS - Validated configuration struct.
%
% Outputs:
%   None.

applySolverSettings(modelName, AOCS);
applyAerospace6DofSettings(modelName);
applyOrbitPropagatorSettings(modelName, AOCS);
applySunEphemerisSettings(modelName, AOCS);
applyEarthFrameSettings(modelName, AOCS);
applyIgrfSettings(modelName);
applyEclipseShadowModelSettings(modelName, AOCS);
end
