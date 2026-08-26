function value = eclipseShadowModelMaskValue(model)
% Description:
%   Maps project eclipse model names to Aerospace Blockset mask values.
%
% Arguments:
%   model - Project-level eclipse model string.
%
% Outputs:
%   value - Mask value accepted by Eclipse Shadow Model.

switch string(model)
    case "dual_cone"
        value = "Dual cone";
    otherwise
        error("AOCS:Simulink:UnsupportedEclipseModel", ...
            "Unsupported eclipse model '%s'.", char(model));
end
end
