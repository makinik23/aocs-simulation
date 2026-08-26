function value = eclipseTimeSourceMaskValue(timeSource)
% Description:
%   Maps project time-source names to Aerospace Blockset mask values.
%
% Arguments:
%   timeSource - Project-level eclipse time source string.
%
% Outputs:
%   value - Mask value accepted by Eclipse Shadow Model.

switch string(timeSource)
    case "dialog"
        value = "Dialog";
    otherwise
        error("AOCS:Simulink:UnsupportedEclipseTimeSource", ...
            "Unsupported eclipse time source '%s'.", char(timeSource));
end
end
