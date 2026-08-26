function value = onOff(flag)
% Description:
%   Converts a logical flag to the on/off strings expected by Simulink masks.
%
% Arguments:
%   flag - Scalar logical.
%
% Outputs:
%   value - 'on' when true, otherwise 'off'.

if flag
    value = "on";
else
    value = "off";
end
end
