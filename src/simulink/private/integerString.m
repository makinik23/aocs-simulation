function value = integerString(value)
% Description:
%   Formats an integer-valued scalar for Simulink mask parameters.
%
% Arguments:
%   value - Numeric scalar.
%
% Outputs:
%   value - Character vector without decimal places.

value = sprintf("%.0f", value);
end
