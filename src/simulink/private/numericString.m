function value = numericString(value)
% Description:
%   Formats a finite scalar for Simulink mask parameters.
%
% Arguments:
%   value - Numeric scalar.
%
% Outputs:
%   value - Character vector preserving useful precision.

value = sprintf("%.15g", value);
end
