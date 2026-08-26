function value = scalarField(parent, fieldName, displayName, mustBePositive)
%SCALARFIELD Read a finite real scalar JSON number.

value = double(requireField(parent, fieldName, displayName));
validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar'}, ...
    mfilename, displayName);
if mustBePositive && value <= 0
    error("AOCS:Config:InvalidField", ...
        "Config field %s must be positive.", displayName);
end
end
