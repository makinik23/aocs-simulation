function value = stringScalarField(parent, fieldName, displayName)
%STRINGSCALARFIELD Read a required non-empty scalar JSON string.

rawValue = requireField(parent, fieldName, displayName);
if ~(ischar(rawValue) || (isstring(rawValue) && isscalar(rawValue)))
    error("AOCS:Config:InvalidField", ...
        "Config field %s must be a scalar string.", displayName);
end

value = string(rawValue);
if strlength(value) == 0
    error("AOCS:Config:InvalidField", ...
        "Config field %s must be non-empty.", displayName);
end
end
