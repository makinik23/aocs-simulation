function value = logicalScalarField(parent, fieldName, displayName)
%LOGICALSCALARFIELD Read a required scalar JSON boolean.

rawValue = requireField(parent, fieldName, displayName);

if islogical(rawValue) && isscalar(rawValue)
    value = rawValue;
else
    error("AOCS:Config:InvalidField", ...
        "Config field %s must be a scalar boolean.", displayName);
end
end
