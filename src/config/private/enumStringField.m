function value = enumStringField(parent, fieldName, displayName, allowedValues)
%ENUMSTRINGFIELD Read a required scalar string constrained to allowedValues.

value = stringScalarField(parent, fieldName, displayName);
allowedValues = string(allowedValues);

if ~any(value == allowedValues)
    error("AOCS:Config:InvalidField", ...
        "Unsupported config field %s '%s'. Expected one of: %s.", ...
        displayName, char(value), strjoin(allowedValues, ", "));
end
end
