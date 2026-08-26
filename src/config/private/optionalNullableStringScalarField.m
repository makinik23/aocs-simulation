function value = optionalNullableStringScalarField(parent, fieldName, displayName, defaultValue)
%OPTIONALNULLABLESTRINGSCALARFIELD Read an optional string, allowing empty.

fieldName = char(fieldName);
if isstruct(parent) && isfield(parent, fieldName)
    rawValue = parent.(fieldName);
    if ~(ischar(rawValue) || (isstring(rawValue) && isscalar(rawValue)))
        error("AOCS:Config:InvalidField", ...
            "Config field %s must be a scalar string.", displayName);
    end
    value = string(rawValue);
else
    value = string(defaultValue);
end
end
