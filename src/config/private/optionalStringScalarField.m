function value = optionalStringScalarField(parent, fieldName, displayName, defaultValue)
%OPTIONALSTRINGSCALARFIELD Read an optional non-empty scalar JSON string.

fieldName = char(fieldName);
if isstruct(parent) && isfield(parent, fieldName)
    value = stringScalarField(parent, fieldName, displayName);
else
    value = string(defaultValue);
end
end
