function value = optionalEnumStringField(parent, fieldName, displayName, allowedValues, defaultValue)
%OPTIONALENUMSTRINGFIELD Read an optional constrained scalar string.

fieldName = char(fieldName);
if isstruct(parent) && isfield(parent, fieldName)
    value = enumStringField(parent, fieldName, displayName, allowedValues);
else
    value = string(defaultValue);
end
end
