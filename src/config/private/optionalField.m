function value = optionalField(parent, fieldName, defaultValue)
%OPTIONALFIELD Read a JSON field or return defaultValue when absent.

fieldName = char(fieldName);
if isstruct(parent) && isfield(parent, fieldName)
    value = parent.(fieldName);
else
    value = defaultValue;
end
end
