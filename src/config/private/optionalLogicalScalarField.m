function value = optionalLogicalScalarField(parent, fieldName, displayName, defaultValue)
%OPTIONALLOGICALSCALARFIELD Read an optional scalar JSON boolean.

fieldName = char(fieldName);
if isstruct(parent) && isfield(parent, fieldName)
    value = logicalScalarField(parent, fieldName, displayName);
else
    value = defaultValue;
end
end
