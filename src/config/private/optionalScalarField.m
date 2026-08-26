function value = optionalScalarField(parent, fieldName, displayName, defaultValue, mustBePositive)
%OPTIONALSCALARFIELD Read an optional finite numeric scalar.

fieldName = char(fieldName);
if isstruct(parent) && isfield(parent, fieldName)
    value = scalarField(parent, fieldName, displayName, mustBePositive);
else
    value = defaultValue;
end
end
