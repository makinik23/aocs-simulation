function value = optionalColumnField(parent, fieldName, displayName, rows, defaultValue)
%OPTIONALCOLUMNFIELD Read an optional fixed-size finite numeric vector.

fieldName = char(fieldName);
if isstruct(parent) && isfield(parent, fieldName)
    value = double(parent.(fieldName));
    value = value(:);
else
    value = defaultValue(:);
end

validateattributes(value, {'numeric'}, {'real', 'finite', 'size', [rows 1]}, ...
    mfilename, displayName);
end
