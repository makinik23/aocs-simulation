function value = columnField(parent, fieldName, displayName, rows)
%COLUMNFIELD Read a finite numeric JSON vector as a fixed-size column.

value = double(requireField(parent, fieldName, displayName));
value = value(:);
validateattributes(value, {'numeric'}, {'real', 'finite', 'size', [rows 1]}, ...
    mfilename, displayName);
end
