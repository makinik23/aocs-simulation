function value = matrixField(parent, fieldName, displayName, rows, cols)
%MATRIXFIELD Read a finite numeric JSON matrix with exact dimensions.

value = double(requireField(parent, fieldName, displayName));
validateattributes(value, {'numeric'}, {'real', 'finite', 'size', [rows cols]}, ...
    mfilename, displayName);
end
