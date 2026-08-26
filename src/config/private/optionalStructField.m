function section = optionalStructField(parent, fieldName)
%OPTIONALSTRUCTFIELD Read an optional scalar JSON object.

fieldName = char(fieldName);
if isstruct(parent) && isfield(parent, fieldName)
    section = parent.(fieldName);
    if ~isstruct(section) || ~isscalar(section)
        error("AOCS:Config:InvalidField", ...
            "Config field %s must be an object.", fieldName);
    end
else
    section = [];
end
end
