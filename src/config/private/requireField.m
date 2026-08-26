function value = requireField(parent, fieldName, displayName)
%REQUIREFIELD Read a required JSON field with a focused config error.

fieldName = char(fieldName);
if ~isstruct(parent) || ~isfield(parent, fieldName)
    error("AOCS:Config:MissingField", ...
        "Missing required config field: %s", displayName);
end

value = parent.(fieldName);
end
