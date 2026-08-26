function section = requireStruct(parent, fieldName, displayName)
%REQUIRESTRUCT Read a required scalar JSON object section.

section = requireField(parent, fieldName, displayName);
if ~isstruct(section)
    error("AOCS:Config:InvalidField", ...
        "Config field %s must be an object.", displayName);
end
end
