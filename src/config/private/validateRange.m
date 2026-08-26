function validateRange(value, minimumValue, maximumValue, displayName)
%VALIDATERANGE Validate a finite scalar against an inclusive numeric range.

if value < minimumValue || value > maximumValue
    error("AOCS:Config:InvalidField", ...
        "Config field %s must be in [%g, %g].", ...
        displayName, minimumValue, maximumValue);
end
end
