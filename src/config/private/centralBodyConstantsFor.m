function constants = centralBodyConstantsFor(centralBody)
%CENTRALBODYCONSTANTSFOR Resolve central-body constants for config loading.

if centralBody ~= "Earth"
    error("AOCS:Config:UnsupportedCentralBody", ...
        "Unsupported orbit.central_body '%s'. Expected 'Earth'.", char(centralBody));
end

constants = struct();
constants.mu_m3_s2 = 3.986004418e14;
constants.radius_m = 6378137.0;
end
