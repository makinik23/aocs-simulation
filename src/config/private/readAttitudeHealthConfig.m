function config = readAttitudeHealthConfig(health)
% Description:
%   Validates estimator-health thresholds and converts angles to SI units.
%
% Arguments:
%   health - JSON object from gnc.attitude_health.
%
% Outputs:
%   config - Positive ordered thresholds for monitoring the attitude estimate.

pairs = ["attitude_std_deg", "state_age_s", "correction_age_s"];
config = struct();
for pair = pairs
    degradedName = "degraded_" + pair;
    lostName = "lost_" + pair;
    degraded = positiveScalar(health, degradedName);
    lost = positiveScalar(health, lostName);
    if lost <= degraded
        error("AOCS:Config:InvalidField", ...
            "gnc.attitude_health.%s must exceed %s.", lostName, degradedName);
    end
    if pair == "attitude_std_deg"
        config.DegradedAttitudeStd_rad = deg2rad(degraded);
        config.LostAttitudeStd_rad = deg2rad(lost);
    elseif pair == "state_age_s"
        config.DegradedStateAge_s = degraded;
        config.LostStateAge_s = lost;
    else
        config.DegradedCorrectionAge_s = degraded;
        config.LostCorrectionAge_s = lost;
    end
end
end

function value = positiveScalar(parent, name)
% Description:
%   Requires one finite positive numeric health threshold.
%
% Arguments:
%   parent - Health configuration object.
%   name - Field to read.
%
% Outputs:
%   value - Validated double scalar.

value = requireField(parent, name, "gnc.attitude_health." + name);
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ...
        ~isfinite(value) || value <= 0
    error("AOCS:Config:InvalidField", ...
        "gnc.attitude_health.%s must be a finite positive scalar.", name);
end
value = double(value);
end
