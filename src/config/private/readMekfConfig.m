function config = readMekfConfig(mekf)
% Description:
%   Validates MEKF priors and filter tuning independently of sensor truth.
%   Converts human-readable standard deviations to SI units.
%
% Arguments:
%   mekf - JSON object from gnc.mekf.
%
% Outputs:
%   config - Initialization priors and independent filter tuning in SI units.

config.InitialBias_B_rad_s = readVector(mekf, "initial_bias_B_rad_s", false);
config.InitialAttitudeStd_rad = deg2rad( ...
    readVector(mekf, "initial_attitude_std_deg", true));
config.InitialBiasStd_rad_s = deg2rad( ...
    readVector(mekf, "initial_bias_std_deg_s", true));
names = ["gyro_noise_density_rad_s_sqrt_Hz", ...
    "bias_random_walk_std_rad_s_sqrt_s", "magnetometer_direction_std_deg", ...
    "css_direction_std_deg", "innovation_gate_squared", ...
    "maximum_correction_deg", "time_alignment_tolerance_s", "reference_max_age_s"];
for name = names
    outputName = name;
    value = requireField(mekf, name, "gnc.mekf." + name);
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ...
            ~isfinite(value) || value <= 0
        error("AOCS:Config:InvalidField", ...
            "gnc.mekf.%s must be a finite positive scalar.", name);
    end
    if endsWith(name, "_deg")
        if value >= 90
            error("AOCS:Config:InvalidField", ...
                "gnc.mekf.%s must be below 90 degrees for a local-error filter.", name);
        end
        outputName = replace(name, "_deg", "_rad");
        value = deg2rad(value);
    end
    config.Tuning.(outputName) = double(value);
end
end

function value = readVector(parent, name, positive)
% Description:
%   Reads a finite three-axis vector and optionally enforces positivity.
%
% Arguments:
%   parent - Configuration object containing the field.
%   name - Required field name.
%   positive - True for strictly positive prior standard deviations.
%
% Outputs:
%   value - Three-element double column vector.

label = "gnc.mekf." + name;
value = requireField(parent, name, label);
if ~isnumeric(value) || ~isreal(value) || ~isvector(value) || ...
        numel(value) ~= 3 || any(~isfinite(value(:)))
    error("AOCS:Config:InvalidField", ...
        "%s must be a finite numeric three-element vector.", label);
end
value = double(value(:));
if positive && any(value <= 0)
    error("AOCS:Config:InvalidField", ...
        "%s entries must be strictly positive.", label);
end
end
