function applyAocsSimulationSettings(modelName, AOCS)
% Description:
%   Applies solver timing/tolerance settings and points Aerospace Blockset
%   block mask parameters at values exposed by the validated AOCS config.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%   AOCS - Validated configuration struct.
%
% Outputs:
%   None.

set_param(modelName, ...
    "StartTime", num2str(AOCS.Sim.StartTime_s), ...
    "StopTime", num2str(AOCS.Sim.StopTime_s), ...
    "Solver", char(AOCS.Sim.Solver), ...
    "RelTol", num2str(AOCS.Sim.RelTol), ...
    "AbsTol", num2str(AOCS.Sim.AbsTol));

applyAerospace6DofSettings(modelName);
applyOrbitPropagatorSettings(modelName);
applyEciToLlaSettings(modelName, AOCS);
applyEciToEcefDcmSettings(modelName, AOCS);
applyIgrfSettings(modelName);
end

function applyAerospace6DofSettings(modelName)
% Description:
%   Finds every EOM6DOFBodyQuat block and binds inertia, initial Euler
%   attitude, and initial body rates to AOCS_Config fields.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%
% Outputs:
%   None.

blocks = find_system(modelName, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "BlockType", "EOM6DOFBodyQuat");

for k = 1:numel(blocks)
    set_param(blocks{k}, ...
        "eul_0", "AOCS_Config.euler_BI_0_rad", ...
        "pm_0", "AOCS_Config.omega_BI_B_0", ...
        "inertia", "AOCS_Config.I_B");
end
end

function applyOrbitPropagatorSettings(modelName)
% Description:
%   Finds Orbit Propagator blocks and binds the Keplerian initial orbit to
%   AOCS_OrbitConfig fields.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%
% Outputs:
%   None.

blocks = find_system(modelName, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "BlockType", "OrbitPropagator");

for k = 1:numel(blocks)
    set_param(blocks{k}, ...
        "propagator", "Kepler (unperturbed)", ...
        "timeFormat", "Gregorian date", ...
        "startDate", "AOCS_OrbitConfig.epoch_utc", ...
        "dateOut", "on", ...
        "stateFormatKep", "Orbital elements", ...
        "orbitType", "Keplerian", ...
        "centralBody", "Earth", ...
        "units", "Metric (m/s)", ...
        "angleUnits", "Radians", ...
        "semiMajorAxis", "AOCS_OrbitConfig.semi_major_axis_m", ...
        "eccentricity", "AOCS_OrbitConfig.eccentricity", ...
        "inclination", "AOCS_OrbitConfig.inclination_rad", ...
        "raan", "AOCS_OrbitConfig.raan_rad", ...
        "argPeriapsis", "AOCS_OrbitConfig.argument_of_periapsis_rad", ...
        "trueAnomaly", "AOCS_OrbitConfig.true_anomaly_rad");
end
end

function applyEciToLlaSettings(modelName, AOCS)
% Description:
%   Finds ECI Position to LLA blocks and applies the JSON epoch as mask
%   date plus a seconds time-increment port.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%   AOCS - Validated configuration struct.
%
% Outputs:
%   None.

blocks = find_system(modelName, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "MaskType", "ECItoLLA");

epoch = AOCS.Epoch.Utc;
monthName = monthNumberToName(epoch(2));

for k = 1:numel(blocks)
    set_param(blocks{k}, ...
        "red", "IAU-2000/2006", ...
        "year", integerString(epoch(1)), ...
        "month", monthName, ...
        "day", integerString(epoch(3)), ...
        "hour", integerString(epoch(4)), ...
        "min", integerString(epoch(5)), ...
        "sec", numericString(epoch(6)), ...
        "deltaT", "Sec", ...
        "errorflag", "Error", ...
        "extraparamflag", "off", ...
        "eunits", "Metric (MKS)", ...
        "earthmodel", "WGS84");
end
end

function applyEciToEcefDcmSettings(modelName, AOCS)
% Description:
%   Finds standalone Direction Cosine Matrix ECI to ECEF blocks and applies
%   the JSON epoch as mask date plus a seconds time-increment port.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%   AOCS - Validated configuration struct.
%
% Outputs:
%   None.

blocks = find_system(modelName, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "MaskType", "DCMECItoECEF");

epoch = AOCS.Epoch.Utc;
monthName = monthNumberToName(epoch(2));

for k = 1:numel(blocks)
    if isWithinMaskedSubsystem(blocks{k}, "ECItoLLA")
        continue;
    end

    set_param(blocks{k}, ...
        "red", "IAU-2000/2006", ...
        "year", integerString(epoch(1)), ...
        "month", monthName, ...
        "day", integerString(epoch(3)), ...
        "hour", integerString(epoch(4)), ...
        "min", integerString(epoch(5)), ...
        "sec", numericString(epoch(6)), ...
        "deltaT", "Sec", ...
        "errorflag", "Error", ...
        "extraparamflag", "off");
end
end

function applyIgrfSettings(modelName)
% Description:
%   Finds IGRF blocks and configures the modern IGRF generation with a
%   decimal-year input port and no secular-variation output ports.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%
% Outputs:
%   None.

blocks = find_system(modelName, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "BlockType", "IGRF");

for k = 1:numel(blocks)
    set_param(blocks{k}, ...
        "generation", "IGRF-14", ...
        "units", "Metric (MKS)", ...
        "time_in", "on", ...
        "action", "Error", ...
        "sv_out", "off");
end
end

function tf = isWithinMaskedSubsystem(block, maskType)
% Description:
%   Checks whether a block is nested inside a masked subsystem of a given
%   type. This avoids setting implementation details inside library-linked
%   masked blocks that already expose their own top-level parameters.
%
% Arguments:
%   block - Block path to test.
%   maskType - Ancestor mask type to look for.
%
% Outputs:
%   tf - True when any parent subsystem has the requested mask type.

tf = false;
parent = get_param(block, "Parent");

while strlength(string(parent)) > 0
    try
        if string(get_param(parent, "MaskType")) == string(maskType)
            tf = true;
            return;
        end
        parent = get_param(parent, "Parent");
    catch
        return;
    end
end
end

function monthName = monthNumberToName(monthNumber)
% Description:
%   Converts a numeric UTC month to the enum string expected by Aerospace
%   Blockset time/date masks.
%
% Arguments:
%   monthNumber - Integer month number in the range 1..12.
%
% Outputs:
%   monthName - Character vector month name.

monthNames = ["January", "February", "March", "April", "May", "June", ...
    "July", "August", "September", "October", "November", "December"];
monthName = char(monthNames(monthNumber));
end

function value = integerString(value)
% Description:
%   Formats an integer-valued scalar for Simulink mask parameters.
%
% Arguments:
%   value - Numeric scalar.
%
% Outputs:
%   value - Character vector without decimal places.

value = sprintf("%.0f", value);
end

function value = numericString(value)
% Description:
%   Formats a finite scalar for Simulink mask parameters.
%
% Arguments:
%   value - Numeric scalar.
%
% Outputs:
%   value - Character vector preserving useful precision.

value = sprintf("%.15g", value);
end
