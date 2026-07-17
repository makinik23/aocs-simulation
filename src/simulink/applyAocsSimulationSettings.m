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
applyPlanetaryEphemerisSettings(modelName, AOCS);
applyEciToLlaSettings(modelName, AOCS);
applyEciToEcefDcmSettings(modelName, AOCS);
applyIgrfSettings(modelName);
applyEclipseShadowModelSettings(modelName, AOCS);
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

function applyPlanetaryEphemerisSettings(modelName, AOCS)
% Description:
%   Finds Planetary Ephemeris blocks and configures the project Sun vector
%   source from environment.sun.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%   AOCS - Validated configuration struct.
%
% Outputs:
%   None.

sun = AOCS.Environment.Sun;
if sun.Model ~= "planet_ephemeris"
    error("AOCS:Simulink:UnsupportedSunModel", ...
        "Unsupported Sun model '%s'.", char(sun.Model));
end

blocks = findPlanetaryEphemerisBlocks(modelName);
if isempty(blocks)
    error("AOCS:Simulink:MissingPlanetaryEphemeris", ...
        "environment.sun.model is 'planet_ephemeris', but no Planetary Ephemeris block was found in model '%s'.", ...
        char(modelName));
end

for k = 1:numel(blocks)
    set_param(blocks{k}, ...
        "units", "m,m/s", ...
        "epochFormat", "Julian date", ...
        "ephemerisModel", char(sun.EphemerisModel), ...
        "center", "Earth", ...
        "target", "Sun", ...
        "useDateRange", onOff(sun.UseEphemerisDateRange), ...
        "startDate", julianDateExpression(sun.EphemerisStartUtc), ...
        "endDate", julianDateExpression(sun.EphemerisEndUtc), ...
        "action", char(sun.Action), ...
        "outputVelocity", "off");
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

function applyEclipseShadowModelSettings(modelName, AOCS)
% Description:
%   Finds Aerospace Blockset Eclipse Shadow Model blocks and applies the
%   project eclipse settings from environment.eclipse.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%   AOCS - Validated configuration struct.
%
% Outputs:
%   None.

eclipse = AOCS.Environment.Eclipse;
blocks = findEclipseShadowModelBlocks(modelName);
if isempty(blocks)
    if ~eclipse.Enabled
        return;
    end

    error("AOCS:Simulink:MissingEclipseShadowModel", ...
        "environment.eclipse.enabled is true, but no Eclipse Shadow Model block was found in model '%s'.", ...
        char(modelName));
end

for k = 1:numel(blocks)
    set_param(blocks{k}, ...
        "units", "Metric (m)", ...
        "shadowModel", eclipseShadowModelMaskValue(eclipse.Model), ...
        "outputShadowRegion", onOff(eclipse.OutputShadowRegion), ...
        "timeSource", eclipseTimeSourceMaskValue(eclipse.TimeSource), ...
        "startDate", julianDateExpression(AOCS.Epoch.Utc), ...
        "centralBody", char(eclipse.CentralBody), ...
        "includeMoon", onOff(eclipse.IncludeMoon), ...
        "includeEarth", onOff(eclipse.IncludeEarth), ...
        "customRadius", numericString(AOCS.Orbit.CentralBodyConstants.radius_m), ...
        "ephemerisModel", char(eclipse.EphemerisModel), ...
        "useEphemerisDateRange", onOff(eclipse.UseEphemerisDateRange), ...
        "ephemerisStartDate", julianDateExpression(eclipse.EphemerisStartUtc), ...
        "ephemerisEndDate", julianDateExpression(eclipse.EphemerisEndUtc), ...
        "action", char(eclipse.Action), ...
        "zeroCrossing", onOff(eclipse.ZeroCrossing));
end
end

function blocks = findEclipseShadowModelBlocks(modelName)
% Description:
%   Finds Eclipse Shadow Model blocks robustly, including built-in block
%   types that are not masked subsystems.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%
% Outputs:
%   blocks - Cell array of matching block paths.

candidates = find_system(modelName, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "Type", "Block");

blocks = {};
for k = 1:numel(candidates)
    try
        if string(get_param(candidates{k}, "BlockType")) == "EclipseShadowModel"
            blocks{end + 1} = candidates{k}; %#ok<AGROW>
        end
    catch
    end
end
end

function blocks = findPlanetaryEphemerisBlocks(modelName)
% Description:
%   Finds Aerospace Blockset Planetary Ephemeris blocks by block type.
%
% Arguments:
%   modelName - Loaded Simulink model name.
%
% Outputs:
%   blocks - Cell array of matching block paths.

candidates = find_system(modelName, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "Type", "Block");

blocks = {};
for k = 1:numel(candidates)
    try
        if string(get_param(candidates{k}, "BlockType")) == "PlanetaryEphem"
            blocks{end + 1} = candidates{k}; %#ok<AGROW>
        end
    catch
    end
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

function value = onOff(flag)
% Description:
%   Converts a logical flag to the on/off strings expected by Simulink masks.
%
% Arguments:
%   flag - Scalar logical.
%
% Outputs:
%   value - 'on' when true, otherwise 'off'.

if flag
    value = "on";
else
    value = "off";
end
end

function value = eclipseShadowModelMaskValue(model)
% Description:
%   Maps project eclipse model names to Aerospace Blockset mask values.
%
% Arguments:
%   model - Project-level eclipse model string.
%
% Outputs:
%   value - Mask value accepted by Eclipse Shadow Model.

switch string(model)
    case "dual_cone"
        value = "Dual cone";
    otherwise
        error("AOCS:Simulink:UnsupportedEclipseModel", ...
            "Unsupported eclipse model '%s'.", char(model));
end
end

function value = eclipseTimeSourceMaskValue(timeSource)
% Description:
%   Maps project time-source names to Aerospace Blockset mask values.
%
% Arguments:
%   timeSource - Project-level eclipse time source string.
%
% Outputs:
%   value - Mask value accepted by Eclipse Shadow Model.

switch string(timeSource)
    case "dialog"
        value = "Dialog";
    otherwise
        error("AOCS:Simulink:UnsupportedEclipseTimeSource", ...
            "Unsupported eclipse time source '%s'.", char(timeSource));
end
end

function value = julianDateExpression(epochUtc)
% Description:
%   Formats a UTC vector as a Simulink mask expression evaluated to Julian
%   date by Aerospace Blockset blocks.
%
% Arguments:
%   epochUtc - 6-by-1 UTC vector [year month day hour minute second]'.
%
% Outputs:
%   value - Character vector expression, e.g. juliandate(2026, 1, 1, 0, 0, 0).

value = sprintf("juliandate(%s, %s, %s, %s, %s, %s)", ...
    integerString(epochUtc(1)), ...
    integerString(epochUtc(2)), ...
    integerString(epochUtc(3)), ...
    integerString(epochUtc(4)), ...
    integerString(epochUtc(5)), ...
    numericString(epochUtc(6)));
end
