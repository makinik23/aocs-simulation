function AttitudeInitializationConfigBus = createAttitudeInitializationConfigBus(targetWorkspace)
% Description:
%   Defines onboard reference-vector and TRIAD initialization settings.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   AttitudeInitializationConfigBus - Simulink.Bus object for onboard
%                                     attitude initialization settings.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("enabled", 1, "1", ...
    "Attitude-initialization enable flag");
elems(2) = busElement("magnetometer_max_age_s", 1, "s", ...
    "Maximum age of a magnetometer report accepted by TRIAD");
elems(3) = busElement("css_max_age_s", 1, "s", ...
    "Maximum age of a coarse-Sun-sensor report accepted by TRIAD");
elems(4) = busElement("sgp4_sample_time_s", 1, "s", ...
    "Execution period of the onboard SGP4 orbit propagator");
elems(5) = busElement("sgp4_bstar", 1, "1", ...
    "SGP4 B-star drag term; zero disables the fitted drag term");
elems(6) = busElement("minimum_vector_norm", 1, "1", ...
    "Numerical lower bound used while normalizing vectors");
elems(7) = busElement("minimum_triad_cross_norm", 1, "1", ...
    "Minimum sine of the angle between normalized TRIAD vectors");
elems(8) = busElement("epoch_utc", [6 1], "1", ...
    "Mission UTC epoch [year month day hour minute second]");
elems(9) = busElement("epoch_utc_jd", 1, "1", ...
    "UTC Julian date used as the onboard SGP4 epoch");
elems(10) = busElement("epoch_tdb_jd", 1, "1", ...
    "TDB Julian date at the mission epoch");
elems(11) = busElement("epoch_decimal_year", 1, "1", ...
    "Decimal UTC year at the mission epoch");
elems(12) = busElement("mu_m3_s2", 1, "m^3/s^2", ...
    "Central-body gravitational parameter carried by the time context");
elems(13) = busElement("sgp4_semi_major_axis_m", 1, "m", ...
    "SGP4 epoch semimajor axis");
elems(14) = busElement("sgp4_eccentricity", 1, "1", ...
    "SGP4 epoch eccentricity");
elems(15) = busElement("sgp4_inclination_deg", 1, "deg", ...
    "SGP4 epoch inclination");
elems(16) = busElement("sgp4_raan_deg", 1, "deg", ...
    "SGP4 epoch right ascension of the ascending node");
elems(17) = busElement("sgp4_argument_of_periapsis_deg", 1, "deg", ...
    "SGP4 epoch argument of periapsis");
elems(18) = busElement("sgp4_true_anomaly_deg", 1, "deg", ...
    "SGP4 epoch true anomaly used to initialize mean elements");
elems(19) = busElement("delta_at_s", 1, "s", ...
    "TAI minus UTC offset used by onboard frame transforms");
elems(20) = busElement("delta_ut1_s", 1, "s", ...
    "UT1 minus UTC offset used by onboard frame transforms");
elems(21) = busElement("polar_motion_rad", [1 2], "rad", ...
    "Earth polar motion used by onboard frame transforms");
elems(22) = busElement("d_cip_rad", [1 2], "rad", ...
    "Celestial intermediate pole correction used by onboard transforms");

AttitudeInitializationConfigBus = Simulink.Bus;
AttitudeInitializationConfigBus.Description = ...
    "Onboard reference-vector and TRIAD initialization configuration";
AttitudeInitializationConfigBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AttitudeInitializationConfigBus", ...
        AttitudeInitializationConfigBus);
end
end

function elem = busElement(name, dimensions, unit, description)
% Description:
%   Keeps bus element construction compact and consistent.
%
% Arguments:
%   name - Bus element name.
%   dimensions - Element dimensions.
%   unit - Physical unit string.
%   description - Human-readable element description.
%
% Outputs:
%   elem - Simulink.BusElement configured as a double.

elem = Simulink.BusElement;
elem.Name = name;
elem.Dimensions = dimensions;
elem.DataType = "double";
elem.Unit = unit;
elem.Description = description;
end
