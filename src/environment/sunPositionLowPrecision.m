function [r_sun_I_m, sun_distance_m, solar_flux_W_m2] = sunPositionLowPrecision(epoch_utc, t_s, solar_constant_W_m2)
% Description:
%   Computes an approximate geocentric Earth-to-Sun vector in the project
%   inertial frame from a UTC epoch plus simulation elapsed seconds.
%
%   The model is the common low-precision solar ephemeris expressed in
%   mean-equator/equinox J2000-like axes. It is suitable for engineering-level
%   Sun vector, eclipse, and SRP modeling, and avoids a dependency on external
%   JPL ephemeris data packages.
%
% Arguments:
%   epoch_utc - 6-by-1 UTC epoch [year month day hour minute second]'.
%   t_s - Elapsed simulation time from the epoch [s].
%   solar_constant_W_m2 - Solar irradiance at 1 AU [W/m^2].
%
% Outputs:
%   r_sun_I_m - Approximate Earth-to-Sun position vector in inertial axes [m].
%   sun_distance_m - Earth-to-Sun distance [m].
%   solar_flux_W_m2 - Solar irradiance scaled by instantaneous Sun distance.

%#codegen

AU_m = 149597870700.0;
deg2rad = pi / 180.0;

epoch = double(epoch_utc);
jd = calendarUtcToJulianDate(epoch(1), epoch(2), epoch(3), ...
    epoch(4), epoch(5), epoch(6)) + double(t_s) / 86400.0;

n_days = jd - 2451545.0;
mean_longitude_deg = mod(280.460 + 0.9856474 * n_days, 360.0);
mean_anomaly_deg = mod(357.528 + 0.9856003 * n_days, 360.0);

mean_anomaly_rad = mean_anomaly_deg * deg2rad;
ecliptic_longitude_rad = (mean_longitude_deg ...
    + 1.915 * sin(mean_anomaly_rad) ...
    + 0.020 * sin(2.0 * mean_anomaly_rad)) * deg2rad;
obliquity_rad = (23.439 - 0.0000004 * n_days) * deg2rad;

sun_distance_AU = 1.00014 ...
    - 0.01671 * cos(mean_anomaly_rad) ...
    - 0.00014 * cos(2.0 * mean_anomaly_rad);

r_sun_I_m = AU_m * sun_distance_AU * [
    cos(ecliptic_longitude_rad);
    cos(obliquity_rad) * sin(ecliptic_longitude_rad);
    sin(obliquity_rad) * sin(ecliptic_longitude_rad)];

sun_distance_m = norm(r_sun_I_m);
solar_flux_W_m2 = solar_constant_W_m2 * (AU_m / sun_distance_m)^2;
end

function jd = calendarUtcToJulianDate(year, month, day, hour, minute, second)
% Description:
%   Converts a Gregorian UTC calendar date to Julian date using arithmetic that
%   is friendly to MATLAB Function blocks and code generation.

y = floor(double(year));
m = floor(double(month));
d = floor(double(day));

if m <= 2
    y = y - 1;
    m = m + 12;
end

a = floor(y / 100.0);
b = 2.0 - a + floor(a / 4.0);

day_fraction = (double(hour) + (double(minute) + double(second) / 60.0) / 60.0) / 24.0;

jd = floor(365.25 * (y + 4716.0)) ...
    + floor(30.6001 * (m + 1.0)) ...
    + d + day_fraction + b - 1524.5;
end
