function [sun_B_unit, sun_I_unit, r_sun_I_m, sun_distance_m, solar_flux_W_m2] = computeSunProducts( ...
    r_sun_I_m, solar_constant_W_m2, r_I_m, DCM_be)
% Description:
%   Computes spacecraft-relative Sun products from an ephemeris-supplied
%   Earth-to-Sun vector.
%
% Arguments:
%   r_sun_I_m - Earth-to-Sun position vector in ICRF axes [m].
%   solar_constant_W_m2 - Solar irradiance at 1 AU [W/m^2].
%   r_I_m - Spacecraft inertial position vector [m].
%   DCM_be - Direction cosine matrix mapping inertial vector components to
%            body-frame vector components.
%
% Outputs:
%   sun_B_unit - Unit vector from spacecraft to Sun expressed in body axes.
%   sun_I_unit - Unit vector from spacecraft to Sun expressed in inertial axes.
%   r_sun_I_m - Earth-to-Sun position vector in ICRF axes [m].
%   sun_distance_m - Spacecraft-to-Sun distance [m].
%   solar_flux_W_m2 - Solar irradiance at spacecraft distance [W/m^2].

%#codegen

AU_m = 149597870700.0;

r_sun_I_m = r_sun_I_m(:);
r_I_m = r_I_m(:);

r_sat_to_sun_I_m = r_sun_I_m - r_I_m;
sun_distance_m = norm(r_sat_to_sun_I_m);

if sun_distance_m > 0
    sun_I_unit = r_sat_to_sun_I_m / sun_distance_m;
else
    sun_I_unit = [1.0; 0.0; 0.0];
end

sun_B_unit = DCM_be * sun_I_unit;
sun_B_norm = norm(sun_B_unit);
if sun_B_norm > 0
    sun_B_unit = sun_B_unit / sun_B_norm;
end

if sun_distance_m > 0
    solar_flux_W_m2 = solar_constant_W_m2 * (AU_m / sun_distance_m)^2;
else
    solar_flux_W_m2 = 0.0;
end
end
