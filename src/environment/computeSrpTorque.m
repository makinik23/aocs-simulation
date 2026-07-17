function [M_srp_B_Nm, F_srp_B_N, P_srp_N_m2] = computeSrpTorque( ...
    sun_B_unit, solar_flux_shadowed_W_m2, srp_enabled, ...
    srp_area_ref_m2, srp_coefficient_reflectivity, srp_center_of_pressure_B_m)
% Description:
%   Computes flat-plate constant-area solar radiation pressure force and
%   torque in spacecraft body axes.
%
% Arguments:
%   sun_B_unit - Unit vector from spacecraft to Sun expressed in body axes.
%   solar_flux_shadowed_W_m2 - Eclipse-shadowed solar irradiance [W/m^2].
%   srp_enabled - Numeric enable flag. Values <= 0.5 disable SRP.
%   srp_area_ref_m2 - Reference illuminated area [m^2].
%   srp_coefficient_reflectivity - SRP reflectivity coefficient [-].
%   srp_center_of_pressure_B_m - Center-of-pressure offset from CoM in body axes [m].
%
% Outputs:
%   M_srp_B_Nm - SRP torque expressed in body axes [N*m].
%   F_srp_B_N - SRP force expressed in body axes [N].
%   P_srp_N_m2 - Solar radiation pressure [N/m^2].

%#codegen

c_m_s = 299792458.0;

M_srp_B_Nm = zeros(3, 1);
F_srp_B_N = zeros(3, 1);
P_srp_N_m2 = 0.0;

if srp_enabled <= 0.5
    return;
end

solar_flux_shadowed_W_m2 = max(solar_flux_shadowed_W_m2, 0.0);
srp_area_ref_m2 = max(srp_area_ref_m2, 0.0);
srp_coefficient_reflectivity = max(srp_coefficient_reflectivity, 0.0);

sunNorm = norm(sun_B_unit);
if sunNorm <= eps
    return;
end

sunDirection_B = sun_B_unit ./ sunNorm;
P_srp_N_m2 = solar_flux_shadowed_W_m2 ./ c_m_s;
F_srp_mag_N = P_srp_N_m2 .* srp_coefficient_reflectivity .* srp_area_ref_m2;

% sun_B_unit points from spacecraft to Sun. Radiation pressure force points
% from Sun to spacecraft, so the force direction is opposite to sun_B_unit.
F_srp_B_N = -F_srp_mag_N .* sunDirection_B;
M_srp_B_Nm = cross(srp_center_of_pressure_B_m, F_srp_B_N);
end
