function [F_srp_I_N, a_srp_I_m_s2] = computeSrpAcceleration(F_srp_B_N, C_BI, mass_kg)
% Description:
%   Converts body-frame SRP force into inertial force and acceleration.
%
% Arguments:
%   F_srp_B_N - 3-by-1 solar radiation pressure force in body axes [N].
%   C_BI - 3-by-3 DCM mapping inertial vectors into body axes.
%   mass_kg - Spacecraft mass [kg].
%
% Outputs:
%   F_srp_I_N - 3-by-1 solar radiation pressure force in inertial axes [N].
%   a_srp_I_m_s2 - 3-by-1 SRP acceleration in inertial axes [m/s^2].

%#codegen

F_srp_I_N = C_BI.' * F_srp_B_N(:);
a_srp_I_m_s2 = zeros(3, 1);

if mass_kg <= 0.0
    return;
end

a_srp_I_m_s2 = F_srp_I_N ./ mass_kg;
end
