function [rho_kg_m3, rho_raw_kg_m3, rho_uncertainty_1sigma_kg_m3, ...
    T_local_K, T_exo_K, n_O_m3, n_N2_m3, n_O2_m3, n_He_m3, ...
    n_H_m3, n_N_m3, v_atm_I_m_s] = computeAtmosphereProducts( ...
    r_I_m, v_I_m_s, lla, environmentConfig, environmentContext)
% Description:
%   Computes the atmosphere product contract and isolates the replaceable
%   DTM2020 backend from Simulink-facing units, scaling, and enable logic.
%
% Arguments:
%   r_I_m - Spacecraft inertial position vector [m].
%   v_I_m_s - Spacecraft inertial velocity vector [m/s].
%   lla - Geodetic latitude [deg], longitude [deg], altitude [m].
%   environmentConfig - Struct/bus matching AOCS_EnvironmentConfigBus.
%   environmentContext - Struct/bus matching AOCS_EnvironmentContextBus.

%#codegen

rho_kg_m3 = 0.0;
rho_raw_kg_m3 = 0.0;
rho_uncertainty_1sigma_kg_m3 = 0.0;
T_local_K = 0.0;
T_exo_K = 0.0;
n_O_m3 = 0.0;
n_N2_m3 = 0.0;
n_O2_m3 = 0.0;
n_He_m3 = 0.0;
n_H_m3 = 0.0;
n_N_m3 = 0.0;
v_atm_I_m_s = zeros(3, 1);

if environmentConfig.atmosphere_enabled <= 0.5
    return;
end

r_I_m = r_I_m(:);
v_I_m_s = v_I_m_s(:); %#ok<NASGU>
lla = lla(:);

omegaEarth_I_rad_s = [0.0; 0.0; 7.2921150e-5];
v_atm_I_m_s = cross(omegaEarth_I_rad_s, r_I_m);

inputs = dtm2020.prepareInputs(lla, environmentConfig, environmentContext);
[rho_raw_kg_m3, rhoUncertaintyFraction, T_local_K, T_exo_K, ...
    nHRaw_m3, nHeRaw_m3, nORaw_m3, nNRaw_m3, nN2Raw_m3, nO2Raw_m3] = ...
    dtm2020.evaluateBackend(inputs);

scale = environmentConfig.rho_scale_factor;
rho_kg_m3 = scale * rho_raw_kg_m3;
n_H_m3 = scale * nHRaw_m3;
n_He_m3 = scale * nHeRaw_m3;
n_O_m3 = scale * nORaw_m3;
n_N_m3 = scale * nNRaw_m3;
n_N2_m3 = scale * nN2Raw_m3;
n_O2_m3 = scale * nO2Raw_m3;

if environmentConfig.atmosphere_uncertainty_enabled > 0.5
    rho_uncertainty_1sigma_kg_m3 = rhoUncertaintyFraction * rho_kg_m3;
end
end
