function [rho_kg_m3, rho_raw_kg_m3, rho_uncertainty_1sigma_kg_m3, ...
    T_local_K, T_exo_K, n_O_m3, n_N2_m3, n_O2_m3, n_He_m3, ...
    n_H_m3, n_N_m3, v_atm_I_m_s] = postprocessNativeOutput( ...
    nativeOutput, r_I_m, environmentConfig)
% Description:
%   Converts the operational DTM2020 native output to the SI atmosphere bus
%   contract and applies the configured density calibration factor.

%#codegen

nativeOutput = nativeOutput(:);
r_I_m = r_I_m(:);

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

gPerCm3ToKgPerM3 = 1000.0;
atomicMassUnit_kg = 1.66053906660e-27;
scale = environmentConfig.rho_scale_factor;

rho_raw_kg_m3 = gPerCm3ToKgPerM3 * nativeOutput(1);
rho_kg_m3 = scale * rho_raw_kg_m3;
T_local_K = nativeOutput(3);
T_exo_K = nativeOutput(4);

speciesMassDensity_kg_m3 = scale * gPerCm3ToKgPerM3 * nativeOutput(5:10);
n_H_m3 = speciesMassDensity_kg_m3(1) / atomicMassUnit_kg;
n_He_m3 = speciesMassDensity_kg_m3(2) / (4.0 * atomicMassUnit_kg);
n_O_m3 = speciesMassDensity_kg_m3(3) / (16.0 * atomicMassUnit_kg);
n_N2_m3 = speciesMassDensity_kg_m3(4) / (28.0 * atomicMassUnit_kg);
n_O2_m3 = speciesMassDensity_kg_m3(5) / (32.0 * atomicMassUnit_kg);
n_N_m3 = speciesMassDensity_kg_m3(6) / (14.0 * atomicMassUnit_kg);

if environmentConfig.atmosphere_uncertainty_enabled > 0.5
    rho_uncertainty_1sigma_kg_m3 = nativeOutput(2) * rho_kg_m3 / 100.0;
end

omegaEarth_I_rad_s = [0.0; 0.0; 7.2921150e-5];
v_atm_I_m_s = cross(omegaEarth_I_rad_s, r_I_m);
end
