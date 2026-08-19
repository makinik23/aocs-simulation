function [rho_raw_kg_m3, rho_uncertainty_fraction_1sigma, ...
    T_local_K, T_exo_K, n_H_raw_m3, n_He_raw_m3, n_O_raw_m3, ...
    n_N_raw_m3, n_N2_raw_m3, n_O2_raw_m3] = evaluateBackend(inputs)
% Description:
%   Defines the single replaceable backend boundary for DTM2020. Until the
%   official Fortran implementation is vendored and wrapped, this function
%   returns an explicitly temporary log-linear atmosphere.

%#codegen

altitude_m = inputs.altitude_km * 1000.0;
rho_raw_kg_m3 = fallbackNeutralDensity(altitude_m);
rho_uncertainty_fraction_1sigma = 0.3;
[T_local_K, T_exo_K] = fallbackNeutralTemperature(altitude_m);
[n_H_raw_m3, n_He_raw_m3, n_O_raw_m3, n_N_raw_m3, ...
    n_N2_raw_m3, n_O2_raw_m3] = fallbackSpeciesNumberDensity(rho_raw_kg_m3);
end

function rho_kg_m3 = fallbackNeutralDensity(altitude_m)
altitudeBreakpoints_m = [ ...
    0.0, 100e3, 150e3, 200e3, 250e3, 300e3, 350e3, 400e3, ...
    450e3, 500e3, 600e3, 700e3, 800e3, 900e3, 1000e3, 1200e3, 1500e3];
densityBreakpoints_kg_m3 = [ ...
    1.225, 5.6e-7, 2.0e-9, 2.8e-10, 7.0e-11, 2.4e-11, 9.5e-12, 3.9e-12, ...
    1.6e-12, 6.8e-13, 1.5e-13, 4.0e-14, 1.3e-14, 5.0e-15, 2.0e-15, ...
    5.0e-16, 1.0e-16];

altitude_m = min(max(altitude_m, altitudeBreakpoints_m(1)), altitudeBreakpoints_m(end));
segmentIndex = 1;
for k = 1:(numel(altitudeBreakpoints_m) - 1)
    if altitude_m <= altitudeBreakpoints_m(k + 1)
        segmentIndex = k;
        break;
    end
end

fraction = (altitude_m - altitudeBreakpoints_m(segmentIndex)) / ...
    (altitudeBreakpoints_m(segmentIndex + 1) - altitudeBreakpoints_m(segmentIndex));
logDensity = log(densityBreakpoints_kg_m3(segmentIndex)) + fraction * ...
    (log(densityBreakpoints_kg_m3(segmentIndex + 1)) - ...
    log(densityBreakpoints_kg_m3(segmentIndex)));
rho_kg_m3 = exp(logDensity);
end

function [T_local_K, T_exo_K] = fallbackNeutralTemperature(altitude_m)
T_exo_K = 1000.0;
T_local_K = 200.0 + 800.0 * (1.0 - exp(-altitude_m / 200e3));
T_local_K = min(max(T_local_K, 180.0), T_exo_K);
end

function [n_H_m3, n_He_m3, n_O_m3, n_N_m3, n_N2_m3, n_O2_m3] = ...
    fallbackSpeciesNumberDensity(rho_kg_m3)
amu_kg = 1.66053906660e-27;
n_H_m3 = 0.005 * rho_kg_m3 / amu_kg;
n_He_m3 = 0.020 * rho_kg_m3 / (4.0 * amu_kg);
n_O_m3 = 0.695 * rho_kg_m3 / (16.0 * amu_kg);
n_N_m3 = 0.0;
n_N2_m3 = 0.200 * rho_kg_m3 / (28.0 * amu_kg);
n_O2_m3 = 0.080 * rho_kg_m3 / (32.0 * amu_kg);
end
