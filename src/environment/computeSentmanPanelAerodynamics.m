function [F_aero_B_N, M_aero_B_Nm, F_aero_I_N, a_aero_I_m_s2, ...
    panel_forces_B_N, q_dyn_N_m2] = computeSentmanPanelAerodynamics( ...
    v_flow_B_m_s, rho_kg_m3, T_local_K, number_densities_m3, C_BI, ...
    mass_kg, aero_enabled, panel_normals_B, panel_areas_m2, ...
    panel_centers_B_m, wall_temperature_K, energy_accommodation)
% Description:
%   Computes force and moment on six flat panels in free-molecular flow.
%   The Sentman diffuse-reemission equations use DTM2020 number densities
%   for O, N2, O2, He, H, and N independently.

%#codegen

F_aero_B_N = zeros(3, 1);
M_aero_B_Nm = zeros(3, 1);
F_aero_I_N = zeros(3, 1);
a_aero_I_m_s2 = zeros(3, 1);
panel_forces_B_N = zeros(3, 6);
q_dyn_N_m2 = 0.0;

v_flow_B_m_s = v_flow_B_m_s(:);
number_densities_m3 = max(number_densities_m3(:), 0.0);
panel_areas_m2 = max(panel_areas_m2(:), 0.0);
wall_temperature_K = max(wall_temperature_K(:), 1.0);
energy_accommodation = min(max(energy_accommodation(:), 0.0), 1.0);

speed_m_s = norm(v_flow_B_m_s);
rho_kg_m3 = max(rho_kg_m3, 0.0);
T_local_K = max(T_local_K, 1.0);
if aero_enabled <= 0.5 || speed_m_s <= 1.0e-9 || ...
        rho_kg_m3 <= 0.0 || mass_kg <= 0.0
    return;
end

atomic_mass_unit_kg = 1.66053906660e-27;
boltzmann_J_K = 1.380649e-23;
species_mass_kg = atomic_mass_unit_kg .* [16.0; 28.0; 32.0; 4.0; 1.0; 14.0];
species_rho_kg_m3 = number_densities_m3 .* species_mass_kg;

% Preserve the total density returned by DTM2020 despite roundoff in the
% partial densities exposed by the reference Fortran implementation.
partial_rho_sum = sum(species_rho_kg_m3);
if partial_rho_sum <= 0.0
    species_rho_kg_m3(1) = rho_kg_m3;
else
    species_rho_kg_m3 = species_rho_kg_m3 .* (rho_kg_m3 / partial_rho_sum);
end

q_dyn_N_m2 = 0.5 * rho_kg_m3 * speed_m_s^2;
flow_direction_B = v_flow_B_m_s / speed_m_s;
sqrt_pi = sqrt(pi);

for panel_index = 1:6
    normal_B = panel_normals_B(:, panel_index);
    normal_norm = norm(normal_B);
    if normal_norm <= 0.0 || panel_areas_m2(panel_index) <= 0.0
        continue;
    end
    normal_B = normal_B / normal_norm;

    cos_delta = -dot(flow_direction_B, normal_B);
    cos_delta = min(max(cos_delta, -1.0), 1.0);
    sin_delta = sqrt(max(0.0, 1.0 - cos_delta^2));

    tangent_B = flow_direction_B + cos_delta * normal_B;
    tangent_norm = norm(tangent_B);
    if tangent_norm > 1.0e-12
        tangent_B = tangent_B / tangent_norm;
    else
        tangent_B = zeros(3, 1);
    end

    panel_force_B_N = zeros(3, 1);
    for species_index = 1:6
        species_rho = species_rho_kg_m3(species_index);
        if species_rho <= 0.0
            continue;
        end

        speed_ratio = speed_m_s * sqrt(species_mass_kg(species_index) / ...
            (2.0 * boltzmann_J_K * T_local_K));
        speed_ratio = max(speed_ratio, 1.0e-8);
        projected_ratio = speed_ratio * cos_delta;
        exp_term = exp(-projected_ratio^2);
        erf_term = 1.0 + erf(projected_ratio);

        alpha = energy_accommodation(panel_index);
        wall_ratio = 2.0 * wall_temperature_K(panel_index) / ...
            (T_local_K * speed_ratio^2);
        reflected_temperature_factor = 0.5 * sqrt(max(0.0, ...
            0.5 * (1.0 + alpha * (wall_ratio - 1.0))));

        c_pressure = (cos_delta^2 + 1.0 / (2.0 * speed_ratio^2)) * erf_term + ...
            cos_delta * exp_term / (sqrt_pi * speed_ratio) + ...
            reflected_temperature_factor * (sqrt_pi * cos_delta * erf_term + ...
            exp_term / speed_ratio);
        c_shear = sin_delta * cos_delta * erf_term + ...
            sin_delta * exp_term / (speed_ratio * sqrt_pi);

        species_q = 0.5 * species_rho * speed_m_s^2;
        panel_force_B_N = panel_force_B_N + panel_areas_m2(panel_index) * ...
            species_q * (c_shear * tangent_B - c_pressure * normal_B);
    end

    panel_forces_B_N(:, panel_index) = panel_force_B_N;
    F_aero_B_N = F_aero_B_N + panel_force_B_N;
    M_aero_B_Nm = M_aero_B_Nm + cross( ...
        panel_centers_B_m(:, panel_index), panel_force_B_N);
end

F_aero_I_N = C_BI.' * F_aero_B_N;
a_aero_I_m_s2 = F_aero_I_N / mass_kg;
end
