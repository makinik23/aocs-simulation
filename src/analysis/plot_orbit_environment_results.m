function figures = plot_orbit_environment_results(resultsFile, exportDirectory)
% Description:
%   Loads the latest simulation result and plots orbit and environment
%   products: 3D inertial orbit, kinematics, ground track, magnetic field, and
%   disturbance torques.
%
% Arguments:
%   resultsFile - Optional MAT-file containing simulation output variable out.
%   exportDirectory - Optional directory. When supplied, PNG copies are saved.
%
% Outputs:
%   figures - Handles to the generated MATLAB figures.

projectRoot = setupAocsPaths();

AOCS = loadAocsSimulationConfig(fullfile(projectRoot, "config", "AocsSimulationConfig.json"), projectRoot);

if nargin < 1 || strlength(string(resultsFile)) == 0
    resultsFile = AOCS.Results.File;
end

if nargin < 2
    exportDirectory = "";
end

loaded = load(resultsFile);
out = loaded.out;
if isfield(loaded, "AOCS")
    AOCS = loaded.AOCS;
end

logsout = out.logsout;
t_s = out.tout(:);
t_min = t_s ./ 60;

r_I_m = loggedVector(logsout, "r_I_m", 3);
v_I_m_s = loggedVector(logsout, "v_I_m_s", 3);
B_NED_T = loggedVector(logsout, "B_NED_T", 3);
B_I_T = loggedVector(logsout, "B_I_T", 3);
B_B_T = loggedVector(logsout, "B_B_T", 3);
zeroTorque_Nm = zeros(numel(t_s), 3);
zeroForce_N = zeros(numel(t_s), 3);
zeroAcceleration_m_s2 = zeros(numel(t_s), 3);
M_rmm_B_Nm = loggedVector(logsout, "M_rmm_B_Nm", 3, zeroTorque_Nm);
M_gg_B_Nm = loggedVector(logsout, "M_gg_B_Nm", 3, zeroTorque_Nm);
M_srp_B_Nm = loggedVector(logsout, "M_srp_B_Nm", 3, zeroTorque_Nm);
M_dist_B_Nm = loggedVector(logsout, "M_dist_B_Nm", 3, zeroTorque_Nm);
F_srp_B_N = loggedVector(logsout, "F_srp_B_N", 3, zeroForce_N);
F_srp_I_N = loggedVector(logsout, "F_srp_I_N", 3, zeroForce_N);
a_srp_I_m_s2 = loggedVector(logsout, "a_srp_I_m_s2", 3, zeroAcceleration_m_s2);
a_aero_I_m_s2 = loggedVector(logsout, "a_aero_I_m_s2", 3, zeroAcceleration_m_s2);
a_dist_I_m_s2 = loggedVector(logsout, "a_dist_I_m_s2", 3, zeroAcceleration_m_s2);

orbit = computeOrbitDiagnostics(r_I_m, v_I_m_s, AOCS);
lla = computeLlaProducts(r_I_m, t_s, AOCS);
summary = computeDisturbanceSummary(B_NED_T, B_I_T, B_B_T, ...
    M_rmm_B_Nm, M_gg_B_Nm, M_srp_B_Nm, M_dist_B_Nm, ...
    F_srp_B_N, F_srp_I_N, a_srp_I_m_s2, a_aero_I_m_s2, a_dist_I_m_s2);

figures = gobjects(4, 1);
figures(1) = plotInertialOrbit3d(r_I_m, t_min, orbit, AOCS);
figures(2) = plotOrbitKinematics(t_min, orbit);
figures(3) = plotGroundTrackAndLla(t_min, lla, orbit);
figures(4) = plotEnvironmentProducts(t_min, B_NED_T, B_I_T, B_B_T, ...
    M_rmm_B_Nm, M_gg_B_Nm, M_srp_B_Nm, M_dist_B_Nm, ...
    F_srp_B_N, F_srp_I_N, a_srp_I_m_s2, a_aero_I_m_s2, a_dist_I_m_s2);

printSummary(orbit, summary);
exportFigures(figures, exportDirectory);
end

function fig = plotInertialOrbit3d(r_I_m, t_min, orbit, AOCS)
% Description:
%   Plots the inertial orbit around a reference Earth sphere.

earthRadius_km = AOCS.Orbit.CentralBodyConstants.radius_m / 1000;
r_I_km = r_I_m ./ 1000;

fig = figure("Name", "Orbit - 3D ICRF view", "Color", "w");
hold on

[earthX, earthY, earthZ] = sphere(96);
surf(earthRadius_km .* earthX, earthRadius_km .* earthY, earthRadius_km .* earthZ, ...
    "EdgeColor", "none", "FaceColor", [0.25 0.45 0.75], "FaceAlpha", 0.22);

plot3(r_I_km(:, 1), r_I_km(:, 2), r_I_km(:, 3), ...
    "Color", [0.10 0.10 0.10], "LineWidth", 0.8);
scatter3(r_I_km(:, 1), r_I_km(:, 2), r_I_km(:, 3), 12, t_min, "filled");
scatter3(r_I_km(1, 1), r_I_km(1, 2), r_I_km(1, 3), 70, ...
    [0.10 0.55 0.20], "filled", "MarkerEdgeColor", "k");
scatter3(r_I_km(end, 1), r_I_km(end, 2), r_I_km(end, 3), 70, ...
    [0.85 0.20 0.15], "filled", "MarkerEdgeColor", "k");

axis equal
grid on
box on
view(36, 24)
xlabel("X_I [km]")
ylabel("Y_I [km]")
zlabel("Z_I [km]")
title("Inertial orbit view")
subtitle(sprintf("altitude %.1f..%.1f km, period estimate %.2f min", ...
    min(orbit.Altitude_m) / 1000, max(orbit.Altitude_m) / 1000, orbit.EstimatedPeriod_s / 60));
colormap(fig, turbo)
cb = colorbar;
ylabel(cb, "Time [min]")
legend("Earth reference", "Orbit path", "Samples", "Start", "End", "Location", "bestoutside")
styleAxes(gca)
end

function fig = plotOrbitKinematics(t_min, orbit)
% Description:
%   Plots altitude, speed decomposition, and osculating-element diagnostics.

fig = figure("Name", "Orbit - kinematics and elements", "Color", "w");
layout = tiledlayout(fig, 3, 2, "TileSpacing", "compact", "Padding", "compact");
title(layout, "Orbit kinematics and consistency diagnostics")

nexttile
plot(t_min, orbit.Altitude_m ./ 1000, "LineWidth", 1.3)
hold on
yline(mean(orbit.Altitude_m) ./ 1000, "--", "Mean")
grid on
xlabel("Time [min]")
ylabel("Altitude [km]")
title("Altitude")
styleAxes(gca)

nexttile
plot(t_min, orbit.Speed_m_s, "LineWidth", 1.3)
grid on
xlabel("Time [min]")
ylabel("Speed [m/s]")
title("Inertial speed")
styleAxes(gca)

nexttile
plot(t_min, orbit.RadialSpeed_m_s, "LineWidth", 1.2)
hold on
plot(t_min, orbit.TangentialSpeed_m_s, "LineWidth", 1.2)
grid on
xlabel("Time [min]")
ylabel("Speed [m/s]")
title("Radial/tangential speed")
legend("v_r", "v_t", "Location", "best")
styleAxes(gca)

nexttile
plot(t_min, orbit.SpecificEnergy_J_kg, "LineWidth", 1.2)
grid on
xlabel("Time [min]")
ylabel("Specific energy [J/kg]")
title("Specific orbital energy")
styleAxes(gca)

nexttile
plot(t_min, orbit.SemiMajorAxis_m ./ 1000, "LineWidth", 1.2)
grid on
xlabel("Time [min]")
ylabel("a [km]")
title("Estimated semi-major axis")
styleAxes(gca)

nexttile
yyaxis left
plot(t_min, orbit.Eccentricity, "LineWidth", 1.2)
ylabel("e [-]")
yyaxis right
plot(t_min, rad2deg(orbit.Inclination_rad), "LineWidth", 1.2)
ylabel("i [deg]")
grid on
xlabel("Time [min]")
title("Estimated eccentricity and inclination")
legend("e", "i", "Location", "best")
styleAxes(gca)
end

function fig = plotGroundTrackAndLla(t_min, lla, orbit)
% Description:
%   Plots geodetic ground track and LLA histories.

[lonPlot, latPlot] = breakLongitudeWraps(lla.Longitude_deg, lla.Latitude_deg);

fig = figure("Name", "Orbit - ground track and LLA", "Color", "w");
layout = tiledlayout(fig, 2, 2, "TileSpacing", "compact", "Padding", "compact");
title(layout, "Geodetic ground track and local orbit geometry")

nexttile([2 1])
plot(lonPlot, latPlot, "LineWidth", 1.3)
hold on
scatter(lla.Longitude_deg(1), lla.Latitude_deg(1), 55, [0.10 0.55 0.20], "filled", ...
    "MarkerEdgeColor", "k")
scatter(lla.Longitude_deg(end), lla.Latitude_deg(end), 55, [0.85 0.20 0.15], "filled", ...
    "MarkerEdgeColor", "k")
grid on
xlim([-180 180])
ylim([-90 90])
xlabel("Longitude [deg]")
ylabel("Latitude [deg]")
title("Ground track")
legend("Track", "Start", "End", "Location", "best")
styleAxes(gca)

nexttile
plot(t_min, lla.Latitude_deg, "LineWidth", 1.2)
hold on
plot(t_min, lla.Longitude_deg, "LineWidth", 1.2)
grid on
xlabel("Time [min]")
ylabel("Angle [deg]")
title("Latitude and longitude")
legend("lat", "lon", "Location", "best")
styleAxes(gca)

nexttile
plot(t_min, lla.Altitude_m ./ 1000, "LineWidth", 1.2)
hold on
plot(t_min, orbit.Altitude_m ./ 1000, "--", "LineWidth", 1.0)
grid on
xlabel("Time [min]")
ylabel("Altitude [km]")
title("Geodetic vs spherical altitude")
legend("LLA altitude", "|r|-R_E", "Location", "best")
styleAxes(gca)
end

function fig = plotEnvironmentProducts(t_min, B_NED_T, B_I_T, B_B_T, ...
    M_rmm_B_Nm, M_gg_B_Nm, M_srp_B_Nm, M_dist_B_Nm, ...
    F_srp_B_N, F_srp_I_N, a_srp_I_m_s2, a_aero_I_m_s2, a_dist_I_m_s2)
% Description:
%   Plots magnetic-field products, disturbance torques, and orbit accelerations.

B_NED_uT = B_NED_T .* 1e6;
B_I_uT = B_I_T .* 1e6;
B_B_uT = B_B_T .* 1e6;
M_rmm_nNm = M_rmm_B_Nm .* 1e9;
M_gg_nNm = M_gg_B_Nm .* 1e9;
M_srp_nNm = M_srp_B_Nm .* 1e9;
M_dist_nNm = M_dist_B_Nm .* 1e9;
F_srp_B_nN = F_srp_B_N .* 1e9;
F_srp_I_nN = F_srp_I_N .* 1e9;
a_srp_nm_s2 = a_srp_I_m_s2 .* 1e9;
a_aero_nm_s2 = a_aero_I_m_s2 .* 1e9;
a_dist_nm_s2 = a_dist_I_m_s2 .* 1e9;

fig = figure("Name", "Orbit environment - fields and disturbances", "Color", "w");
layout = tiledlayout(fig, 4, 2, "TileSpacing", "compact", "Padding", "compact");
title(layout, "Magnetic field and disturbance products")

nexttile
plot(t_min, B_NED_uT, "LineWidth", 1.1)
grid on
xlabel("Time [min]")
ylabel("B_NED [uT]")
title("IGRF field in NED")
legend("N", "E", "D", "Location", "best")
styleAxes(gca)

nexttile
plot(t_min, vecnorm(B_NED_uT, 2, 2), "LineWidth", 1.3)
hold on
plot(t_min, vecnorm(B_I_uT, 2, 2), "--", "LineWidth", 1.0)
plot(t_min, vecnorm(B_B_uT, 2, 2), ":", "LineWidth", 1.5)
grid on
xlabel("Time [min]")
ylabel("|B| [uT]")
title("Magnetic-field norm by frame")
legend("NED", "I", "B", "Location", "best")
styleAxes(gca)

nexttile
plot(t_min, B_I_uT, "LineWidth", 1.1)
grid on
xlabel("Time [min]")
ylabel("B_I [uT]")
title("Magnetic field in inertial axes")
legend("x", "y", "z", "Location", "best")
styleAxes(gca)

nexttile
plot(t_min, B_B_uT, "LineWidth", 1.1)
grid on
xlabel("Time [min]")
ylabel("B_B [uT]")
title("Magnetic field in body axes")
legend("x", "y", "z", "Location", "best")
styleAxes(gca)

nexttile
plot(t_min, vecnorm(M_rmm_nNm, 2, 2), "LineWidth", 1.2)
hold on
plot(t_min, vecnorm(M_gg_nNm, 2, 2), "LineWidth", 1.2)
plot(t_min, vecnorm(M_srp_nNm, 2, 2), "LineWidth", 1.2)
plot(t_min, vecnorm(M_dist_nNm, 2, 2), "LineWidth", 1.4)
grid on
xlabel("Time [min]")
ylabel("|M| [nN*m]")
title("Disturbance torque norms")
legend("RMM", "gravity gradient", "SRP", "total", "Location", "best")
styleAxes(gca)

nexttile
plot(t_min, M_dist_nNm, "LineWidth", 1.1)
grid on
xlabel("Time [min]")
ylabel("M_{dist,B} [nN*m]")
title("Total disturbance torque components")
legend("x", "y", "z", "Location", "best")
styleAxes(gca)

nexttile
plot(t_min, vecnorm(F_srp_B_nN, 2, 2), "LineWidth", 1.1)
hold on
plot(t_min, vecnorm(F_srp_I_nN, 2, 2), "--", "LineWidth", 1.0)
grid on
xlabel("Time [min]")
ylabel("|F_{SRP}| [nN]")
title("SRP force norm by frame")
legend("B", "I", "Location", "best")
styleAxes(gca)

nexttile
plot(t_min, vecnorm(a_aero_nm_s2, 2, 2), "LineWidth", 1.2)
hold on
plot(t_min, vecnorm(a_srp_nm_s2, 2, 2), "LineWidth", 1.2)
plot(t_min, vecnorm(a_dist_nm_s2, 2, 2), "LineWidth", 1.4)
grid on
xlabel("Time [min]")
ylabel("|a| [nm/s^2]")
title("Orbit disturbance acceleration norms")
legend("aero", "SRP", "total", "Location", "best")
styleAxes(gca)
end

function printSummary(orbit, summary)
% Description:
%   Prints a compact numerical summary matching the plotted diagnostics.

fprintf("\nOrbit/environment plot summary\n");
fprintf("Altitude [km] min/mean/max : %.3f / %.3f / %.3f\n", ...
    min(orbit.Altitude_m) / 1000, mean(orbit.Altitude_m) / 1000, max(orbit.Altitude_m) / 1000);
fprintf("Speed [m/s] min/mean/max   : %.6f / %.6f / %.6f\n", ...
    min(orbit.Speed_m_s), mean(orbit.Speed_m_s), max(orbit.Speed_m_s));
fprintf("a [km] mean, e mean, i mean: %.6f / %.9f / %.6f deg\n", ...
    mean(orbit.SemiMajorAxis_m) / 1000, mean(orbit.Eccentricity), mean(rad2deg(orbit.Inclination_rad)));
fprintf("Period estimate [min]      : %.6f\n", orbit.EstimatedPeriod_s / 60);
fprintf("|B_NED| [uT] min/mean/max  : %.6f / %.6f / %.6f\n", ...
    summary.Magnetic.B_NED_norm_uT.Min, summary.Magnetic.B_NED_norm_uT.Mean, summary.Magnetic.B_NED_norm_uT.Max);
fprintf("|B_I|-|B_B| max diff [T]   : %.3e\n", summary.Magnetic.B_I_B_norm_max_diff_T);
fprintf("|M_rmm| [nN*m] min/mean/max: %.6f / %.6f / %.6f\n", ...
    summary.Torques.M_rmm_norm_nNm.Min, summary.Torques.M_rmm_norm_nNm.Mean, summary.Torques.M_rmm_norm_nNm.Max);
fprintf("|M_gg| [nN*m] min/mean/max : %.6f / %.6f / %.6f\n", ...
    summary.Torques.M_gg_norm_nNm.Min, summary.Torques.M_gg_norm_nNm.Mean, summary.Torques.M_gg_norm_nNm.Max);
fprintf("|M_srp| [nN*m] min/mean/max: %.6f / %.6f / %.6f\n", ...
    summary.Torques.M_srp_norm_nNm.Min, summary.Torques.M_srp_norm_nNm.Mean, summary.Torques.M_srp_norm_nNm.Max);
fprintf("|M_dist| [nN*m] min/mean/max: %.6f / %.6f / %.6f\n", ...
    summary.Torques.M_dist_norm_nNm.Min, summary.Torques.M_dist_norm_nNm.Mean, summary.Torques.M_dist_norm_nNm.Max);
if isfield(summary, "Forces")
    fprintf("|F_srp| [nN] min/mean/max: %.6f / %.6f / %.6f\n", ...
        summary.Forces.F_srp_I_norm_nN.Min, summary.Forces.F_srp_I_norm_nN.Mean, summary.Forces.F_srp_I_norm_nN.Max);
    fprintf("|F_srp_B|-|F_srp_I| max diff [N]: %.3e\n", ...
        summary.Forces.F_srp_B_I_norm_max_diff_N);
end
if isfield(summary, "Accelerations")
    fprintf("|a_srp| [nm/s^2] min/mean/max: %.6f / %.6f / %.6f\n", ...
        summary.Accelerations.a_srp_norm_nm_s2.Min, summary.Accelerations.a_srp_norm_nm_s2.Mean, ...
        summary.Accelerations.a_srp_norm_nm_s2.Max);
    fprintf("|a_dist| [nm/s^2] min/mean/max: %.6f / %.6f / %.6f\n\n", ...
        summary.Accelerations.a_dist_norm_nm_s2.Min, summary.Accelerations.a_dist_norm_nm_s2.Mean, ...
        summary.Accelerations.a_dist_norm_nm_s2.Max);
else
    fprintf("\n");
end
end

function exportFigures(figures, exportDirectory)
% Description:
%   Saves figures as PNG files when an export directory is supplied.

exportDirectory = string(exportDirectory);
if strlength(exportDirectory) == 0
    return;
end

if ~isfolder(exportDirectory)
    mkdir(exportDirectory);
end

fileNames = ["orbit_3d", "orbit_kinematics", "ground_track_lla", "environment_products"];
for k = 1:numel(figures)
    exportgraphics(figures(k), fullfile(exportDirectory, fileNames(k) + ".png"), "Resolution", 180);
end
end

function [lonPlot, latPlot] = breakLongitudeWraps(lon_deg, lat_deg)
% Description:
%   Inserts NaNs at longitude wrap jumps so ground-track lines do not cross
%   the map.

lonPlot = lon_deg(:);
latPlot = lat_deg(:);
wrapJumps = find(abs(diff(lonPlot)) > 180);

for k = numel(wrapJumps):-1:1
    idx = wrapJumps(k) + 1;
    lonPlot = [lonPlot(1:idx-1); NaN; lonPlot(idx:end)]; %#ok<AGROW>
    latPlot = [latPlot(1:idx-1); NaN; latPlot(idx:end)]; %#ok<AGROW>
end
end

function styleAxes(ax)
% Description:
%   Applies consistent readable styling to plot axes.

ax.FontName = "Helvetica";
ax.FontSize = 10;
ax.LineWidth = 0.8;
ax.GridAlpha = 0.22;
ax.MinorGridAlpha = 0.12;
end
