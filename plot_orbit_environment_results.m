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

projectRoot = fileparts(mfilename("fullpath"));
addpath(fullfile(projectRoot, "src", "analysis"));
addpath(fullfile(projectRoot, "src", "config"));

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
M_rmm_B_Nm = loggedVector(logsout, "M_rmm_B_Nm", 3);
M_gg_B_Nm = loggedVector(logsout, "M_gg_B_Nm", 3);
M_dist_B_Nm = loggedVector(logsout, "M_dist_B_Nm", 3);

orbit = computeOrbitDiagnostics(r_I_m, v_I_m_s, AOCS);
lla = computeLlaProducts(r_I_m, t_s, AOCS);

figures = gobjects(4, 1);
figures(1) = plotInertialOrbit3d(r_I_m, t_min, orbit, AOCS);
figures(2) = plotOrbitKinematics(t_min, orbit);
figures(3) = plotGroundTrackAndLla(t_min, lla, orbit);
figures(4) = plotEnvironmentProducts(t_min, B_NED_T, B_I_T, B_B_T, ...
    M_rmm_B_Nm, M_gg_B_Nm, M_dist_B_Nm);

printSummary(orbit, B_NED_T, B_I_T, B_B_T, M_rmm_B_Nm, M_gg_B_Nm, M_dist_B_Nm);
exportFigures(figures, exportDirectory);
end

function data = loggedVector(logsout, signalName, width)
% Description:
%   Reads a logged vector signal and returns an N-by-width matrix.
%
% Arguments:
%   logsout - Simulink logsout dataset.
%   signalName - Logged signal name.
%   width - Expected vector width.
%
% Outputs:
%   data - N-by-width numeric matrix.

element = logsout.getElement(char(signalName));
if isempty(element)
    error("AOCS:Analysis:MissingSignal", "Missing logged signal '%s'.", char(signalName));
end

data = loggedSignalMatrix(element.Values.Data, width, signalName);
end

function orbit = computeOrbitDiagnostics(r_I_m, v_I_m_s, AOCS)
% Description:
%   Computes scalar orbit diagnostics useful for plotting and sanity checks.
%
% Arguments:
%   r_I_m - N-by-3 inertial position samples [m].
%   v_I_m_s - N-by-3 inertial velocity samples [m/s].
%   AOCS - Validated configuration struct.
%
% Outputs:
%   orbit - Struct of orbit diagnostics.

mu = AOCS.Orbit.CentralBodyConstants.mu_m3_s2;
earthRadius_m = AOCS.Orbit.CentralBodyConstants.radius_m;

rNorm_m = vecnorm(r_I_m, 2, 2);
vNorm_m_s = vecnorm(v_I_m_s, 2, 2);
rHat_I = r_I_m ./ rNorm_m;

radialSpeed_m_s = sum(r_I_m .* v_I_m_s, 2) ./ rNorm_m;
tangentialSpeed_m_s = sqrt(max(vNorm_m_s.^2 - radialSpeed_m_s.^2, 0));

h_I_m2_s = cross(r_I_m, v_I_m_s, 2);
hNorm_m2_s = vecnorm(h_I_m2_s, 2, 2);
inclination_rad = acos(clamp(h_I_m2_s(:, 3) ./ hNorm_m2_s, -1, 1));

specificEnergy_J_kg = 0.5 .* vNorm_m_s.^2 - mu ./ rNorm_m;
semiMajorAxis_m = -mu ./ (2 .* specificEnergy_J_kg);
eccentricityVector = cross(v_I_m_s, h_I_m2_s, 2) ./ mu - rHat_I;
eccentricity = vecnorm(eccentricityVector, 2, 2);
period_s = 2 * pi * sqrt(mean(semiMajorAxis_m, "omitnan")^3 / mu);

orbit = struct();
orbit.Radius_m = rNorm_m;
orbit.Altitude_m = rNorm_m - earthRadius_m;
orbit.Speed_m_s = vNorm_m_s;
orbit.RadialSpeed_m_s = radialSpeed_m_s;
orbit.TangentialSpeed_m_s = tangentialSpeed_m_s;
orbit.SpecificEnergy_J_kg = specificEnergy_J_kg;
orbit.SpecificAngularMomentum_m2_s = hNorm_m2_s;
orbit.SemiMajorAxis_m = semiMajorAxis_m;
orbit.Eccentricity = eccentricity;
orbit.Inclination_rad = inclination_rad;
orbit.EstimatedPeriod_s = period_s;
end

function lla = computeLlaProducts(r_I_m, t_s, AOCS)
% Description:
%   Converts inertial position samples to geodetic latitude, longitude, and
%   altitude using the same IAU-2000/2006 reduction family used in the model.
%
% Arguments:
%   r_I_m - N-by-3 inertial position samples [m].
%   t_s - N-by-1 simulation time samples [s].
%   AOCS - Validated configuration struct.
%
% Outputs:
%   lla - Struct with Latitude_deg, Longitude_deg, and Altitude_m.

epoch = AOCS.Epoch.Utc;
epochDate = datetime(epoch(1), epoch(2), epoch(3), epoch(4), epoch(5), epoch(6), ...
    "TimeZone", "UTC");
utc = datevec(epochDate + seconds(t_s));

llaData = eci2lla(r_I_m, utc, "IAU-2000/2006");

lla = struct();
lla.Latitude_deg = llaData(:, 1);
lla.Longitude_deg = wrapDegrees180(llaData(:, 2));
lla.Altitude_m = llaData(:, 3);
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
    M_rmm_B_Nm, M_gg_B_Nm, M_dist_B_Nm)
% Description:
%   Plots magnetic-field products and disturbance torques.

B_NED_uT = B_NED_T .* 1e6;
B_I_uT = B_I_T .* 1e6;
B_B_uT = B_B_T .* 1e6;
M_rmm_nNm = M_rmm_B_Nm .* 1e9;
M_gg_nNm = M_gg_B_Nm .* 1e9;
M_dist_nNm = M_dist_B_Nm .* 1e9;

fig = figure("Name", "Orbit environment - magnetic field and torques", "Color", "w");
layout = tiledlayout(fig, 3, 2, "TileSpacing", "compact", "Padding", "compact");
title(layout, "Magnetic field and disturbance torque products")

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
plot(t_min, vecnorm(M_dist_nNm, 2, 2), "LineWidth", 1.4)
grid on
xlabel("Time [min]")
ylabel("|M| [nN*m]")
title("Disturbance torque norms")
legend("RMM", "gravity gradient", "total", "Location", "best")
styleAxes(gca)

nexttile
plot(t_min, M_dist_nNm, "LineWidth", 1.1)
grid on
xlabel("Time [min]")
ylabel("M_{dist,B} [nN*m]")
title("Total disturbance torque components")
legend("x", "y", "z", "Location", "best")
styleAxes(gca)
end

function printSummary(orbit, B_NED_T, B_I_T, B_B_T, M_rmm_B_Nm, M_gg_B_Nm, M_dist_B_Nm)
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
fprintf("|B_NED| [uT] min/mean/max  : %.6f / %.6f / %.6f\n", vectorNormStats(B_NED_T .* 1e6));
fprintf("|B_I|-|B_B| max diff [T]   : %.3e\n", ...
    max(abs(vecnorm(B_I_T, 2, 2) - vecnorm(B_B_T, 2, 2))));
fprintf("|M_rmm| [nN*m] min/mean/max: %.6f / %.6f / %.6f\n", vectorNormStats(M_rmm_B_Nm .* 1e9));
fprintf("|M_gg| [nN*m] min/mean/max : %.6f / %.6f / %.6f\n", vectorNormStats(M_gg_B_Nm .* 1e9));
fprintf("|M_dist| [nN*m] min/mean/max: %.6f / %.6f / %.6f\n\n", vectorNormStats(M_dist_B_Nm .* 1e9));
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

function stats = vectorNormStats(data)
% Description:
%   Returns min/mean/max of vector norms in a row for fprintf expansion.

norms = vecnorm(data, 2, 2);
stats = [min(norms), mean(norms), max(norms)];
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

function wrapped = wrapDegrees180(degrees)
% Description:
%   Wraps degrees to [-180, 180).

wrapped = mod(degrees + 180, 360) - 180;
end

function value = clamp(value, lowerBound, upperBound)
% Description:
%   Clamps numeric values to a closed interval.

value = min(max(value, lowerBound), upperBound);
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
