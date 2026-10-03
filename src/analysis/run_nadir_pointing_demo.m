function [out, AOCS, runDirectory, report] = run_nadir_pointing_demo(options)
%RUN_NADIR_POINTING_DEMO Closed-loop acquisition from a specified LVLH error.
% Uses the complete plant, sensors, drivers, TRIAD and MEKF.
% Example: run_nadir_pointing_demo(InitialError_deg=90, StopTime_s=600)
arguments
    options.InitialError_deg (1,1) double {mustBeFinite,mustBeNonnegative} = 90
    options.StopTime_s (1,1) double {mustBeFinite,mustBePositive} = 600
    options.DisturbancesEnabled (1,1) logical = true
    options.MakePlots (1,1) logical = true
end

assert(options.InitialError_deg <= 180, 'InitialError_deg must not exceed 180.');
root = setupAocsPaths();

base = loadAocsSimulationConfig(fullfile(root,'config','scenarios', ...
    'nadir_pointing.json'), root);
s = base.GNCConfig.AttitudeInitialization;

orbit = propagateOnboardSgp4([0; s.epoch_utc_jd; ...
    s.sgp4_semi_major_axis_m; s.sgp4_eccentricity; ...
    s.sgp4_inclination_deg; s.sgp4_raan_deg; ...
    s.sgp4_argument_of_periapsis_deg; s.sgp4_true_anomaly_deg; s.sgp4_bstar]);

assert(orbit(7) == 1, 'Initial onboard SGP4 propagation failed.');

r = orbit(1:3); v = orbit(4:6); h = cross(r,v);
z = -r/norm(r); y = -h/norm(h); x = cross(y,z);

qRef = dcm2quat([x';y';z']);
axis = [1 1 1]/sqrt(3);

qError = [cosd(options.InitialError_deg/2), ...
    axis*sind(options.InitialError_deg/2)];

qInitial = quatmultiply(qRef,qError);
omegaRef = [0;-norm(h)/dot(r,r);0];

raw = base.Raw;
raw.initial_conditions.q_BI = qInitial(:);
raw.initial_conditions.omega_BI_B_rad_s = quat2dcm(qError)*omegaRef;

if isfield(raw.initial_conditions,'euler_BI_0_rad')
    raw.initial_conditions = rmfield(raw.initial_conditions,'euler_BI_0_rad');
end

raw.simulation.start_time_s = 0;
raw.simulation.stop_time_s = options.StopTime_s;
raw.environment.disturbances.enabled = options.DisturbancesEnabled;

if ~options.DisturbancesEnabled
    raw.results.file = 'aocs_nadir_pointing_no_disturbances.mat';
    raw.mission.name = 'quaternion_pd_nadir_no_disturbances';
end

% The archived configuration contains the complete resolved experiment.
configFile = [tempname '.json'];
cleanup = onCleanup(@() delete(configFile)); %#ok<NASGU>

writeAocsJson(configFile,raw);

[out,AOCS,runDirectory] = run_aocs_simulation(configFile);

report = plot_nadir_pointing_results(out,AOCS,MakePlots=options.MakePlots);
save(fullfile(runDirectory,'nadir_report.mat'),'report');

if options.MakePlots
    exportgraphics(gcf,fullfile(runDirectory,'nadir_pointing.png'),'Resolution',160);
end

end
