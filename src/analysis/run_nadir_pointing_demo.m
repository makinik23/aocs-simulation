function [out, AOCS, runDirectory, report] = run_nadir_pointing_demo(options)
%RUN_NADIR_POINTING_DEMO Nadir pointing for a given reference quaternion.
% Example: run_nadir_pointing_demo(InitialError_deg=90, StopTime_s=600)
arguments
    options.InitialError_deg (1,1) double {mustBeFinite,mustBeNonnegative} = 90
    options.StopTime_s (1,1) double {mustBeFinite,mustBePositive} = 600
    options.DisturbancesEnabled (1,1) logical = true
    options.Controller (1,1) string {mustBeMember(options.Controller,["PD","PID"])} = "PD"
    options.PIDGains double {mustBeFinite,mustBePositive} = []
    options.AntiWindupTime_s double {mustBeFinite,mustBePositive} = []
    options.InitialErrorAxis (3,1) double {mustBeFinite} = [1;1;1]
    options.InitialRateError_deg_s (3,1) double {mustBeFinite} = zeros(3,1)
    options.MakePlots (1,1) logical = true
end

assert(options.InitialError_deg <= 180, 'InitialError_deg must not exceed 180.');
root = setupAocsPaths();

base = loadAocsSimulationConfig(fullfile(root,'config','scenarios', ...
    'nadir_pointing.json'), root);
if base.Orbit.InitialState.Type == "cartesian"
    r = base.Orbit.InitialCartesian.position_I_m;
    v = base.Orbit.InitialCartesian.velocity_I_m_s;
else
    kepler = base.Orbit.InitialKeplerian;
    epochJd = base.GNCConfig.AttitudeInitialization.epoch_utc_jd;
    [r, v] = propagateOrbit(epochJd, kepler.semi_major_axis_m, ...
        kepler.eccentricity, rad2deg(kepler.inclination_rad), rad2deg(kepler.raan_rad), ...
        rad2deg(kepler.argument_of_periapsis_rad), rad2deg(kepler.true_anomaly_rad), ...
        Epoch=epochJd, PropModel="two-body-keplerian", ...
        OutputCoordinateFrame="inertial");
    r = reshape(r, 3, 1);
    v = reshape(v, 3, 1);
end

h = cross(r,v);
z = -r/norm(r); y = -h/norm(h); x = cross(y,z);

qRef = dcm2quat([x';y';z']);
assert(norm(options.InitialErrorAxis)>0,'InitialErrorAxis must be nonzero.');
axis = options.InitialErrorAxis'/norm(options.InitialErrorAxis);

qError = [cosd(options.InitialError_deg/2), ...
    axis*sind(options.InitialError_deg/2)];

qInitial = quatmultiply(qRef,qError);
omegaRef = [0;-norm(h)/dot(r,r);0];

raw = base.Raw;
raw.initial_conditions.q_BI = qInitial(:);
raw.initial_conditions.omega_BI_B_rad_s = quat2dcm(qError)*omegaRef ...
    + deg2rad(options.InitialRateError_deg_s);

if isfield(raw.initial_conditions,'euler_BI_0_rad')
    raw.initial_conditions = rmfield(raw.initial_conditions,'euler_BI_0_rad');
end

raw.simulation.start_time_s = 0;
raw.simulation.stop_time_s = options.StopTime_s;
raw.environment.disturbances.enabled = options.DisturbancesEnabled;
raw.gnc.attitude_control.use_integral = options.Controller == "PID";

if ~isempty(options.PIDGains)
    assert(options.Controller=="PID" && isvector(options.PIDGains) && numel(options.PIDGains)==3, ...
        'PIDGains must be [Kp Kd Ki] and requires Controller="PID".');
    raw.gnc.attitude_control.pid_kp_s2 = options.PIDGains(1);
    raw.gnc.attitude_control.pid_kd_s = options.PIDGains(2);
    raw.gnc.attitude_control.ki_s3 = options.PIDGains(3);
end

if ~isempty(options.AntiWindupTime_s)
    assert(isscalar(options.AntiWindupTime_s),'AntiWindupTime_s must be scalar.');
    raw.gnc.attitude_control.anti_windup_time_s = options.AntiWindupTime_s;
end

if ~options.DisturbancesEnabled
    raw.results.file = 'aocs_nadir_pointing_no_disturbances.mat';
    raw.mission.name = 'quaternion_pd_nadir_no_disturbances';
end

if options.Controller == "PID"
    [~,name,extension] = fileparts(raw.results.file);
    raw.results.file = [char(name) '_pid' char(extension)];
    raw.mission.name = [char(raw.mission.name) '_integral_feedback'];
end

% The archived configuration contains the complete resolved experiment.
configFile = [tempname '.json'];
cleanup = onCleanup(@() delete(configFile));

writeAocsJson(configFile,raw);

[out,AOCS,runDirectory] = run_aocs_simulation(configFile);

report = plot_nadir_pointing_results(out,AOCS,MakePlots=options.MakePlots);
save(fullfile(runDirectory,'nadir_report.mat'),'report');

if options.MakePlots
    exportgraphics(gcf,fullfile(runDirectory,'nadir_pointing.png'),'Resolution',160);
end

end