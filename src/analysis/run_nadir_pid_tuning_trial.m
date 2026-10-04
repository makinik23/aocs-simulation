function trial = run_nadir_pid_tuning_trial(label,gains,options)
%RUN_NADIR_PID_TUNING_TRIAL Run and append a reproducible full-plant PID trial.
% Gains are [Kp Kd Ki], in [s^-2 s^-1 s^-3]. The baseline PD is unchanged.
% Run trials sequentially, with unique labels, to preserve the audit order.
arguments
    label (1,1) string
    gains (1,3) double {mustBeFinite,mustBePositive}
    options.AntiWindupTime_s (1,1) double {mustBeFinite,mustBePositive} = 10
    options.InitialError_deg (1,1) double {mustBeFinite,mustBeNonnegative} = 30
    options.StopTime_s (1,1) double {mustBeFinite,mustBePositive} = 1200
    options.InitialErrorAxis (3,1) double {mustBeFinite} = [1;1;1]
    options.InitialRateError_deg_s (3,1) double {mustBeFinite} = zeros(3,1)
    options.DisturbancesEnabled (1,1) logical = true
    options.Rationale (1,1) string = ""
end
root=setupAocsPaths(); directory=fullfile(root,'docs','data','nadir_pid_tuning');
if ~isfolder(directory), mkdir(directory); end
historyFile=fullfile(directory,'history.json');
if isfile(historyFile), history=jsondecode(fileread(historyFile)); else, history=struct([]); end
assert(isempty(history) || ~any(strcmp({history.Label},label)),'Trial labels must be unique.');
assert(norm(options.InitialErrorAxis)>0,'InitialErrorAxis must be nonzero.');
trial=struct('Sequence',numel(history)+1,'Label',char(label), ...
    'StartedUTC',char(datetime('now',TimeZone='UTC',Format='yyyy-MM-dd''T''HH:mm:ssXXX')), ...
    'Status','running','Kp_s2',gains(1),'Kd_s',gains(2),'Ki_s3',gains(3), ...
    'AntiWindupTime_s',options.AntiWindupTime_s, ...
    'InitialError_deg',options.InitialError_deg,'StopTime_s',options.StopTime_s, ...
    'InitialErrorAxis',options.InitialErrorAxis'/norm(options.InitialErrorAxis), ...
    'InitialRateError_deg_s',options.InitialRateError_deg_s', ...
    'DisturbancesEnabled',options.DisturbancesEnabled, ...
    'Rationale',char(options.Rationale),'Decision','pending review','RunDirectory','', ...
    'FinalError_deg',NaN,'TailRmsError_deg',NaN,'MaxErrorAfter300s_deg',NaN, ...
    'ReboundAfterFirstBelow1deg_deg',NaN,'SettlingWithin1deg_s',NaN, ...
    'TailRateRms_deg_s',NaN,'SaturationFraction',NaN, ...
    'MaxIntegralAcceleration_rad_s2',NaN,'MaxIntegralTorque_Nm',NaN, ...
    'FinalIntegralAcceleration_rad_s2',[NaN NaN NaN]);
if isempty(history), history=trial; else, history(end+1)=trial; end
writeAocsJson(historyFile,history);
try
    [out,aocs,runDirectory,report]=run_nadir_pointing_demo(Controller="PID", ...
        PIDGains=gains,AntiWindupTime_s=options.AntiWindupTime_s, ...
        InitialError_deg=options.InitialError_deg,StopTime_s=options.StopTime_s, ...
        InitialErrorAxis=options.InitialErrorAxis,InitialRateError_deg_s=options.InitialRateError_deg_s, ...
        DisturbancesEnabled=options.DisturbancesEnabled,MakePlots=false);
    trial.Status='completed'; trial.RunDirectory=char(erase(string(runDirectory),string(root)+filesep));
    trial.FinalError_deg=report.FinalError_deg; trial.TailRmsError_deg=report.TailRmsError_deg;
    trial.MaxErrorAfter300s_deg=max(report.TrueError_deg(report.Time_s>=min(300,options.StopTime_s)));
    first=find(report.TrueError_deg<1,1);
    if ~isempty(first), trial.ReboundAfterFirstBelow1deg_deg=max(report.TrueError_deg(first:end)); end
    last=find(report.TrueError_deg>1,1,'last');
    if isempty(last), trial.SettlingWithin1deg_s=0;
    elseif last<numel(report.Time_s), trial.SettlingWithin1deg_s=report.Time_s(last+1); end
    trial.SaturationFraction=report.SaturationFraction;
    signal=out.logsout.get('pid_integral_acceleration_rad_s2').Values;
    xi=loggedSignalMatrix(signal.Data,3,'integral acceleration');
    trial.MaxIntegralAcceleration_rad_s2=max(abs(xi),[],'all');
    trial.MaxIntegralTorque_Nm=max(abs(xi*aocs.Spacecraft.I_B'),[],'all');
    trial.FinalIntegralAcceleration_rad_s2=xi(end,:);
    rate=out.logsout.get('pid_omega_error_BD_B_rad_s').Values;
    ew=loggedSignalMatrix(rate.Data,3,'estimated rate error');
    trial.TailRateRms_deg_s=rad2deg(sqrt(mean(sum(ew(rate.Time>=rate.Time(end)-60,:).^2,2))));
    save(fullfile(runDirectory,'pid_tuning_trial.mat'),'trial','report');
catch problem
    trial.Status=['failed: ' problem.identifier];
    history(end)=trial; writeAocsJson(historyFile,history); rethrow(problem);
end
history(end)=trial; writeAocsJson(historyFile,history);
flat=rmfield(history,{'InitialErrorAxis','InitialRateError_deg_s','FinalIntegralAcceleration_rad_s2'});
writetable(struct2table(flat),fullfile(directory,'trials.csv'));
fprintf('TUNING_TRIAL %s: tail RMS %.4f deg; rebound %.3f deg; settles <1 deg at %.1f s; saturation %.2f%%\n', ...
    label,trial.TailRmsError_deg,trial.ReboundAfterFirstBelow1deg_deg,trial.SettlingWithin1deg_s,100*trial.SaturationFraction);
end
