function [out,AOCS,runDirectory,report] = run_long_pd_diagnostic(stopTime_s)
%RUN_LONG_PD_DIAGNOSTIC Archive a nominal PD and torque EKF nadir run.
% Example: run_long_pd_diagnostic(15000)
% Quaternions are scalar-first, C_BI maps inertial I to body B; D is LVLH.
	if nargin < 1, stopTime_s = 15000; end
	validateattributes(stopTime_s,{'double'},{'scalar','finite','positive'});
	root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
	addpath(root);
	setupAocsPaths();
	configureAocsFileGeneration();
	requireDtm2020Native();

	% 1. Preserve scenario gains and noise; initialize 5 deg from onboard LVLH.
	base = loadAocsSimulationConfig(fullfile(root,'config','scenarios','nadir_pointing.json'),root);
	s = base.GNCConfig.AttitudeInitialization;
	orbit = propagateOnboardSgp4([0;s.epoch_utc_jd;s.sgp4_semi_major_axis_m; ...
		s.sgp4_eccentricity;s.sgp4_inclination_deg;s.sgp4_raan_deg; ...
		s.sgp4_argument_of_periapsis_deg;s.sgp4_true_anomaly_deg;s.sgp4_bstar]);
	assert(orbit(7)==1,'Initial onboard SGP4 propagation failed.');
	r = orbit(1:3);
	h = cross(r,orbit(4:6));
	z = -r/norm(r);
	y = -h/norm(h);
	x = cross(y,z);
	qRef = dcm2quat([x.';y.';z.']);
	qError = [cosd(2.5),sind(2.5)*ones(1,3)/sqrt(3)];
	wRef = [0;-norm(h)/dot(r,r);0];
	raw = base.Raw;
	raw.initial_conditions.q_BI = quatmultiply(qRef,qError).';
	raw.initial_conditions.omega_BI_B_rad_s = quat2dcm(qError)*wRef;
	if isfield(raw.initial_conditions,'euler_BI_0_rad')
		raw.initial_conditions = rmfield(raw.initial_conditions,'euler_BI_0_rad');
	end
	raw.simulation.start_time_s = 0;
	raw.simulation.stop_time_s = stopTime_s;
	raw.gnc.attitude_control.use_integral = false;
	ObserverCfg = defaultDisturbanceObserverConfig();
	assert(ObserverCfg.Enabled,'Default torque EKF must be enabled.');

	% 2. Create a unique archive before simulation; retain the resolved config.
	resultRoot = fullfile(root,'experiments','nadir_pd','results');
	if ~isfolder(resultRoot), mkdir(resultRoot); end
	runDirectory = tempname(resultRoot);
	mkdir(runDirectory);
	configFile = fullfile(runDirectory,'scenario.json');
	writeAocsJson(configFile,raw);
	[AOCS,variables] = setupAocsSimulation(configFile,false);
	model = char(AOCS.Model.Name);
	assert(strcmp(model,'aocs_plant'),'This diagnostic requires the main aocs_plant model.');
	wasLoaded = bdIsLoaded(model);
	load_system(fullfile(root,'models',[model '.slx']));
	cleanup = onCleanup(@() closeIfOpened(model,wasLoaded)); %#ok<NASGU>
	in = Simulink.SimulationInput(model);
	names = fieldnames(variables);
	for k = 1:numel(names)
		in = in.setVariable(names{k},variables.(names{k}));
	end
	in = in.setVariable('ObserverCfg',ObserverCfg,'Workspace',model);
	in = applyAocsSimulationSettings(model,AOCS,in);
	in = in.setModelParameter('ReturnWorkspaceOutputs','on');
	% Use the signal instrumentation already saved in the main model.
	% The unwired actuator placeholder is bypassed by the current plant torque path.
	actuators = Simulink.ID.getFullName([model ':5616']);
	ports = get_param(actuators,'PortHandles');
	assert(all(arrayfun(@(p) get_param(p,'Line')==-1,ports.Inport)), ...
		'Actuators is wired: inspect its topology before bypassing it.');
	in = in.setBlockParameter(actuators,'Commented','on');
	metadata = struct('InitialError_deg',5,'InitialErrorAxis',[1;1;1]/sqrt(3), ...
		'InitialRateError_rad_s',zeros(3,1),'RandomSeed',1,'Window_s',500, ...
		'CommentedActuatorsSID',5616,'StartedUTC',char(datetime('now','TimeZone','UTC')));
	save(fullfile(runDirectory,'configuration.mat'),'AOCS','raw','ObserverCfg','metadata');
	fprintf('PD + default torque EKF, %.0f s; archive: %s\n',stopTime_s,runDirectory);

	% 3. Archive raw results immediately, before any diagnostic computation.
	rng(1);
	wallClock = tic;
	progress = timer('ExecutionMode','fixedSpacing','Period',45, ...
		'TimerFcn',@(~,~) reportProgress(model));
	progressCleanup = onCleanup(@() stopProgress(progress));
	start(progress);
	out = sim(in);
	wallTime_s = toc(wallClock);
	clear progressCleanup;
	save(fullfile(runDirectory,'raw_simulation.mat'),'out','AOCS','raw', ...
		'ObserverCfg','metadata','wallTime_s','-v7.3');

	% 4. Compare truth, estimate and onboard reference on the GNC timeline.
	logs = out.logsout;
	ref = logs.get('q_ref_DI').Values;
	t = ref.Time(:);
	qReference = sample(ref,4,t);
	qTrue = sample(logs.get('q_be').Values,4,t);
	estimate = logs.get('AttitudeEstimate').Values;
	qEstimated = sample(estimate.q_BI,4,t);
	trueError = attitudeAngle(qTrue,qReference);
	estimatedError = attitudeAngle(qEstimated,qReference);
	estimationError = attitudeAngle(qEstimated,qTrue);
	omegaTrue = sample(logs.get('omega_b').Values,3,t);
	omegaEstimated = sample(estimate.omega_BI_B_rad_s,3,t);
	omegaReference = sample(logs.get('omega_ref_DI_D_rad_s').Values,3,t);
	omegaError = zeros(numel(t),3);
	for k = 1:numel(t)
		C_BD = quat2dcm(qTrue(k,:))*quat2dcm(qReference(k,:)).';
		omegaError(k,:) = omegaTrue(k,:) - (C_BD*omegaReference(k,:).').';
	end
	tauTruth = optionalLog(logs,'M_dist_B_Nm',3,t);
	tauEstimate = optionalOutput(out,'DisturbanceObserverTorque',3,t);
	tauDemand = optionalLog(logs,'tau_demand_B_Nm',3,t);
	tauApplied = optionalLog(logs,'tau_control_B_Nm',3,t);
	active = optionalLog(logs,'control_active',1,t);
	observerValid = optionalOutput(out,'EKFObserverValid',1,t);
	estimateValid = sample(estimate.valid,1,t);
	health = logs.get('AttitudeHealth').Values;
	healthUsable = sample(health.control_usable,1,t);
	correctionAge = sample(health.correction_age_s,1,t);
	healthMode = sample(health.mode_id,1,t);
	solarFlux = optionalLog(logs,'solar_flux_shadowed_W_m2',1,t);
	% Logged PD demand precedes observer subtraction; test the compensated demand.
	compensatedDemand = optionalLog(logs,'tau_compensated_demand_B_Nm',3,t);
	if all(isnan(compensatedDemand),'all'), compensatedDemand = tauDemand-tauEstimate; end
	saturated = double(any(abs(compensatedDemand)>AOCS.Control.MaxTorque_Nm+1e-12,2));
	if all(isnan(tauDemand),'all'), saturated(:)=NaN; end
	trace = struct('Time_s',t,'TrueError_deg',trueError, ...
		'EstimatedError_deg',estimatedError,'EstimationError_deg',estimationError, ...
		'OmegaError_B_rad_s',omegaError,'OmegaEstimationError_B_rad_s',omegaEstimated-omegaTrue, ...
		'TorqueTruth_B_Nm',tauTruth,'TorqueEstimate_B_Nm',tauEstimate, ...
		'TorqueDemand_B_Nm',tauDemand,'CompensatedDemand_B_Nm',compensatedDemand,'TorqueApplied_B_Nm',tauApplied, ...
		'ControlActive',active,'ObserverValid',observerValid,'EstimateValid',estimateValid, ...
		'HealthUsable',healthUsable,'HealthMode',healthMode,'CorrectionAge_s',correctionAge, ...
		'SolarFlux_W_m2',solarFlux,'Saturated',saturated);

	% 5. Summarize nonoverlapping 500 s windows, including any final partial window.
	edges = unique([0:500:stopTime_s,stopTime_s]);
	rows = struct([]);
	for k = 1:numel(edges)-1
		mask = t>=edges(k) & t<edges(k+1);
		if k==numel(edges)-1, mask = t>=edges(k) & t<=edges(k+1); end
		row = struct('Start_s',edges(k),'End_s',edges(k+1),'Samples',nnz(mask), ...
			'TrueRMS_deg',rmsValue(trueError(mask)), ...
			'TrueMax_deg',max(trueError(mask),[],'omitnan'), ...
			'EstimatedRMS_deg',rmsValue(estimatedError(mask)), ...
			'EstimationRMS_deg',rmsValue(estimationError(mask)), ...
			'RateRMS_deg_s',rad2deg(rmsValue(vecnorm(omegaError(mask,:),2,2))), ...
			'RateEstimationRMS_deg_s',rad2deg(rmsValue(vecnorm(omegaEstimated(mask,:)-omegaTrue(mask,:),2,2))), ...
			'TorqueErrorRMS_microNm',1e6*rmsValue(vecnorm(tauEstimate(mask,:)-tauTruth(mask,:),2,2)), ...
			'ControlActiveFraction',mean(active(mask),'omitnan'), ...
			'ObserverValidFraction',mean(observerValid(mask),'omitnan'), ...
			'EstimateValidFraction',mean(estimateValid(mask),'omitnan'), ...
			'HealthUsableFraction',mean(healthUsable(mask),'omitnan'), ...
			'MaxCorrectionAge_s',max(correctionAge(mask),[],'omitnan'), ...
			'SaturationActiveFraction',mean(saturated(mask & active>0.5),'omitnan'));
		if k==1, rows=row; else, rows(k)=row; end %#ok<AGROW>
	end
	report = struct2table(rows);
	writetable(report,fullfile(runDirectory,'windows_500s.csv'));
	save(fullfile(runDirectory,'diagnostics.mat'),'report','trace','wallTime_s');
	disp(report);

	% 6. Export readable diagnostic plots; raw data remains available if plotting fails.
	fig = figure('Name','Long nominal PD and default torque EKF','Color','w','DefaultAxesColor','w', ...
		'DefaultAxesXColor',[.15 .15 .15],'DefaultAxesYColor',[.15 .15 .15], ...
		'DefaultTextColor',[.15 .15 .15], ...
		'DefaultAxesColorOrder',[0 .447 .741;.85 .325 .098;.25 .55 .15], ...
		'Position',[80 80 1200 1100]);
	tiledlayout(6,1,'TileSpacing','compact');
	nexttile; plot(t,[trueError,estimatedError,estimationError]); grid on;
	ylabel('Attitude [deg]'); legend('True tracking','Estimated tracking','Estimation');
	nexttile; plot(t,rad2deg(omegaError)); grid on; ylabel('True rate error [deg/s]');
	nexttile; plot(t,1e6*tauTruth); hold on; plot(t,1e6*tauEstimate,'--'); grid on;
	ylabel('Disturbance [uNm]'); legend('Truth x','Truth y','Truth z','EKF x','EKF y','EKF z');
	nexttile; plot(t,1e6*(tauEstimate-tauTruth)); grid on; ylabel('EKF minus truth [uNm]');
	nexttile; plot(t,1e6*tauApplied); grid on; ylabel('Applied [uNm]');
	yline(1e6*AOCS.Control.MaxTorque_Nm,':'); yline(-1e6*AOCS.Control.MaxTorque_Nm,':');
	nexttile; stairs(t,[active,observerValid,estimateValid,healthUsable]); grid on;
	ylabel('Validity / activity'); xlabel('Time [s]');
	legend('Control active','Torque EKF valid','MEKF valid','Health usable');
	exportgraphics(fig,fullfile(runDirectory,'diagnostics.png'),'Resolution',160);
	if stopTime_s>=1000, analyze_long_pd_events(runDirectory); end
end

function values = sample(ts,width,t)
	data = double(loggedSignalMatrix(ts.Data,width,ts.Name));
	[times,index] = unique(ts.Time(:),'last');
	values = interp1(times,data(index,:),t,'previous',NaN);
end

function values = optionalLog(logs,name,width,t)
	values = nan(numel(t),width);
	if any(strcmp(logs.getElementNames,name))
		values = sample(logs.get(name).Values,width,t);
	end
end

function values = optionalOutput(out,name,width,t)
	values = nan(numel(t),width);
	if any(strcmp(out.who,name)), values = sample(out.get(name),width,t); end
end

function angle = attitudeAngle(q1,q2)
	q1 = q1./vecnorm(q1,2,2);
	q2 = q2./vecnorm(q2,2,2);
	angle = 2*acosd(min(1,abs(sum(q1.*q2,2))));
end

function value = rmsValue(values)
	value = sqrt(mean(values.^2,'omitnan'));
end

function closeIfOpened(model,wasLoaded)
	if ~wasLoaded && bdIsLoaded(model), close_system(model,0); end
end

function reportProgress(model)
	if bdIsLoaded(model)
		fprintf('Simulation progress: %.1f s\n',get_param(model,'SimulationTime'));
	end
end

function stopProgress(progress)
	stop(progress);
	delete(progress);
end
