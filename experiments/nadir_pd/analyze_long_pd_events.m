function report = analyze_long_pd_events(runDirectory)
%ANALYZE_LONG_PD_EVENTS Inspect an archived run without rerunning the plant.
	archive = load(fullfile(runDirectory,'raw_simulation.mat'),'out','AOCS');
	out = archive.out;
	logs = out.logsout;
	ref = logs.get('q_ref_DI').Values;
	t = ref.Time(:);
	estimate = logs.get('AttitudeEstimate').Values;
	qr = sample(ref,4,t);
	qt = sample(logs.get('q_be').Values,4,t);
	qe = sample(estimate.q_BI,4,t);
	truth = angleError(qt,qr);
	estimated = angleError(qe,qr);
	estimation = angleError(qe,qt);
	% Check the commanded reference rate against quaternion increments.
	omegaReference = sample(logs.get('omega_ref_DI_D_rad_s').Values,3,t);
	deltaQ = quatmultiply(quatconj(qr(1:end-1,:)),qr(2:end,:));
	deltaQ(deltaQ(:,1)<0,:) = -deltaQ(deltaQ(:,1)<0,:);
	omegaFromQuaternion = 2*deltaQ(:,2:4)./diff(t); % Small increments at Ts=0.1 s.
	referenceRateMismatch = omegaFromQuaternion-0.5*(omegaReference(1:end-1,:)+omegaReference(2:end,:));
	rateTime = 0.5*(t(1:end-1)+t(2:end));
	available = string(logs.getElementNames);
	requested = ["solar_flux_shadowed_W_m2","Sun_Vector_Update_Accepted", ...
		"Sun_Vector_Update_NIS","Magnetometer_Update_Accepted", ...
		"Magnetometer_Update_NIS","M_dist_B_Nm","r_I_m"];
	missing = requested(~ismember(requested,available));
	flux = readLog(logs,requested(1),1,t);
	sunAccepted = readLog(logs,requested(2),1,t);
	sunNIS = readLog(logs,requested(3),1,t);
	magAccepted = readLog(logs,requested(4),1,t);
	magNIS = readLog(logs,requested(5),1,t);
	tauTruth = readLog(logs,requested(6),3,t);
	% +Z axis pointing excludes rotation about +Z; use actual logged orbit truth.
	rInertial = readLog(logs,'r_I_m',3,t);
	nadirInertial = -rInertial./vecnorm(rInertial,2,2);
	zBodyInertial = inertialZ(qt);
	zEstimatedInertial = inertialZ(qe);
	zReferenceInertial = inertialZ(qr);
	physicalNadir = axisAngle(zBodyInertial,nadirInertial);
	estimatedNadir = axisAngle(zEstimatedInertial,nadirInertial);
	referenceNadir = axisAngle(zReferenceInertial,nadirInertial);
	tauEstimate = nan(numel(t),3);
	observerValid = nan(size(t));
	outputs = string(out.who);
	if ismember("DisturbanceObserverTorque",outputs)
		tauEstimate = sample(out.get('DisturbanceObserverTorque'),3,t);
	else
		missing(end+1) = "DisturbanceObserverTorque";
	end
	if ismember("EKFObserverValid",outputs)
		observerValid = sample(out.get('EKFObserverValid'),1,t);
	else
		missing(end+1) = "EKFObserverValid";
	end
	health = logs.get('AttitudeHealth').Values;
	usable = sample(health.control_usable,1,t);
	mode = sample(health.mode_id,1,t);
	correctionAge = sample(health.correction_age_s,1,t);
	active = readLog(logs,'control_active',1,t);
	torqueResidual = vecnorm(tauEstimate-tauTruth,2,2);
	lit = flux>1 & isfinite(flux);
	dark = flux<=1 & isfinite(flux);
	tail = t>=1000;

	% Reconstruct references on a 30 s grid, independently of the logged plant field.
	geometry = analyzeReferenceGeometry(out,archive.AOCS,runDirectory);

	% 1. Compare steady-run errors by inferred illumination state.
	labels = ["Tail_all","Tail_lit","Tail_dark"];
	masks = {tail,tail & lit,tail & dark};
	rows = cell(3,1);
	for k = 1:3
		m = masks{k};
		rateMask = m(1:end-1) & m(2:end);
		rows{k} = struct('Subset',labels(k),'Samples',nnz(m), ...
			'TrueRMS_deg',rmsValue(truth(m)),'TrueMax_deg',maxValue(truth(m)), ...
			'EstimatedRMS_deg',rmsValue(estimated(m)), ...
			'EstimationRMS_deg',rmsValue(estimation(m)), ...
			'ReferenceRateMismatchRMS_deg_s',rad2deg(rmsValue(vecnorm(referenceRateMismatch(rateMask,:),2,2))), ...
			'TrueZPhysicalNadirRMS_deg',rmsValue(physicalNadir(m)), ...
			'TrueZPhysicalNadirMax_deg',maxValue(physicalNadir(m)), ...
			'EstimatedZPhysicalNadirRMS_deg',rmsValue(estimatedNadir(m)), ...
			'EstimatedZPhysicalNadirMax_deg',maxValue(estimatedNadir(m)), ...
			'ReferenceZPhysicalNadirRMS_deg',rmsValue(referenceNadir(m)), ...
			'ReferenceZPhysicalNadirMax_deg',maxValue(referenceNadir(m)), ...
			'SunUpdateAcceptedFraction',mean(sunAccepted(m),'omitnan'), ...
			'MagneticUpdateAcceptedFraction',mean(magAccepted(m),'omitnan'), ...
			'HealthUsableFraction',mean(usable(m),'omitnan'), ...
			'ControlActiveFraction',mean(active(m),'omitnan'), ...
			'ObserverValidFraction',mean(observerValid(m),'omitnan'), ...
			'MaxCorrectionAge_s',maxValue(correctionAge(m)), ...
			'TorqueResidualRMS_microNm',1e6*rmsValue(torqueResidual(m)), ...
			'CorrTrueEstimation',correlation(truth(m),estimation(m)), ...
			'CorrTrueTorqueResidual',correlation(truth(m),torqueResidual(m)));
	end
	statistics = struct2table(vertcat(rows{:}));

	% 2. Locate the worst sample within each nonoverlapping 500 s window.
	edges = unique([0:500:t(end),t(end)]);
	rows = cell(numel(edges)-1,1);
	for k = 1:numel(rows)
		m = t>=edges(k) & t<edges(k+1);
		if k==numel(rows), m = t>=edges(k) & t<=edges(k+1); end
		indices = find(m);
		[peak,j] = max(truth(indices));
		p = indices(j);
		rows{k} = struct('Start_s',edges(k),'End_s',edges(k+1), ...
			'PeakTime_s',t(p),'TruePeak_deg',peak,'EstimatedAtPeak_deg',estimated(p), ...
			'EstimationAtPeak_deg',estimation(p),'SolarFluxAtPeak_W_m2',flux(p), ...
			'TorqueResidualAtPeak_microNm',1e6*torqueResidual(p), ...
			'HealthModeAtPeak',mode(p),'ControlActiveAtPeak',active(p));
	end
	worstWindows = struct2table(vertcat(rows{:}));

	% 3. Group contiguous excursions above 1 deg after the 1000 s startup cutoff.
	above = tail & truth>1;
	starts = find(diff([false;above])==1);
	stops = find(diff([above;false])==-1);
	rows = cell(numel(starts),1);
	for k = 1:numel(starts)
		indices = starts(k):stops(k);
		[peak,j] = max(truth(indices));
		p = indices(j);
		rows{k} = struct('Start_s',t(starts(k)),'End_s',t(stops(k)), ...
			'Duration_s',t(stops(k))-t(starts(k)),'PeakTime_s',t(p), ...
			'TruePeak_deg',peak,'EstimatedAtPeak_deg',estimated(p), ...
			'EstimationAtPeak_deg',estimation(p),'SolarFluxAtPeak_W_m2',flux(p));
	end
	if isempty(rows)
		episodes = array2table(zeros(0,8),'VariableNames',{'Start_s','End_s', ...
			'Duration_s','PeakTime_s','TruePeak_deg','EstimatedAtPeak_deg', ...
			'EstimationAtPeak_deg','SolarFluxAtPeak_W_m2'});
	else
		episodes = struct2table(vertcat(rows{:}));
	end

	% 4. Record observable gates; validity transitions are not reset evidence.
	healthDrop = find(diff(usable>0.5)==-1)+1;
	observerDrop = find(diff(observerValid>0.5)==-1)+1;
	transitionTimes = [t(healthDrop);t(observerDrop)];
	transitionTypes = [repmat("Health gate disabled",numel(healthDrop),1); ...
		repmat("Torque EKF invalid",numel(observerDrop),1)];
	transitions = table(transitionTimes,transitionTypes,'VariableNames',{'Time_s','Event'});
	transitions = sortrows(transitions,'Time_s');
	resetLog = "observer_reset";
	resetTimes_s = [];
	if ismember(resetLog,available)
		reset = readLog(logs,resetLog,1,t);
		resetTimes_s = t(find(diff([false;reset>0.5])==1));
	else
		missing(end+1) = resetLog;
	end
	report = struct('Statistics',statistics,'WorstWindows',worstWindows, ...
		'EpisodesAbove1deg',episodes,'GateTransitions',transitions, ...
		'ObserverResetTimes_s',resetTimes_s,'MissingLogs',missing, ...
		'AvailableLogs',available,'ReferenceGeometry',geometry,'IlluminationThreshold_W_m2',1,'TailStart_s',1000);
	trace = struct('Time_s',t,'TrueError_deg',truth,'EstimatedError_deg',estimated, ...
		'EstimationError_deg',estimation,'SolarFlux_W_m2',flux, ...
		'TrueZPhysicalNadir_deg',physicalNadir,'EstimatedZPhysicalNadir_deg',estimatedNadir, ...
		'ReferenceZPhysicalNadir_deg',referenceNadir,'Position_I_m',rInertial, ...
		'SunAccepted',sunAccepted,'SunNIS',sunNIS,'MagneticAccepted',magAccepted, ...
		'MagneticNIS',magNIS,'HealthUsable',usable,'HealthMode',mode, ...
		'ObserverValid',observerValid,'TorqueResidual_Nm',torqueResidual, ...
		'ReferenceRateTime_s',rateTime,'ReferenceRateMismatch_D_rad_s',referenceRateMismatch);
	writetable(statistics,fullfile(runDirectory,'event_tail_statistics.csv'));
	writetable(worstWindows,fullfile(runDirectory,'event_worst_500s.csv'));
	writetable(episodes,fullfile(runDirectory,'event_episodes_above_1deg.csv'));
	writetable(transitions,fullfile(runDirectory,'event_gate_transitions.csv'));
	save(fullfile(runDirectory,'event_diagnostics.mat'),'report','trace');
	disp(statistics);
	fprintf('Missing logs: %s\n',strjoin(missing,', '));
	fprintf('Observer reset times require an explicit reset log; gate changes alone do not prove resets.\n');

	% 5. Show event timing alongside illumination and correction acceptance.
	fig = figure('Name','Archived long PD event diagnosis','Color','w','DefaultAxesColor','w', ...
		'DefaultAxesXColor',[.15 .15 .15],'DefaultAxesYColor',[.15 .15 .15], ...
		'DefaultTextColor',[.15 .15 .15], ...
		'DefaultAxesColorOrder',[0 .447 .741;.85 .325 .098;.25 .55 .15], ...
		'Position',[100 80 1100 850]);
	tiledlayout(4,1,'TileSpacing','compact');
	nexttile; plot(t,[truth,estimated,estimation]); yline(1,':'); grid on;
	ylabel('Angle [deg]'); legend('True tracking','Estimated tracking','Estimation','Location','northoutside','Orientation','horizontal');
	nexttile; plot(t,flux); grid on; ylabel('Shadowed flux [W/m^2]');
	nexttile; stairs(t,[sunAccepted,magAccepted,usable,observerValid]); grid on;
	ylabel('Accepted / usable'); legend('Sun update','Magnetic update','Health gate','Torque EKF');
	nexttile; plot(t,1e6*torqueResidual); grid on;
	ylabel('Torque residual [uNm]'); xlabel('Time [s]');
	set(findall(fig,'Type','legend'),'Color','w','TextColor',[.15 .15 .15]);
	exportgraphics(fig,fullfile(runDirectory,'event_diagnostics.png'),'Resolution',160);
	physicalFig = figure('Name','Physical nadir versus onboard reference','Color','w','DefaultAxesColor','w', ...
		'DefaultAxesXColor',[.15 .15 .15],'DefaultAxesYColor',[.15 .15 .15], ...
		'DefaultTextColor',[.15 .15 .15], ...
		'DefaultAxesColorOrder',[0 .447 .741;.85 .325 .098;.25 .55 .15], ...
		'Position',[100 80 1100 850]);
	tiledlayout(3,1,'TileSpacing','compact');
	nexttile; plot(t,[physicalNadir,estimatedNadir]); grid on;
	ylabel('+Z to physical nadir [deg]'); legend('True body +Z','Estimated body +Z','Location','northoutside','Orientation','horizontal');
	title('Axis pointing relative to -r_I/||r_I||; roll about +Z is excluded');
	nexttile; plot(t,referenceNadir); grid on;
	ylabel('Reference +Z to nadir [deg]'); xlabel('Time [s]');
	nexttile; plot(rateTime,rad2deg(referenceRateMismatch)); grid on;
	ylabel('Reference rate mismatch [deg/s]'); xlabel('Time [s]');
	set(findall(physicalFig,'Type','legend'),'Color','w','TextColor',[.15 .15 .15]);
	exportgraphics(physicalFig,fullfile(runDirectory,'event_physical_nadir.png'),'Resolution',160);
end

function values = sample(ts,width,t)
	data = double(loggedSignalMatrix(ts.Data,width,ts.Name));
	[times,index] = unique(ts.Time(:),'last');
	values = interp1(times,data(index,:),t,'previous',NaN);
end

function values = readLog(logs,name,width,t)
	values = nan(numel(t),width);
	if any(strcmp(logs.getElementNames,name)), values = sample(logs.get(char(name)).Values,width,t); end
end

function angle = angleError(q1,q2)
	q1 = q1./vecnorm(q1,2,2);
	q2 = q2./vecnorm(q2,2,2);
	angle = 2*acosd(min(1,abs(sum(q1.*q2,2))));
end

function value = rmsValue(x)
	value = sqrt(mean(x.^2,'omitnan'));
end

function value = maxValue(x)
	value = NaN;
	if ~isempty(x), value = max(x,[],'omitnan'); end
end

function value = correlation(x,y)
	valid = isfinite(x) & isfinite(y);
	value = NaN;
	if nnz(valid)>2 && std(x(valid))>0 && std(y(valid))>0
		matrix = corrcoef(x(valid),y(valid));
		value = matrix(1,2);
	end
end

function zInertial = inertialZ(q)
	% C_FI maps I to frame F; C_FI row 3 is +Z_F expressed in I.
	q = q./vecnorm(q,2,2);
	zInertial = nan(size(q,1),3);
	for k = 1:size(q,1)
		if all(isfinite(q(k,:)))
			C_FI = quat2dcm(q(k,:));
			zInertial(k,:) = C_FI(3,:);
		end
	end
end

function angle = axisAngle(a,b)
	dotProduct = sum(a.*b,2);
	angle = acosd(max(-1,min(1,dotProduct)));
	angle(~all(isfinite(a),2) | ~all(isfinite(b),2)) = NaN;
end
