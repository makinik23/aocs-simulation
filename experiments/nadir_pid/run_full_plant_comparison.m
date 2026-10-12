% Compare truth feedback and MEKF feedback on the full AOCS plant.

comparisonDuration_s = 6000;
comparisonAngles_deg = [5 30];

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root);
setupAocsPaths();
configureAocsFileGeneration();
requireDtm2020Native();
close(findall(groot,'Type','figure','Tag','FullPidComparison'));

% 1. Read the full scenario.
base = loadAocsSimulationConfig(fullfile(root,'config','scenarios','nadir_pointing.json'),root);
s = base.GNCConfig.AttitudeInitialization;
orbit = propagateOnboardSgp4([0;s.epoch_utc_jd;s.sgp4_semi_major_axis_m; ...
	s.sgp4_eccentricity;s.sgp4_inclination_deg;s.sgp4_raan_deg; ...
	s.sgp4_argument_of_periapsis_deg;s.sgp4_true_anomaly_deg;s.sgp4_bstar]);
assert(orbit(7)==1,'Initial SGP4 propagation failed.');
r = orbit(1:3);
h = cross(r,orbit(4:6));
z = -r/norm(r);
y = -h/norm(h);
x = cross(y,z);

qReference = dcm2quat([x.';y.';z.']);
wReference = [0;-norm(h)/dot(r,r);0];
tauExternal = [1;-1;1]*1e-6; % Constant body-frame torque [Nm].

% 2. Keep the saved plant untouched; close and remove only our temporary copy.
folder = tempname;
mkdir(folder);
model = 'aocs_pid_comparison';
assert(~bdIsLoaded(model),'Close aocs_pid_comparison before starting this experiment.');
cleanup = onCleanup(@() cleanupComparison(model,folder));
outputFolder = fullfile(root,'docs','data','nadir_pid_stability','full_plant', ...
	sprintf('duration_%gs',comparisonDuration_s));
if ~isfolder(outputFolder), mkdir(outputFolder); end
results = struct([]);
traces = struct([]);
caseIndex = 0;

try
for initialAngle = comparisonAngles_deg
	for idealFeedback = [true false]
		% 3. Use identical gains, initial states and external torque in both runs.
		qError = [cosd(initialAngle/2),sind(initialAngle/2)*ones(1,3)/sqrt(3)];
		raw = base.Raw;
		raw.initial_conditions.q_BI = quatmultiply(qReference,qError).';
		raw.initial_conditions.omega_BI_B_rad_s = quat2dcm(qError)*wReference;
		if isfield(raw.initial_conditions,'euler_BI_0_rad')
			raw.initial_conditions = rmfield(raw.initial_conditions,'euler_BI_0_rad');
		end
		raw.simulation.start_time_s = 0;
		raw.simulation.stop_time_s = comparisonDuration_s;
		raw.environment.disturbances.enabled = false;
		raw.gnc.attitude_control.use_integral = true;
		configFile = fullfile(folder,'scenario.json');
		writeAocsJson(configFile,raw);
		[AOCS,variables] = setupAocsSimulation(configFile,false);

		copyfile(fullfile(root,'models','aocs_plant.slx'),fullfile(folder,[model '.slx']));
		load_system(fullfile(folder,[model '.slx']));
		injectTorque(model,tauExternal);
		if idealFeedback
			useTruthFeedback(model);
		end

		% 4. Run the actual Guidance, sensors and MEKF in both configurations.
		input = Simulink.SimulationInput(model);
		names = fieldnames(variables);
		for j = 1:numel(names)
			input = input.setVariable(names{j},variables.(names{j}));
		end
		input = applyAocsSimulationSettings(model,AOCS,input);
		rng(1); % Repeatable MATLAB-side random initialization.
		out = sim(input);
		metrics = plot_nadir_pointing_results(out,AOCS,MakePlots=false);
		active = out.logsout.get('pid_control_active').Values;
		metrics.ControlActiveFraction = mean(double(active.Data(:)));
		demand = sampleLog(out.logsout,'tau_pid_demand_B_Nm',3,active.Time(:));
		mask = logical(active.Data(:));
		metrics.SaturationFraction = sum(any(abs(demand)>AOCS.Control.MaxTorque_Nm+1e-12,2) & mask)/max(1,sum(mask));

		% 5. Evaluate the PDF function using true tracking errors in both cases.
		t = metrics.Time_s;
		q = sampleLog(out.logsout,'q_be',4,t);
		qr = sampleLog(out.logsout,'q_ref_DI',4,t);
		w = sampleLog(out.logsout,'omega_b',3,t);
		wr = sampleLog(out.logsout,'omega_ref_DI_D_rad_s',3,t);
		xi = sampleLog(out.logsout,'pid_integral_acceleration_rad_s2',3,t);
		qe = quatmultiply(quatconj(qr),q);
		qe(qe(:,1)<0,:) = -qe(qe(:,1)<0,:);
		for j = 1:numel(t)
			w(j,:) = w(j,:) - (quat2dcm(qe(j,:))*wr(j,:).').';
		end
		lyapunov = pdfLyapunov(qe,w,xi,AOCS.Spacecraft.I_B,tauExternal,AOCS);
		tauIntegral = -(AOCS.Spacecraft.I_B*xi.').';
		caseIndex = caseIndex+1;
		mode = "MEKF";
		if idealFeedback, mode = "Truth"; end
		row = struct('Feedback',mode,'Initial_deg',initialAngle, ...
			'Final_deg',metrics.FinalError_deg,'TailRMS_deg',metrics.TailRmsError_deg, ...
			'ActiveFraction',metrics.ControlActiveFraction,'SaturationFraction',metrics.SaturationFraction, ...
			'FinalBalance_Nm',norm(tauIntegral(end,:).'+tauExternal), ...
			'FinalV',lyapunov.V(end),'MaxDeltaV',max(lyapunov.deltaV));
		if caseIndex==1, results=row; else, results(caseIndex)=row; end
		trace = struct('t',t,'angle',metrics.TrueError_deg, ...
			'estimation',metrics.EstimationError_deg,'tauIntegral',tauIntegral, ...
			'V',lyapunov.V,'deltaV',lyapunov.deltaV,'label',sprintf('%s, %g deg',mode,initialAngle));
		if caseIndex == 1
			traces = trace;
		else
			traces(caseIndex) = trace;
		end
		save(fullfile(outputFolder,sprintf('%s_%gdeg.mat',mode,initialAngle)), ...
			'AOCS','metrics','lyapunov','tauExternal','tauIntegral','t','row');
		close_system(model,0);
	end
end

catch problem
	clear cleanup;
	rethrow(problem);
end

% 6. Compare true pointing, torque compensation and the PDF Lyapunov function.
report = struct2table(results);
disp(report);
writetable(report,fullfile(outputFolder,'comparison.csv'));
save(fullfile(outputFolder,'comparison.mat'),'report','traces','tauExternal');
fig = figure('Name','Full AOCS plant: truth versus MEKF','Tag','FullPidComparison', ...
	'Color','w','Position',[100 80 1150 900]);
tiledlayout(4,1,'TileSpacing','compact');
for panel = 1:4
	nexttile;
	hold on;
	for j = 1:numel(traces)
		v = traces(j);
		switch panel
			case 1, plot(v.t,v.angle,'DisplayName',v.label);
			case 2, plot(v.t,1e6*vecnorm(v.tauIntegral+tauExternal.',2,2),'DisplayName',v.label);
			case 3, plot(v.t,v.V,'DisplayName',v.label);
			case 4, plot(v.t(1:end-1),v.deltaV,'DisplayName',v.label);
		end
	end
	grid on;
	labels = {'True attitude error [deg]','Torque imbalance [microNm]','V from PDF','Delta V from PDF'};
	ylabel(labels{panel});
	if panel < 4, set(gca,'YScale','log'); else, yline(0,':'); end
	if panel == 1, legend('Location','best'); end
end
xlabel('Time [s]');
exportgraphics(fig,fullfile(outputFolder,'comparison.png'),'Resolution',160);
clear cleanup;

function injectTorque(model,tau)
	% Add disturbance after the controller limit, directly at the plant input.
	ports = get_param([model '/Flight Dynamics'],'PortHandles');
	line = get_param(ports.Inport(1),'Line');
	source = get_param(line,'SrcPortHandle');
	delete_line(line);
	add_block('simulink/Math Operations/Sum',[model '/Test Torque Sum'],'Inputs','++');
	add_block('simulink/Sources/Constant',[model '/Test External Torque'], ...
		'Value',mat2str(tau,17),'VectorParams1D','off');
	p = get_param([model '/Test Torque Sum'],'PortHandles');
	add_line(model,source,p.Inport(1));
	add_line(model,'Test External Torque/1','Test Torque Sum/2');
	add_line(model,p.Outport,ports.Inport(1));
end

function useTruthFeedback(model)
	% Carry the true attitude bus explicitly into GNC; keep MEKF running.
	gnc = [model '/GNC'];
	add_block('simulink/Ports & Subsystems/In1',[gnc '/Test Truth']);
	add_line(model,'Flight Dynamics/1','GNC/1');
	add_block('simulink/Signal Routing/Bus Selector',[gnc '/Test Truth Fields'], ...
		'OutputSignals','AttitudeState.q_be,AttitudeState.omega_b');
	add_line(gnc,'Test Truth/1','Test Truth Fields/1');
	control = [gnc '/Attitude Control'];
	for j = 1:2
		name = sprintf('Test Sample %d',j);
		add_block('simulink/Discrete/Zero-Order Hold',[gnc '/' name], ...
			'SampleTime','AOCS.GNC.SampleTime_s');
		add_block('simulink/Ports & Subsystems/In1',[control '/' name], ...
			'Port',num2str(6+j));
		add_line(gnc,sprintf('Test Truth Fields/%d',j),[name '/1']);
		add_line(gnc,[name '/1'],sprintf('Attitude Control/%d',6+j));
	end

	% Override inside the function-call subsystem to preserve execution order.
	add_block('simulink/Signal Routing/Bus Assignment',[control '/Test Estimate Override'], ...
		'AssignedSignals','q_BI,omega_BI_B_rad_s');
	ports = get_param([control '/AttitudeEstimate shared'],'PortHandles');
	line = get_param(ports.Inport,'Line');
	source = get_param(line,'SrcPortHandle');
	delete_line(control,source,ports.Inport);
	p = get_param([control '/Test Estimate Override'],'PortHandles');
	add_line(control,source,p.Inport(1));
	add_line(control,'Test Sample 1/1','Test Estimate Override/2');
	add_line(control,'Test Sample 2/1','Test Estimate Override/3');
	add_line(control,p.Outport,ports.Inport);

	% Truth feedback does not wait for MEKF health; reference validity remains.
	block = [gnc '/Attitude Control/PID/Control Health'];
	ports = get_param(block,'PortHandles');
	line = get_param(ports.Outport,'Line');
	dest = get_param(line,'DstPortHandle');
	delete_line(line);
	add_block('simulink/Sinks/Terminator',[gnc '/Attitude Control/PID/Test Unused Health']);
	add_line([gnc '/Attitude Control/PID'],'Control Health/1','Test Unused Health/1');
	add_block('simulink/Sources/Constant',[gnc '/Attitude Control/PID/Test Truth Valid'], ...
		'Value','true','OutDataTypeStr','boolean');
	p = get_param([gnc '/Attitude Control/PID/Test Truth Valid'],'PortHandles');
	add_line([gnc '/Attitude Control/PID'],p.Outport,dest);
end

function data = sampleLog(logs,name,width,t)
	ts = logs.get(name).Values;
	if ts.IsTimeFirst, rows = reshape(ts.Data,numel(ts.Time),width);
	else, rows = reshape(ts.Data,width,numel(ts.Time)).'; end
	[times,idx] = unique(ts.Time,'last');
	data = interp1(times,rows(idx,:),t,'previous');
	assert(all(isfinite(data),'all'),'Nonfinite or missing samples in %s.',name);
end

function cleanupComparison(model,folder)
	if bdIsLoaded(model), close_system(model,0); end
	if isfolder(folder), rmdir(folder,'s'); end
end
