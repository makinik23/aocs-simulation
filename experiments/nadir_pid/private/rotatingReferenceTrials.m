function report = rotatingReferenceTrials(sourceModel, aocs, P, root, useNadir)
	if nargin < 5
		useNadir = false;
	end
	% Work on a temporary copy; preserve the saved model and PID layout.
	folder = tempname;
	mkdir(folder);
	model = 'nadir_pid_stability_run';
	assert(~bdIsLoaded(model), 'Close nadir_pid_stability_run before running this script.');
	cleanup = onCleanup(@() removeTemporaryModel(model, folder));
	copyfile(get_param(sourceModel, 'FileName'), fullfile(folder, [model '.slx']));
	load_system(fullfile(folder, [model '.slx']));

	% Replace only the three reference sources in the temporary model.
	names = {'Quaternion', 'Omega', 'Alpha'};
	variables = {'referenceQuaternion', 'referenceRate', 'referenceAcceleration'};
	for k = 1:3
		block = [model '/Reference/' names{k}];
		position = get_param(block, 'Position');
		ports = get_param(block, 'PortHandles');
		line = get_param(ports.Outport, 'Line');
		destination = get_param(line, 'DstPortHandle');
		delete_line(line);
		delete_block(block);
		add_block('simulink/Sources/From Workspace', block, ...
			'VariableName', variables{k}, 'SampleTime', 'Ts', ...
			'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value', ...
			'Position', position);
		ports = get_param(block, 'PortHandles');
		add_line([model '/Reference'], ports.Outport, destination);
	end

	% Reference: smooth rotation about a mixed axis, with nonzero acceleration.
	Ts = aocs.GNC.SampleTime_s;
	t = (0:Ts:3000).';
	axis = [1;2;3] / sqrt(14);
	rate0 = 1.1e-3;                  % rad/s; illustrative orbital-rate scale.
	frequency = 2*pi/600;           % rad/s; modulation period is 600 s.
	phase = rate0*(t + 0.2/frequency*(1-cos(frequency*t)));
	qReference = [cos(phase/2), sin(phase/2)*axis.'];
	wReference = rate0*(1 + 0.2*sin(frequency*t))*axis.';
	aReference = 0.2*rate0*frequency*cos(frequency*t)*axis.';

	tauExternal = zeros(3,1);
	if useNadir
		% Reproduce the current Guidance formulas using the scenario orbit.
		addpath(root);
		setupAocsPaths();
		config = loadAocsSimulationConfig(fullfile(root,'config','scenarios','nadir_pointing.json'),root);
		orbit = config.GNCConfig.AttitudeInitialization;
		t = (0:Ts:6000).';
		[rOrbit,vOrbit] = propagateOrbit(orbit.epoch_utc_jd+t.'/86400, ...
			orbit.sgp4_semi_major_axis_m,orbit.sgp4_eccentricity, ...
			orbit.sgp4_inclination_deg,orbit.sgp4_raan_deg, ...
			orbit.sgp4_argument_of_periapsis_deg,orbit.sgp4_true_anomaly_deg, ...
			Epoch=orbit.epoch_utc_jd,PropModel="sgp4",BStar=orbit.sgp4_bstar, ...
			OutputCoordinateFrame="inertial");
		rOrbit = reshape(rOrbit,3,[]);
		vOrbit = reshape(vOrbit,3,[]);
		qReference = zeros(numel(t),4);
		wReference = zeros(numel(t),3);
		for j = 1:numel(t)
			r = rOrbit(:,j);
			h = cross(r,vOrbit(:,j));
			z = -r/norm(r);
			y = -h/norm(h);
			x = cross(y,z);
			qReference(j,:) = dcm2quat([x.';y.';z.']);
			if j > 1 && dot(qReference(j,:),qReference(j-1,:)) < 0
				qReference(j,:) = -qReference(j,:);
			end
			wReference(j,:) = [0,-norm(h)/dot(r,r),0];
		end
		aReference = [zeros(1,3);diff(wReference)/Ts];
		tauExternal = [1;-1;1]*1e-6;
	end

	% q_DI, omega_DI^D and its component derivative use the same trajectory.
	referenceQuaternion = timeseries(qReference, t);
	referenceRate = timeseries(wReference, t);
	referenceAcceleration = timeseries(aReference, t);

	% Uniform uncertainty plus two physically admissible anisotropic cases.
	labels = ["nominal", "minus05", "plus05", "minus10", "plus10", ...
		"minus20", "plus20", "axis_mix_A", "axis_mix_B"];
	factors = [1 1 1; .95 .95 .95; 1.05 1.05 1.05; .9 .9 .9; ...
		1.1 1.1 1.1; .8 .8 .8; 1.2 1.2 1.2; 1.05 .95 1; .95 1.05 1];
	Jcontroller = aocs.Spacecraft.I_B;
	assert(norm(Jcontroller-diag(diag(Jcontroller)), 'fro') < 1e-12, ...
		'These uncertainty cases assume diagonal nominal inertia.');
	initialAngles = [5 30];
	initialRateError = deg2rad([0.1;-0.1;0.1]);
	results = struct([]);
	traces = struct([]);
	caseIndex = 0;

	% First check tracking from zero error, then test acquisition and uncertainty.
	variants = 0:numel(labels);
	if useNadir
		variants = 1;
		initialRateError = zeros(3,1);
	end

	for variant = variants
		if variant == 0
			angles = 0;
			factor = [1 1 1];
			label = "tracking_check";
		else
			angles = initialAngles;
			factor = factors(variant,:);
			label = labels(variant);
		end
		Jplant = diag(diag(Jcontroller).*factor.');
		moments = eig(Jplant);
		assert(all(moments>0) && 2*max(moments)<=sum(moments), ...
			'Plant inertia must satisfy the principal-moment triangle inequality.');

		for initialAngle = angles
			caseIndex = caseIndex + 1;
			q0 = [cosd(initialAngle/2); sind(initialAngle/2)*ones(3,1)/sqrt(3)];
			omega0 = quat2dcm(q0.')*wReference(1,:).';
			q0 = quatmultiply(qReference(1,:),q0.').';
			if variant ~= 0
				omega0 = omega0 + initialRateError;
			end

			% Change plant inertia only; the controller keeps its nominal J.
			input = Simulink.SimulationInput(model);
			input = input.setVariable('AOCS', aocs, 'Workspace', model);
			input = input.setVariable('J', Jplant, 'Workspace', model);
			input = input.setVariable('q0', q0, 'Workspace', model);
			input = input.setVariable('omega0', omega0, 'Workspace', model);
			input = input.setVariable('tau_ext', tauExternal, 'Workspace', model);
			input = input.setVariable('referenceQuaternion', referenceQuaternion, 'Workspace', model);
			input = input.setVariable('referenceRate', referenceRate, 'Workspace', model);
			input = input.setVariable('referenceAcceleration', referenceAcceleration, 'Workspace', model);
			input = input.setModelParameter('StopTime', num2str(t(end)));
			output = sim(input);

			q = sampledRows(output.logsout, 'q_error', 4, t);
			w = sampledRows(output.logsout, 'omega_error_B_rad_s', 3, t);
			xi = sampledRows(output.logsout, 'pid_integral_acceleration_rad_s2', 3, t);
			demand = sampledRows(output.logsout, 'tau_pid_demand_B_Nm', 3, t);
			torque = sampledRows(output.logsout, 'tau_applied_B_Nm', 3, t);
			angle = 2*atan2d(vecnorm(q(:,2:4),2,2), abs(q(:,1)));
			rate = rad2deg(vecnorm(w,2,2));

			% The nominal P is a diagnostic here, not a robustness certificate.
			X = [2*q(:,2:4), w, xi-(Jcontroller\tauExternal).'].';
			V = sum(X.*(P*X),1);
			dV = diff(V);
			saturated = any(abs(demand)>aocs.Control.MaxTorque_Nm*(1+1e-9),2);
			lastOutside = find(angle>0.1 | rate>0.01,1,'last');
			settling = NaN;
			if isempty(lastOutside)
				settling = 0;
			elseif lastOutside < numel(t)
				settling = t(lastOutside+1);
			end

			row = struct('Variant',label,'Initial_deg',initialAngle, ...
				'Jx_factor',factor(1),'Jy_factor',factor(2),'Jz_factor',factor(3), ...
				'Final_angle_deg',angle(end),'Tail60_RMS_deg',sqrt(mean(angle(t>=t(end)-60).^2)), ...
				'Final_rate_deg_s',rate(end),'Peak_angle_deg',max(angle), ...
				'Settling_s',settling,'Saturation_s',sum(saturated(1:end-1))*Ts, ...
				'Peak_torque_Nm',max(abs(torque),[],'all'),'Max_delta_V',max(dV), ...
				'Positive_delta_V_steps',sum(dV>1e-12*max(1,max(V))), ...
				'Final_balance_Nm',norm(-Jcontroller*xi(end,:).'+tauExternal));
			if caseIndex == 1
				results = row;
			else
				results(caseIndex) = row;
			end

			traces(caseIndex).angle_deg = angle;
			traces(caseIndex).rate_deg_s = rate;
			traces(caseIndex).V = V;
			traces(caseIndex).tauIntegral = -(Jcontroller*xi.').';
			traces(caseIndex).pdf = pdfLyapunov(q,w,xi,Jcontroller,tauExternal,aocs);
			traces(caseIndex).initialAngle_deg = initialAngle;
			fprintf('%02d %-14s %2g deg: final %.5g deg, settle %.1f s, dV+ %d\n', ...
				caseIndex,label,initialAngle,angle(end),settling,row.Positive_delta_V_steps);
		end
	end

	% Save the scenarios, unchanged gains and results for later comparison.
	report = struct2table(results);
	outputFolder = fullfile(root,'docs','data','nadir_pid_stability');
	if ~isfolder(outputFolder)
		mkdir(outputFolder);
	end

	stem = 'rotating_reference_inertia';
	if useNadir
		stem = 'nadir_constant_torque';
	end
	writetable(report,fullfile(outputFolder,[stem '.csv']));
	save(fullfile(outputFolder,[stem '.mat']), ...
		'report','traces','t','aocs','P','factors','initialRateError', ...
		'qReference','wReference','aReference','rate0','frequency','axis','tauExternal','Jcontroller');

	if useNadir
		plotNadirTrial(t,traces,tauExternal,outputFolder);
	else
		figure('Name','Inertia uncertainty','Tag','NadirInertia');
		hold on;
		for k = 1:numel(traces)
			plot(t,traces(k).angle_deg);
		end
		yline(0.1,'--');
		grid on;
		xlabel('Time [s]');
		ylabel('Attitude error [deg]');
	end
end
