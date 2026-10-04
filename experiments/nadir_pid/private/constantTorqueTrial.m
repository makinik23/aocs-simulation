function report = constantTorqueTrial(model, aocs, P, root)
	% Match the proof: constant body torque, exact inertia, fixed reference.
	Ts = aocs.GNC.SampleTime_s;
	t = (0:Ts:6000).';
	J = aocs.Spacecraft.I_B;
	tauExternal = [1; -1; 1]*1e-6; % Nm, constant in the body frame.
	xiEquilibrium = J \ tauExternal;

	% Start aligned with zero rate and the controller's zero integral state.
	input = Simulink.SimulationInput(model);
	input = input.setVariable('AOCS', aocs, 'Workspace', model);
	input = input.setVariable('J', J, 'Workspace', model);
	input = input.setVariable('q0', [1;0;0;0], 'Workspace', model);
	input = input.setVariable('q_ref', [1;0;0;0], 'Workspace', model);
	input = input.setVariable('omega0', zeros(3,1), 'Workspace', model);
	input = input.setVariable('tau_ext', tauExternal, 'Workspace', model);
	input = input.setModelParameter('StopTime', '6000');
	output = sim(input);

	% Compare the integral torque with the required balancing torque.
	q = sampledRows(output.logsout, 'q_error', 4, t);
	w = sampledRows(output.logsout, 'omega_error_B_rad_s', 3, t);
	xi = sampledRows(output.logsout, 'pid_integral_acceleration_rad_s2', 3, t);
	demand = sampledRows(output.logsout, 'tau_pid_demand_B_Nm', 3, t);
	tauIntegral = -(J*xi.').';
	balance = tauIntegral + tauExternal.';
	angle = 2*atan2d(vecnorm(q(:,2:4),2,2), abs(q(:,1)));
	rate = rad2deg(vecnorm(w,2,2));

	% Evaluate V around the disturbed equilibrium, not around xi = 0.
	X = [2*q(:,2:4), w, xi-xiEquilibrium.'].';
	V = sum(X.*(P*X),1);
	deltaV = diff(V);
	tail = t >= t(end)-60;
	tailAngle = max(angle(tail));
	tailBalance = max(vecnorm(balance(tail,:),2,2));
	peakDemand = max(abs(demand),[],'all');

	report = table(tailAngle, max(rate(tail)), tailBalance, peakDemand, max(deltaV), ...
		'VariableNames', {'Tail60_max_angle_deg','Tail60_max_rate_deg_s', ...
		'Tail60_max_balance_Nm','Peak_demand_Nm','Max_shifted_delta_V'});
	folder = fullfile(root,'docs','data','nadir_pid_stability');
	if ~isfolder(folder)
		mkdir(folder);
	end

	writetable(report,fullfile(folder,'constant_torque.csv'));
	save(fullfile(folder,'constant_torque.mat'), 'report','t','q','w','xi', ...
		'tauExternal','tauIntegral','balance','xiEquilibrium','V','deltaV','P','aocs');

	figure('Name','Constant external torque rejection','Tag','NadirConstant');
	subplot(4,1,1);
	plot(t,angle);
	grid on;
	ylabel('Attitude error [deg]');

	subplot(4,1,2);
	plot(t,1e6*tauIntegral);
	hold on;
	plot(t,repmat(-1e6*tauExternal.',numel(t),1),'--');
	grid on;
	ylabel('Integral torque [microNm]');
	legend('X','Y','Z','Target X','Target Y','Target Z');

	subplot(4,1,3);
	semilogy(t,max(V,realmin));
	grid on;
	ylabel('Shifted V');

	subplot(4,1,4);
	plot(t(1:end-1),deltaV);
	yline(0, ':');
	grid on;
	ylabel('Delta V');
	xlabel('Time [s]');

	% These tolerances check this trajectory, not global stability.
	assert(peakDemand < aocs.Control.MaxTorque_Nm, 'The proof comparison must remain unsaturated.');
	assert(max(deltaV) <= 1e-12*max(1,max(V)), 'Shifted V increases beyond numerical tolerance.');
	assert(tailAngle < 1e-3, 'The attitude error has not settled below 0.001 deg.');
	assert(tailBalance < 1e-9, 'The integral torque has not balanced the disturbance to 1 nNm.');
end
