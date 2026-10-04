close(findall(groot,'Type','figure','Tag','NadirLinear'));
[model, root, Ts, aocs, Ad, P, rho, relativeResidual] = loadPidExperiment;

% 1. Simulate a small initial error with the unchanged nonlinear PID.
angle0 = deg2rad(0.1);
q0 = [cos(angle0/2); sin(angle0/2); 0; 0];

aocs.Control.Enabled = true;
aocs.Control.UseIntegral = true;

in = Simulink.SimulationInput(model);
in = in.setVariable('AOCS', aocs, 'Workspace', model);
in = in.setVariable('q0', q0, 'Workspace', model);
in = in.setVariable('q_ref', [1; 0; 0; 0], 'Workspace', model);
in = in.setVariable('omega0', zeros(3,1), 'Workspace', model);
in = in.setVariable('tau_ext', zeros(3,1), 'Workspace', model);
in = in.setModelParameter('StopTime', '600');

out = sim(in);

% 2. Read the error and integral state at the controller sample times.
t = (0:Ts:600).';
q = sampledRows(out.logsout, 'q_error', 4, t);
w = sampledRows(out.logsout, 'omega_error_B_rad_s', 3, t);
xi = sampledRows(out.logsout, 'pid_integral_acceleration_rad_s2', 3, t);

% State order: [2*epsilon; rate error; integral state].
Xsim = [2*q(:,2:4), w, xi].';

% 3. Propagate the linear model from the same initial state.
Xlin = zeros(size(Xsim));
Xlin(:,1) = Xsim(:,1);

for k = 1:numel(t)-1
	Xlin(:,k+1) = Ad * Xlin(:,k);
end

figure('Name','Linear PID comparison','Tag','NadirLinear');
plot(t, rad2deg(Xsim(1,:)), ...
	t, rad2deg(Xlin(1,:)), '--');

grid on;
xlabel('Time [s]');
ylabel('First attitude-error component [deg]');
legend('Simulink: 2\epsilon_1', 'Linear model', ...
	'Location', 'best');

% 4. Compare the Lyapunov value and its change between samples.
Vsim = sum(Xsim .* (P * Xsim), 1);
Vlin = sum(Xlin .* (P * Xlin), 1);

dVsim = diff(Vsim);
dVlin = diff(Vlin);

figure('Name','Discrete Lyapunov comparison','Tag','NadirLinear');

subplot(2,1,1);
plot(t, Vsim, t, Vlin, '--');
grid on;
ylabel('V');
legend('Simulink', 'Linear model');

subplot(2,1,2);
plot(t(1:end-1), dVsim, ...
	t(1:end-1), dVlin, '--');
yline(0, ':');
grid on;
xlabel('Time [s]');
ylabel('\Delta V');

% 5. Report the nominal comparison; this is not a global stability proof.
maxAngleDifference_deg = max(vecnorm(Xsim(1:3,:) - Xlin(1:3,:), 2, 1)) * 180/pi;
torque = sampledRows(out.logsout, 'tau_applied_B_Nm', 3, t);
peakTorque_Nm = max(abs(torque), [], 'all');
assert(peakTorque_Nm < aocs.Control.MaxTorque_Nm, 'Torque reaches the limit: reduce the initial error for the local comparison.');

fprintf('Maximum eigenvalue magnitude: %.9f\n', rho);
fprintf('Lyapunov relative residual: %.3e\n', relativeResidual);
fprintf('Maximum attitude-component difference: %.3e deg\n', maxAngleDifference_deg);
fprintf('Maximum simulated Delta V: %.3e\n', max(dVsim));
fprintf('Peak applied torque: %.3e Nm (limit %.3e Nm)\n', peakTorque_Nm, aocs.Control.MaxTorque_Nm);

