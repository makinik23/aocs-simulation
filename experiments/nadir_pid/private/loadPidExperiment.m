function [model, root, Ts, aocs, Ad, P, rho, relativeResidual] = loadPidExperiment
% 1. Load the saved model and read the controller parameters.

model = 'nadir_pid_testbench';
root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
load_system(fullfile(root, 'models', [model '.slx']));

ws = get_param(model, 'ModelWorkspace');
Ts = ws.getVariable('Ts');
aocs = ws.getVariable('AOCS');

% 2. Linearize the nominal, unsaturated loop around a fixed reference.
assert(abs(Ts - aocs.GNC.SampleTime_s) < 1e-12, ...
	'Controller sampling and integral update must use the same Ts.');
assert(norm(ws.getVariable('J') - aocs.Spacecraft.I_B, 'fro') < 1e-12, ...
	'Plant and controller inertia must match for this linearization.');

kp = aocs.Control.PIDKp_s2;
kd = aocs.Control.PIDKd_s;
ki = aocs.Control.Ki_s3;
I = eye(3);
Z = zeros(3);

% Sampled state: x(k+1) = Ad*x(k), with x = [2*epsilon; rate error; integral].
Ad = [
	(1 - 0.5*Ts^2*kp)*I, (Ts - 0.5*Ts^2*kd)*I, -0.5*Ts^2*I;
	-Ts*kp*I,            (1 - Ts*kd)*I,         -Ts*I;
	Ts*ki*I,             Z,                     I
];

% 3. Check the poles and solve the discrete Lyapunov equation.
lambda = eig(Ad);
rho = max(abs(lambda));
assert(rho < 1, 'The linearized closed loop is not asymptotically stable.');

Q = eye(size(Ad));
P = dlyap(Ad', Q);
P = (P + P') / 2;
[~, flag] = chol(P);
assert(flag == 0, 'P must be positive definite.');
relativeResidual = norm(Ad' * P * Ad - P + Q, 'fro') / norm(Q, 'fro');


aocs.Control.Enabled = true;
aocs.Control.UseIntegral = true;
end
