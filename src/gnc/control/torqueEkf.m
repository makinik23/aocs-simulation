function output = torqueEkf(input)
%TORQUEEKF Estimate body disturbance torque using an EKF.
	input = input(:);
	p = input(51:66);
	cfg = struct('Enabled',logical(p(11)),'TorqueRW_Nm2_s',p(12), ...
		'RateSigma_rad_s',p(13),'InitialTorqueSigma_Nm',p(14), ...
		'NISLimit',p(15),'MaxEstimate_Nm',p(16));
	J = reshape(p(1:9),3,3);
	Ts = p(10);
	state = input(8:50);
	output = zeros(47,1);
 
	if input(7) < 0.5 || ~cfg.Enabled
		return;
	end

	omega = input(1:3);
	torque = input(4:6);
	reset = state(43) < 0.5 || any(~isfinite(state));
	valid = all(isfinite(input(1:7)));
	[tau,x,P,valid] = advanceEkf(state(1:6),reshape(state(7:42),6,6), ...
		omega,torque,J,Ts,cfg.TorqueRW_Nm2_s*eye(3), ...
		cfg.RateSigma_rad_s^2*eye(3), ...
		diag([repmat(cfg.RateSigma_rad_s^2,1,3),repmat(cfg.InitialTorqueSigma_Nm^2,1,3)]), ...
		cfg.NISLimit,reset,valid);
	state(1:6) = x;
	state(7:42) = P(:);
	state(43) = double(valid);
	output(1:43) = state;

	if valid
		output(44:46) = max(-cfg.MaxEstimate_Nm,min(cfg.MaxEstimate_Nm,tau));
		output(47) = 1;
	end
end

function [tauHat, xNext, PNext, isValid] = advanceEkf( ...
		xHat, P, omegaMeasured, torquePrevious, J, Ts, torqueRandomWalk, ...
		rateCovariance, resetCovariance, nisLimit, reset, measurementValid)
%ADVANCEEKF Predict and correct angular rate and disturbance torque.

tauHat = zeros(3, 1);
xNext = xHat;
PNext = P;
isValid = false;

if ~isfinite(Ts) || Ts <= 0 || any(~isfinite(torquePrevious)) || ...
		~positiveDefinite(J) || ~positiveDefinite(rateCovariance) || ...
		~positiveDefinite(resetCovariance) || ...
		~positiveSemidefinite3(torqueRandomWalk) || ...
		isnan(nisLimit) || nisLimit <= 0
	return
end

if reset
	if measurementValid && all(isfinite(omegaMeasured))
		xNext = [omegaMeasured; zeros(3, 1)];
		PNext = resetCovariance;
		isValid = true;
	end
	return
end

if any(~isfinite(xHat)) || ~positiveDefinite(P)
	return
end

% Propagate the nonlinear rigid body using the completed interval input.
k1 = dynamics(xHat, torquePrevious, J);
k2 = dynamics(xHat + 0.5 * Ts * k1, torquePrevious, J);
k3 = dynamics(xHat + 0.5 * Ts * k2, torquePrevious, J);
k4 = dynamics(xHat + Ts * k3, torquePrevious, J);

xMinus = xHat + (Ts / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
omega = xHat(1:3);
F = zeros(6);
F(1:3, 1:3) = J \ (skew3(J * omega) - skew3(omega) * J);
F(1:3, 4:6) = J \ eye(3);
Qc = zeros(6);
Qc(4:6, 4:6) = (torqueRandomWalk + torqueRandomWalk.') / 2;
vanLoan = expm([F, Qc; zeros(6), -F.'] * Ts);
Phi = vanLoan(1:6, 1:6);
Qd = vanLoan(1:6, 7:12) * Phi.';
Qd = (Qd + Qd.') / 2;
pMinus = Phi * P * Phi.' + Qd;
pMinus = (pMinus + pMinus.') / 2;

if any(~isfinite(xMinus)) || ~positiveDefinite(pMinus)
	return
end
xNext = xMinus;
PNext = pMinus;
tauHat = xMinus(4:6);
if ~measurementValid || any(~isfinite(omegaMeasured))
	return
end

innovation = omegaMeasured - xMinus(1:3);
S = pMinus(1:3, 1:3) + rateCovariance;
if ~positiveDefinite(S)
	xNext = xHat;
	PNext = P;
	tauHat = zeros(3, 1);
	return
end
nis = innovation.' * (S \ innovation);
if ~isfinite(nis) || nis > nisLimit
	return
end
gain = pMinus(:, 1:3) / S;
xCandidate = xMinus + gain * innovation;
H = [eye(3), zeros(3)];
correction = eye(6) - gain * H;
pCandidate = correction * pMinus * correction.' + ...
	gain * rateCovariance * gain.';
pCandidate = (pCandidate + pCandidate.') / 2;
if any(~isfinite(xCandidate)) || ~positiveDefinite(pCandidate)
	xNext = xHat;
	PNext = P;
	tauHat = zeros(3, 1);
	return
end
xNext = xCandidate;
PNext = pCandidate;
tauHat = xCandidate(4:6);
isValid = true;
end

function derivative = dynamics(state, torque, inertia)
%DYNAMICS Compute angular acceleration and disturbance torque dynamics.
omega = state(1:3);
derivative = [inertia \ (torque + state(4:6) - ...
	cross(omega, inertia * omega)); zeros(3, 1)];
end

function matrix = skew3(vector)
%SKEW3 Create the matrix representing the cross product.
matrix = [0, -vector(3), vector(2); ...
	vector(3), 0, -vector(1); -vector(2), vector(1), 0];
end

function ok = positiveDefinite(matrix)
%POSITIVEDEFINITE Check that a matrix is finite, symmetric and positive definite.
ok = false;
if any(~isfinite(matrix(:))) || ...
		norm(matrix - matrix.', 'fro') > ...
		1e-10 * max(norm(matrix, 'fro'), realmin)
	return
end
[~, flag] = chol((matrix + matrix.') / 2);
ok = flag == 0;
end

function ok = positiveSemidefinite3(matrix)
%POSITIVESEMIDEFINITE3 Check that a 3x3 matrix is finite, symmetric and positive semidefinite.
ok = false;
if any(~isfinite(matrix(:))) || ...
		norm(matrix - matrix.', 'fro') > ...
		1e-10 * max(norm(matrix, 'fro'), realmin)
	return
end
eigenvalues = eig((matrix + matrix.') / 2);
ok = all(eigenvalues >= 0);
end
