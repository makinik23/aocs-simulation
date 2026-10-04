function result = pdfLyapunov(q,w,xi,J,tauExternal,aocs)
% Use the scaled continuous-time function from Section 4 of the PID note.
	omega0 = 0.06; % s^-1; time scale used in the PDF.
	kp = aocs.Control.PIDKp_s2 / omega0^2;
	kd = aocs.Control.PIDKd_s / omega0;
	ki = aocs.Control.Ki_s3 / omega0^3;
	A0 = [0 1 0; -kp -kd -1; ki 0 0];
	assert(max(real(eig(A0))) < 0, 'The PDF construction requires a Hurwitz A0.');
	P0 = lyap(A0',eye(3));
	P0 = (P0+P0')/2;
	[~,flag] = chol(P0);
	assert(flag == 0, 'The continuous Lyapunov matrix must be positive definite.');
	residual = norm(A0'*P0+P0*A0+eye(3),'fro');
	assert(residual < 1e-10, 'The continuous Lyapunov equation residual is too large.');

	% Assemble the 9-state matrix in the same block order as the PDF.
	I = eye(3);
	P = [P0(1,1)*I P0(1,2)*I P0(1,3)*I;
		P0(2,1)*I P0(2,2)*I P0(2,3)*I;
		P0(3,1)*I P0(3,2)*I P0(3,3)*I];
	X = [2*q(:,2:4),w/omega0,(xi-(J\tauExternal).')/omega0^2].';
	V = sum(X.*(P*X),1);

	% Sample differences measure the actual trajectory, including saturation.
	result = struct('omega0',omega0,'A0',A0,'P0',P0,'P',P, ...
		'V',V,'deltaV',diff(V),'residual',residual);
end
