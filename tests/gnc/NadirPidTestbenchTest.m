classdef NadirPidTestbenchTest < matlab.unittest.TestCase
	methods (Test, TestTags = "Integration")
		function zeroErrorIsAnEquilibrium(testCase)
			[model,ws,p,cleanup] = fixture(); %#ok<ASGLU>

			p.q0=[1;0;0;0];
			p.tStop=0.3;

			setParameters(ws,p);

			out=sim(model);

			testCase.verifyLessThan(max(abs(values(out,'attitude_error_deg',1)),[],'all'),1e-10);
			testCase.verifyEqual(values(out,'omega_BI_B_rad_s',3), ...
				zeros(size(values(out,'omega_BI_B_rad_s',3))),'AbsTol',1e-13);
			testCase.verifyLessThan(max(abs(values(out,'tau_applied_B_Nm',3)),[],'all'),1e-14);
		end

		function knownConstantBodyRateMatchesQuaternion(testCase)
			[model,ws,p,cleanup] = fixture(); %#ok<ASGLU>

			p.AOCS.Control.Enabled=false;
			setParameters(ws,p);

			w=[0.03;-0.02;0.01];
			p.q0=[cos(.2);sin(.2)*[1;2;3]/sqrt(14)];
			p.omega0=w;
			p.tStop=2;

			% Analytic solution: the imposed torque cancels the plant gyro
			% term, so body rate is constant even for non-principal rotation.
			p.tau_ext=cross(w,p.J*w);
			setParameters(ws,p);

			out=sim(model);

			ts=out.logsout.get('q_BI').Values;
			q=values(out,'q_BI',4);
			t=ts.Time(:);
			a=norm(w)*t/2;
			delta=[cos(a),sin(a)*(w'/norm(w))];
			expected=quatmultiply(repmat(p.q0',numel(t),1),delta);

			testCase.verifyEqual(q,expected,'AbsTol',1e-9);

			rates=values(out,'omega_BI_B_rad_s',3);

			testCase.verifyEqual(rates,repmat(w',size(rates,1),1),'AbsTol',1e-10);
			testCase.verifyLessThan(max(abs(values(out,'quaternion_norm',1)-1)),1e-10);
		end

		function copiedPidAcquires(testCase)
			[model,ws,p,cleanup] = fixture(); %#ok<ASGLU>

			% One nominal run confirms closure and conventions, not a proof.
			setParameters(ws,p);

			out=sim(model);

			angle=values(out,'attitude_error_deg',1);

			testCase.verifyEqual(angle(1),5,'AbsTol',1e-9);
			testCase.verifyLessThan(angle(end),0.5);
			testCase.verifyLessThanOrEqual(max(abs(values(out,'tau_applied_B_Nm',3)),[],'all'), ...
				p.AOCS.Control.MaxTorque_Nm+1e-12);

			fprintf('PID testbench: %.6f -> %.6f deg in %.1f s.\n',angle(1),angle(end),p.tStop);
		end
	end
end

function [model,ws,p,cleanup]=fixture()
	% Load the saved standalone model directly: no project setup or bus creation.
	root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
	model='nadir_pid_testbench';
	wasLoaded=bdIsLoaded(model);
	load_system(fullfile(root,'models',[model '.slx']));
	ws=get_param(model,'ModelWorkspace');
	variables=whos(ws);
	p=struct();
	for k=1:numel(variables)
		p.(variables(k).name)=ws.getVariable(variables(k).name);
	end
	dirty=get_param(model,'Dirty');
	cleanup=onCleanup(@() restore(model,ws,p,wasLoaded,dirty));
end

function setParameters(ws,p)
	names=fieldnames(p);
	for k=1:numel(names), ws.assignin(names{k},p.(names{k})); end
end

function restore(model,ws,p,wasLoaded,dirty)
	setParameters(ws,p);
	if wasLoaded, set_param(model,'Dirty',dirty); else, close_system(model,0); end
end

function data=values(out,name,width)
	ts=out.logsout.get(name).Values;

	% Use the time dimension explicitly, also when samples == signal width.

	if ts.IsTimeFirst
		data=reshape(ts.Data,numel(ts.Time),width);
	else
		data=reshape(ts.Data,width,numel(ts.Time))';
	end

end
