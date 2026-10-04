classdef QuaternionPdControllerTest < matlab.unittest.TestCase
	% Tests the actual block controller against rigid-body error dynamics.

	methods (Test, TestTags = ["Integration", "FullPlant"])
		function compensatedErrorDynamicsAndEquilibrium(testCase)
			setupAocsPaths([],true);
			[aocs,variables] = setupAocsSimulation('',false);
			load_system(aocs.Model.File);
			h = 'quaternion_pd_block_test';
			new_system(h);
			cleanup = onCleanup(@() close_system(h,0)); %#ok<NASGU>

			set_param(h,'Solver','FixedStepDiscrete','FixedStep','0.1','StopTime','0.1');
			add_block([char(aocs.Model.Name) '/GNC/Attitude Control/PD/Controller'],[h '/Controller']);

			names = {'qe','ew','wr','estimate','ar'};
			for k=1:5
				block=[h '/' names{k}];

				add_block('simulink/Sources/Constant',block,'Value',names{k},'VectorParams1D','off');

				if k==4, set_param(block,'OutDataTypeStr','Bus: AttitudeEstimateBus'); end

				add_line(h,[names{k} '/1'],sprintf('Controller/%d',k));
			end
			add_block('simulink/Sinks/To Workspace',[h '/Torque'], ...
				'VariableName','tau','SaveFormat','Timeseries');
			add_line(h,'Controller/1','Torque/1');

			J=aocs.Spacecraft.I_B;
			for k=1:3
				estimate=Simulink.Bus.createMATLABStruct('AttitudeEstimateBus');
				if k==1
					qe=[1;0;0;0];
					wr=zeros(3,1);
					ar=zeros(3,1);
					ew=zeros(3,1);
				elseif k==2
					qe=[1;0;0;0];
					wr=[.01;-.03;.02];
					ar=[.001;-.002;.003];
					ew=zeros(3,1);
				else
					qe=angle2quat(.7,-.4,.9)';
					wr=[.01;-.03;.02];
					ar=[.001;-.002;.003];
					ew=[.02;.03;-.04];
				end
				C=quat2dcm(qe');
				w=ew+C*wr;
				estimate.omega_BI_B_rad_s=w;
				input=Simulink.SimulationInput(h);
				fields=fieldnames(variables);
				for j=1:numel(fields), input=input.setVariable(fields{j},variables.(fields{j})); end
				values={qe,ew,wr,estimate,ar};
				for j=1:5, input=input.setVariable(names{j},values{j}); end

				output=sim(input);

				ts=output.tau;
				tau=reshape(getdatasamples(ts,numel(ts.Time)),3,1);

				% Euler's plant dynamics and the rotating-reference transport law.
				wdot=J\(tau-cross(w,J*w));
				errorDerivative=wdot+cross(ew,C*wr)-C*ar;
				expected=-2*aocs.Control.Kp_s2*qe(2:4)-aocs.Control.Kd_s*ew;

				testCase.verifyEqual(errorDerivative,expected,'AbsTol',1e-12);

				if k==1, testCase.verifyEqual(tau,zeros(3,1),'AbsTol',1e-14); end
			end
		end

		function controllerIsOptInAndRejectsInvalidGain(testCase)
			root=setupAocsPaths([],true);
			aocs=loadAocsSimulationConfig('',root);

			testCase.verifyFalse(aocs.Control.Enabled);
			testCase.verifyFalse(aocs.Control.UseIntegral);

			raw=aocs.Raw;
			raw.gnc.attitude_control.kp_s2=-1;
			file=[tempname '.json'];
			cleanup=onCleanup(@() delete(file)); %#ok<NASGU>
			writeAocsJson(file,raw);

			testCase.verifyError(@() loadAocsSimulationConfig(file,root),'AOCS:Config:InvalidField');
		end

		function integralFeedbackSelectionLimitsAndHealth(testCase)
			setupAocsPaths([],true);
			[aocs,variables] = setupAocsSimulation('',false);
			load_system(aocs.Model.File);
			h='quaternion_pid_block_test';
			new_system(h);
			cleanup=onCleanup(@() close_system(h,0)); %#ok<NASGU>

			set_param(h,'Solver','FixedStepDiscrete','FixedStep','0.1','StopTime','2');
			add_block([char(aocs.Model.Name) '/GNC/Attitude Control'],[h '/Control']);
			add_block('simulink/Ports & Subsystems/Function-Call Generator',[h '/Tick'],'sample_time','0.1');
			add_line(h,'Tick/1','Control/Trigger');

			names={'health','estimate','reference','qref','wref','aref'};
			busTypes={'AttitudeHealthBus','AttitudeEstimateBus','ReferenceVectorBus'};
			for k=1:numel(names)
				block=[h '/' names{k}];

				add_block('simulink/Sources/Constant',block,'Value',names{k},'VectorParams1D','off');

				if k<=3, set_param(block,'OutDataTypeStr',['Bus: ' busTypes{k}]); end

				add_line(h,[names{k} '/1'],sprintf('Control/%d',k));
			end
			add_block('simulink/Sinks/To Workspace',[h '/Torque'], ...
				'VariableName','tau','SaveFormat','Timeseries');
			add_line(h,'Control/1','Torque/1');

			estimate=Simulink.Bus.createMATLABStruct('AttitudeEstimateBus');
			estimate.q_BI=[cosd(5);sind(5);0;0];
			estimate.omega_BI_B_rad_s=zeros(3,1);
			health=Simulink.Bus.createMATLABStruct('AttitudeHealthBus');
			reference=Simulink.Bus.createMATLABStruct('ReferenceVectorBus');
			eTheta=2*estimate.q_BI(2:4);
			aocs.Control.PIDKp_s2=0.00055; % Detect accidental reuse of PD gains.
			aocs.Control.PIDKd_s=0.047;
			for k=1:6
				aocs.Control.UseIntegral=k~=1;
				aocs.Control.Enabled=k~=4;
				health.control_usable=k~=5;
				reference.valid=double(k~=6);
				aocs.Control.MaxTorque_Nm=2e-5;
				if k==3, aocs.Control.MaxTorque_Nm=1e-8; end
				input=Simulink.SimulationInput(h);
				variables.AOCS=aocs;
				fields=fieldnames(variables);
				for j=1:numel(fields), input=input.setVariable(fields{j},variables.(fields{j})); end
				values={health,estimate,reference,[1;0;0;0],zeros(3,1),zeros(3,1)};
				for j=1:numel(names), input=input.setVariable(names{j},values{j}); end

				output=sim(input);

				ts=output.tau;
				actual=reshape(getdatasamples(ts,numel(ts.Time)),3,1);
				expected=-aocs.Spacecraft.I_B*aocs.Control.Kp_s2*eTheta;
				if k==2
					expected=-aocs.Spacecraft.I_B*(aocs.Control.PIDKp_s2 ...
						+aocs.Control.Ki_s3*ts.Time(end))*eTheta;
				elseif k==3
					testCase.verifyLessThanOrEqual(max(abs(ts.Data),[],'all'),aocs.Control.MaxTorque_Nm+1e-14);

					continue;
				elseif k>=4
					expected=zeros(3,1);
				end

				testCase.verifyEqual(actual,expected,'AbsTol',1e-13);
			end
		end

		function integralResetAndSaturationRecovery(testCase)
			% Exercise the actual accumulator with actuator clipping in feedback.
			setupAocsPaths([],true);
			[aocs,variables]=setupAocsSimulation('',false);
			load_system(aocs.Model.File);
			h='pid_integral_recovery_test';
			new_system(h);
			cleanup=onCleanup(@() close_system(h,0)); %#ok<NASGU>

			set_param(h,'Solver','FixedStepDiscrete','FixedStep','0.1','StopTime','160');
			add_block('simulink/Ports & Subsystems/Subsystem',[h '/Loop']);

			Simulink.SubSystem.deleteContents([h '/Loop']);
			p=[h '/Loop'];

			add_block('simulink/Ports & Subsystems/Trigger',[p '/Tick'],'TriggerType','function-call');
			add_block('simulink/Ports & Subsystems/Function-Call Generator',[h '/Tick'],'sample_time','0.1');
			add_line(h,'Tick/1','Loop/Trigger');
			add_block([char(aocs.Model.Name) '/GNC/Attitude Control/PID/Controller/Integral Action'],[p '/I']);
			add_block('simulink/Sources/Constant',[p '/Error'],'Value','[1;0;0]','VectorParams1D','off');

			% Active -> reset for 10 s -> re-enable.
			add_block('simulink/Sources/Step',[p '/Disable'],'Time','120','Before','1','After','0','SampleTime','-1');
			add_block('simulink/Sources/Step',[p '/Reenable'],'Time','130','Before','0','After','1','SampleTime','-1');
			add_block('simulink/Math Operations/Sum',[p '/Active'],'Inputs','++');
			add_block('simulink/Math Operations/Gain',[p '/Torque'],'Gain','-AOCS.Spacecraft.I_B','Multiplication','Matrix(K*u)');
			add_block('simulink/Discontinuities/Saturation',[p '/Limit'],'UpperLimit','1e-7','LowerLimit','-1e-7');
			add_block('simulink/Math Operations/Sum',[p '/Residual'],'Inputs','+-');
			add_block('simulink/Sinks/To Workspace',[p '/State'],'VariableName','xi','SaveFormat','Timeseries');

			lines={'Error/1','I/1';'Disable/1','Active/1';'Reenable/1','Active/2'; ...
				'Active/1','I/2';'I/1','Torque/1';'Torque/1','Limit/1'; ...
				'Limit/1','Residual/1';'Torque/1','Residual/2';'Residual/1','I/3';'I/1','State/1'};
			for k=1:size(lines,1), add_line(p,lines{k,1},lines{k,2}); end
			input=Simulink.SimulationInput(h);
			fields=fieldnames(variables);
			for k=1:numel(fields), input=input.setVariable(fields{k},variables.(fields{k})); end

			output=sim(input);

			ts=output.xi;
			values=loggedSignalMatrix(ts.Data,3,'integral test');

			% For the diagonal inertia, the held-error saturated steady value is
			% xi_x = limit/Jxx + Ki*Taw, rather than an unbounded ramp.
			target=1e-7/aocs.Spacecraft.I_B(1,1)+aocs.Control.Ki_s3*aocs.Control.AntiWindupTime_s;

			testCase.verifyEqual(values(find(ts.Time<120,1,'last'),1),target,'RelTol',1e-3);
			testCase.verifyEqual(values(ts.Time>=120 & ts.Time<=130,:), ...
				zeros(nnz(ts.Time>=120 & ts.Time<=130),3),'AbsTol',1e-14);

			first=find(ts.Time>130,1);

			testCase.verifyEqual(values(first,1),aocs.GNC.SampleTime_s*aocs.Control.Ki_s3,'AbsTol',1e-14);
		end
	end
end
