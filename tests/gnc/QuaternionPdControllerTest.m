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
            add_block([char(aocs.Model.Name) '/GNC/PD feedback linearisation/Controller'],[h '/Controller']);
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
                    qe=[1;0;0;0]; wr=zeros(3,1); ar=zeros(3,1); ew=zeros(3,1);
                elseif k==2
                    qe=[1;0;0;0]; wr=[.01;-.03;.02]; ar=[.001;-.002;.003]; ew=zeros(3,1);
                else
                    qe=angle2quat(.7,-.4,.9)'; wr=[.01;-.03;.02]; ar=[.001;-.002;.003]; ew=[.02;.03;-.04];
                end
                C=quat2dcm(qe'); w=ew+C*wr; estimate.omega_BI_B_rad_s=w;
                input=Simulink.SimulationInput(h);
                fields=fieldnames(variables);
                for j=1:numel(fields), input=input.setVariable(fields{j},variables.(fields{j})); end
                values={qe,ew,wr,estimate,ar};
                for j=1:5, input=input.setVariable(names{j},values{j}); end
                output=sim(input); ts=output.tau; tau=reshape(getdatasamples(ts,numel(ts.Time)),3,1);
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
            raw=aocs.Raw; raw.gnc.attitude_control.kp_s2=-1;
            file=[tempname '.json']; cleanup=onCleanup(@() delete(file)); %#ok<NASGU>
            writeAocsJson(file,raw);
            testCase.verifyError(@() loadAocsSimulationConfig(file,root),'AOCS:Config:InvalidField');
        end
    end
end
