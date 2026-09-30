classdef MekfPlantIntegrationTest < matlab.unittest.TestCase
    % Description:
    %   Checks default high-rate MEKF convergence through the complete plant.

    methods (Test, TestTags = ["Integration", "FullPlant"])
        function defaultPlantConvergesWithSensorAndReferenceErrors(testCase)
            % Description:
            %   Simulates 50 s of plant, sensors, drivers and onboard references.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            setupAocsPaths([],true);
            [input,AOCS]=createAocsSimulationInput();
            cleanup=onCleanup(@() close_system(AOCS.Model.Name,0)); %#ok<NASGU>
            input=input.setModelParameter('StopTime','50');
            out=sim(input);

            estimate=out.logsout.get('AttitudeEstimate').Values;
            truth=out.logsout.get('q_be').Values;
            time=estimate.q_BI.Time(:);
            q=reshape(estimate.q_BI.Data,4,[]).';
            qTrue=interp1(truth.Time(:),truth.Data,time,'linear');
            qTrue=qTrue./vecnorm(qTrue,2,2);
            errorDeg=2*acosd(min(1,abs(sum(q.*qTrue,2))));
            settled=time>=10 & time<=50;
            testCase.verifyTrue(all(estimate.initialized.Data(settled)));
            testCase.verifyLessThan(max(errorDeg(settled)),0.8);

            variables=input.Variables;
            sensor=variables(strcmp({variables.Name},'AOCS_SensorConfig')).Value;
            bias=reshape(estimate.gyro_bias_B_rad_s.Data,3,[]).';
            initialTrueBias=sensor.Value.Gyro.bias_initial_rad_s(:);
            testCase.verifyLessThan(norm(bias(end,:)'-initialTrueBias),5e-4);

            health=out.logsout.get('AttitudeHealth').Values;
            testCase.verifyEqual(health.mode_id.Data(end),uint8(1));
            testCase.verifyTrue(health.control_usable.Data(end));
            gnc=variables(strcmp({variables.Name},'AOCS_GNCConfig')).Value;
            testCase.verifyLessThan(health.correction_age_s.Data(end), ...
                gnc.Value.AttitudeHealth.degraded_correction_age_s);
        end
    end
end
