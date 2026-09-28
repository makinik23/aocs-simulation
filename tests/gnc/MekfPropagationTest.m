classdef MekfPropagationTest < matlab.unittest.TestCase
    % Description:
    %   Regression tests for block-only gyro prediction with updates disabled.
    methods (TestClassSetup)
        function prepareEnvironment(~)
            % Description:
            %   Loads production paths and typed configuration buses.
            % Arguments:
            %   None.
            % Outputs:
            %   None.
            setupAocsPaths([], true);
            setupAocsSimulation;
        end
    end
    methods (Test)
        function positiveYawMatchesPlantConvention(testCase)
            % Description:
            %   Checks positive body yaw against the passive Aerospace DCM.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f = fixture(); f.rate = repmat([0 0 0.2], 6, 1);
            s = simulate(f);
            angle = 0.2 * 0.5;
            expected = [cos(angle) sin(angle) 0; -sin(angle) cos(angle) 0; 0 0 1];
            testCase.verifyEqual(quat2dcm(s.q(:,end)'), expected, 'AbsTol', 1e-12);
            testCase.verifyEqual(vecnorm(s.q), ones(1,6), 'AbsTol', 1e-12);
        end
        function nonidentityAttitudeUsesRightQuaternionProduct(testCase)
            % Description:
            %   Distinguishes multiplication side for noncommuting rotations.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f = fixture(); f.q = [cos(0.4);sin(0.4);0;0];
            f.rate = repmat([0.12 -0.08 0.21],6,1);
            s = simulate(f); expected = f.q;
            w = f.rate(1,:)';
            for k=2:6
                delta = [cos(norm(w)*0.1/2); sin(norm(w)*0.1/2)*w/norm(w)];
                expected = quatmultiply(expected',delta')';
            end
            testCase.verifyEqual(s.q(:,end),expected,'AbsTol',1e-12);
        end
        function changingBodyRateUsesAdjacentSamplesAndConing(testCase)
            % Description:
            %   Checks midpoint and right-product coning correction at high rate.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture();
            f.rate=[0.20 0.10 1.50;0.21 0.08 1.50;0.22 0.05 1.50; ...
                0.21 0.02 1.50;0.18 -0.01 1.50;0.15 -0.03 1.50];
            s=simulate(f); expected=f.q;
            for k=2:numel(f.t)
                current=f.rate(k,:);
                if k==2
                    w=current;
                else
                    previous=f.rate(k-1,:);
                    if k==3
                        w=(previous+current)/2;
                    else
                        w=(5*current+8*previous-f.rate(k-2,:))/12;
                    end
                    w=w+0.1/12*cross(previous,current);
                end
                v=0.1*w; delta=[cos(norm(v)/2);sin(norm(v)/2)*v'/norm(v)];
                expected=quatmultiply(expected',delta')';
                testCase.verifyEqual(s.q(:,k),expected,'AbsTol',2e-12);
            end
        end
        function gyroGapDoesNotInterpolateStaleSamples(testCase)
            % Description:
            %   Falls back to the current rate after missed gyro receipts.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(); f.fresh=logical([1;1;0;0;1;1]);
            f.rate=[0.2 0.1 1.5;0.2 0.1 1.5;0.3 0.2 1.5; ...
                0.4 0.3 1.5;0.1 -0.2 1.5;0.15 -0.15 1.5];
            s=simulate(f);
            first=0.1*f.rate(2,:); gap=0.3*f.rate(5,:);
            d1=[cos(norm(first)/2) sin(norm(first)/2)*first/norm(first)];
            d2=[cos(norm(gap)/2) sin(norm(gap)/2)*gap/norm(gap)];
            expected=quatmultiply(d1,d2)';
            testCase.verifyEqual(s.q(:,5),expected,'AbsTol',2e-12);
            testCase.verifyEqual(s.q(:,2:4),repmat(d1',1,3),'AbsTol',2e-12);
            midpoint=(f.rate(5,:)+f.rate(6,:))/2;
            coning=0.1/12*cross(f.rate(5,:),f.rate(6,:));
            v=0.1*(midpoint+coning);
            delta=[cos(norm(v)/2) sin(norm(v)/2)*v/norm(v)];
            testCase.verifyEqual(s.q(:,6),quatmultiply(expected',delta)',...
                'AbsTol',2e-12);
        end
        function fastVaryingRateStaysCloseToContinuousTruth(testCase)
            % Description:
            %   Compares ten seconds of high-rate propagation to 1 ms truth.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(); f.t=(0:0.1:10)'; f.receipt=f.t;
            f.rate=[0.2*cos(1.2*f.t),0.2*sin(1.2*f.t),1.5*ones(size(f.t))];
            f.valid=true(size(f.t)); f.fresh=f.valid; f.overrun=false(size(f.t));
            s=simulate(f); truth=f.q;
            for time=0.0005:0.001:10
                w=[0.2*cos(1.2*time);0.2*sin(1.2*time);1.5];
                v=0.001*w; delta=[cos(norm(v)/2);sin(norm(v)/2)*v/norm(v)];
                truth=quatmultiply(truth',delta')';
            end
            angle=2*acosd(min(1,abs(dot(s.q(:,end),truth))));
            testCase.verifyLessThan(angle,0.1);
        end
        function covarianceUsesNegativeSkewAndConfiguredNoise(testCase)
            % Description:
            %   Verifies anisotropic covariance, coupling sign and PSD.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(); f.rate=repmat([0.2 -0.1 0.3],6,1);
            s=simulate(f); P=f.P; w=f.rate(1,:)';
            skew=[0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];
            F=[-skew -eye(3);zeros(3,6)]; Phi=eye(6)+0.1*F;
            config=evalin('base','AOCS_GNCConfig'); g=config.Value.MEKF;
            Q=0.1*blkdiag(eye(3)*g.gyro_noise_density_rad_s_sqrt_Hz^2, ...
                eye(3)*g.bias_random_walk_std_rad_s_sqrt_s^2);
            for k=2:6
                P=Phi*P*Phi'+Q;
                testCase.verifyEqual(s.P(:,:,k),P,'AbsTol',1e-12);
                testCase.verifyGreaterThan(min(eig(s.P(:,:,k))),0);
            end
        end
        function subtractsIndependentBiasPrior(testCase)
            % Description:
            %   Equal reported rate and prior yield zero corrected rotation.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(); f.bias=[0.01;-0.02;0.03];
            f.rate=repmat(f.bias',6,1); s=simulate(f);
            testCase.verifyEqual(s.q,repmat(f.q,1,6),'AbsTol',1e-12);
            testCase.verifyEqual(s.rate,zeros(3,6),'AbsTol',1e-12);
            testCase.verifyEqual(s.bias,repmat(f.bias,1,6),'AbsTol',1e-12);
        end
        function missingSamplesHoldAndResumeWithElapsedTime(testCase)
            % Description:
            %   Holds state/time for repeated receipts and integrates the gap.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(); f.rate=repmat([0 0 0.2],6,1);
            f.fresh=logical([1;1;0;0;1;1]); s=simulate(f);
            testCase.verifyEqual(s.q(:,2:4),repmat(s.q(:,2),1,3));
            testCase.verifyEqual(s.time,[0;0.1;0.1;0.1;0.4;0.5],'AbsTol',1e-12);
            angle=0.2*0.5;
            testCase.verifyEqual(s.q(:,end),[cos(angle/2);0;0;sin(angle/2)],'AbsTol',1e-12);
            testCase.verifyEqual(s.valid,logical([0;1;0;0;1;1]));
        end
        function rejectsRepeatedAndBackwardTimestamps(testCase)
            % Description:
            %   Fresh flags cannot advance state using a nonpositive interval.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(); f.receipt=[0;0.1;0.1;0.05;0.4;0.5];
            s=simulate(f);
            testCase.verifyEqual(s.time,[0;0.1;0.1;0.1;0.4;0.5],'AbsTol',1e-12);
            testCase.verifyEqual(s.valid,logical([0;1;0;0;1;1]));
        end
        function invalidGyroHoldsWithoutPoisoningState(testCase)
            % Description:
            %   Invalid nonfinite samples must not contaminate held memory.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(); f.rate(3,:)=[NaN Inf -Inf]; f.valid(3)=false;
            s=simulate(f);
            testCase.verifyTrue(all(isfinite(s.q),'all'));
            testCase.verifyTrue(all(isfinite(s.P),'all'));
            testCase.verifyEqual(s.q(:,3),s.q(:,2));
            testCase.verifyEqual(s.rate(:,3),s.rate(:,2));
        end
        function overrunKeepsLatestValidReportUsable(testCase)
            % Description:
            %   Matches the driver contract: overrun does not invalidate data.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(); f.overrun(3)=true; s=simulate(f);
            testCase.verifyTrue(s.valid(3));
            testCase.verifyEqual(s.time(3),0.2,'AbsTol',1e-12);
        end
        function rejectsNonfiniteRateEvenWhenMarkedValid(testCase)
            % Description:
            %   A malformed report cannot rely on its valid flag alone.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(); f.rate(3,:)=[NaN Inf -Inf];
            s=simulate(f);
            testCase.verifyFalse(s.valid(3));
            testCase.verifyEqual(s.time(3),s.time(2));
            testCase.verifyTrue(all(isfinite(s.q),'all'));
            testCase.verifyTrue(all(isfinite(s.P),'all'));
        end
    end
end

function f=fixture()
% Description:
%   Supplies a six-tick scenario with independent anisotropic uncertainty.
% Arguments:
%   None.
% Outputs:
%   f - Deterministic report, initialization and prior data.
f.t=(0:0.1:0.5)'; f.receipt=f.t; f.q=[1;0;0;0];
f.rate=repmat([0.1 0.2 -0.1],6,1); f.bias=zeros(3,1);
f.P=diag([0.01 0.02 0.04 1e-5 2e-5 3e-5]);
f.valid=true(6,1); f.fresh=true(6,1); f.overrun=false(6,1);
end

function s=simulate(f)
% Description:
%   Executes the public estimator harness with vector corrections disabled.
% Arguments:
%   f - Prediction scenario.
% Outputs:
%   s - Logged estimator state histories.
s = simulateMekfFixture(f);
end
