classdef MekfCorrectionTest < matlab.unittest.TestCase
    % Description:
    %   Verifies saved native-block MEKF corrections and deterministic convergence.
    methods (TestClassSetup)
        function prepareEnvironment(~)
            % Description:
            %   Publishes the production buses and independently tuned filter.
            % Arguments:
            %   None.
            % Outputs:
            %   None.
            setupAocsPaths([],true); setupAocsSimulation;
        end
    end
    methods (Test)
        function zeroInnovationReducesCovarianceWithoutMovingState(testCase)
            % Description:
            %   Checks zero-residual sequential updates and first-tick exclusion.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(0.5); f.q=f.truth(:,1); f.rate(:)=0;
            f.mag=repmat(f.mag(1,:),6,1); f.sun=repmat(f.sun(1,:),6,1);
            s=simulateMekfFixture(f);
            testCase.verifyEqual(s.q,repmat(f.q,1,6),'AbsTol',1e-12);
            testCase.verifyEqual(s.bias,zeros(3,6),'AbsTol',1e-12);
            testCase.verifyFalse(s.magAccepted(1)); testCase.verifyFalse(s.sunAccepted(1));
            testCase.verifyTrue(all(s.magAccepted(2:end) & s.sunAccepted(2:end)));
            testCase.verifyLessThan(trace(s.P(1:3,1:3,end)),trace(f.P(1:3,1:3))/10);
            verifyState(testCase,s);
        end

        function oneStepMatchesIndependentJosephAndResetEquations(testCase)
            % Description:
            %   Checks both updates, right injection, bias feedback and covariance.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(0.1); f.bias=[1e-4;-2e-4;3e-4];
            s=simulateMekfFixture(f); c=configuration();
            w=f.rate(2,:)'-f.bias; dt=0.1;
            q=quatmultiply(f.q',rotationQuaternion(dt*w)')';
            F=[-skew(w),-eye(3);zeros(3,6)]; Phi=eye(6)+dt*F;
            Q=dt*blkdiag(eye(3)*c.gyro_noise_density_rad_s_sqrt_Hz^2, ...
                eye(3)*c.bias_random_walk_std_rad_s_sqrt_s^2);
            P=Phi*f.P*Phi'+Q; b=f.bias;
            [q,b,P,magNIS]=referenceUpdate(q,b,P,f.mag(2,:)',f.magI(2,:)',c.magnetometer_direction_std_rad);
            [q,b,P,sunNIS]=referenceUpdate(q,b,P,f.sun(2,:)',f.sunI(2,:)',c.css_direction_std_rad);
            testCase.verifyTrue(s.magAccepted(2) && s.sunAccepted(2));
            testCase.verifyEqual(s.q(:,2),q,'AbsTol',2e-12);
            testCase.verifyEqual(s.bias(:,2),b,'AbsTol',2e-12);
            testCase.verifyEqual(s.P(:,:,2),P,'AbsTol',2e-12);
            testCase.verifyEqual(s.rate(:,2),f.rate(2,:)'-b,'AbsTol',2e-12);
            testCase.verifyEqual(s.magNIS(2),magNIS,'AbsTol',2e-10);
            testCase.verifyEqual(s.sunNIS(2),sunNIS,'AbsTol',2e-10);
        end

        function posteriorBiasDrivesNextPrediction(testCase)
            % Description:
            %   Ensures the next gyro step uses the corrected bias, not its prior.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(0.2); f.magFresh=[true;true;false]; f.sunFresh=f.magFresh;
            s=simulateMekfFixture(f);
            testCase.verifyGreaterThan(norm(s.bias(:,2)-f.bias),1e-9);
            w=f.rate(3,:)'-s.bias(:,2);
            previous=f.rate(2,:)'-f.bias;
            integrated=(previous+w)/2+0.1/12*cross(previous,w);
            expected=quatmultiply(s.q(:,2)',rotationQuaternion(integrated*0.1)')';
            testCase.verifyEqual(s.q(:,3),expected,'AbsTol',1e-12);
            testCase.verifyEqual(s.rate(:,3),w,'AbsTol',1e-12);
            testCase.verifyEqual(s.bias(:,3),s.bias(:,2));
        end

        function rejectsInvalidAndRepeatedVectorReports(testCase)
            % Description:
            %   Rejects NaN, zero, invalid and repeated reports without poisoning.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(0.5); f.mag(2,:)=[NaN Inf -Inf]; f.mag(3,:)=0;
            f.magValid=[true;true;true;false;true;true];
            f.magFresh=[true;true;true;true;false;true];
            f.sunValid=false(6,1);
            s=simulateMekfFixture(f);
            testCase.verifyEqual(s.magAccepted,logical([0;0;0;0;0;1]));
            testCase.verifyFalse(any(s.sunAccepted)); verifyState(testCase,s);
            testCase.verifyEqual(s.bias(:,1:5),zeros(3,5),'AbsTol',1e-15);
        end

        function rejectsMisalignedReportsAndStaleOrFutureReferences(testCase)
            % Description:
            %   Enforces state-time alignment and finite bounded reference age.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(0.7); f.magTime=f.t; f.sunTime=f.t;
            f.magTime(2)=f.t(2)-0.05; f.sunTime(2)=f.magTime(2);
            f.magTime(3)=f.t(3)+0.05; f.sunTime(3)=f.magTime(3);
            f.refTime=f.t; f.refTime(4)=f.t(4)-3; f.refTime(5)=f.t(5)+1;
            f.refTime(6)=NaN; f.refValid=true(8,1); f.refValid(7)=false;
            s=simulateMekfFixture(f);
            testCase.verifyEqual(s.magAccepted,logical([zeros(7,1);1]));
            testCase.verifyEqual(s.sunAccepted,logical([zeros(7,1);1]));
            verifyState(testCase,s);
        end

        function rejectsInnovationOutlierAndExcessiveCorrection(testCase)
            % Description:
            %   Checks independent NIS and local-angle gates against gyro-only output.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(0.1); f.q=f.truth(:,1); f.mag(2,:)=-f.mag(2,:);
            f.sunValid=false(2,1); s=simulateMekfFixture(f);
            testCase.verifyFalse(s.magAccepted(2));
            testCase.verifyGreaterThan(s.magNIS(2),configuration().innovation_gate_squared);
            baseline=f; baseline.magFresh=false(2,1); held=simulateMekfFixture(baseline);
            testCase.verifyEqual(s.q,held.q); testCase.verifyEqual(s.P,held.P);
            f=fixture(0.1); f.tuning.maximum_correction_rad=1e-8;
            s=simulateMekfFixture(f);
            testCase.verifyFalse(s.magAccepted(2) || s.sunAccepted(2));
            testCase.verifyEqual(s.bias,zeros(3,2)); verifyState(testCase,s);
        end

        function convergesWithBiasAndSurvivesVectorDropouts(testCase)
            % Description:
            %   Runs five minutes with wrong attitude/bias, CSS eclipse and total dropout.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            f=fixture(300); n=numel(f.t); f.sunValid=true(n,1); f.magValid=true(n,1);
            eclipse=f.t>=60 & f.t<120; outage=f.t>=180 & f.t<200;
            f.sunValid(eclipse | outage)=false; f.magValid(outage)=false;
            s=simulateMekfFixture(f); verifyState(testCase,s);
            error=attitudeError(s.q,f.truth);
            biasError=vecnorm(s.bias-f.truthBias);
            testCase.verifyLessThan(rad2deg(error(end)),0.05);
            testCase.verifyLessThan(biasError(end),5e-6);
            testCase.verifyFalse(any(s.sunAccepted(eclipse | outage)));
            testCase.verifyFalse(any(s.magAccepted(outage)));
            testCase.verifyGreaterThan(mean(s.magAccepted(~outage & f.t>0)),0.99);
            testCase.verifyTrue(s.sunAccepted(end));
            testCase.verifyLessThan(error(end),error(1)/50);
            fprintf('MEKF 300 s: attitude error %.6f deg; bias error %.3g rad/s\n', ...
                rad2deg(error(end)),biasError(end));
        end

        function observationAndResetSignsMatchFiniteDifferences(testCase)
            % Description:
            %   Independently checks local-error measurement and reset Jacobian signs.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            q=rotationQuaternion([0.3;-0.2;0.4]); ref=[0.2;0.3;0.9]; ref=ref/norm(ref);
            h=quat2dcm(q')*ref; step=1e-7; derivative=zeros(3);
            for k=1:3
                v=zeros(3,1); v(k)=step;
                perturbed=quatmultiply(q',rotationQuaternion(v)');
                derivative(:,k)=(quat2dcm(perturbed)*ref-h)/step;
            end
            testCase.verifyEqual(derivative,skew(h),'AbsTol',1e-7);
            injected=[0.001;-0.002;0.003]; J=zeros(3);
            inverse=rotationQuaternion(-injected);
            for k=1:3
                v=zeros(3,1); v(k)=step;
                dq=quatmultiply(inverse',rotationQuaternion(injected+v)');
                J(:,k)=2*dq(2:4)'/step;
            end
            testCase.verifyEqual(J,eye(3)-0.5*skew(injected),'AbsTol',3e-6);
        end
    end
end

function f=fixture(stop)
% Description:
%   Generates truth outside the estimator for a rotating body with gyro bias.
% Arguments:
%   stop - Duration [s], a multiple of the 0.1 s GNC period.
% Outputs:
%   f - Public input histories, priors and separate assertion-only truth.
f.t=(0:0.1:stop)'; n=numel(f.t); f.bias=zeros(3,1);
f.P=diag(deg2rad([5;5;5;0.1;0.1;0.1]).^2);
q0=rotationQuaternion([0.3;-0.2;0.5]); rate=[0.015;0.023;-0.008];
f.truthBias=[5e-4;-3e-4;2e-4]; f.rate=repmat((rate+f.truthBias)',n,1);
f.q=quatmultiply(q0',rotationQuaternion(deg2rad([3;-2;1]))')';
mag=[0.3;0.1;0.5]; mag=4e-5*mag/norm(mag);
sun=[-0.2;0.9;0.2]; sun=sun/norm(sun);
f.magI=repmat(mag',n,1); f.sunI=repmat(sun',n,1);
f.truth=zeros(4,n); f.mag=zeros(n,3); f.sun=zeros(n,3);
for k=1:n
    f.truth(:,k)=quatmultiply(q0',rotationQuaternion(rate*f.t(k))')';
    C=quat2dcm(f.truth(:,k)'); f.mag(k,:)=(C*mag)'; f.sun(k,:)=(C*sun)';
end
end

function c=configuration()
% Description:
%   Reads estimator tuning, never the physical sensor truth configuration.
% Arguments:
%   None.
% Outputs:
%   c - Numeric MEKF configuration.
p=evalin('base','AOCS_GNCConfig'); c=p.Value.MEKF;
end

function q=rotationQuaternion(v)
% Description:
%   Forms an independent exact scalar-first Hamilton rotation quaternion.
% Arguments:
%   v - Rotation vector [rad].
% Outputs:
%   q - Four-element unit quaternion.
a=norm(v); if a==0, q=[1;0;0;0]; else, q=[cos(a/2);sin(a/2)*v/a]; end
end

function S=skew(v)
% Description:
%   Defines the cross-product matrix for independent equation assertions.
% Arguments:
%   v - Three-element vector.
% Outputs:
%   S - Matrix satisfying S*w = cross(v,w).
S=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end

function [q,b,P,nis]=referenceUpdate(q,b,P,z,r,sigma)
% Description:
%   Implements a separate MATLAB oracle for one accepted vector correction.
% Arguments:
%   q, b, P - Nominal quaternion, bias [rad/s] and mixed-unit covariance.
%   z, r, sigma - Measured/reference vectors and angular noise [rad].
% Outputs:
%   q, b, P, nis - Posterior state and normalized innovation squared.
h=quat2dcm(q')*(r/norm(r)); innovation=z/norm(z)-h;
H=[skew(h),zeros(3)]; R=eye(3)*sigma^2;
S=H*P*H'+R; K=(S\(H*P))'; nis=innovation'*(S\innovation);
dx=K*innovation; q=quatmultiply(q',rotationQuaternion(dx(1:3))')'; q=q/norm(q);
b=b+dx(4:6); A=eye(6)-K*H; P=A*P*A'+K*R*K';
G=blkdiag(eye(3)-0.5*skew(dx(1:3)),eye(3)); P=G*P*G'; P=(P+P')/2;
end

function verifyState(testCase,s)
% Description:
%   Checks all saved samples for finite state, unit quaternions and symmetric PSD P.
% Arguments:
%   testCase, s - Test instance and logged simulation results.
% Outputs:
%   None.
testCase.verifyTrue(all(isfinite(s.q),'all') && all(isfinite(s.bias),'all'));
testCase.verifyTrue(all(isfinite(s.P),'all'));
testCase.verifyEqual(vecnorm(s.q),ones(1,size(s.q,2)),'AbsTol',2e-12);
testCase.verifyEqual(s.P,permute(s.P,[2 1 3]),'AbsTol',2e-12);
minimum=Inf;
for k=1:size(s.P,3), minimum=min(minimum,min(eig(s.P(:,:,k)))); end
testCase.verifyGreaterThanOrEqual(minimum,-1e-14);
end

function angle=attitudeError(q,truth)
% Description:
%   Computes sign-invariant attitude error for comparison with external truth.
% Arguments:
%   q, truth - Column-oriented unit quaternion histories.
% Outputs:
%   angle - Rotation errors [rad].
d=abs(sum(q.*truth,1)); angle=2*acos(min(1,d));
end
