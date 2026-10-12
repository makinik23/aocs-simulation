function design = designNadirPidPolePlacement(omegaN_rad_s, integralPole_s, damping, sampleTime_s)
%DESIGNNADIRPIDPOLEPLACEMENT Match the local attitude-error characteristic polynomial.
% P(s) = s^3+Kd*s^2+Kp*s+Ki = (s+pI)*(s^2+2*zeta*wn*s+wn^2).
% Assumes exact feedback linearization, small attitude errors, no clipping,
% and continuous control. The sampled poles below also include the existing
% forward-Euler integral and zero-order-held acceleration command.
arguments
    omegaN_rad_s (1,1) double {mustBeFinite,mustBePositive}
    integralPole_s (1,1) double {mustBeFinite,mustBePositive}
    damping (1,1) double {mustBeFinite,mustBePositive} = 1
    sampleTime_s (1,1) double {mustBeFinite,mustBePositive} = 0.1
end
w=omegaN_rad_s; p=integralPole_s; z=damping; T=sampleTime_s;
kp=w^2+2*z*w*p; kd=2*z*w+p; ki=w^2*p;
Ad=[1-.5*kp*T^2,T-.5*kd*T^2,-.5*T^2; -kp*T,1-kd*T,-T; ki*T,0,1];
continuousPoles=roots([1 kd kp ki]); sampledPoles=eig(Ad);
design=struct('Method','Augmented-state pole placement by coefficient matching', ...
    'OmegaN_rad_s',w,'IntegralPole_s',p,'Damping',z, ...
    'Kp_s2',kp,'Kd_s',kd,'Ki_s3',ki, ...
    'PolesFormat','Each row contains real part and imaginary part', ...
    'ContinuousPoles_s',[real(continuousPoles) imag(continuousPoles)], ...
    'SampleTime_s',T,'SampledPoles',[real(sampledPoles) imag(sampledPoles)], ...
    'SampledSpectralRadius',max(abs(sampledPoles)));
assert(design.SampledSpectralRadius<1,'The chosen gains destabilize the local sampled model.');
end
