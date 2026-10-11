function [nextState, navigation, health] = orbitNavigationStep( ...
    previousState, measurement, gnss_update, time_s, reset, cfg) %#codegen
%ORBITNAVIGATIONSTEP Perform one orbital EKF step with prediction and GNSS correction.

nextState = previousState;
nisValue = 0.0;
nisAvailable = false;
initializedThisStep = false;
stateChanged = false;
timeTolerance_s = 1e-6;
timeValid = isfinite(time_s) && time_s >= 0;

if reset
    nextState = cfg.initial_state;
end

validMeasurement = timeValid && measurement.valid ...
    && all(isfinite(measurement.z_gnss)) ...
    && isfinite(measurement.measurement_time_s) ...
    && measurement.measurement_time_s >= 0 ...
    && measurement.measurement_time_s <= time_s + timeTolerance_s;

if ~reset && ~nextState.initialized && gnss_update && validMeasurement
    [P0, covarianceValid] = gnssMeasurementCovariance( ...
        measurement.z_gnss, cfg.gnss_covariance_RTN);

    if covarianceValid ...
            && validOrbitState(measurement.z_gnss, cfg.earth_radius_m) ...
            && positiveDefinite(P0)
        nextState.x_hat = measurement.z_gnss;
        nextState.P = P0;
        nextState.epoch_s = measurement.measurement_time_s;
        nextState.initialized = true;
        nextState.last_accepted_gnss_time_s = measurement.measurement_time_s;
        nextState.last_processed_sequence_id = measurement.sequence_id;
        initializedThisStep = true;
        stateChanged = true;
    end
end

stateValid = nextState.initialized ...
    && validOrbitState(nextState.x_hat, cfg.earth_radius_m) ...
    && positiveDefinite(nextState.P) ...
    && isfinite(nextState.epoch_s) && nextState.epoch_s >= 0;

if ~reset && timeValid && stateValid
    dt = time_s - nextState.epoch_s;

    if dt > 0 && dt <= 1.0 ...
            && isfinite(cfg.acceleration_noise_psd_m2_s3) ...
            && cfg.acceleration_noise_psd_m2_s3 >= 0
        [xPredicted, propagationValid, Phi] = propagateOrbitJ2( ...
            nextState.x_hat, dt, cfg.mu_m3_s2, ...
            cfg.earth_radius_m, cfg.J2, cfg.pole_I);

        if propagationValid
            Qd = orbitProcessNoise(dt, cfg.acceleration_noise_psd_m2_s3);
            Ppredicted = Phi * nextState.P * Phi.' + Qd;
            Ppredicted = 0.5 * (Ppredicted + Ppredicted.');

            if positiveDefinite(Ppredicted)
                nextState.x_hat = xPredicted;
                nextState.P = Ppredicted;
                nextState.epoch_s = time_s;
                stateChanged = true;
            end
        end
    end
end

timeSynchronized = timeValid ...
    && abs(nextState.epoch_s - time_s) <= timeTolerance_s;
canCorrect = ~reset && stateValid && timeSynchronized ...
    && ~initializedThisStep && gnss_update && validMeasurement ...
    && measurement.sequence_id ~= nextState.last_processed_sequence_id ...
    && abs(measurement.measurement_time_s - nextState.epoch_s) <= timeTolerance_s;

if canCorrect
    [nextState, nisValue, nisAvailable, correctionApplied] = ...
        correctGnss(nextState, measurement, cfg);
    stateChanged = stateChanged || correctionApplied;
end

if stateChanged
    if nextState.solution_id == intmax('uint32')
        nextState.solution_id = uint32(0);
    else
        nextState.solution_id = nextState.solution_id + uint32(1);
    end
end

solutionValid = ~reset && timeSynchronized && nextState.initialized ...
    && validOrbitState(nextState.x_hat, cfg.earth_radius_m) ...
    && positiveDefinite(nextState.P);
mode = uint8(0);
stateSource = uint8(0);
if nextState.initialized
    mode = uint8(3);
    if solutionValid
        mode = uint8(2);
        stateSource = uint8(2);
        if abs(nextState.epoch_s - nextState.last_accepted_gnss_time_s) ...
                <= timeTolerance_s
            mode = uint8(1);
            stateSource = uint8(1);
        end
    end
end

navigation.position_I_m = nextState.x_hat(1:3);
navigation.velocity_I_m_s = nextState.x_hat(4:6);
navigation.covariance = nextState.P;
navigation.epoch_s = nextState.epoch_s;
navigation.state_source = stateSource;
navigation.valid = solutionValid;
navigation.solution_id = nextState.solution_id;

health.mode = mode;
health.valid = solutionValid;
health.gnss_age_s = -1.0;
health.nis = nisValue;
health.nis_available = nisAvailable;
health.algorithm_ready = true;
health.update_requested = logical(gnss_update);
health.measurement_sequence_id = measurement.sequence_id;
health.execution_time_s = time_s;

if nextState.initialized && timeValid
    health.gnss_age_s = time_s - nextState.last_accepted_gnss_time_s;
end

end


function [state, nisValue, nisAvailable, applied] = correctGnss(state, measurement, cfg)
%CORRECTGNSS Correct the orbital state with GNSS and report NIS.

nisValue = 0.0;
nisAvailable = false;
applied = false;
state.last_processed_sequence_id = measurement.sequence_id;
innovation = measurement.z_gnss - state.x_hat;
[R, covarianceValid] = gnssMeasurementCovariance( ...
    state.x_hat, cfg.gnss_covariance_RTN);

if ~covarianceValid || ~positiveDefinite(R)
    return
end

S = state.P + R;
S = 0.5 * (S + S.');
if any(~isfinite(S(:)))
    return
end

[L, cholFlag] = chol(S, 'lower');
if cholFlag ~= 0
    return
end

[nisValue, nisAvailable] = innovationNis(innovation, L);
K = (L.' \ (L \ state.P.')).';
xCorrected = state.x_hat + K * innovation;
IminusK = eye(6) - K;
Pcorrected = IminusK * state.P * IminusK.' + K * R * K.';
Pcorrected = 0.5 * (Pcorrected + Pcorrected.');

if ~validOrbitState(xCorrected, cfg.earth_radius_m) ...
        || ~positiveDefinite(Pcorrected)
    return
end

state.x_hat = xCorrected;
state.P = Pcorrected;
state.last_accepted_gnss_time_s = measurement.measurement_time_s;
applied = true;
end


function Qd = orbitProcessNoise(dt, accelerationNoisePsd)
%ORBITPROCESSNOISE Compute the discrete orbital process noise covariance.

I3 = eye(3);
Qd = accelerationNoisePsd * [ ...
    dt^3/3 * I3, dt^2/2 * I3; ...
    dt^2/2 * I3, dt * I3];
end


function [R, valid] = gnssMeasurementCovariance(x, covarianceRTN)
%GNSSMEASUREMENTCOVARIANCE Transform the GNSS covariance to GCRF.

R = zeros(6);
valid = false;

r_I = x(1:3);
v_I = x(4:6);
h_I = cross(r_I, v_I);

rNorm = norm(r_I);
hNorm = norm(h_I);

if ~isfinite(rNorm) || ~isfinite(hNorm) ...
        || rNorm <= 0 || hNorm <= 0
    return
end

radial_I = r_I / rNorm;
normal_I = h_I / hNorm;

transverse_I = cross(normal_I, radial_I);
C_I_RTN = [radial_I, transverse_I, normal_I];
T_I_RTN = blkdiag(C_I_RTN, C_I_RTN);

R = T_I_RTN * covarianceRTN * T_I_RTN.';
R = 0.5 * (R + R.');

valid = all(isfinite(R(:)));

end


function [value, available] = innovationNis(innovation, L)
%INNOVATIONNIS Compute the normalized innovation squared.

value = 0.0;
whitened = L \ innovation;
candidate = whitened.' * whitened;
available = isfinite(candidate);

if available
    value = candidate;
end

end


function valid = positiveDefinite(P)
%POSITIVEDEFINITE Check that a covariance is finite, symmetric and positive definite.

valid = false;

if any(~isfinite(P(:))) ...
        || norm(P - P.', 'fro') > 1e-10 * max(norm(P, 'fro'), realmin)
    return
end

[~, flag] = chol(0.5 * (P + P.'));
valid = flag == 0;

end


function valid = validOrbitState(x, earthRadius)
%VALIDORBITSTATE Check that an orbital state has valid RTN geometry.

rNorm = norm(x(1:3));
vNorm = norm(x(4:6));
hNorm = norm(cross(x(1:3), x(4:6)));

valid = all(isfinite(x)) && isfinite(earthRadius) && earthRadius > 0 ...
    && isfinite(rNorm) && rNorm > earthRadius ...
    && isfinite(vNorm) && vNorm > 0 && isfinite(hNorm) ...
    && hNorm > 1e-8 * rNorm * vNorm;

end
