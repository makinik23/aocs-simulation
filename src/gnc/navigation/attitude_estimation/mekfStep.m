function [nextState, health, diagnostics] = mekfStep( ...
    previousState, reports, status, initialization, references, time_s, cfg) %#codegen
%MEKFSTEP Perform one attitude MEKF step with gyro prediction and vector corrections.

nextState = previousState;
estimate = previousState.estimate;
previouslyInitialized = estimate.initialized;
initializedThisStep = ~previouslyInitialized && initialization.valid > 0.5;

if initializedThisStep
    estimate.q_BI = initialization.q_BI;
    estimate.gyro_bias_B_rad_s = cfg.MEKF.initial_bias_B_rad_s;
    estimate.P_error = cfg.MEKF.initial_covariance;
    estimate.omega_BI_B_rad_s = zeros(3, 1);
    estimate.update_time_s = initialization.solution_time_s;
    estimate.initialized = true;
end

nextState.estimate = estimate;
[nextState, predictionAccepted] = predictGyro( ...
    nextState, reports.Gyro, status.Gyro, previouslyInitialized, cfg);
estimate = nextState.estimate;

[estimate, magAccepted, magNis] = correctVector( ...
    estimate, reports.Magnetometer.B_B_T, references.B_I_T, ...
    reports.Magnetometer.receive_time_s, reports.Magnetometer.valid, ...
    status.Magnetometer, references, cfg.MEKF, ...
    cfg.MEKF.magnetometer_direction_std_rad);

[estimate, sunAccepted, sunNis] = correctVector( ...
    estimate, reports.CoarseSunSensors.sun_B_unit, references.sun_I_unit, ...
    reports.CoarseSunSensors.receive_time_s, reports.CoarseSunSensors.valid, ...
    status.CoarseSunSensors, references, cfg.MEKF, ...
    cfg.MEKF.css_direction_std_rad);

vectorAccepted = magAccepted || sunAccepted;
if initializedThisStep || vectorAccepted
    nextState.last_correction_time_s = time_s;
end

nextState.estimate = estimate;
health = evaluateHealth(estimate, time_s, ...
    nextState.last_correction_time_s, vectorAccepted, cfg.AttitudeHealth);

diagnostics.initialized_this_step = initializedThisStep;
diagnostics.prediction_accepted = predictionAccepted;
diagnostics.mag_accepted = magAccepted;
diagnostics.sun_accepted = sunAccepted;
diagnostics.mag_nis = magNis;
diagnostics.sun_nis = sunNis;

end


function [state, accepted] = predictGyro(state, report, status, previouslyInitialized, cfg)
%PREDICTGYRO Propagate attitude and covariance with an accepted gyro sample.

estimate = state.estimate;
estimate.valid = false;
state.estimate = estimate;
accepted = false;

omega_B = report.omega_rad_s - estimate.gyro_bias_B_rad_s;
dt = report.receive_time_s - estimate.update_time_s;

if ~previouslyInitialized || ~status.has_data || ~status.new_data ...
        || ~(report.valid > 0.5) || ~(omega_B.' * omega_B < Inf) ...
        || ~(dt > 0 && dt < Inf)
    return
end

adjacent = dt <= 1.5 * cfg.sample_time_s;
integratedRate_B = omega_B;

if state.have_previous_rate && adjacent
    integratedRate_B = 0.5 * (omega_B + state.previous_rate_B_rad_s);

    if state.have_two_rates && state.previous_gap_adjacent
        integratedRate_B = (5 * omega_B ...
            + 8 * state.previous_rate_B_rad_s ...
            - state.older_rate_B_rad_s) / 12;
    end

    integratedRate_B = integratedRate_B ...
        + dt / 12 * cross(state.previous_rate_B_rad_s, omega_B);
end

estimate.q_BI = normalizeQuaternion(quaternionProduct( ...
    estimate.q_BI, rotationQuaternion(dt * integratedRate_B)));

F = [-skew(omega_B), -eye(3); zeros(3), zeros(3)];
Phi = eye(6) + F * dt;
Qd = dt * blkdiag( ...
    cfg.MEKF.gyro_noise_density_rad_s_sqrt_Hz^2 * eye(3), ...
    cfg.MEKF.bias_random_walk_std_rad_s_sqrt_s^2 * eye(3));
P = Phi * estimate.P_error * Phi.' + Qd;

estimate.P_error = 0.5 * (P + P.');
estimate.omega_BI_B_rad_s = omega_B;
estimate.update_time_s = report.receive_time_s;
estimate.valid = true;

state.estimate = estimate;
state.older_rate_B_rad_s = state.previous_rate_B_rad_s;
state.previous_rate_B_rad_s = omega_B;
state.have_two_rates = state.have_previous_rate;
state.have_previous_rate = true;
state.previous_gap_adjacent = adjacent;
accepted = true;

end


function [estimate, accepted, nis] = correctVector( ...
    estimate, measured_B, reference_I, reportTime_s, reportValid, ...
    status, references, cfg, directionStd_rad)
%CORRECTVECTOR Correct attitude and gyro bias with a normalized vector measurement.

accepted = false;
nis = 0.0;
measuredNormSquared = measured_B.' * measured_B;
referenceNormSquared = reference_I.' * reference_I;
referenceAge_s = estimate.update_time_s - references.orbit_update_time_s;

if ~estimate.valid || ~status.has_data || ~status.new_data ...
        || ~(reportValid > 0.5) || ~(references.valid > 0.5) ...
        || ~(measuredNormSquared > 1e-24 && measuredNormSquared < Inf) ...
        || ~(referenceNormSquared > 1e-24 && referenceNormSquared < Inf) ...
        || ~(abs(reportTime_s - estimate.update_time_s) ...
            <= cfg.time_alignment_tolerance_s) ...
        || ~(referenceAge_s >= -cfg.time_alignment_tolerance_s ...
            && referenceAge_s <= cfg.reference_max_age_s)
    return
end

measured_B = measured_B / max(sqrt(measuredNormSquared), 1e-12);
reference_I = reference_I / max(sqrt(referenceNormSquared), 1e-12);
predicted_B = rotateInertialToBody(estimate.q_BI, reference_I);
innovation = measured_B - predicted_B;

H = [skew(predicted_B), zeros(3)];
P = estimate.P_error;
R = directionStd_rad^2 * eye(3);
S = H * P * H.' + R;

if any(~isfinite(S(:)))
    return
end

[L, flag] = chol(S, 'lower');
if flag ~= 0
    return
end

solution = L.' \ (L \ [H * P, innovation]);
K = solution(:, 1:6).';
nis = innovation.' * solution(:, 7);
delta = K * innovation;
deltaTheta = delta(1:3);
deltaBias = delta(4:6);

if ~(nis >= 0 && nis <= cfg.innovation_gate_squared) ...
        || ~(norm(deltaTheta) <= cfg.maximum_correction_rad)
    return
end

estimate.q_BI = normalizeQuaternion(quaternionProduct( ...
    estimate.q_BI, rotationQuaternion(deltaTheta)));
estimate.gyro_bias_B_rad_s = estimate.gyro_bias_B_rad_s + deltaBias;
estimate.omega_BI_B_rad_s = estimate.omega_BI_B_rad_s - deltaBias;

IminusKH = eye(6) - K * H;
Pcorrected = IminusKH * P * IminusKH.' + K * R * K.';
Pcorrected = 0.5 * (Pcorrected + Pcorrected.');
G = blkdiag(eye(3) - 0.5 * skew(deltaTheta), eye(3));
Pcorrected = G * Pcorrected * G.';

estimate.P_error = 0.5 * (Pcorrected + Pcorrected.');
accepted = true;

end


function health = evaluateHealth(estimate, time_s, correctionTime_s, vectorAccepted, cfg)
%EVALUATEHEALTH Classify attitude availability from uncertainty and measurement ages.

attitudeVariances = diag(estimate.P_error(1:3, 1:3));
attitudeStd_rad = sqrt(max(0, max(attitudeVariances)));
stateAge_s = time_s - estimate.update_time_s;
correctionAge_s = time_s - correctionTime_s;

notLost = attitudeStd_rad < cfg.lost_attitude_std_rad ...
    && stateAge_s >= 0 && stateAge_s <= cfg.lost_state_age_s ...
    && correctionAge_s >= 0 && correctionAge_s <= cfg.lost_correction_age_s ...
    && min(attitudeVariances) >= 0;
tracking = estimate.valid && attitudeStd_rad < cfg.degraded_attitude_std_rad ...
    && stateAge_s <= cfg.degraded_state_age_s ...
    && correctionAge_s <= cfg.degraded_correction_age_s;

if ~estimate.initialized
    mode = uint8(0);
elseif ~notLost
    mode = uint8(3);
elseif tracking
    mode = uint8(1);
else
    mode = uint8(2);
end

health.mode_id = mode;
health.control_usable = mode == uint8(1);
health.attitude_std_max_rad = attitudeStd_rad;
health.state_age_s = stateAge_s;
health.correction_age_s = correctionAge_s;
health.vector_update_accepted = vectorAccepted;

end


function q = rotationQuaternion(theta)
%ROTATIONQUATERNION Convert a rotation vector to a scalar first quaternion.

angle = sqrt(theta.' * theta);
scale = 0.5;
if angle > 1e-12
    scale = sin(0.5 * angle) / max(angle, 1e-12);
end

q = [cos(0.5 * angle); scale * theta];

end


function product = quaternionProduct(q, p)
%QUATERNIONPRODUCT Compute the Hamilton product of two quaternions.

product = [q(1) * p(1) - q(2:4).' * p(2:4); ...
    q(1) * p(2:4) + p(1) * q(2:4) + cross(q(2:4), p(2:4))];

end


function q = normalizeQuaternion(q)
%NORMALIZEQUATERNION Normalize a quaternion with a bounded denominator.

q = q / max(sqrt(q.' * q), 1e-12);

end


function vector_B = rotateInertialToBody(q_BI, vector_I)
%ROTATEINERTIALTOBODY Rotate a GCRF vector into body axes.

q0 = q_BI(1);
qv = q_BI(2:4);
C_BI = (q0^2 - qv.' * qv) * eye(3) ...
    + 2 * (qv * qv.') - 2 * q0 * skew(qv);
vector_B = C_BI * vector_I;

end


function S = skew(v)
%SKEW Create the cross product matrix of a vector.

S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];

end
