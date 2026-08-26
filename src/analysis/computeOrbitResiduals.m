function residuals = computeOrbitResiduals( ...
    timeModel_s, rModel_I_m, vModel_I_m_s, timeRef_s, rRef_I_m, vRef_I_m_s)
% Description:
%   Interpolates modeled orbit states to reference epochs and computes
%   inertial and RTN-frame position/velocity residual diagnostics.
%
% Arguments:
%   timeModel_s - M-by-1 modeled elapsed times [s].
%   rModel_I_m - M-by-3 modeled inertial position samples [m].
%   vModel_I_m_s - M-by-3 modeled inertial velocity samples [m/s].
%   timeRef_s - N-by-1 reference elapsed times [s].
%   rRef_I_m - N-by-3 reference inertial position samples [m].
%   vRef_I_m_s - N-by-3 reference inertial velocity samples [m/s].
%
% Outputs:
%   residuals - Struct containing interpolated states, residual vectors, RTN
%               components, and scalar summary statistics.

timeModel_s = timeModel_s(:);
timeRef_s = timeRef_s(:);
validateOrbitResidualInputs(timeModel_s, rModel_I_m, vModel_I_m_s, ...
    timeRef_s, rRef_I_m, vRef_I_m_s);

rModelAtRef_I_m = interp1(timeModel_s, rModel_I_m, timeRef_s, "linear");
vModelAtRef_I_m_s = interp1(timeModel_s, vModel_I_m_s, timeRef_s, "linear");

positionError_I_m = rModelAtRef_I_m - rRef_I_m;
velocityError_I_m_s = vModelAtRef_I_m_s - vRef_I_m_s;

rtnBasis = referenceRtnBasis(rRef_I_m, vRef_I_m_s);
positionError_RTN_m = projectRows(positionError_I_m, rtnBasis);
velocityError_RTN_m_s = projectRows(velocityError_I_m_s, rtnBasis);

residuals = struct();
residuals.Time_s = timeRef_s;
residuals.ModelAtReference.r_I_m = rModelAtRef_I_m;
residuals.ModelAtReference.v_I_m_s = vModelAtRef_I_m_s;
residuals.Reference.r_I_m = rRef_I_m;
residuals.Reference.v_I_m_s = vRef_I_m_s;
residuals.PositionError_I_m = positionError_I_m;
residuals.VelocityError_I_m_s = velocityError_I_m_s;
residuals.PositionError_RTN_m = positionError_RTN_m;
residuals.VelocityError_RTN_m_s = velocityError_RTN_m_s;
residuals.Summary = residualSummary(positionError_I_m, velocityError_I_m_s, ...
    positionError_RTN_m, velocityError_RTN_m_s);
end

function validateOrbitResidualInputs( ...
    timeModel_s, rModel_I_m, vModel_I_m_s, timeRef_s, rRef_I_m, vRef_I_m_s)
% Description:
%   Validates dimensions, finiteness, and interpolation coverage.

validateattributes(timeModel_s, {'numeric'}, {'real', 'finite', 'column'}, ...
    mfilename, "timeModel_s");
validateattributes(timeRef_s, {'numeric'}, {'real', 'finite', 'column'}, ...
    mfilename, "timeRef_s");
validateattributes(rModel_I_m, {'numeric'}, {'real', 'finite', 'size', [numel(timeModel_s), 3]}, ...
    mfilename, "rModel_I_m");
validateattributes(vModel_I_m_s, {'numeric'}, {'real', 'finite', 'size', [numel(timeModel_s), 3]}, ...
    mfilename, "vModel_I_m_s");
validateattributes(rRef_I_m, {'numeric'}, {'real', 'finite', 'size', [numel(timeRef_s), 3]}, ...
    mfilename, "rRef_I_m");
validateattributes(vRef_I_m_s, {'numeric'}, {'real', 'finite', 'size', [numel(timeRef_s), 3]}, ...
    mfilename, "vRef_I_m_s");

if any(diff(timeModel_s) <= 0.0) || any(diff(timeRef_s) <= 0.0)
    error("AOCS:Analysis:NonMonotonicOrbitTime", ...
        "Modeled and reference orbit times must be strictly increasing.");
end

if timeRef_s(1) < timeModel_s(1) || timeRef_s(end) > timeModel_s(end)
    error("AOCS:Analysis:ReferenceOutsideModelTime", ...
        "Reference epochs must lie within the modeled time span.");
end
end

function basis = referenceRtnBasis(r_I_m, v_I_m_s)
% Description:
%   Builds row-wise RTN basis vectors from reference inertial states.

rHat = normalizeRows(r_I_m);
hHat = normalizeRows(cross(r_I_m, v_I_m_s, 2));
tHat = normalizeRows(cross(hHat, rHat, 2));

basis = struct();
basis.R = rHat;
basis.T = tHat;
basis.N = hHat;
end

function projected = projectRows(vectors_I, basis)
% Description:
%   Projects inertial row vectors into row-wise RTN basis components.

projected = [ ...
    sum(vectors_I .* basis.R, 2), ...
    sum(vectors_I .* basis.T, 2), ...
    sum(vectors_I .* basis.N, 2)];
end

function rows = normalizeRows(rows)
% Description:
%   Normalizes each row vector and rejects zero-norm rows.

norms = vecnorm(rows, 2, 2);
if any(norms <= 0.0)
    error("AOCS:Analysis:DegenerateRtnBasis", ...
        "Reference position and angular momentum rows must be non-zero.");
end

rows = rows ./ norms;
end

function summary = residualSummary( ...
    positionError_I_m, velocityError_I_m_s, positionError_RTN_m, velocityError_RTN_m_s)
% Description:
%   Computes scalar position, velocity, and component-wise RTN residual stats.

positionNorm_m = vecnorm(positionError_I_m, 2, 2);
velocityNorm_m_s = vecnorm(velocityError_I_m_s, 2, 2);

summary = struct();
summary.Position.Norm_m = vectorStats(positionNorm_m);
summary.Position.RTN_m = componentStats(positionError_RTN_m);
summary.Velocity.Norm_m_s = vectorStats(velocityNorm_m_s);
summary.Velocity.RTN_m_s = componentStats(velocityError_RTN_m_s);
end

function stats = vectorStats(values)
% Description:
%   Computes RMS, max, final, and mean statistics for a scalar vector.

values = values(:);
stats = struct();
stats.Rms = sqrt(mean(values.^2));
stats.Max = max(values);
stats.Final = values(end);
stats.Mean = mean(values);
end

function stats = componentStats(values)
% Description:
%   Computes per-component RMS, maximum absolute, final, and mean statistics.

stats = struct();
stats.Rms = sqrt(mean(values.^2, 1));
stats.MaxAbs = max(abs(values), [], 1);
stats.Final = values(end, :);
stats.Mean = mean(values, 1);
end
