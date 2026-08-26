function orbit = computeOrbitDiagnostics(r_I_m, v_I_m_s, AOCS)
% Description:
%   Computes scalar orbit diagnostics useful for plotting, validation, and
%   downstream guidance/navigation/control telemetry products.
%
% Arguments:
%   r_I_m - N-by-3 inertial position samples [m].
%   v_I_m_s - N-by-3 inertial velocity samples [m/s].
%   AOCS - Validated configuration struct.
%
% Outputs:
%   orbit - Struct of orbit diagnostics.

mu = AOCS.Orbit.CentralBodyConstants.mu_m3_s2;
earthRadius_m = AOCS.Orbit.CentralBodyConstants.radius_m;

rNorm_m = vecnorm(r_I_m, 2, 2);
vNorm_m_s = vecnorm(v_I_m_s, 2, 2);
rHat_I = r_I_m ./ rNorm_m;

radialSpeed_m_s = sum(r_I_m .* v_I_m_s, 2) ./ rNorm_m;
tangentialSpeed_m_s = sqrt(max(vNorm_m_s.^2 - radialSpeed_m_s.^2, 0));

h_I_m2_s = cross(r_I_m, v_I_m_s, 2);
hNorm_m2_s = vecnorm(h_I_m2_s, 2, 2);
inclination_rad = acos(clamp(h_I_m2_s(:, 3) ./ hNorm_m2_s, -1, 1));

specificEnergy_J_kg = 0.5 .* vNorm_m_s.^2 - mu ./ rNorm_m;
semiMajorAxis_m = -mu ./ (2 .* specificEnergy_J_kg);
eccentricityVector = cross(v_I_m_s, h_I_m2_s, 2) ./ mu - rHat_I;
eccentricity = vecnorm(eccentricityVector, 2, 2);
period_s = 2 * pi * sqrt(mean(semiMajorAxis_m, "omitnan")^3 / mu);

orbit = struct();
orbit.Radius_m = rNorm_m;
orbit.Altitude_m = rNorm_m - earthRadius_m;
orbit.Speed_m_s = vNorm_m_s;
orbit.RadialSpeed_m_s = radialSpeed_m_s;
orbit.TangentialSpeed_m_s = tangentialSpeed_m_s;
orbit.SpecificEnergy_J_kg = specificEnergy_J_kg;
orbit.SpecificAngularMomentum_m2_s = hNorm_m2_s;
orbit.SemiMajorAxis_m = semiMajorAxis_m;
orbit.Eccentricity = eccentricity;
orbit.Inclination_rad = inclination_rad;
orbit.EstimatedPeriod_s = period_s;
end

function value = clamp(value, lowerBound, upperBound)
% Description:
%   Clamps numeric values to a closed interval.

value = min(max(value, lowerBound), upperBound);
end
