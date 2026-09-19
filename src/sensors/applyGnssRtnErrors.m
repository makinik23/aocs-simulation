function [rMeasured_I_m, vMeasured_I_m_s, C_I_RTN] = applyGnssRtnErrors( ...
    rTruth_I_m, vTruth_I_m_s, positionError_RTN_m, velocityError_RTN_m_s)
% Description:
%   Applies local radial, along-track, and cross-track GNSS PVT errors to
%   an inertial truth state. The RTN frame is formed from the sampled truth
%   position and velocity and expressed by C_I_RTN = [R T N].
%
% Arguments:
%   rTruth_I_m - Truth inertial position [3x1, m].
%   vTruth_I_m_s - Truth inertial velocity [3x1, m/s].
%   positionError_RTN_m - Position error in the RTN frame [3x1, m].
%   velocityError_RTN_m_s - Velocity error in the RTN frame [3x1, m/s].
%
% Outputs:
%   rMeasured_I_m - Inertial position measurement [3x1, m].
%   vMeasured_I_m_s - Inertial velocity measurement [3x1, m/s].
%   C_I_RTN - Direction-cosine matrix mapping RTN vectors to inertial [3x3].

radialNorm = norm(rTruth_I_m);
orbitNormal_I = cross(rTruth_I_m, vTruth_I_m_s);
normalNorm = norm(orbitNormal_I);

if radialNorm <= 1.0e-12 || normalNorm <= 1.0e-12
    C_I_RTN = eye(3);
else
    radial_I = rTruth_I_m / radialNorm;
    normal_I = orbitNormal_I / normalNorm;
    transverse_I = cross(normal_I, radial_I);
    transverseNorm = norm(transverse_I);

    if transverseNorm <= 1.0e-12
        C_I_RTN = eye(3);
    else
        transverse_I = transverse_I / transverseNorm;
        C_I_RTN = [radial_I, transverse_I, normal_I];
    end
end

rMeasured_I_m = rTruth_I_m + C_I_RTN * positionError_RTN_m;
vMeasured_I_m_s = vTruth_I_m_s + C_I_RTN * velocityError_RTN_m_s;
end
