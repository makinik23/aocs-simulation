function summary = computeDisturbanceSummary(B_NED_T, B_I_T, B_B_T, ...
    M_rmm_B_Nm, M_gg_B_Nm, M_srp_B_Nm, M_dist_B_Nm, ...
    F_srp_B_N, F_srp_I_N, a_srp_I_m_s2, a_aero_I_m_s2, a_dist_I_m_s2)
% Description:
%   Computes reusable magnetic-field, disturbance-torque, and acceleration metrics.
%
% Arguments:
%   B_NED_T - N-by-3 magnetic field in NED axes [T].
%   B_I_T - N-by-3 magnetic field in inertial axes [T].
%   B_B_T - N-by-3 magnetic field in body axes [T].
%   M_rmm_B_Nm - N-by-3 residual magnetic moment torque [N*m].
%   M_gg_B_Nm - N-by-3 gravity-gradient torque [N*m].
%   M_srp_B_Nm - N-by-3 solar radiation pressure torque [N*m].
%   M_dist_B_Nm - N-by-3 total modeled disturbance torque [N*m].
%   F_srp_B_N - Optional N-by-3 solar radiation pressure force in body axes [N].
%   F_srp_I_N - Optional N-by-3 solar radiation pressure force in inertial axes [N].
%   a_srp_I_m_s2 - Optional N-by-3 solar radiation pressure acceleration [m/s^2].
%   a_aero_I_m_s2 - Optional N-by-3 aerodynamic acceleration [m/s^2].
%   a_dist_I_m_s2 - Optional N-by-3 total disturbance acceleration [m/s^2].
%
% Outputs:
%   summary - Struct with norm summaries and frame consistency metrics.

summary = struct();

summary.Magnetic = struct();
summary.Magnetic.B_NED_norm_uT = vectorNormSummary(B_NED_T .* 1e6);
summary.Magnetic.B_I_norm_uT = vectorNormSummary(B_I_T .* 1e6);
summary.Magnetic.B_B_norm_uT = vectorNormSummary(B_B_T .* 1e6);
summary.Magnetic.B_I_B_norm_max_diff_T = max(abs(vecnorm(B_I_T, 2, 2) - vecnorm(B_B_T, 2, 2)));

summary.Torques = struct();
summary.Torques.M_rmm_norm_nNm = vectorNormSummary(M_rmm_B_Nm .* 1e9);
summary.Torques.M_gg_norm_nNm = vectorNormSummary(M_gg_B_Nm .* 1e9);
summary.Torques.M_srp_norm_nNm = vectorNormSummary(M_srp_B_Nm .* 1e9);
summary.Torques.M_dist_norm_nNm = vectorNormSummary(M_dist_B_Nm .* 1e9);
summary.Torques.M_dist_norm_Nm = vectorNormSummary(M_dist_B_Nm);

if nargin >= 9
    summary.Forces = struct();
    summary.Forces.F_srp_B_norm_nN = vectorNormSummary(F_srp_B_N .* 1e9);
    summary.Forces.F_srp_I_norm_nN = vectorNormSummary(F_srp_I_N .* 1e9);
    summary.Forces.F_srp_B_I_norm_max_diff_N = ...
        max(abs(vecnorm(F_srp_B_N, 2, 2) - vecnorm(F_srp_I_N, 2, 2)));
end

if nargin >= 12
    summary.Accelerations = struct();
    summary.Accelerations.a_srp_norm_nm_s2 = vectorNormSummary(a_srp_I_m_s2 .* 1e9);
    summary.Accelerations.a_aero_norm_nm_s2 = vectorNormSummary(a_aero_I_m_s2 .* 1e9);
    summary.Accelerations.a_dist_norm_nm_s2 = vectorNormSummary(a_dist_I_m_s2 .* 1e9);
    summary.Accelerations.a_dist_norm_m_s2 = vectorNormSummary(a_dist_I_m_s2);
end
end

function stats = vectorNormSummary(data)
% Description:
%   Returns min/mean/max of vector norms as named scalar fields.

norms = vecnorm(data, 2, 2);

stats = struct();
stats.Min = min(norms);
stats.Mean = mean(norms);
stats.Max = max(norms);
end
