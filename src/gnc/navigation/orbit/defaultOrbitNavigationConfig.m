function cfg = defaultOrbitNavigationConfig(AOCS)
    %DEFAULTORBITNAVIGATIONCONFIG Create the orbital EKF configuration.
    
    cfg.sample_time_s = AOCS.GNC.SampleTime_s;
    cfg.mu_m3_s2 = AOCS.Orbit.CentralBodyConstants.mu_m3_s2;
    cfg.earth_radius_m = AOCS.Orbit.CentralBodyConstants.radius_m;
    cfg.J2 = 1.08262668e-3;

    eop = AOCS.Environment.EarthOrientation;
    
    C_FI = dcmeci2ecef("IAU-2000/2006", ...
        AOCS.Epoch.Utc(:).', ...
        eop.DeltaAT_s, eop.DeltaUT1_s, eop.PolarMotion_rad, ...
        "dCIP", eop.DCIP_rad);
    
    cfg.pole_I = C_FI.' * [0; 0; 1];
    cfg.acceleration_noise_psd_m2_s3 = 1e-8;
    
    gnss = AOCS.Sensors.GNSS;
    
    positionVariance = gnss.AntennaNoiseScale^2 * ( ...
        gnss.WhiteNoise.PositionStd_RTN_m.^2 ...
        + gnss.GaussMarkov.PositionStd_RTN_m.^2);
    
    velocityVariance = gnss.AntennaNoiseScale^2 * ( ...
        gnss.WhiteNoise.VelocityStd_RTN_m_s.^2 ...
        + gnss.GaussMarkov.VelocityStd_RTN_m_s.^2);
    
    cfg.gnss_covariance_RTN = diag( ...
    [positionVariance; velocityVariance]);
    
    initialState.x_hat = zeros(6, 1);
    initialState.P = zeros(6, 6);
    initialState.epoch_s = 0.0;
    initialState.initialized = false;
    initialState.last_accepted_gnss_time_s = -1.0;
    initialState.last_processed_sequence_id = uint32(0);
    initialState.solution_id = uint32(0);
    cfg.initial_state = initialState;
end
