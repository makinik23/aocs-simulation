function state = propagateOnboardSgp4(input)
%PROPAGATEONBOARDSGP4 Propagate the onboard orbit using SGP4.

arguments
    input (9, 1) double
end

t_s = max(input(1), 0.0);
epochUtcJd = input(2);
semiMajorAxis_m = input(3);
eccentricity = input(4);
inclination_deg = input(5);
raan_deg = input(6);
argumentOfPeriapsis_deg = input(7);
trueAnomaly_deg = input(8);
bstar = input(9);

try
    currentUtcJd = epochUtcJd + t_s / 86400.0;
    [r_I_m, v_I_m_s] = propagateOrbit(currentUtcJd, ...
        semiMajorAxis_m, eccentricity, inclination_deg, raan_deg, ...
        argumentOfPeriapsis_deg, trueAnomaly_deg, ...
        Epoch=epochUtcJd, PropModel="sgp4", BStar=bstar, ...
        OutputCoordinateFrame="inertial");
    state = [reshape(r_I_m, 3, 1); reshape(v_I_m_s, 3, 1); 1.0];
catch
    state = zeros(7, 1);
end
end
