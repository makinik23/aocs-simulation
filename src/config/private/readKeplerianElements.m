function keplerian = readKeplerianElements(initialKeplerian)
%READKEPLERIANELEMENTS Validate classical Keplerian elements.

keplerian = struct();
keplerian.semi_major_axis_m = scalarField(initialKeplerian, ...
    "semi_major_axis_m", "orbit.initial_keplerian.semi_major_axis_m", true);
keplerian.eccentricity = scalarField(initialKeplerian, ...
    "eccentricity", "orbit.initial_keplerian.eccentricity", false);
keplerian.inclination_rad = scalarField(initialKeplerian, ...
    "inclination_rad", "orbit.initial_keplerian.inclination_rad", false);
keplerian.raan_rad = scalarField(initialKeplerian, ...
    "raan_rad", "orbit.initial_keplerian.raan_rad", false);
keplerian.argument_of_periapsis_rad = scalarField(initialKeplerian, ...
    "argument_of_periapsis_rad", "orbit.initial_keplerian.argument_of_periapsis_rad", false);
keplerian.true_anomaly_rad = scalarField(initialKeplerian, ...
    "true_anomaly_rad", "orbit.initial_keplerian.true_anomaly_rad", false);

if keplerian.eccentricity < 0 || keplerian.eccentricity >= 1
    error("AOCS:Config:InvalidKeplerianElements", ...
        "orbit.initial_keplerian.eccentricity must satisfy 0 <= e < 1 for the initial elliptical propagator.");
end

if keplerian.inclination_rad < 0 || keplerian.inclination_rad > pi
    error("AOCS:Config:InvalidKeplerianElements", ...
        "orbit.initial_keplerian.inclination_rad must satisfy 0 <= i <= pi.");
end
end
