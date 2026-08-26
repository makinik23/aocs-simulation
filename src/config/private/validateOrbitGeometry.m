function validateOrbitGeometry(keplerian, centralBodyRadius_m)
%VALIDATEORBITGEOMETRY Check that the initial ellipse is above the body.

periapsisRadius_m = keplerian.semi_major_axis_m * (1 - keplerian.eccentricity);
if periapsisRadius_m <= centralBodyRadius_m
    error("AOCS:Config:InvalidKeplerianElements", ...
        "orbit.initial_keplerian gives a periapsis radius below the central-body radius.");
end
end
