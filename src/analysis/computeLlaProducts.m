function lla = computeLlaProducts(r_I_m, t_s, AOCS)
% Description:
%   Converts inertial position samples to geodetic latitude, longitude, and
%   altitude using the same IAU-2000/2006 reduction family used in the model.
%
% Arguments:
%   r_I_m - N-by-3 inertial position samples [m].
%   t_s - N-by-1 simulation time samples [s].
%   AOCS - Validated configuration struct.
%
% Outputs:
%   lla - Struct with Latitude_deg, Longitude_deg, and Altitude_m.

epoch = AOCS.Epoch.Utc;
epochDate = datetime(epoch(1), epoch(2), epoch(3), epoch(4), epoch(5), epoch(6), ...
    "TimeZone", "UTC");
utc = datevec(epochDate + seconds(t_s));

llaData = eci2lla(r_I_m, utc, "IAU-2000/2006");

lla = struct();
lla.Latitude_deg = llaData(:, 1);
lla.Longitude_deg = wrapDegrees180(llaData(:, 2));
lla.Altitude_m = llaData(:, 3);
end

function wrapped = wrapDegrees180(degrees)
% Description:
%   Wraps degrees to [-180, 180).

wrapped = mod(degrees + 180, 360) - 180;
end
