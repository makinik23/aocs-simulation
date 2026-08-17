function inputs = prepareInputs(lla, environmentConfig, environmentContext)
% Description:
%   Prepares the fixed-shape input contract used by the DTM2020 operational
%   dtm3 and research dtm5 routines.

%#codegen

lla = lla(:);
latitude_rad = lla(1) * pi / 180.0;
longitude_rad = lla(2) * pi / 180.0;
time = dtm2020.modelTime(environmentContext.epoch_utc, ...
    environmentContext.t_s, longitude_rad);

f = zeros(2, 1);
fbar = zeros(2, 1);
akp = zeros(4, 1);
ap60 = zeros(10, 1);

if environmentConfig.atmosphere_mode_id <= 1.5
    f(1) = environmentConfig.f10_7_sfu;
    fbar(1) = environmentConfig.f10_7_81d_sfu;
    akp(1) = environmentConfig.kp;
    akp(3) = environmentConfig.kp;
else
    f(1) = rescaleF30(environmentConfig.f30_sfu, time.decimal_year);
    fbar(1) = rescaleF30(environmentConfig.f30_81d_sfu, time.decimal_year);
    nominalAp60 = geomagneticIndexToAp(environmentConfig.hp60);
    ap60(:) = nominalAp60;
end

inputs = struct();
inputs.mode_id = environmentConfig.atmosphere_mode_id;
inputs.year = time.year;
inputs.day_of_year = time.day_of_year;
inputs.decimal_year = time.decimal_year;
inputs.seconds_of_day = time.seconds_of_day;
inputs.latitude_rad = latitude_rad;
inputs.longitude_rad = longitude_rad;
inputs.altitude_km = max(lla(3), 0.0) / 1000.0;
inputs.local_solar_time_rad = time.local_solar_time_rad;
inputs.f = f;
inputs.fbar = fbar;
inputs.akp = akp;
inputs.ap60 = ap60;
end

function value = rescaleF30(f30_sfu, decimalYear)
% DTM2020 research-model conversion documented by the SWAMI reference code.
value = -1.5998 + 1.553755 * f30_sfu + ...
    (0.22446 * decimalYear - 447.13328);
end

function ap = geomagneticIndexToAp(index)
% Standard Kp/Hp to linear ap conversion, interpolated for decimal indices.
indexGrid = (0.0:(1.0 / 3.0):9.0);
apGrid = [0.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 9.0, 12.0, ...
    15.0, 18.0, 22.0, 27.0, 32.0, 39.0, 48.0, 56.0, 67.0, ...
    80.0, 94.0, 111.0, 132.0, 154.0, 179.0, 207.0, 236.0, ...
    300.0, 400.0];

index = min(max(index, indexGrid(1)), indexGrid(end));
segment = min(floor(index * 3.0) + 1.0, numel(indexGrid) - 1.0);
fraction = (index - indexGrid(segment)) / ...
    (indexGrid(segment + 1.0) - indexGrid(segment));
ap = apGrid(segment) + fraction * (apGrid(segment + 1.0) - apGrid(segment));
end
