function nativeInput = prepareNativeInput(lla, environmentConfig, environmentContext)
% Description:
%   Packs the operational DTM2020 S-Function input vector.

%#codegen

inputs = dtm2020.prepareInputs(lla, environmentConfig, environmentContext);
nativeInput = zeros(9, 1);
nativeInput(1) = inputs.day_of_year;
nativeInput(2) = inputs.f(1);
nativeInput(3) = inputs.fbar(1);
nativeInput(4) = inputs.akp(1);
nativeInput(5) = inputs.akp(3);
nativeInput(6) = inputs.altitude_km;
nativeInput(7) = inputs.local_solar_time_rad;
nativeInput(8) = inputs.latitude_rad;
nativeInput(9) = inputs.longitude_rad;
end
