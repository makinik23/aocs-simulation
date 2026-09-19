function id = magnetometerModeId(mode)
%MAGNETOMETERMODEID Convert magnetometer mode string to numeric bus value.

switch string(mode)
    case "nominal"
        id = 1.0;
    case "failure"
        id = 2.0;
    otherwise
        error("AOCS:Config:InvalidSensorMode", ...
            "Unsupported magnetometer mode '%s'.", char(mode));
end
end
