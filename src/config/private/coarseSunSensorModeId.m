function id = coarseSunSensorModeId(mode)
%COARSESUNSENSORMODEID Convert coarse-sun-sensor mode string to numeric bus value.

switch string(mode)
    case "nominal"
        id = 1.0;
    case "failure"
        id = 2.0;
    otherwise
        error("AOCS:Config:InvalidSensorMode", ...
            "Unsupported coarse sun sensor mode '%s'.", char(mode));
end
end
