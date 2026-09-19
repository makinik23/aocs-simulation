function id = gnssModeId(mode)
%GNSSMODEID Convert GNSS receiver mode string to numeric bus value.

switch string(mode)
    case "nominal"
        id = 1.0;
    case "dropout"
        id = 2.0;
    case "failure"
        id = 3.0;
    otherwise
        error("AOCS:Config:InvalidSensorMode", ...
            "Unsupported GNSS mode '%s'.", char(mode));
end
end
