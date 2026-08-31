function id = gyroModeId(mode)
%GYROMODEID Map gyro operating modes to numeric bus IDs.

switch string(mode)
    case "nominal"
        id = 1.0;
    case "failure"
        id = 2.0;
    otherwise
        error("AOCS:Config:InvalidGyro", ...
            "Unsupported gyro mode '%s'.", char(mode));
end
end
