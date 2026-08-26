function id = atmosphereModeId(mode)
%ATMOSPHEREMODEID Map atmosphere model driver modes to numeric bus IDs.

switch string(mode)
    case "operational"
        id = 1.0;
    case "research"
        id = 2.0;
    otherwise
        error("AOCS:Config:InvalidAtmosphere", ...
            "Unsupported atmosphere mode '%s'.", char(mode));
end
end
