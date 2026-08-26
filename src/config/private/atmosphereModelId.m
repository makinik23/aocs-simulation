function id = atmosphereModelId(model)
%ATMOSPHEREMODELID Map atmosphere model names to numeric bus IDs.

switch string(model)
    case "dtm2020"
        id = 1.0;
    otherwise
        error("AOCS:Config:InvalidAtmosphere", ...
            "Unsupported atmosphere model '%s'.", char(model));
end
end
