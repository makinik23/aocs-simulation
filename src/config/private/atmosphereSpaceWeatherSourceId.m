function id = atmosphereSpaceWeatherSourceId(source)
%ATMOSPHERESPACEWEATHERSOURCEID Map space-weather source names to bus IDs.

switch string(source)
    case "nominal"
        id = 1.0;
    case "file"
        id = 2.0;
    otherwise
        error("AOCS:Config:InvalidAtmosphere", ...
            "Unsupported atmosphere space weather source '%s'.", char(source));
end
end
