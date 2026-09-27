function script = plantFiniteCheckScript()
%PLANTFINITECHECKSCRIPT Generate a check for every numeric plant-state leaf.
paths = numericLeaves("PlantStateBus", "state");
script = "function ok = finiteState(state)" + newline + ...
    "%#codegen" + newline + "ok = true;";
for path = paths(:).'
    script = script + newline + "ok = ok && all(isfinite(" + path + "(:)));";
end
script = char(script + newline + "end" + newline);
end

function paths = numericLeaves(busName, prefix)
bus = evalin("base", busName);
paths = strings(0, 1);
for element = bus.Elements(:).'
    name = string(prefix) + "." + element.Name;
    dataType = string(element.DataType);
    if startsWith(dataType, "Bus: ")
        nestedBus = extractAfter(dataType, "Bus: ");
        paths = [paths; numericLeaves(nestedBus, name)]; %#ok<AGROW>
    else
        paths(end + 1, 1) = name; %#ok<AGROW>
    end
end
end
