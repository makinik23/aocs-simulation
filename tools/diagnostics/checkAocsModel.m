function report = checkAocsModel(model)
%CHECKAOCSMODEL Enforce the project's documented, small MAB-inspired subset.
% This is not the licensed MAB Model Advisor suite or a compliance certificate.
if nargin < 1
    model = 'aocs_plant';
end
load_system(model);
violations = strings(0,1);
parameters = ["UnconnectedInputMsg", "UnconnectedOutputMsg", ...
    "UnconnectedLineMsg"];
for parameter = parameters
    if string(get_param(model, parameter)) ~= "error"
        violations(end+1,1) = parameter + " must be error"; %#ok<AGROW>
    end
end
if strcmp(get_param(model, 'AssertionControl'), 'DisableAll')
    violations(end+1,1) = "Model verification must not be globally disabled";
end
check = string(model) + '/Flight Dynamics/Numerical Checks/Finite Plant State';
if getSimulinkBlockHandle(check) < 0
    violations(end+1,1) = "Missing finite plant-state assertion";
elseif ~strcmp(get_param(check, 'enabled'), 'on')
    violations(end+1,1) = "Finite plant-state assertion is disabled";
elseif ~strcmp(get_param(check, 'stopWhenAssertionFail'), 'on')
    violations(end+1,1) = "Finite plant-state assertion must stop the simulation";
end
chart = find(sfroot, '-isa', 'Stateflow.EMChart', ...
    'Path', char(string(model) + '/Flight Dynamics/Numerical Checks/Finite Values'));
if isempty(chart) || ~strcmp(chart.Script, plantFiniteCheckScript())
    violations(end+1,1) = "Finite-value monitor does not match the current bus schema";
end
% Check project-owned blocks, not the implementation of vendor library links.
blocks = find_system(model, 'LookUnderMasks', 'all', 'FollowLinks', 'off', 'Type', 'Block');
for k=1:numel(blocks)
    block = blocks{k};
    if ~strcmp(get_param(block, 'LinkStatus'), 'none')
        continue;
    end
    ports = get_param(block, 'PortHandles');
    for port = [ports.Inport ports.Outport]
        if get_param(port, 'Line') < 0
            violations(end+1,1) = string(block) + ": unconnected data port"; %#ok<AGROW>
        end
    end
end
lines = find_system(model, 'FindAll', 'on', 'LookUnderMasks', 'all', ...
    'FollowLinks', 'off', 'Type', 'line');
for line = lines(:).'
    destination = get_param(line, 'DstPortHandle');
    if isempty(get_param(line, 'LineChildren')) && ...
            (isempty(destination) || all(destination < 0))
        violations(end+1,1) = string(get_param(line, 'Parent')) + ...
            ": dangling signal line or branch"; %#ok<AGROW>
    end
end
report = struct('Model', string(model), 'Violations', violations, ...
    'Passed', isempty(violations), 'Scope', "Project MAB subset; see docs/engineering.md");
if ~report.Passed
    error('AOCS:Model:Guidelines', '%s', strjoin(violations, newline));
end
end
