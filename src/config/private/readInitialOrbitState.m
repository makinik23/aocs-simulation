function state = readInitialOrbitState(orbit, defaultKeplerian)
% Description:
%   Reads the optional orbit.initial_state section while preserving the legacy
%   orbit.initial_keplerian source as the default initial state.
%
% Arguments:
%   orbit - JSON orbit object from the merged AOCS configuration.
%   defaultKeplerian - Validated Keplerian elements from orbit.initial_keplerian.
%
% Outputs:
%   state - Struct containing Type, Keplerian, and Cartesian initial states.

state = defaultInitialOrbitState(defaultKeplerian);
initialState = optionalStructField(orbit, "initial_state");

if isempty(initialState)
    return;
end

state.Type = enumStringField(initialState, "type", "orbit.initial_state.type", ...
    ["keplerian", "cartesian"]);

switch state.Type
    case "keplerian"
        if isfield(initialState, "keplerian")
            state.Keplerian = readKeplerianElements(requireStruct( ...
                initialState, "keplerian", "orbit.initial_state.keplerian"));
        end

    case "cartesian"
        state.Cartesian.position_I_m = columnField(initialState, ...
            "position_I_m", "orbit.initial_state.position_I_m", 3);
        state.Cartesian.velocity_I_m_s = columnField(initialState, ...
            "velocity_I_m_s", "orbit.initial_state.velocity_I_m_s", 3);
end
end

function state = defaultInitialOrbitState(keplerian)
% Description:
%   Builds the backwards-compatible initial-state struct.
%
% Arguments:
%   keplerian - Validated legacy Keplerian elements.
%
% Outputs:
%   state - Initial-state struct with zeroed Cartesian placeholders.

state = struct();
state.Type = "keplerian";
state.Keplerian = keplerian;
state.Cartesian = struct( ...
    "position_I_m", zeros(3, 1), ...
    "velocity_I_m_s", zeros(3, 1));
end
