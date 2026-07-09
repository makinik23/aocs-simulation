function AOCS = loadAocsSimulationConfig(configFile, projectRoot)
% Description:
%   Reads the JSON source of truth, validates required fields, normalizes the
%   initial quaternion, derives Aerospace Blockset Euler initial conditions,
%   and builds resolved paths for the model and results.
%
% Arguments:
%   configFile - Optional path to an AocsSimulationConfig JSON file.
%   projectRoot - Optional project root used to resolve model and results paths.
%
% Outputs:
%   AOCS - Validated configuration struct. AOCS.Config contains only the
%          numeric plant inputs exposed through AOCS_ConfigBus.

if nargin < 2 || strlength(string(projectRoot)) == 0
    projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
end

if nargin < 1 || strlength(string(configFile)) == 0
    configFile = fullfile(projectRoot, "config", "AocsSimulationConfig.json");
end

if ~isfile(configFile)
    error("AOCS:Config:MissingFile", "AOCS config file not found: %s", configFile);
end

raw = jsondecode(fileread(configFile));

schema = string(requireField(raw, "schema", "schema"));
if schema ~= "AocsSimulationConfig/v1"
    error("AOCS:Config:UnsupportedSchema", ...
        "Unsupported AOCS config schema '%s'. Expected 'AocsSimulationConfig/v1'.", char(schema));
end

mission = requireStruct(raw, "mission", "mission");
models = requireStruct(raw, "models", "models");
results = requireStruct(raw, "results", "results");
sim = requireStruct(raw, "simulation", "simulation");
spacecraft = requireStruct(raw, "spacecraft", "spacecraft");
massProps = requireStruct(spacecraft, "mass_properties", "spacecraft.mass_properties");
initial = requireStruct(raw, "initial_conditions", "initial_conditions");
environment = requireStruct(raw, "environment", "environment");
numerics = requireStruct(raw, "numerics", "numerics");
conventions = requireStruct(raw, "conventions", "conventions");

I_B = matrixField(massProps, "inertia_B_kg_m2", "spacecraft.mass_properties.inertia_B_kg_m2", 3, 3);
if any(any(abs(I_B - I_B.') > 1e-12))
    error("AOCS:Config:InvalidInertia", "Inertia matrix I_B must be symmetric.");
end

if any(eig(I_B) <= 0)
    error("AOCS:Config:InvalidInertia", "Inertia matrix I_B must be positive definite.");
end

q_BI = columnField(initial, "q_BI", "initial_conditions.q_BI", 4);
qNorm = norm(q_BI);
if qNorm <= 0
    error("AOCS:Config:InvalidQuaternion", "Initial quaternion q_BI must have non-zero norm.");
end
q_BI = q_BI ./ qNorm;

euler_BI_0_rad = optionalColumnField(initial, "euler_BI_0_rad", "initial_conditions.euler_BI_0_rad", 3, quaternionToEuler321(q_BI));
omega_BI_B = columnField(initial, "omega_BI_B_rad_s", "initial_conditions.omega_BI_B_rad_s", 3);
M_ext_B = columnField(environment, "external_torque_B_Nm", "environment.external_torque_B_Nm", 3);

AOCS = struct();
AOCS.Meta.Schema = schema;
AOCS.Meta.ProjectRoot = string(projectRoot);
AOCS.Meta.ConfigFile = string(configFile);
AOCS.Meta.MissionName = string(requireField(mission, "name", "mission.name"));
AOCS.Meta.Description = string(optionalField(mission, "description", ""));

AOCS.Model.Name = string(requireField(models, "plant", "models.plant"));
AOCS.Model.Directory = fullfile(projectRoot, string(requireField(models, "directory", "models.directory")));
AOCS.Model.File = fullfile(AOCS.Model.Directory, AOCS.Model.Name + ".slx");

AOCS.Results.Directory = fullfile(projectRoot, string(requireField(results, "directory", "results.directory")));
AOCS.Results.File = fullfile(AOCS.Results.Directory, string(requireField(results, "file", "results.file")));

AOCS.Sim.StartTime_s = scalarField(sim, "start_time_s", "simulation.start_time_s", false);
AOCS.Sim.StopTime_s = scalarField(sim, "stop_time_s", "simulation.stop_time_s", true);
AOCS.Sim.SampleTime_s = scalarField(sim, "sample_time_s", "simulation.sample_time_s", true);
AOCS.Sim.Solver = string(requireField(sim, "solver", "simulation.solver"));
AOCS.Sim.RelTol = scalarField(sim, "relative_tolerance", "simulation.relative_tolerance", true);
AOCS.Sim.AbsTol = scalarField(sim, "absolute_tolerance", "simulation.absolute_tolerance", true);

if AOCS.Sim.StopTime_s <= AOCS.Sim.StartTime_s
    error("AOCS:Config:InvalidTimeSpan", "simulation.stop_time_s must be greater than simulation.start_time_s.");
end

AOCS.Spacecraft.Id = string(requireField(spacecraft, "id", "spacecraft.id"));
AOCS.Spacecraft.I_B = I_B;

AOCS.Initial.q_BI = q_BI;
AOCS.Initial.euler_BI_0_rad = euler_BI_0_rad;
AOCS.Initial.omega_BI_B = omega_BI_B;

AOCS.Environment.M_ext_B = M_ext_B;

AOCS.Numerics.QuatNormEpsilon = scalarField(numerics, "quat_norm_epsilon", "numerics.quat_norm_epsilon", true);
AOCS.Numerics.MaxAllowedEnergyDrift = scalarField(numerics, "max_allowed_energy_drift", "numerics.max_allowed_energy_drift", true);
AOCS.Numerics.MaxAllowedHnormDrift = scalarField(numerics, "max_allowed_Hnorm_drift", "numerics.max_allowed_Hnorm_drift", true);

AOCS.Convention.QuaternionOrder = string(requireField(conventions, "quaternion_order", "conventions.quaternion_order"));
AOCS.Convention.q_BI = string(requireField(conventions, "q_BI", "conventions.q_BI"));
AOCS.Convention.C_BI = string(requireField(conventions, "C_BI", "conventions.C_BI"));
AOCS.Convention.omega_BI_B = string(requireField(conventions, "omega_BI_B", "conventions.omega_BI_B"));

AOCS.Config = buildBusConfig(AOCS);
AOCS.Raw = raw;
end

function config = buildBusConfig(AOCS)
% Description:
%   Selects only plant-facing numeric values from the full configuration.
%
% Arguments:
%   AOCS - Validated AOCS configuration struct.
%
% Outputs:
%   config - Struct matching createAocsConfigBus element names and dimensions.

config = struct();
config.I_B = AOCS.Spacecraft.I_B;
config.euler_BI_0_rad = AOCS.Initial.euler_BI_0_rad;
config.omega_BI_B_0 = AOCS.Initial.omega_BI_B;
config.M_ext_B = AOCS.Environment.M_ext_B;
end

function section = requireStruct(parent, fieldName, displayName)
% Description:
%   Combines required-field lookup with a type check for JSON object sections.
%
% Arguments:
%   parent - Struct containing the requested field.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%
% Outputs:
%   section - Struct stored in parent.(fieldName).

section = requireField(parent, fieldName, displayName);
if ~isstruct(section)
    error("AOCS:Config:InvalidField", "Config field %s must be an object.", displayName);
end
end

function value = requireField(parent, fieldName, displayName)
% Description:
%   Throws a focused config error when a required JSON field is missing.
%
% Arguments:
%   parent - Struct containing the requested field.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%
% Outputs:
%   value - Value stored in parent.(fieldName).

fieldName = char(fieldName);
if ~isstruct(parent) || ~isfield(parent, fieldName)
    error("AOCS:Config:MissingField", "Missing required config field: %s", displayName);
end
value = parent.(fieldName);
end

function value = optionalField(parent, fieldName, defaultValue)
% Description:
%   Keeps optional JSON metadata reads explicit and local.
%
% Arguments:
%   parent - Struct that may contain fieldName.
%   fieldName - Field name to read.
%   defaultValue - Value returned when fieldName is absent.
%
% Outputs:
%   value - Field value or defaultValue.

fieldName = char(fieldName);
if isstruct(parent) && isfield(parent, fieldName)
    value = parent.(fieldName);
else
    value = defaultValue;
end
end

function value = scalarField(parent, fieldName, displayName, mustBePositive)
% Description:
%   Converts JSON numeric values to double and enforces scalar shape plus
%   optional positivity.
%
% Arguments:
%   parent - Struct containing the requested field.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%   mustBePositive - True when zero and negative values are invalid.
%
% Outputs:
%   value - Validated finite real scalar double.

value = double(requireField(parent, fieldName, displayName));
validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar'}, mfilename, displayName);
if mustBePositive && value <= 0
    error("AOCS:Config:InvalidField", "Config field %s must be positive.", displayName);
end
end

function value = columnField(parent, fieldName, displayName, rows)
% Description:
%   Converts a JSON vector to a MATLAB column vector and checks its length.
%
% Arguments:
%   parent - Struct containing the requested field.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%   rows - Required number of rows.
%
% Outputs:
%   value - rows-by-1 finite real double vector.

value = double(requireField(parent, fieldName, displayName));
value = value(:);
validateattributes(value, {'numeric'}, {'real', 'finite', 'size', [rows 1]}, mfilename, displayName);
end

function value = optionalColumnField(parent, fieldName, displayName, rows, defaultValue)
% Description:
%   Provides optional vector fields while still enforcing the same numeric
%   contract as required vector fields.
%
% Arguments:
%   parent - Struct that may contain fieldName.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%   rows - Required number of rows.
%   defaultValue - Vector used when fieldName is absent.
%
% Outputs:
%   value - rows-by-1 finite real double vector.

fieldName = char(fieldName);
if isstruct(parent) && isfield(parent, fieldName)
    value = double(parent.(fieldName));
    value = value(:);
else
    value = defaultValue(:);
end
validateattributes(value, {'numeric'}, {'real', 'finite', 'size', [rows 1]}, mfilename, displayName);
end

function value = matrixField(parent, fieldName, displayName, rows, cols)
% Description:
%   Converts a JSON matrix to double and enforces exact matrix dimensions.
%
% Arguments:
%   parent - Struct containing the requested field.
%   fieldName - Field name to read.
%   displayName - Human-readable field path for error messages.
%   rows - Required number of rows.
%   cols - Required number of columns.
%
% Outputs:
%   value - rows-by-cols finite real double matrix.

value = double(requireField(parent, fieldName, displayName));
validateattributes(value, {'numeric'}, {'real', 'finite', 'size', [rows cols]}, mfilename, displayName);
end

function euler321 = quaternionToEuler321(q)
% Description:
%   Computes the Aerospace Blockset 6DOF initial Euler orientation from the
%   JSON quaternion. Pitch is clamped before asin to avoid roundoff overflow.
%
% Arguments:
%   q - 4-by-1 scalar-first quaternion [q0 q1 q2 q3]'.
%
% Outputs:
%   euler321 - 3-by-1 [roll pitch yaw]' vector [rad].

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

roll = atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1^2 + q2^2));
pitchArgument = 2 * (q0*q2 - q3*q1);
pitchArgument = min(max(pitchArgument, -1), 1);
pitch = asin(pitchArgument);
yaw = atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2^2 + q3^2));

euler321 = [roll; pitch; yaw];
end
