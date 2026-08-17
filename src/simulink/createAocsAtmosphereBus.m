function AOCS_AtmosphereBus = createAocsAtmosphereBus(targetWorkspace)
% Description:
%   Defines atmosphere runtime products consumed by aerodynamic force and
%   torque models.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   AOCS_AtmosphereBus - Simulink.Bus object for atmosphere products.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("rho_kg_m3", 1, "kg/m^3", ...
    "Scaled neutral mass density");
elems(2) = busElement("rho_raw_kg_m3", 1, "kg/m^3", ...
    "Unscaled DTM2020 neutral mass density");
elems(3) = busElement("rho_uncertainty_1sigma_kg_m3", 1, "kg/m^3", ...
    "One-sigma neutral density uncertainty");
elems(4) = busElement("T_local_K", 1, "K", ...
    "Local neutral temperature");
elems(5) = busElement("T_exo_K", 1, "K", ...
    "Exospheric temperature");
elems(6) = busElement("n_O_m3", 1, "1/m^3", ...
    "Atomic oxygen number density");
elems(7) = busElement("n_N2_m3", 1, "1/m^3", ...
    "Molecular nitrogen number density");
elems(8) = busElement("n_O2_m3", 1, "1/m^3", ...
    "Molecular oxygen number density");
elems(9) = busElement("n_He_m3", 1, "1/m^3", ...
    "Helium number density");
elems(10) = busElement("n_H_m3", 1, "1/m^3", ...
    "Atomic hydrogen number density");
elems(11) = busElement("n_N_m3", 1, "1/m^3", ...
    "Atomic nitrogen number density");
elems(12) = busElement("v_atm_I_m_s", [3 1], "m/s", ...
    "Atmosphere co-rotation and wind velocity expressed in inertial axes");

AOCS_AtmosphereBus = Simulink.Bus;
AOCS_AtmosphereBus.Description = "Runtime atmosphere product bus";
AOCS_AtmosphereBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AOCS_AtmosphereBus", AOCS_AtmosphereBus);
end
end

function elem = busElement(name, dimensions, unit, description)
% Description:
%   Keeps bus element construction compact and consistent.

elem = Simulink.BusElement;
elem.Name = name;
elem.Dimensions = dimensions;
elem.DataType = "double";
elem.Unit = unit;
elem.Description = description;
end
