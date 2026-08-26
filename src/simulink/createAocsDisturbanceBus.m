function AOCS_DisturbanceBus = createAocsDisturbanceBus(targetWorkspace)
% Description:
%   Defines modeled disturbance torque products.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("M_dist_B_Nm", [3 1], "N*m", "Total modeled disturbance torque expressed in body axes");
elems(2) = busElement("M_srp_B_Nm", [3 1], "N*m", "Solar radiation pressure torque expressed in body axes");
elems(3) = busElement("M_aero_B_Nm", [3 1], "N*m", "Free-molecular aerodynamic torque expressed in body axes");
elems(4) = busElement("F_srp_B_N", [3 1], "N", "Solar radiation pressure force expressed in body axes");
elems(5) = busElement("F_srp_I_N", [3 1], "N", "Solar radiation pressure force expressed in ICRF axes");
elems(6) = busElement("a_srp_I_m_s2", [3 1], "m/s^2", "Solar radiation pressure acceleration fed back to the orbit propagator");
elems(7) = busElement("F_aero_B_N", [3 1], "N", "Free-molecular aerodynamic force expressed in body axes");
elems(8) = busElement("F_aero_I_N", [3 1], "N", "Free-molecular aerodynamic force expressed in ICRF axes");
elems(9) = busElement("a_aero_I_m_s2", [3 1], "m/s^2", "Aerodynamic acceleration fed back to the orbit propagator");
elems(10) = busElement("a_dist_I_m_s2", [3 1], "m/s^2", "Total translational disturbance acceleration fed back to the orbit propagator");
elems(11) = busElement("q_dyn_N_m2", 1, "N/m^2", "Dynamic pressure based on DTM2020 total density");
elems(12) = busElement("v_rel_norm_m_s", 1, "m/s", "Spacecraft-atmosphere relative speed");

AOCS_DisturbanceBus = Simulink.Bus;
AOCS_DisturbanceBus.Description = "Disturbance torque product bus";
AOCS_DisturbanceBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AOCS_DisturbanceBus", AOCS_DisturbanceBus);
end
end

function elem = busElement(name, dimensions, unit, description)
elem = Simulink.BusElement;
elem.Name = name;
elem.Dimensions = dimensions;
elem.DataType = "double";
elem.Unit = unit;
elem.Description = description;
end
