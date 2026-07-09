function invariants = computeAocsInvariants(omega_b, I_B)
% Description:
%   Computes angular momentum in body axes, rotational kinetic energy, and
%   angular momentum norm for torque-free validation and plotting.
%
% Arguments:
%   omega_b - N-by-3 body angular rate history [rad/s].
%   I_B - 3-by-3 inertia matrix expressed in body axes [kg*m^2].
%
% Outputs:
%   invariants - Struct with H_B, E_rot, and H_norm histories.

H_B = (I_B * omega_b.').';

invariants = struct();
invariants.H_B = H_B;
invariants.E_rot = 0.5 * sum(omega_b .* H_B, 2);
invariants.H_norm = sqrt(sum(H_B.^2, 2));
end
