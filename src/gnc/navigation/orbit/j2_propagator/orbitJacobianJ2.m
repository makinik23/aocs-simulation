function [A, valid] = orbitJacobianJ2(x, mu, Re, J2, pole_I) %#codegen
    %ORBITJACOBIANJ2 Compute the orbital dynamics Jacobian with J2.
    
    A = zeros(6);
    valid = false;
    
    if any(~isfinite(x)) || any(~isfinite(pole_I)) ...
            || any(~isfinite([mu; Re; J2])) ...
            || mu <= 0 || Re <= 0 || J2 < 0 ...
            || norm(x(1:3)) <= Re || norm(pole_I) < 1e-12
        return
    end
    
    n_I = pole_I / norm(pole_I);
    r_I = x(1:3);

    rNorm = norm(r_I);
    rSquared = rNorm^2;

    z = dot(r_I, n_I);
    s = z^2 / rSquared;
    rr = r_I * r_I.' / rSquared;
    
    Gcentral = mu / rNorm^3 * (3*rr - eye(3));
    factor = 1.5 * J2 * mu * Re^2 / rNorm^5;
    Gj2 = factor * ((5*s - 1)*eye(3) - 2*(n_I*n_I.') ...
        + 10*z/rSquared * (r_I*n_I.' + n_I*r_I.') ...
        + (5 - 35*s)*rr);
    
    A = [zeros(3), eye(3); Gcentral + Gj2, zeros(3)];

    valid = all(isfinite(A(:)));
    if ~valid
        A = zeros(6);
    end
end
