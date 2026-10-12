function [xNext, valid, Phi] = propagateOrbitJ2(x, dt, mu, Re, J2, pole_I) %#codegen
%PROPAGATEORBITJ2 Propagate the orbital state with central gravity and J2.

xNext = x;
valid = false;
Phi = eye(6);

if any(~isfinite(x)) || any(~isfinite(pole_I)) ...
        || any(~isfinite([dt; mu; Re; J2])) ...
        || dt < 0 || dt > 60 || mu <= 0 || Re <= 0 || J2 < 0 ...
        || norm(x(1:3)) <= Re || norm(pole_I) < 1e-12
    return
end

pole_I = pole_I / norm(pole_I);

if dt == 0
    valid = true;
    return
end

steps = ceil(dt / 1.0);
h = dt / steps;

candidate = x;
candidatePhi = Phi;

for k = 1:60
    if k > steps
        break
    end

    k1 = dynamicsJ2(candidate, mu, Re, J2, pole_I);

    [A1, jacobianValid] = ...
        orbitJacobianJ2(candidate, mu, Re, J2, pole_I);
    
    if ~jacobianValid
        return
    end
    
    L1 = A1 * candidatePhi;

    x2 = candidate + h/2 * k1;
    Phi2 = candidatePhi + h/2 * L1;
    
    k2 = dynamicsJ2(x2, mu, Re, J2, pole_I);
    
    [A2, jacobianValid] = ...
        orbitJacobianJ2(x2, mu, Re, J2, pole_I);
    
    if ~jacobianValid
        return
    end

    L2 = A2 * Phi2;

    x3 = candidate + h/2 * k2;
    Phi3 = candidatePhi + h/2 * L2;
    
    k3 = dynamicsJ2(x3, mu, Re, J2, pole_I);
    
    [A3, jacobianValid] = ...
        orbitJacobianJ2(x3, mu, Re, J2, pole_I);
    
    if ~jacobianValid
        return
    end
    
    L3 = A3 * Phi3;
    
    x4 = candidate + h * k3;
    Phi4 = candidatePhi + h * L3;
    
    k4 = dynamicsJ2(x4, mu, Re, J2, pole_I);
    
    [A4, jacobianValid] = ...
        orbitJacobianJ2(x4, mu, Re, J2, pole_I);
    
    if ~jacobianValid
        return
    end
    
    L4 = A4 * Phi4;

    candidatePhi = candidatePhi + h/6*(L1 + 2*L2 + 2*L3 + L4);
    candidate = candidate + h/6*(k1 + 2*k2 + 2*k3 + k4);
 
    if any(~isfinite(candidate)) ...
            || any(~isfinite(candidatePhi(:))) ...
            || norm(candidate(1:3)) <= Re
        return
    end

end

xNext = candidate;
Phi = candidatePhi;
valid = true;
end

function derivative = dynamicsJ2(x, mu, Re, J2, pole_I)
%DYNAMICSJ2 Compute orbital derivatives with central gravity and J2.

r_I = x(1:3);
rNorm = norm(r_I);
z = dot(r_I, pole_I);
s = z^2 / rNorm^2;

central = -mu * r_I / rNorm^3;
factor = 1.5 * J2 * mu * Re^2 / rNorm^5;
oblateness = factor * ((5*s - 1)*r_I - 2*z*pole_I);

derivative = [x(4:6); central + oblateness];
end
