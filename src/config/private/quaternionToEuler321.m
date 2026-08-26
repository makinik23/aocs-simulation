function euler321 = quaternionToEuler321(q)
%QUATERNIONTOEULER321 Convert scalar-first quaternion to 3-2-1 Euler angles.

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
