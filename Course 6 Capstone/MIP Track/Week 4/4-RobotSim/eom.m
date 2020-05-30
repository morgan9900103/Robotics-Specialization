function qdd = eom(params, th, phi, dth, dphi, u)
    % This is the starter file for the week5 assignment

    % Provided params are
    % params.g: gravitational constant
    % params.mr: mass of the "rod"
    % params.ir: rotational inertia of the rod
    % params.d: distance of rod CoM from the wheel axis
    % params.r: wheel radius

    % Provided states are:
    % th: wheel angle (relative to body)
    % phi: body pitch
    % dth, dphi: time-derivatives of above
    % u: torque applied at the wheel

    g = params.g;
    m = params.mr;
    i = params.ir;
    l = params.d;
    r = params.r;
    
    % A*ddth + B*ddphi = C
    % D*ddth + E*ddphi = F
    A = m*r^2;
    B = m*r^2 + m*r*l*cos(phi);
    C = u + m*r*l*dphi^2*sin(phi);
    D = m*r^2 + m*r*l*cos(phi);
    E = m*r^2 + m*l^2 + 2*m*r*l*cos(phi) + i;
    F = m*g*l*sin(phi) + m*r*l*dphi^2*sin(phi);
    
    qdd = [A, B; D, E]\[C; F];

end