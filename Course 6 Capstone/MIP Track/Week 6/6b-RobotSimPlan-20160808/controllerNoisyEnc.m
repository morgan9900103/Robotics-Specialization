
function u = controllerNoisyEnc(params, t, obs, th, dth)
    % This is the starter file for the week5 assignment
    % Now you only receive noisy measurements for theta, and must use your EKF from week 3 to filter the data and get an estimate of the state
    % obs = [ay; az; gx] (same as last week)
    % New for 6b: you also have access to params.traj(t)

    % Template code (same as last week)
    xhat = EKFupdate(params, t, obs);
    phi = xhat(1);
    phidot = xhat(2);
    
    % Position Controller
    x = params.r*(th + phi);
    xdot = params.r*(dth + phidot);
    
    % Gains
    kpx = 0.2;
    kdx = 0.3;
    
    phides = -kpx*(x - params.traj(t)) - kdx*xdot;
    phides = asin(phides);
    
    % Balancing Controller
    kpphi = 0.095;
    kdphi = 0.008;
    
    u = kpphi*sin(phi - phides) + kdphi*phidot;
end

function xhatOut = EKFupdate(params, t, z)
    % z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
    % You can borrow most of your week 3 solution, but this must only implement a single predict-update step of the EKF
    % Recall (from assignment 5b) that you can use persistent variables to create/update any additional state that you need.

    % Student completes this
    % Noise
    Q = diag([0.01 0.01]);
    R = diag([0.1 0.1 0.1]);

    persistent x_last P t_last
    if isempty(P)
      P = eye(2);
      t_last = 0;
      x_last = [0;0];
    end
    
    % dt
    dt = t - t_last;
    
    % Matrices
    A = [1 dt; 0 1];
    x_last = A*x_last;

    % Predict
    P = A*P*A' + Q;
    
    phi = x_last(1);
    phidot = x_last(2);

    H = [cos(phi), 0; -sin(phi), 0; 0, 1];

    % Optimal Kalman Gain
    K = P*H'/(H*P*H' + R);
    
    h = [sin(phi); cos(phi); phidot];

    % Update
    x_last = x_last + K*(z - h);
    P = (eye(2) - K*H)*P;
    t_last = t;
    
    xhatOut = x_last;
end
