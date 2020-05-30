function u = controllerNoisy(params, t, obs)
    % This is the starter file for the week5 assignment
    % Now you only receive noisy measurements for phi, and must use your EKF from week 3 to filter the data and get an estimate of the state
    % obs = [ay; az; gx] with a* in units of g's, and gx in units of rad/s

    % This template code calls the function EKFupdate that you must complete below
    xhat = EKFupdate(params, t, obs);
    phi = xhat(1);
    phidot = xhat(2);

    % The rest of this function should ideally be identical to your solution in week 4
    % Student completes this
    
    % Initialize
    persistent error t_last
    if isempty(error)
      error = 0;
      t_last = 0;
    end

    kp = 9;
    kd = 0.09;
    ki = 100;

    dt = t - t_last;
    error = error + phi*dt;

    u = kp*phi + kd*phidot + ki*error;
    t_last = t;

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
