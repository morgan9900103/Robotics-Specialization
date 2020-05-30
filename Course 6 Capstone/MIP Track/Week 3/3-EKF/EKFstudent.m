
function xhat = EKFstudent(t, z)
    % In this exercise, you will batch-process this data:
    % you are provided a vector of timestamps (of length T), and a 3xT matrix of observations, z.
    xhat = zeros(2,length(t));
    xhat(:, 1) = [0, 0]';

    T = length(t);
    
    % Noise
    Q = diag([0.1,0.1]);
    R = diag([0.001, 0.001, 0.001]);
    
    % Initialize
    P0 = 5 * ones(2);
    
    for k = 2:T
        % dt
        dt = t(k) - t(k-1);
        
        % Matrices
        % A - 2 x 2
        % Xkp - 2 x 1
        % h - 3 x 1
        % H - 3 x 2
        A = [1 dt; 0 1];
        xkp = A * xhat(:, k-1);
        h = [sind(xkp(1)); cosd(xkp(1)); xkp(2)];
        H = [cosd(xkp(1))*(pi/180), 0; -sind(xkp(1))*(pi/180), 0; 0, 1];
        
        % Predict
        % P1 - 2 x 2
        P1 = A*P0*A' + Q;
        
        % Optimal Kalman Gain
        % K - 2 x 2
        K = P1*H'/(H*P1*H' + R);
        
        % Update
        xhat(:, k) = xkp + K*(z(:, k) - h);
        P0 = (eye(2) - K*H)*P1;
        
    end
  % Student completes this
end
