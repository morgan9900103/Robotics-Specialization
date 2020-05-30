function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)
    dt = 0.033;
    
    % Noise Covariance Matrices
    % sigma_m - 4 x 4
    % sigma_o - 2 x 2
    sigma_px = 1*exp(6);
    sigma_py = 1*exp(6);
    sigma_vx = 1*exp(6);
    sigma_vy = 1*exp(6);
    Sigma_m = [sigma_px^2 0 0 0; 0 sigma_py^2 0 0; 0 0 sigma_vx^2 0; 0 0 0 sigma_vy^2];
    
    sigma_zx = 0.01;
    sigma_zy = 0.01;
    Sigma_o = eye(2)*[sigma_zx^2 0; 0 sigma_zy^2];
    
    % Transition Matrix - 4 x 4
    A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
    
    % Observation Matrix - 2 x 4
    C = [1 0 0 0; 0 1 0 0];

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0]';
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % % As an example, here is a Naive estimate without a Kalman filter
    % % You should replace this code
    % vx = (x - state(1)) / (t - previous_t);
    % vy = (y - state(2)) / (t - previous_t);
    % % Predict 330ms into the future
    % predictx = x + vx * 0.330;
    % predicty = y + vy * 0.330;
    % % State is a four dimensional element
    % state = [x, y, vx, vy];
    
    % Predicted State & Covariance Matrix
    % Xkp - 4 x 1
    % Pkp - 4 x 4
    Xkp = A*state; 
    Pkp = A*param.P*A' + Sigma_m;
    
    % Kalman Gain - 4 x 2
    K = Pkp*C'/(C*Pkp*C' + Sigma_o);
    
    % Z Measurement - 2 x 1
    Z = C*[x y 0 0]';
    
    % Update Covariance Matrix
    param.P = Pkp - K*C*Pkp;
    
    % Update State
    state = Xkp + K*(Z - C*Xkp);
    
    % Prediction
    predictx = state(1);
    predicty = state(2);
    
end
