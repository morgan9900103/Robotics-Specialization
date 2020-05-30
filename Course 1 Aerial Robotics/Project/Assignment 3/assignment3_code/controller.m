function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Gains
Kp_x        = 500;
Kd_x        = 10;
Kp_y        = 500;
Kd_y        = 10;
Kp_z        = 500;
Kd_z        = 10;

Kp_phi      = 100;
Kd_phi      = 1;
Kp_theta    = 100;
Kd_theta    = 1;
Kp_psi      = 100;
Kd_psi      = 1;

r_ddot_1_des = des_state.acc(1) + Kd_x * (des_state.vel(1) - state.vel(1)) + Kp_x * (des_state.pos(1) - state.pos(1));
r_ddot_2_des = des_state.acc(2) + Kd_y * (des_state.vel(2) - state.vel(2)) + Kp_y * (des_state.pos(2) - state.pos(2));
r_ddot_3_des = des_state.acc(3) + Kd_z * (des_state.vel(3) - state.vel(3)) + Kp_z * (des_state.pos(3) - state.pos(3));

des_phi     = (r_ddot_1_des * sin(des_state.yaw) - r_ddot_2_des * cos(des_state.yaw))/params.gravity;
des_theta   = (r_ddot_1_des * cos(des_state.yaw) + r_ddot_2_des * sin(des_state.yaw))/params.gravity;
des_psi     = des_state.yaw;

M1 = Kp_phi   * (des_phi - state.rot(1))   + Kd_phi   * (0 - state.omega(1));
M2 = Kp_theta * (des_theta - state.rot(2)) + Kd_theta * (0 - state.omega(2));
M3 = Kp_psi   * (des_psi - state.rot(3))   + Kd_psi   * (des_state.yawdot - state.omega(3));

% Thrust
F = params.mass * (params.gravity + r_ddot_3_des);

% Moment
M = [M1; M2; M3];

% =================== Your code ends here ===================

end
