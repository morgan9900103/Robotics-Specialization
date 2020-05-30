function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% Gains
Kpz = 50;
Kvz = 20;
Kpy = 25;
Kvy = 8;
Kpphi = 60;
Kvphi = 5;

% Errors
ez = des_state.pos(2) - state.pos(2);
ez_dot = des_state.vel(2) - state.vel(2);
ey = des_state.pos(1) - state.pos(1);
ey_dot = des_state.vel(1) - state.vel(1);

phi_c = -1/params.gravity * (des_state.acc(1) + Kvy * ey_dot + Kpy * ey);

ephi = phi_c - state.rot;
ephi_dot = 0 - state.omega;

% controller

u1 = params.mass * (params.gravity + des_state.acc(2) + Kvz * ez_dot + Kpz * ez);
u2 = Kvphi * ephi_dot + Kpphi * ephi;


% FILL IN YOUR CODE HERE

end

