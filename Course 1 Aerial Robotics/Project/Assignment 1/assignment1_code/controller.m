function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

Kv = 20;
Kp = 200;
e = s_des(1) - s(1);
edot = s_des(2) - s(2);

u = params.mass * (0 + Kp * e + Kv * edot + params.gravity);


% FILL IN YOUR CODE HERE


end

