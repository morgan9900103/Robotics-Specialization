function [ desired_state ] = fixed_set_point(t, z_des)
%FIXED_SET_POINT  Outputs a constant desired state = [z_des;0] except at t = 0 where it returns [0;0]

if t == 0
  desired_state.pos = [0; 0; 0];
  desired_state.vel = [0; 0; 0];
  desired_state.acc = [0; 0; 0];
  desired_state.yaw = 0;
  desired_state.yawdot = 0;
else
  desired_state.pos = [0; 0; 1];
  desired_state.vel = [0; 0; 0];
  desired_state.acc = [0; 0; 0];
  desired_state.yaw = 0;
  desired_state.yawdot = 0;
end

end
