function desired_state = traj_step(t, ~)

acc = [0;0;0];
vel = [0;0;0];
if t < 0.2
  pos = [0;0;0];
else
  pos = [1.0;0.0;0.0];
end

desired_state.pos = pos;
desired_state.vel = vel;
desired_state.acc = acc;
desired_state.yaw = 0;
desired_state.yawdot = 0;

end
