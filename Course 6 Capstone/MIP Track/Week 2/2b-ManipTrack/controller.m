
function u = controller(params, t, X)
  u = [0;0];
  % 1. write out the forward kinematics, such that p = FK(theta1, theta2)
  % 2. Let e = p - params.traj(t) be the task-space error
  % 3. Calculate the manipulator Jacobian J = d p / d theta
  % 4. Use a "natural motion" PD controller, u = - kp * J^T * e - kd * [dth1; dth2]
  
  % X - [th1; th2; dth1; dth2]
  % p = l(cos(th1), sin(th1)) + l(cos(th2), sin(th2))
  % J = D*p
  % u = -kp*J'(p-r(t)) - kd*q_dot, q = [th1, th2], r(t) = params.traj(t)
  
  % Gains
  kp = 10000;
  kd = 10;
  
  p = params.l*[cos(X(1)); sin(X(1))] + params.l*[cos(X(1) + X(2)); sin(X(1) + X(2))];
  
  e = p - params.traj(t);
  
  J = params.l*[-sin(X(1)) - sin(X(1)+X(2)), -sin(X(1)+X(2));...
                 cos(X(1)) + cos(X(1)+X(2)),  cos(X(1)+X(2))];
  
  u = -kp*J'*e - kd*[X(3); X(4)];
  
end

