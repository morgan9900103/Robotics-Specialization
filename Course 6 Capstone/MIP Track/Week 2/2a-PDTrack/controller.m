
function u = controller(params, t, x, xd)
  % x = current position
  % xd = current velocity

  % Use params.traj(t) to get the reference trajectory
  % e.g. (x - params.traj(t)) represents the instaneous trajectory error

  % params can be initialized in the initParams function, which is called before the simulation starts
  
  % SOLUTION GOES HERE -------------
  
    % Duffing equation - x_ddot + delts * x_dot + alpha * x + beta * x^3 = gamma * cos(omega * t)
    
    delta = 150;
    alpha = 600;
    beta = 50;
    gamma = 0.01;   
    omega = 1;
    e = params.traj(t) - x;

    u = -delta*xd + alpha*e + beta*e^3 + gamma*cos(omega*t);
  
  
end