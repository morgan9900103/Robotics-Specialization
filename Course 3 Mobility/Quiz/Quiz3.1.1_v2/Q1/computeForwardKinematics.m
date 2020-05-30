function endeff = computeForwardKinematics(rads)

% Parameters
L = 1;
theta = 2;

x = L * cos(theta);
y = L * sin(theta);


endeff = [x,y];

