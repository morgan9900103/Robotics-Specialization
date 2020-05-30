function [endeff] = computeMiniForwardKinematics(rads1,rads2)

% Parameters
L1 = 1;
L2 = 2;
theta1 = 3.5;
theta2 = 1.5;

q = (theta1+theta2)/2;
d = L1*sin(q-theta2);
phi = asin(d/L2);
psi = q - phi + pi - theta2;

x1 = L1*cos(theta2);
y1 = L1*sin(theta2);

x2 = x1+L2*cos(theta2+psi);
y2 = y1+L2*sin(theta2+psi);

endeff = [x2,y2];